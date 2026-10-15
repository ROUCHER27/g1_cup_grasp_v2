from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import struct
from contextlib import ExitStack
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("navigate_station", ROOT / "tools" / "navigate_station.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


def sample(stamp=1, **changes):
    result = {
        "agv_status": 8, "map_id": 6, "position_conf": 99.0,
        "agv_pos_x": 1.542, "agv_pos_y": 1.344, "agv_angle": 0.0,
        "linear_speed": 0.0, "angular_speed": 0.0, "timestamp_ns": stamp,
    }
    result.update(changes)
    return result


class FakeChassisInfoSlam:
    def __init__(self, reflect):
        self.reflect = reflect
        self.values = {"max_linear_speed": 0.6, "max_angular_speed": 0.8,
                       "min_collision_distance": 0.15}
        self.timestamp_ns = 0
        self.chassis_info_writes = []

    def get_chassis_info(self):
        self.timestamp_ns += 1
        return dict(self.values), self.timestamp_ns

    def set_chassis_info(self, max_linear_speed, max_angular_speed, min_collision_distance):
        requested = (max_linear_speed, max_angular_speed, min_collision_distance)
        self.chassis_info_writes.append(requested)
        if self.reflect:
            self.values = dict(zip(("max_linear_speed", "max_angular_speed", "min_collision_distance"),
                                   requested))

    def set_agv_col_level(self, level):
        return SimpleNamespace(error_code=0)

    def set_agv_col_ctrl(self, enabled):
        return SimpleNamespace(success=enabled is True)


class NavigationExecutionTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / "tests/fixtures/transport_config_test.yaml").read_text())
        self.samples = [sample(index) for index in range(1, 6)]
        self.hmi = {
            "map_assignment_matches": True,
            "state": {"isNormal": True, "isScram": False, "isEdged": False, "isPause": False,
                      "isHardAuto": True, "isSoftAuto": True,
                      "batteryStateList": [{"isCharging": False}]},
        }
        self.controller = {"map_selection_matches": True, "serial_number": "424", "auto_motion_parameters": {"maxSpeed": 100}}
        self.target = self.config["stations"]["B"]

    def test_transport_test_fixture_keeps_navigation_examples_stable(self):
        config = yaml.safe_load((ROOT / "tests/fixtures/transport_config_test.yaml").read_text())
        self.assertEqual(config["map_id"], 6)
        self.assertEqual(config["stations"]["A"], {"x_m": 1.542, "y_m": 1.344, "yaw_rad": 0.0})
        self.assertEqual(config["stations"]["B"], {"x_m": 0.646, "y_m": 1.098, "yaw_rad": math.pi})

    def test_runbook_uses_native_feedback_no_cup_and_labeled_reports(self):
        runbook = (ROOT / "docs/field_test_runbook.md").read_text(encoding="utf-8")
        self.assertIn("python tools/navigate_station.py --station A --native-feedback", runbook)
        self.assertIn("--dispatch --native-feedback --no-cup --carry-clearance-verified", runbook)
        self.assertIn("--label obstacle", runbook)
        self.assertIn("no_progress_timeout_s` 设为 25 s", runbook)
        self.assertNotIn("需给它接原生反馈", runbook)
        self.assertNotIn("报告 `cp` 一份", runbook)
        self.assertNotIn("no_progress_timeout_s` 也是 10 s", runbook)

    def test_request_uses_mm_and_degrees_while_config_remains_si(self):
        station_b = {"x_m": 0.646, "y_m": 1.098, "yaw_rad": math.pi}
        request = nav.navigation_request(station_b)
        self.assertEqual((request["x_mm"], request["y_mm"], request["yaw_deg"]), (646, 1098, 180))
        request = nav.navigation_request({"x_m": 1.542, "y_m": 1.344, "yaw_rad": 0.0})
        self.assertEqual((request["x_mm"], request["y_mm"], request["yaw_deg"]), (1542, 1344, 0))
        self.assertEqual(station_b["x_m"], 0.646)
        self.assertEqual(nav.navigation_request({**station_b, "yaw_rad": -math.pi})["yaw_deg"], 180)
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            nav.navigation_request({**station_b, "x_m": float("nan")})

    def test_readme_collision_level_matches_hal_verified_sensitivity(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("config/transport.yaml` 当前设为 2", readme)
        self.assertIn("1→0.8×、2→1.0×、3→0.8×", readme)
        self.assertIn("g1_nav_api_report.md` §1.1", readme)
        self.assertNotIn("config/transport.yaml` 当前设为 3", readme)

    def test_native_map_is_required_even_with_matching_controller_configuration(self):
        nav.validate_preflight(self.samples, self.hmi, self.controller, self.config)
        for map_id in (None, 0, 5, "6"):
            samples = [{**s, "map_id": map_id} for s in self.samples]
            with self.assertRaisesRegex(RuntimeError, "runtime map ID"):
                nav.validate_preflight(samples, self.hmi, self.controller, self.config)
        frozen = [{**s, "timestamp_ns": 1} for s in self.samples]
        self.assertFalse(nav.runtime_map_check(frozen, 6)["verified"])

    def test_unknown_charging_and_non_finite_feedback_are_not_safe(self):
        self.hmi["state"]["batteryStateList"] = []
        with self.assertRaisesRegex(RuntimeError, "charging state is unknown"):
            nav.validate_preflight(self.samples, self.hmi, self.controller, self.config)
        self.hmi["state"]["batteryStateList"] = [{"isCharging": False}]
        self.samples[-1]["linear_speed"] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "invalid preflight"):
            nav.validate_preflight(self.samples, self.hmi, self.controller, self.config)

    def call_main(self, args, config, fake_module):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            (root / "config/transport.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
            with patch.object(nav, "PROJECT_ROOT", root), patch.dict(
                "sys.modules", {"a2d_sdk.robot": fake_module}
            ), patch("builtins.print"):
                result = nav.main(args)
            path = next((root / "outputs/navigation_check").glob("*.json"))
            return result, json.loads(path.read_text(encoding="utf-8"))

    def test_disabled_dispatch_and_offline_preview_never_construct_sdk(self):
        module = ModuleType("a2d_sdk.robot")
        module.Slam = Mock(side_effect=AssertionError("SDK must not be initialized"))
        code, report = self.call_main(
            ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
            self.config, module
        )
        self.assertEqual(code, 1)
        self.assertTrue(report["read_only"])
        self.assertFalse(report["request_attempted"])
        module.Slam.assert_not_called()
        code, report = self.call_main(["--station", "B", "--preview"], self.config, module)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "OFFLINE_REQUEST_PREVIEW")
        expected = nav.navigation_request(self.config["stations"]["B"])
        self.assertEqual(report["planned_request"]["x_mm"], expected["x_mm"])
        module.Slam.assert_not_called()

    def test_preview_label_is_added_to_report_filename(self):
        module = ModuleType("a2d_sdk.robot")
        module.Slam = Mock(side_effect=AssertionError("SDK must not be initialized"))
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            (root / "config/transport.yaml").write_text(yaml.safe_dump(self.config), encoding="utf-8")
            with ExitStack() as stack:
                stack.enter_context(patch.object(nav, "PROJECT_ROOT", root))
                stack.enter_context(patch.dict("sys.modules", {"a2d_sdk.robot": module}))
                stack.enter_context(patch("builtins.print"))
                code = nav.main(["--station", "B", "--preview", "--label", "obstacle"])
            paths = list((root / "outputs/navigation_check").glob("*.json"))
        self.assertEqual(code, 0)
        self.assertEqual(len(paths), 1)
        self.assertTrue(paths[0].name.endswith("_B_navigation_obstacle.json"))
        module.Slam.assert_not_called()

    def test_dispatch_requires_native_feedback_before_sdk_initialization(self):
        module = ModuleType("a2d_sdk.robot")
        module.Slam = Mock(side_effect=AssertionError("SDK must not be initialized"))
        code, report = self.call_main(
            ["--station", "B", "--dispatch", "--carry-clearance-verified"],
            {**self.config, "enable_real_navigation": True}, module,
        )
        self.assertEqual(code, 1)
        self.assertIn("--native-feedback", report["error"])
        self.assertTrue(report["read_only"])
        self.assertFalse(report["request_attempted"])
        module.Slam.assert_not_called()

    def test_dispatch_uses_native_feedback_for_samples_and_navigation(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[self.config["map_id"]])
        module.Slam = Mock(return_value=slam)
        native = Mock()
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        with ExitStack() as stack:
            native_factory = stack.enter_context(
                patch.object(nav, "NativeChassisFeedback", return_value=native))
            collect = stack.enter_context(
                patch.object(nav, "collect_samples", return_value=self.samples))
            stack.enter_context(patch.object(nav, "read_hmi", return_value=self.hmi))
            stack.enter_context(patch.object(nav, "read_controller", return_value=self.controller))
            stack.enter_context(patch.object(nav, "validate_preflight"))
            stack.enter_context(patch.object(nav, "load_config", return_value={"robot": {}}))
            stack.enter_context(patch.object(nav, "GdkRightArm", return_value=arm))
            execute = stack.enter_context(patch.object(nav, "execute_navigation"))
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
                {**self.config, "enable_real_navigation": True}, module,
            )
        self.assertEqual(code, 0, report)
        native_factory.assert_called_once_with(self.config["native_feedback"])
        collect.assert_called_once_with(native)
        self.assertIs(execute.call_args.args[1], native)
        slam.get_map_list.assert_called_once_with()
        native.close.assert_called_once_with()

    def test_no_cup_main_accepts_empty_gripper_and_records_flag(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[self.config["map_id"]])
        module.Slam = Mock(return_value=slam)
        native = Mock()
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[120.0, 0.0])
        with ExitStack() as stack:
            stack.enter_context(patch.object(nav, "NativeChassisFeedback", return_value=native))
            stack.enter_context(patch.object(nav, "collect_samples", return_value=self.samples))
            stack.enter_context(patch.object(nav, "read_hmi", return_value=self.hmi))
            stack.enter_context(patch.object(nav, "read_controller", return_value=self.controller))
            preflight = stack.enter_context(patch.object(nav, "validate_preflight"))
            stack.enter_context(patch.object(nav, "load_config", return_value={"robot": {}}))
            arm_factory = stack.enter_context(patch.object(nav, "GdkRightArm", return_value=arm))
            execute = stack.enter_context(patch.object(nav, "execute_navigation"))
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--no-cup",
                 "--carry-clearance-verified"],
                {**self.config, "enable_real_navigation": True}, module,
            )
        self.assertEqual(code, 0, report.get("error"))
        self.assertTrue(report["no_cup"])
        preflight.assert_called_once()
        arm_factory.assert_called_once()
        self.assertTrue(execute.call_args.kwargs["no_cup"])

    def test_no_cup_keeps_the_carry_clearance_preflight_gate(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[self.config["map_id"]])
        module.Slam = Mock(return_value=slam)
        native = Mock()
        with ExitStack() as stack:
            stack.enter_context(patch.object(nav, "NativeChassisFeedback", return_value=native))
            stack.enter_context(patch.object(nav, "collect_samples", return_value=self.samples))
            stack.enter_context(patch.object(nav, "read_hmi", return_value=self.hmi))
            stack.enter_context(patch.object(nav, "read_controller", return_value=self.controller))
            preflight = stack.enter_context(patch.object(nav, "validate_preflight"))
            arm_factory = stack.enter_context(patch.object(nav, "GdkRightArm"))
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--no-cup"],
                {**self.config, "enable_real_navigation": True}, module,
            )
        self.assertEqual(code, 1)
        self.assertTrue(report["no_cup"])
        self.assertIn("cup/arm route clearance is unverified", report["error"])
        preflight.assert_called_once()
        arm_factory.assert_not_called()

    def test_enabling_flag_does_not_bypass_missing_runtime_map(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        module.Slam = Mock(return_value=slam)
        native = Mock()
        with patch.object(nav, "NativeChassisFeedback", return_value=native), \
             patch.object(nav, "collect_samples", return_value=[{**s, "map_id": None} for s in self.samples]), \
             patch.object(nav, "read_hmi", return_value=self.hmi), \
             patch.object(nav, "read_controller", return_value=self.controller), \
             patch.object(nav, "GdkRightArm") as arm:
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
                {**self.config, "enable_real_navigation": True}, module,
            )
        self.assertEqual(code, 1)
        self.assertIn("runtime map ID", report["error"])
        slam.navigate_to_pose.assert_not_called()
        slam.switch_nav_mode.assert_not_called()
        slam.pause_walk.assert_not_called()
        arm.assert_not_called()

    def watchdog(self):
        return nav.NavigationWatchdog(self.target, 6, self.samples[-1], 0.0)

    def test_watchdog_rejects_stale_feedback_and_low_confidence(self):
        watchdog = self.watchdog()
        with self.assertRaisesRegex(RuntimeError, "stale"):
            watchdog.update(sample(5), 5, 2.1)
        watchdog = self.watchdog()
        with self.assertRaisesRegex(RuntimeError, "confidence is below 50"):
            watchdog.update(sample(6, position_conf=49.0), 6, 0.1)


    def test_confidence_drop_is_allowed_while_at_least_50(self):
        for confidence in (83.,70.,60.,50.):
            result=self.watchdog().update(sample(6,position_conf=confidence),6,.1)
            self.assertFalse(result["arrived"])

    def test_preflight_confidence_boundary_is_50(self):
        samples=[{**s,"position_conf":50.} for s in self.samples]
        nav.validate_preflight(samples,self.hmi,self.controller,self.config)
        samples[-1]["position_conf"]=49.
        with self.assertRaisesRegex(RuntimeError,"confidence is below 50"):
            nav.validate_preflight(samples,self.hmi,self.controller,self.config)

    def test_linear_speed_has_no_trial_cap(self):
        for speed in (.104,-.104,.2,-.2):
            result=self.watchdog().update(sample(6,agv_status=7,linear_speed=speed),6,.1)
            self.assertFalse(result["arrived"])
        for speed in (float("nan"),float("inf")):
            with self.assertRaisesRegex(RuntimeError,"invalid chassis feedback"):
                self.watchdog().update(sample(6,linear_speed=speed),6,.1)

    def test_configured_linear_speed_has_no_trial_cap(self):
        for speed in (150,200):
            self.controller["auto_motion_parameters"]["maxSpeed"]=speed
            nav.validate_preflight(self.samples,self.hmi,self.controller,self.config)

    def test_linear_movement_at_goal_is_not_arrival(self):
        watchdog=self.watchdog()
        at_goal=sample(6,agv_pos_x=self.target["x_m"],agv_pos_y=self.target["y_m"],
                       agv_angle=self.target["yaw_rad"],linear_speed=.104)
        self.assertFalse(watchdog.update(at_goal,6,.1)["arrived"])
        at_goal["linear_speed"]=0.
        self.assertFalse(watchdog.update(at_goal,7,.2)["arrived"])
        self.assertFalse(watchdog.update(at_goal,8,.8)["arrived"])
        self.assertTrue(watchdog.update(at_goal,9,1.3)["arrived"])

    def test_turning_has_no_trial_speed_cap(self):
        for speed in (.2094, -.2094, .5, -.5):
            result=self.watchdog().update(sample(6, agv_status=7, angular_speed=speed),6,.1)
            self.assertFalse(result["arrived"])
        for speed in (float("nan"),float("inf")):
            with self.assertRaisesRegex(RuntimeError,"invalid chassis feedback"):
                self.watchdog().update(sample(6,angular_speed=speed),6,.1)

    def test_turning_at_goal_is_not_arrival(self):
        watchdog=self.watchdog()
        at_goal=sample(6,agv_pos_x=self.target["x_m"],agv_pos_y=self.target["y_m"],
                       agv_angle=self.target["yaw_rad"],angular_speed=.5)
        self.assertFalse(watchdog.update(at_goal,6,.1)["arrived"])
        at_goal["angular_speed"]=0.
        self.assertFalse(watchdog.update(at_goal,7,.2)["arrived"])
        self.assertFalse(watchdog.update(at_goal,8,.8)["arrived"])
        self.assertTrue(watchdog.update(at_goal,9,1.3)["arrived"])

    def test_watchdog_rejects_moving_away_after_near_target(self):
        watchdog = self.watchdog()
        close = sample(6, agv_pos_x=self.target["x_m"] + 0.20, agv_pos_y=self.target["y_m"])
        watchdog.update(close, 6, 0.1)
        farther = {**close, "timestamp_ns": 7, "agv_pos_x": self.target["x_m"] + 0.31}
        with self.assertRaisesRegex(RuntimeError, "moved away"):
            watchdog.update(farther, 7, 0.2)

    def test_arrival_requires_idle_stopped_updating_samples_for_one_second(self):
        watchdog = self.watchdog()
        at_goal = sample(6, agv_pos_x=self.target["x_m"], agv_pos_y=self.target["y_m"],
                         agv_angle=-math.pi, agv_status=7)
        self.assertFalse(watchdog.update(at_goal, 6, 0.1)["arrived"])
        at_goal["agv_status"] = 8
        self.assertFalse(watchdog.update(at_goal, 7, 0.2)["arrived"])
        self.assertFalse(watchdog.update(at_goal, 8, 0.8)["arrived"])
        self.assertTrue(watchdog.update(at_goal, 9, 1.3)["arrived"])

    def test_replayed_failed_trace_is_rejected_before_touch(self):
        trace = json.loads((ROOT / "tests/fixtures/navigation_failure_20261013.json").read_text(encoding="utf-8-sig"))
        start = {**trace[0], "map_id": 6}
        watchdog = nav.NavigationWatchdog(self.target, 6, start, 0.0)
        stopped_at = None
        caught = None
        for index, row in enumerate(trace[1:], 1):
            # Isolate watchdog behavior; the incident itself did not have runtime map evidence.
            state = {**row, "map_id": 6}
            elapsed = (state["timestamp_ns"] - start["timestamp_ns"]) / 1e9
            try:
                watchdog.update(state, state["timestamp_ns"], elapsed)
            except RuntimeError as exc:
                stopped_at = index
                caught = exc
                break
        self.assertIsNotNone(stopped_at)
        self.assertRegex(str(caught), "moved away after approaching")
        self.assertNotIsInstance(caught, nav.NavigationAvoidTimeout)
        self.assertLess(stopped_at, len(trace) - 1)
        self.assertNotEqual(trace[stopped_at]["agv_status"], 1)

    def test_pause_ack_does_not_prove_stationary_feedback(self):
        clock = [0.0]
        slam = Mock()
        slam.pause_walk.return_value = SimpleNamespace(success=True)
        slam.get_chassis_position.return_value = ({"linear_speed": 0, "angular_speed": 0}, 100)
        def advance(seconds):
            clock[0] += seconds
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]),              patch.object(nav.time, "sleep", side_effect=advance):
            result = nav.pause_and_confirm(slam, 100, timeout_s=1.0)
        self.assertTrue(result["pause_acknowledged"])
        self.assertFalse(result["stop_confirmed"])

    def test_pause_confirms_only_new_zero_speed_feedback(self):
        clock, stamp = [0.0], [100]
        slam = Mock()
        slam.pause_walk.return_value = SimpleNamespace(success=True)
        def feedback():
            stamp[0] += 1
            return {"linear_speed": 0, "angular_speed": 0}, stamp[0]
        slam.get_chassis_position.side_effect = feedback
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]),              patch.object(nav.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0]+seconds)):
            result = nav.pause_and_confirm(slam, 100, timeout_s=2.0)
        self.assertTrue(result["stop_confirmed"])
        self.assertGreaterEqual(len(result["feedback"]), 3)

    def test_avoid_state_is_tolerated_within_budget(self):
        watchdog = self.watchdog()
        for index in range(1, 11):
            result = watchdog.update(sample(5 + index, agv_status=5), 5 + index, index * 0.5)
        self.assertTrue(result["avoiding"])
        self.assertAlmostEqual(result["avoid_elapsed_s"], 4.5)
        self.assertEqual(watchdog.avoidance_summary()["episode_count"], 1)
        result = watchdog.update(sample(16, agv_status=7), 16, 5.5)
        self.assertFalse(result["avoiding"])
        summary = watchdog.avoidance_summary()
        self.assertAlmostEqual(summary["episodes"][0]["duration_s"], 5.0)
        self.assertEqual(summary["rebaseline_count"], 0)

    def test_single_avoid_episode_timeout_aborts(self):
        watchdog = self.watchdog()
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "avoidance exceeded 30"):
            for index in range(62):
                watchdog.update(sample(6 + index, agv_status=5), 6 + index, index * 0.5)

    def test_cumulative_avoid_timeout_aborts(self):
        watchdog = self.watchdog()
        stamp = 5
        distance = math.hypot(self.target["x_m"] - 1.542, self.target["y_m"] - 1.344)
        direction = ((self.target["x_m"] - 1.542) / distance,
                     (self.target["y_m"] - 1.344) / distance)

        def push(now, status, route_progress=0.0):
            nonlocal stamp
            stamp += 1
            current = sample(stamp, agv_status=status,
                             agv_pos_x=1.542 + direction[0] * route_progress,
                             agv_pos_y=1.344 + direction[1] * route_progress)
            return watchdog.update(current, stamp, now)

        for step in range(51):
            push(step * 0.5, 5)
        push(25.5, 7, 0.02)
        for step in range(1, 21):
            push(25.5 + step * 0.5, 7, 0.02 + step * 0.004)
        for step in range(51):
            push(36.0 + step * 0.5, 5, 0.10)
        push(61.5, 7, 0.10)
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "cumulative"):
            for step in range(22):
                push(62.0 + step * 0.5, 5, 0.10)

    def test_avoid_episode_count_cap(self):
        watchdog = self.watchdog()
        stamp, now = 5, 0.0
        for episode in range(5):
            stamp += 1
            watchdog.update(sample(stamp, agv_status=5), stamp, now)
            now += 0.5
            stamp += 1
            watchdog.update(sample(stamp, agv_status=7), stamp, now)
            now += 0.5
        stamp += 1
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "started more than 5"):
            watchdog.update(sample(stamp, agv_status=5), stamp, now)

    def test_avoid_departure_cap(self):
        watchdog = self.watchdog()
        watchdog.update(sample(6, agv_status=5), 6, 0.1)
        far = sample(7, agv_status=5, agv_pos_x=self.target["x_m"] + 1.95,
                     agv_pos_y=self.target["y_m"])
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "too far from the route"):
            watchdog.update(far, 7, 0.6)

    def test_departure_cap_applies_during_grace(self):
        watchdog = self.watchdog()
        near = sample(6, agv_pos_x=self.target["x_m"] + 0.154,
                      agv_pos_y=self.target["y_m"])
        watchdog.update(near, 6, 0.1)
        watchdog.update({**near, "timestamp_ns": 7, "agv_status": 5}, 7, 0.2)
        watchdog.update({**near, "timestamp_ns": 8}, 8, 0.3)
        far_during_grace = sample(9, agv_status=7, linear_speed=0.3,
                                  agv_pos_x=self.target["x_m"] + 1.854,
                                  agv_pos_y=self.target["y_m"])
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "too far from the route"):
            watchdog.update(far_during_grace, 9, 1.0)

    def test_total_departure_cap_across_episodes(self):
        watchdog = nav.NavigationWatchdog(
            self.target, 6,
            sample(5, agv_pos_x=self.target["x_m"] + 0.2,
                   agv_pos_y=self.target["y_m"]),
            0.0,
            settings={"avoid_max_departure_m": 1.0,
                      "avoid_total_departure_m": 2.0,
                      "avoid_grace_s": 1.0},
        )
        stamp = 5

        def push(now, status, distance):
            nonlocal stamp
            stamp += 1
            row = sample(stamp, agv_status=status,
                         agv_pos_x=self.target["x_m"] + distance,
                         agv_pos_y=self.target["y_m"])
            return watchdog.update(row, stamp, now)

        push(0.1, 7, 0.2)
        for episode in range(3):
            start = 0.2 + episode * 1.6
            push(start, 5, 0.2)
            if episode == 2:
                with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "cumulative.*departure"):
                    push(start + 0.1, 5, 0.9)
                break
            push(start + 0.1, 5, 0.9)
            push(start + 0.2, 7, 0.9)
            push(start + 1.3, 7, 0.2)

    def test_moved_away_is_suspended_during_avoid_and_grace_then_rebaselined(self):
        watchdog = self.watchdog()
        near = sample(6, agv_pos_x=self.target["x_m"] + 0.20,
                      agv_pos_y=self.target["y_m"])
        watchdog.update(near, 6, 0.1)
        during_avoid = sample(7, agv_status=5, agv_pos_x=self.target["x_m"] + 0.50,
                              agv_pos_y=self.target["y_m"])
        self.assertTrue(watchdog.update(during_avoid, 7, 0.6)["avoiding"])
        grace = sample(8, agv_status=7, agv_pos_x=self.target["x_m"] + 0.70,
                       agv_pos_y=self.target["y_m"])
        self.assertFalse(watchdog.update(grace, 8, 2.0)["avoiding"])
        still_in_grace = {**grace, "timestamp_ns": 9, "agv_pos_x": self.target["x_m"] + 0.80}
        watchdog.update(still_in_grace, 9, 6.9)
        after_grace = {**grace, "timestamp_ns": 10, "agv_pos_x": self.target["x_m"] + 0.85}
        watchdog.update(after_grace, 10, 7.1)
        self.assertEqual(watchdog.rebaseline_count, 1)

    def test_rebaseline_count_matches_actual_rebaselines(self):
        watchdog = self.watchdog()
        near = sample(6, agv_pos_x=self.target["x_m"] + 0.20,
                      agv_pos_y=self.target["y_m"])
        watchdog.update(near, 6, 0.1)
        watchdog.update({**near, "timestamp_ns": 7, "agv_status": 5}, 7, 0.2)
        watchdog.update({**near, "timestamp_ns": 8}, 8, 0.3)
        watchdog.update({**near, "timestamp_ns": 9, "agv_status": 5}, 9, 0.4)
        watchdog.update({**near, "timestamp_ns": 10}, 10, 0.5)
        watchdog.update({**near, "timestamp_ns": 11}, 11, 5.6)
        self.assertEqual(watchdog.avoidance_summary()["rebaseline_count"], 1)

    def test_no_progress_timer_is_frozen_during_avoid(self):
        settings = {"no_progress_timeout_s": 2.0}
        watchdog = nav.NavigationWatchdog(self.target, 6, self.samples[-1], 0.0, settings=settings)
        watchdog.update(sample(6, agv_status=7), 6, 0.1)
        watchdog.update(sample(7, agv_status=5), 7, 0.2)
        watchdog.update(sample(8, agv_status=5), 8, 10.0)
        watchdog.update(sample(9, agv_status=7), 9, 10.1)
        result = watchdog.update(sample(10, agv_status=7), 10, 15.2)
        self.assertFalse(result["avoiding"])
        self.assertEqual(watchdog.rebaseline_count, 1)

    def test_other_protection_states_still_abort_immediately(self):
        for status in (0, 1, 2, 3, 4, 6, 9, None):
            with self.subTest(status=status):
                with self.assertRaisesRegex(RuntimeError, "protection/state"):
                    self.watchdog().update(sample(6, agv_status=status), 6, 0.1)

    def test_halted_walking_is_not_avoid_unless_configured(self):
        row = sample(6, agv_status=7, linear_speed=0.0, angular_speed=0.0)
        normal = self.watchdog().update(row, 6, 0.1)
        self.assertFalse(normal["avoiding"])
        watchdog = nav.NavigationWatchdog(
            self.target, 6, self.samples[-1], 0.0,
            settings={"treat_halted_walking_as_avoid": True},
        )
        avoided = watchdog.update(row, 6, 0.1)
        self.assertTrue(avoided["avoiding"])
        self.assertEqual(watchdog.avoidance_summary()["episodes"][0]["kind"], "halted_walking")

    def test_replayed_avoid_trace_arrives(self):
        trace = json.loads((ROOT / "tests/fixtures/navigation_avoid_trace.json").read_text())
        initial = {**trace[0], "map_id": 6}
        watchdog = nav.NavigationWatchdog(self.target, 6, initial, 0.0)
        result = None
        for row in trace[1:]:
            current = {**row, "map_id": 6}
            now = (current["timestamp_ns"] - initial["timestamp_ns"]) / 1e9
            result = watchdog.update(current, current["timestamp_ns"], now)
        self.assertTrue(result["arrived"])
        summary = watchdog.avoidance_summary()
        self.assertEqual(summary["episode_count"], 1)
        self.assertIsNone(summary["open_episode"])
        self.assertTrue(7.5 <= summary["episodes"][0]["duration_s"] <= 8.5)
        self.assertEqual(summary["rebaseline_count"], 0)

    def test_replayed_near_goal_avoid_trace_arrives_only_with_avoid_logic(self):
        trace = json.loads((ROOT / "tests/fixtures/navigation_avoid_near_goal_trace.json").read_text())
        initial = {**trace[0], "map_id": 6}
        watchdog = nav.NavigationWatchdog(self.target, 6, initial, 0.0)
        result = None
        for row in trace[1:]:
            current = {**row, "map_id": 6}
            now = (current["timestamp_ns"] - initial["timestamp_ns"]) / 1e9
            result = watchdog.update(current, current["timestamp_ns"], now)
        self.assertTrue(result["arrived"])
        summary = watchdog.avoidance_summary()
        self.assertEqual(summary["rebaseline_count"], 1)
        self.assertGreaterEqual(summary["episodes"][0]["duration_s"], 6.0)

        without_avoid_logic = [{**row, "agv_status": 7}
                               if row["agv_status"] == 5 else row for row in trace]
        watchdog = nav.NavigationWatchdog(
            self.target, 6, {**without_avoid_logic[0], "map_id": 6}, 0.0)
        with self.assertRaisesRegex(RuntimeError, "moved away after approaching"):
            for row in without_avoid_logic[1:]:
                current = {**row, "map_id": 6}
                now = (current["timestamp_ns"] - initial["timestamp_ns"]) / 1e9
                watchdog.update(current, current["timestamp_ns"], now)

    def test_replayed_long_avoid_trace_aborts_by_avoid_timeout(self):
        trace = json.loads((ROOT / "tests/fixtures/navigation_avoid_long_trace.json").read_text())
        initial = {**trace[0], "map_id": 6}
        watchdog = nav.NavigationWatchdog(self.target, 6, initial, 0.0)
        with self.assertRaisesRegex(nav.NavigationAvoidTimeout, "avoidance exceeded 30"):
            for row in trace[1:]:
                current = {**row, "map_id": 6}
                now = (current["timestamp_ns"] - initial["timestamp_ns"]) / 1e9
                watchdog.update(current, current["timestamp_ns"], now)

    def test_arrival_criteria_unchanged_after_avoid(self):
        watchdog = self.watchdog()
        watchdog.update(sample(6, agv_status=5), 6, 0.1)
        watchdog.update(sample(7, agv_status=7), 7, 0.6)
        at_goal = sample(8, agv_status=8, agv_pos_x=self.target["x_m"],
                         agv_pos_y=self.target["y_m"], agv_angle=-math.pi)
        self.assertFalse(watchdog.update(at_goal, 8, 5.7)["arrived"])
        at_goal["timestamp_ns"] = 9
        self.assertFalse(watchdog.update(at_goal, 9, 6.2)["arrived"])
        at_goal["timestamp_ns"] = 10
        self.assertTrue(watchdog.update(at_goal, 10, 6.7)["arrived"])

    def test_leg_timeout_scales_with_distance_and_is_capped(self):
        settings = nav.navigation_settings({})
        self.assertAlmostEqual(nav.leg_timeout_s(settings, 0.93), 175.8)
        self.assertEqual(nav.leg_timeout_s(settings, 10.0), 300.0)
        self.assertEqual(nav.leg_timeout_s(settings, 0.0), 120.0)

    def test_navigation_settings_rejects_unknown_and_invalid_values(self):
        invalid = (
            {"navigation": {"unexpected": 1}},
            {"navigation": {"no_progress_timeout_s": float("nan")}},
            {"navigation": {"timeout_base_s": 0}},
            {"navigation": {"treat_halted_walking_as_avoid": 1}},
            {"navigation": {"avoid_timeout_s": 61.0}},
            {"navigation": {"detour": {"unexpected": True}}},
            {"navigation": {"detour": {"lateral_offsets_m": []}}},
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(RuntimeError):
                nav.navigation_settings(config)

    def test_detour_wait_and_no_progress_timeout_do_not_conflict(self):
        config = yaml.safe_load((ROOT / "config/transport.yaml").read_text())
        self.assertEqual(config["navigation"]["no_progress_timeout_s"], 25.0)
        self.assertEqual(nav.navigation_settings({})["no_progress_timeout_s"], 25.0)
        self.assertIs(config["navigation"]["treat_halted_walking_as_avoid"], False)
        lines = (ROOT / "config/transport.yaml").read_text().splitlines()
        timeout_index = next(i for i, line in enumerate(lines)
                             if "no_progress_timeout_s:" in line)
        self.assertIn("10", lines[timeout_index - 1])
        self.assertTrue(any("no_progress_timeout_s" in line and "25" in line for line in lines))

    def test_navigation_settings_rejects_detour_enabled_until_implemented(self):
        with self.assertRaisesRegex(RuntimeError, "detour is not implemented yet"):
            nav.navigation_settings({"navigation": {"detour": {"enabled": True}}})
        for threshold in (0, 101):
            with self.subTest(threshold=threshold), self.assertRaises(RuntimeError):
                nav.navigation_settings({"navigation": {"detour": {
                    "occupied_threshold": threshold,
                }}})
        self.assertEqual(nav.navigation_settings({"navigation": {"detour": {
            "occupied_threshold": 1,
        }}})["detour"]["occupied_threshold"], 1)
        self.assertEqual(nav.navigation_settings({"navigation": {"detour": {
            "occupied_threshold": 100,
        }}})["detour"]["occupied_threshold"], 100)

    def test_total_departure_limit_is_documented_in_transport_yaml(self):
        lines = (ROOT / "config/transport.yaml").read_text(encoding="utf-8").splitlines()
        index = next(i for i, line in enumerate(lines)
                     if "avoid_total_departure_m:" in line)
        self.assertIn("#", lines[index - 1])
        self.assertIn("episode", lines[index - 1].lower())

    def test_collision_avoidance_tolerates_missing_chassis_info_readback(self):
        slam = Mock()
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        report = {}
        nav.enable_chassis_collision_avoidance(slam, {"agv_collision_level": 2}, report)
        self.assertTrue(report["chassis_collision_avoidance"]["enabled"])
        self.assertFalse(report["chassis_collision_avoidance"]["chassis_info_before"]["available"])
        self.assertFalse(report["chassis_collision_avoidance"]["chassis_info_after"]["available"])

    def test_chassis_info_applied_only_with_confirmed_readback(self):
        slam = FakeChassisInfoSlam(reflect=True)
        original_set = slam.set_chassis_info

        def reflect_float32(max_linear_speed, max_angular_speed, min_collision_distance):
            original_set(max_linear_speed, max_angular_speed, min_collision_distance)
            slam.values = {
                key: struct.unpack("f", struct.pack("f", value))[0]
                for key, value in slam.values.items()
            }

        slam.set_chassis_info = reflect_float32
        report = {}
        config = {"agv_collision_level": 2,
                  "chassis_info": {"max_linear_speed": 0.4, "max_angular_speed": 0.5,
                                   "min_collision_distance": 0.12}}
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock.__setitem__(0, clock[0] + 0.1) or clock[0]), \
             patch.object(nav.time, "sleep"):
            nav.enable_chassis_collision_avoidance(slam, config, report)
        entry = report["chassis_collision_avoidance"]
        self.assertTrue(entry["applied"])
        expected = {key: struct.unpack("f", struct.pack("f", value))[0]
                    for key, value in config["chassis_info"].items()}
        self.assertEqual(entry["chassis_info_after"]["values"], expected)
        self.assertEqual(len(slam.chassis_info_writes), 1)

    def test_chassis_info_rollback_failure_does_not_replace_readback_error(self):
        slam = FakeChassisInfoSlam(reflect=False)
        original_set = slam.set_chassis_info
        calls = [0]

        def fail_rollback(*values):
            calls[0] += 1
            original_set(*values)
            if calls[0] == 2:
                raise RuntimeError("rollback write failed")

        slam.set_chassis_info = fail_rollback
        before = nav.read_chassis_info(slam)
        entry = {}
        requested = {"max_linear_speed": 0.4, "max_angular_speed": 0.5,
                     "min_collision_distance": 0.12}
        with patch.object(nav.time, "monotonic", side_effect=[0.0, 2.0]):
            with self.assertRaisesRegex(RuntimeError, "read-back did not confirm"):
                nav.apply_chassis_info(slam, requested, before, entry, timeout_s=1.0)
        self.assertFalse(entry["applied"])
        self.assertIn("rollback write failed", entry["rollback_error"])

    def test_chassis_info_mismatch_rolls_back_and_rejects(self):
        slam = FakeChassisInfoSlam(reflect=False)
        report = {}
        config = {"agv_collision_level": 2,
                  "chassis_info": {"max_linear_speed": 0.4, "max_angular_speed": 0.5,
                                   "min_collision_distance": 0.12}}
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
            with self.assertRaisesRegex(RuntimeError, "previous values republished"):
                nav.enable_chassis_collision_avoidance(slam, config, report)
        self.assertEqual(len(slam.chassis_info_writes), 2)
        self.assertEqual(slam.chassis_info_writes[1], (0.6, 0.8, 0.15))
        self.assertFalse(report["chassis_collision_avoidance"]["applied"])
        self.assertEqual(report["chassis_collision_avoidance"]["rollback"]["values"],
                         {"max_linear_speed": 0.6, "max_angular_speed": 0.8,
                          "min_collision_distance": 0.15})

    def test_chassis_info_refuses_blind_write(self):
        slam = FakeChassisInfoSlam(reflect=True)
        entry = {}
        with self.assertRaisesRegex(RuntimeError, "refusing a blind set_chassis_info"):
            nav.apply_chassis_info(slam, {"max_linear_speed": 0.4, "max_angular_speed": 0.5,
                                          "min_collision_distance": 0.12},
                                   {"available": False}, entry)
        self.assertEqual(slam.chassis_info_writes, [])

    def test_collision_level_failure_stops_before_ctrl_and_chassis_info(self):
        slam = Mock()
        slam.get_chassis_info.side_effect = RuntimeError("unavailable")
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=1)
        config = {"agv_collision_level": 2,
                  "chassis_info": {"max_linear_speed": 0.4, "max_angular_speed": 0.5,
                                   "min_collision_distance": 0.12}}
        with self.assertRaisesRegex(RuntimeError, "collision level"):
            nav.enable_chassis_collision_avoidance(slam, config, {})
        slam.set_agv_col_ctrl.assert_not_called()
        slam.set_chassis_info.assert_not_called()

    def test_collision_ctrl_false_is_never_sent(self):
        scenarios = []
        success = Mock()
        success.get_chassis_info.side_effect = RuntimeError("not exposed")
        success.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        success.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        scenarios.append((success, {"agv_collision_level": 2}))
        level_failure = Mock()
        level_failure.get_chassis_info.side_effect = RuntimeError("not exposed")
        level_failure.set_agv_col_level.return_value = SimpleNamespace(error_code=1)
        scenarios.append((level_failure, {"agv_collision_level": 2}))
        enable_failure = Mock()
        enable_failure.get_chassis_info.side_effect = RuntimeError("not exposed")
        enable_failure.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        enable_failure.set_agv_col_ctrl.return_value = SimpleNamespace(success=False)
        scenarios.append((enable_failure, {"agv_collision_level": 2}))
        blind_write = Mock()
        blind_write.get_chassis_info.side_effect = RuntimeError("not exposed")
        blind_write.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        blind_write.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        scenarios.append((blind_write, {"agv_collision_level": 2,
                                        "chassis_info": {"max_linear_speed": 0.4,
                                                         "max_angular_speed": 0.5,
                                                         "min_collision_distance": 0.12}}))

        def assert_never_false(slam):
            self.assertTrue(all(
                all(argument is not False for argument in call.args) and
                all(argument is not False for argument in call.kwargs.values())
                for call in slam.set_agv_col_ctrl.call_args_list
            ))

        for slam, config in scenarios:
            try:
                nav.enable_chassis_collision_avoidance(slam, config, {})
            except RuntimeError:
                pass
            assert_never_false(slam)

        success = Mock()
        success.get_chassis_info.side_effect = RuntimeError("not exposed")
        success.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        success.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        success.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.current_end_pose.return_value = SimpleNamespace(position_m=[0.0, 0.0, 0.0],
                                                           rpy_rad=[0.0, 0.0, 0.0])
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        source = Mock()
        stamp = [5]

        def at_goal():
            stamp[0] += 1
            return sample(stamp[0], agv_pos_x=self.target["x_m"],
                          agv_pos_y=self.target["y_m"],
                          agv_angle=self.target["yaw_rad"]), stamp[0]

        source.get_chassis_position.side_effect = at_goal
        report = {"planned_request": nav.navigation_request(self.target)}
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock.__setitem__(0, clock[0] + 0.25) or clock[0]), \
             patch.object(nav.time, "sleep"), patch("builtins.print"):
            nav.execute_navigation(success, source, arm, self.target, self.config,
                                   self.samples[-1], report, timeout_s=10.0)
        self.assertEqual(report["status"], "ARRIVED")
        success.set_agv_col_ctrl.assert_called_once_with(True)
        assert_never_false(success)

        aborted = Mock()
        aborted.get_chassis_info.side_effect = RuntimeError("not exposed")
        aborted.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        aborted.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        aborted.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        source = Mock()
        source.get_chassis_position.return_value = (sample(6, agv_status=2), 6)
        report = {"planned_request": nav.navigation_request(self.target)}
        with patch.object(nav.time, "monotonic", return_value=0.0), \
             self.assertRaisesRegex(RuntimeError, "protection/state 2"):
            nav.execute_navigation(aborted, source, arm, self.target, self.config,
                                   self.samples[-1], report, timeout_s=10.0)
        aborted.set_agv_col_ctrl.assert_called_once_with(True)
        assert_never_false(aborted)

    def test_execute_navigation_saves_avoidance_summary_on_abort(self):
        slam = Mock()
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.current_end_pose.return_value = SimpleNamespace(position_m=[0.0, 0.0, 0.0],
                                                            rpy_rad=[0.0, 0.0, 0.0])
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        source = Mock()
        source.get_chassis_position.return_value = (sample(6, agv_status=2), 6)
        report = {"planned_request": nav.navigation_request(self.target)}
        with patch.object(nav.time, "monotonic", return_value=0.0), \
             self.assertRaisesRegex(RuntimeError, "protection/state 2"):
            nav.execute_navigation(slam, source, arm, self.target, self.config,
                                   self.samples[-1], report)
        self.assertIn("avoidance", report)
        self.assertEqual(report["avoidance"]["episode_count"], 0)
        slam.set_agv_col_ctrl.assert_called_once_with(True)

    def test_no_cup_skips_gripper_and_carry_drift_checks_but_keeps_readiness(self):
        slam = Mock()
        slam.get_chassis_info.side_effect = RuntimeError("not exposed")
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.current_end_pose.return_value = SimpleNamespace(
            position_m=[0.5, 0.5, 0.5], rpy_rad=[1.0, 1.0, 1.0])
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 120.0])
        source = Mock()
        stamp = [5]

        def at_goal():
            stamp[0] += 1
            return sample(stamp[0], agv_pos_x=self.target["x_m"],
                          agv_pos_y=self.target["y_m"],
                          agv_angle=self.target["yaw_rad"]), stamp[0]

        source.get_chassis_position.side_effect = at_goal
        report = {"planned_request": nav.navigation_request(self.target)}
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock.__setitem__(0, clock[0] + 0.25) or clock[0]), patch.object(nav.time, "sleep"), patch("builtins.print"):
            nav.execute_navigation(slam, source, arm, self.target, self.config,
                                   self.samples[-1], report, timeout_s=10.0, no_cup=True)
        self.assertEqual(report["status"], "ARRIVED")
        self.assertTrue(report["no_cup"])
        arm.assert_execution_ready.assert_called()
        slam.set_agv_col_ctrl.assert_called_once_with(True)

    def test_no_cup_progress_report_handles_missing_arm_drift_measurements(self):
        slam = Mock()
        slam.get_chassis_info.side_effect = RuntimeError("not exposed")
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 120.0])
        start = self.config["stations"]["A"]
        distance = math.hypot(self.target["x_m"] - start["x_m"],
                              self.target["y_m"] - start["y_m"])
        fraction = 0.05 / distance
        position = sample(6,
                          agv_pos_x=start["x_m"] + (self.target["x_m"] - start["x_m"]) * fraction,
                          agv_pos_y=start["y_m"] + (self.target["y_m"] - start["y_m"]) * fraction)
        source = Mock()
        source.get_chassis_position.return_value = (position, 6)
        report = {"planned_request": nav.navigation_request(self.target)}
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock.__setitem__(0, clock[0] + 6.0) or clock[0]), patch.object(nav.time, "sleep"), patch("builtins.print"):
            with self.assertRaisesRegex(RuntimeError, "navigation timeout"):
                nav.execute_navigation(slam, source, arm, self.target, self.config,
                                       self.samples[-1], report, timeout_s=10.0, no_cup=True)

    def test_avoidance_summary_failure_does_not_mask_error(self):
        slam = Mock()
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.current_end_pose.return_value = SimpleNamespace(position_m=[0.0, 0.0, 0.0],
                                                            rpy_rad=[0.0, 0.0, 0.0])
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        source = Mock()
        source.get_chassis_position.return_value = (sample(6, agv_status=2), 6)
        report = {"planned_request": nav.navigation_request(self.target)}
        with patch.object(nav.NavigationWatchdog, "avoidance_summary",
                          side_effect=RuntimeError("summary failure")), \
             patch.object(nav.time, "monotonic", return_value=0.0), \
             self.assertRaisesRegex(RuntimeError, "protection/state 2"):
            nav.execute_navigation(slam, source, arm, self.target, self.config,
                                   self.samples[-1], report)
        self.assertIn("summary failure", report["avoidance"]["error"])

    def test_main_computes_timeout_from_config_when_flag_absent(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        native = Mock()
        stamps = [5]
        def feedback():
            stamps[0] += 1
            return sample(stamps[0], agv_status=8, agv_pos_x=self.target["x_m"],
                          agv_pos_y=self.target["y_m"], agv_angle=self.target["yaw_rad"]), stamps[0]
        native.get_chassis_position.side_effect = feedback
        slam.set_agv_col_level.return_value = SimpleNamespace(error_code=0)
        slam.set_agv_col_ctrl.return_value = SimpleNamespace(success=True)
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        module.Slam = Mock(return_value=slam)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.current_end_pose.return_value = SimpleNamespace(position_m=[0.0, 0.0, 0.0],
                                                            rpy_rad=[0.0, 0.0, 0.0])
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        clock = [0.0]
        config = {**self.config, "enable_real_navigation": True}
        with patch.object(nav, "NativeChassisFeedback", return_value=native), \
             patch.object(nav, "collect_samples", return_value=self.samples), \
             patch.object(nav, "read_hmi", return_value=self.hmi), \
             patch.object(nav, "read_controller", return_value=self.controller), \
             patch.object(nav, "load_config", return_value={"robot": {}}), \
             patch.object(nav, "GdkRightArm", return_value=arm), \
             patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
                config, module,
            )
        self.assertEqual(code, 0, report.get("error"))
        expected = nav.leg_timeout_s(nav.navigation_settings(config),
                                     math.hypot(1.542 - self.target["x_m"], 1.344 - self.target["y_m"]))
        self.assertAlmostEqual(report["navigation_timeout_s"], expected)

    def test_main_pause_uses_last_feedback_stamp(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        module.Slam = Mock(return_value=slam)
        native = Mock()
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        config = {**self.config, "enable_real_navigation": True}

        def fail_navigation(*args, **kwargs):
            report = args[6]
            report["request_attempted"] = True
            report["navigation_feedback"] = [{"timestamp_ns": 123456789}]
            raise RuntimeError("navigation failed")

        with patch.object(nav, "NativeChassisFeedback", return_value=native), \
             patch.object(nav, "collect_samples", return_value=self.samples), \
             patch.object(nav, "read_hmi", return_value=self.hmi), \
             patch.object(nav, "read_controller", return_value=self.controller), \
             patch.object(nav, "load_config", return_value={"robot": {}}), \
             patch.object(nav, "GdkRightArm", return_value=arm), \
             patch.object(nav, "execute_navigation", side_effect=fail_navigation), \
             patch.object(nav, "pause_and_confirm", return_value={"stop_confirmed": True}) as pause:
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
                config, module,
            )
        self.assertEqual(code, 1)
        self.assertIn("navigation failed", report["error"])
        pause.assert_called_once_with(slam, 123456789, feedback=native)

    def test_avoid_timeout_leads_to_pause_in_main(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        slam.pause_walk.return_value = SimpleNamespace(success=True)
        native = Mock()
        feedback_stamp = [100]

        def stopped_feedback():
            feedback_stamp[0] += 1
            return {"linear_speed": 0.0, "angular_speed": 0.0}, feedback_stamp[0]

        native.get_chassis_position.side_effect = stopped_feedback
        module.Slam = Mock(return_value=slam)
        arm = Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[0.0, 50.0])
        config = {**self.config, "enable_real_navigation": True}

        def avoid_timeout(*args, **kwargs):
            report = args[6]
            report["request_attempted"] = True
            report["navigation_feedback"] = [{"timestamp_ns": 100}]
            raise nav.NavigationAvoidTimeout("avoid departure budget exceeded")

        clock = [0.0]

        def monotonic():
            current = clock[0]
            clock[0] += 0.1
            return current

        def sleep(seconds):
            clock[0] += seconds

        with patch.object(nav, "NativeChassisFeedback", return_value=native), \
             patch.object(nav, "collect_samples", return_value=self.samples), \
             patch.object(nav, "read_hmi", return_value=self.hmi), \
             patch.object(nav, "read_controller", return_value=self.controller), \
             patch.object(nav, "load_config", return_value={"robot": {}}), \
             patch.object(nav, "GdkRightArm", return_value=arm), \
             patch.object(nav, "execute_navigation", side_effect=avoid_timeout), \
             patch.object(nav.time, "monotonic", side_effect=monotonic), \
             patch.object(nav.time, "sleep", side_effect=sleep):
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--native-feedback", "--carry-clearance-verified"],
                config, module,
            )
        self.assertEqual(code, 1)
        self.assertIn("NavigationAvoidTimeout: avoid departure budget exceeded", report["error"])
        slam.pause_walk.assert_called_once_with(True)

    def test_error_path_pauses_but_never_releases(self):
        slam = Mock()
        slam.pause_walk.return_value = SimpleNamespace(success=True)
        slam.get_chassis_position.return_value = ({"linear_speed": 0.0, "angular_speed": 0.0}, 10)
        clock = [0.0]
        with patch.object(nav.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(nav.time, "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)):
            nav.pause_and_confirm(slam, 10, timeout_s=0.5)
        self.assertTrue(slam.pause_walk.called)
        self.assertTrue(all(call.args == (True,) for call in slam.pause_walk.call_args_list))

    def test_transport_config_rejects_place_on_other_map(self):
        config = dict(self.config)
        config["place"] = {"base_pose": {"map_id": 5}}
        with self.assertRaisesRegex(RuntimeError, "place base_pose was taught on another map"):
            nav.validate_transport_config(config)


if __name__ == "__main__":
    unittest.main()

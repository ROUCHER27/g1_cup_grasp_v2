from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
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


class NavigationExecutionTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / "config/transport.yaml").read_text(encoding="utf-8-sig"))
        self.samples = [sample(index) for index in range(1, 6)]
        self.hmi = {
            "map_assignment_matches": True,
            "state": {"isNormal": True, "isScram": False, "isEdged": False, "isPause": False,
                      "isHardAuto": True, "isSoftAuto": True,
                      "batteryStateList": [{"isCharging": False}]},
        }
        self.controller = {"map_selection_matches": True, "serial_number": "424", "auto_motion_parameters": {"maxSpeed": 100}}
        self.target = self.config["stations"]["B"]

    def test_request_uses_mm_and_degrees_while_config_remains_si(self):
        request = nav.navigation_request(self.target)
        self.assertEqual((request["x_mm"], request["y_mm"], request["yaw_deg"]), (646, 1098, 180))
        request = nav.navigation_request(self.config["stations"]["A"])
        self.assertEqual((request["x_mm"], request["y_mm"], request["yaw_deg"]), (1542, 1344, 0))
        self.assertEqual(self.target["x_m"], 0.646)
        self.assertEqual(nav.navigation_request({**self.target, "yaw_rad": -math.pi})["yaw_deg"], 180)
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            nav.navigation_request({**self.target, "x_m": float("nan")})

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
            ["--station", "B", "--dispatch", "--carry-clearance-verified"], self.config, module
        )
        self.assertEqual(code, 1)
        self.assertTrue(report["read_only"])
        self.assertFalse(report["request_attempted"])
        module.Slam.assert_not_called()
        code, report = self.call_main(["--station", "B", "--preview"], self.config, module)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "OFFLINE_REQUEST_PREVIEW")
        self.assertEqual(report["planned_request"]["x_mm"], 646)
        module.Slam.assert_not_called()

    def test_enabling_flag_does_not_bypass_missing_runtime_map(self):
        module = ModuleType("a2d_sdk.robot")
        slam = Mock()
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        module.Slam = Mock(return_value=slam)
        with patch.object(nav, "collect_samples", return_value=[{**s, "map_id": None} for s in self.samples]),              patch.object(nav, "read_hmi", return_value=self.hmi),              patch.object(nav, "read_controller", return_value=self.controller),              patch.object(nav, "GdkRightArm") as arm:
            code, report = self.call_main(
                ["--station", "B", "--dispatch", "--carry-clearance-verified"],
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
        for index, row in enumerate(trace[1:], 1):
            # Isolate watchdog behavior; the incident itself did not have runtime map evidence.
            state = {**row, "map_id": 6}
            elapsed = (state["timestamp_ns"] - start["timestamp_ns"]) / 1e9
            try:
                watchdog.update(state, state["timestamp_ns"], elapsed)
            except RuntimeError:
                stopped_at = index
                break
        self.assertIsNotNone(stopped_at)
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


if __name__ == "__main__":
    unittest.main()

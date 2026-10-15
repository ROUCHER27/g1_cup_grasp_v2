from __future__ import annotations

import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools")]
import navigate_station as nav


class AvoidanceControlIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / "tests/fixtures/transport_config_test.yaml").read_text())
        self.config["avoidance_control"] = {"mode": "preserve_verified", "manifest_path": "approved.json"}

    def test_unknown_mode_and_misspelled_options_are_rejected(self):
        for control in ({"mode": "skip"}, {"mode": "preserve_verified"},
                        {"mode": "preserve_verified", "manifest_path": ""},
                        {"mode": "hal_level", "skip_checks": True}):
            with self.subTest(control=control), self.assertRaises(RuntimeError):
                nav.validate_transport_config({**self.config, "avoidance_control": control})

    def test_preserve_mode_checks_manifest_without_any_level_or_override_write(self):
        slam = Mock()
        monitor = Mock()
        monitor.summary.return_value = {"configuration_manifest_match": True}
        report = {}
        with patch.object(nav, "load_avoidance_manifest", return_value={"kind": "approved"}), \
             patch.object(nav, "AvoidanceMonitor", return_value=monitor):
            result = nav.enable_chassis_collision_avoidance(slam, self.config, report)
        self.assertIs(result, monitor)
        monitor.start.assert_called_once()
        monitor.check.assert_called_once()
        slam.set_agv_col_level.assert_not_called()
        slam.set_agv_col_ctrl.assert_not_called()
        slam.set_chassis_info.assert_not_called()
        self.assertEqual(report["chassis_collision_avoidance"]["mode"], "preserve_verified")

    def test_preserve_mode_refuses_speed_writes_that_would_change_the_manifest(self):
        self.config["chassis_info"] = {"max_linear_speed": .1, "max_angular_speed": .1, "min_collision_distance": .15}
        with self.assertRaisesRegex(RuntimeError, "chassis_info"):
            nav.validate_transport_config(self.config)

    def test_start_failure_closes_monitor_and_never_dispatches(self):
        slam = Mock(); monitor = Mock(); monitor.start.side_effect = RuntimeError("profile differs")
        with patch.object(nav, "load_avoidance_manifest", return_value={}), \
             patch.object(nav, "AvoidanceMonitor", return_value=monitor):
            with self.assertRaisesRegex(RuntimeError, "profile differs"):
                nav.enable_chassis_collision_avoidance(slam, self.config, {})
        monitor.close.assert_called_once()
        slam.navigate_to_pose.assert_not_called()
        slam.set_agv_col_level.assert_not_called()
        slam.set_agv_col_ctrl.assert_not_called()

    def test_changed_profile_stops_execution_loop_and_closes_monitor(self):
        initial = {"agv_pos_x": 1.542, "agv_pos_y": 1.344, "agv_angle": 0., "map_id": 6,
                   "hal_sha256": nav.HAL_SHA256, "timestamp_ns": 1, "position_conf": 99.,
                   "agv_status": 8, "linear_speed": 0., "angular_speed": 0.}
        slam, arm, source, monitor = Mock(), Mock(), Mock(), Mock()
        arm.assert_execution_ready = Mock()
        source.get_chassis_position.side_effect = AssertionError("feedback read before failed profile gate")
        slam.navigate_to_pose.return_value.error_code = 0
        monitor.check.side_effect = [None, RuntimeError("avoidance configuration changed")]
        monitor.summary.return_value = {"configuration_manifest_match": False}
        report = {"planned_request": nav.navigation_request(self.config["stations"]["B"])}
        def enable(*args):
            report["chassis_collision_avoidance"] = {"mode": "preserve_verified"}
            return monitor
        with patch.object(nav, "enable_chassis_collision_avoidance", side_effect=enable):
            with self.assertRaisesRegex(RuntimeError, "configuration changed"):
                nav.execute_navigation(slam, source, arm, self.config["stations"]["B"], self.config,
                                       initial, report, no_cup=True)
        self.assertTrue(report["request_attempted"])
        monitor.close.assert_called_once()
        source.get_chassis_position.assert_not_called()

    def test_dispatched_profile_failure_reaches_main_pause_and_stop_confirmation(self):
        initial = {"agv_pos_x": 1.542, "agv_pos_y": 1.344, "agv_angle": 0., "map_id": 6,
                   "hal_sha256": nav.HAL_SHA256, "timestamp_ns": 1, "position_conf": 99.,
                   "agv_status": 8, "linear_speed": 0., "angular_speed": 0.}
        samples = [{**initial, "timestamp_ns": i} for i in range(1, 6)]
        module = ModuleType("a2d_sdk.robot")
        slam = Mock(); module.Slam = Mock(return_value=slam)
        slam.get_map_list.return_value = SimpleNamespace(error_code=0, map_id=[6])
        slam.navigate_to_pose.return_value = SimpleNamespace(error_code=0)
        native, arm, monitor = Mock(), Mock(), Mock()
        arm.assert_execution_ready = Mock()
        arm.read_ready_state.return_value = SimpleNamespace(gripper_state=[120., 0.])
        monitor.check.side_effect = [None, RuntimeError("avoidance configuration changed")]
        monitor.summary.return_value = {"configuration_manifest_match": False}
        hmi = {"map_assignment_matches": True, "state": {"isNormal": True, "isPause": False,
               "isScram": False, "isEdged": False, "isHardAuto": True, "isSoftAuto": True,
               "batteryStateList": [{"isCharging": False}]}}
        controller = {"map_selection_matches": True, "serial_number": "424"}
        def enable(slam, config, report):
            report["chassis_collision_avoidance"] = {"mode": "preserve_verified"}
            return monitor
        with TemporaryDirectory() as folder:
            root = Path(folder); (root / "config").mkdir()
            (root / "config/transport.yaml").write_text(yaml.safe_dump({**self.config, "enable_real_navigation": True}))
            with patch.object(nav, "PROJECT_ROOT", root), patch.dict(sys.modules, {"a2d_sdk.robot": module}), \
                 patch.object(nav, "load_avoidance_manifest", return_value={}), \
                 patch.object(nav, "NativeChassisFeedback", return_value=native), \
                 patch.object(nav, "collect_samples", return_value=samples), \
                 patch.object(nav, "read_hmi", return_value=hmi), \
                 patch.object(nav, "read_controller", return_value=controller), \
                 patch.object(nav, "load_config", return_value={"robot": {}}), \
                 patch.object(nav, "GdkRightArm", return_value=arm), \
                 patch.object(nav, "enable_chassis_collision_avoidance", side_effect=enable), \
                 patch.object(nav, "pause_and_confirm", return_value={"stop_confirmed": True}) as pause, \
                 patch("builtins.print"):
                code = nav.main(["--station", "B", "--dispatch", "--native-feedback", "--no-cup", "--carry-clearance-verified"])
            report = json.loads(next((root / "outputs/navigation_check").glob("*.json")).read_text())
        self.assertEqual(code, 1)
        self.assertTrue(report["request_attempted"])
        self.assertTrue(report["stop"]["stop_confirmed"])
        self.assertIn("configuration changed", report["error"])
        pause.assert_called_once_with(slam, 0, feedback=native)
        monitor.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()

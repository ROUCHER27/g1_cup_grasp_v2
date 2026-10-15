from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("check_navigation", ROOT / "tools" / "check_navigation.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


class FakeSlam:
    def __init__(self):
        self.reads = 0

    def get_map_list(self):
        return SimpleNamespace(error_code=0, map_id=[6], map_name=["CUP"])

    def get_chassis_position(self):
        self.reads += 1
        return {
            "agv_status": 9, "position_conf": 77, "agv_pos_x": 0.646,
            "agv_pos_y": 1.098, "agv_angle": -math.pi,
            "linear_speed": 0.0, "angular_speed": 0.0, "agv_task_state": {},
        }, self.reads * 1000000000


class NavigationCheckTests(unittest.TestCase):
    def test_equivalent_pi_headings_have_zero_error(self):
        self.assertAlmostEqual(nav.angle_error_rad(-math.pi, math.pi), 0.0)

    def test_frozen_feedback_is_rejected(self):
        slam = FakeSlam()
        slam.get_chassis_position = lambda: ({}, 0)
        with patch.object(nav.time, "monotonic", side_effect=[0, 0, 1]), patch.object(nav.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "received 0"):
                nav.collect_samples(slam, count=1, timeout_s=0.5)

    def test_inspection_uses_only_getters_and_does_not_confirm_active_map(self):
        fake_module = ModuleType("a2d_sdk.robot")
        fake_module.Slam = FakeSlam
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            (root / "config" / "transport.yaml").write_text(
                (ROOT / "tests" / "fixtures" / "transport_config_test.yaml").read_text(encoding="utf-8"),
                encoding="utf-8"
            )
            with patch.object(nav, "PROJECT_ROOT", root), patch.object(nav.time, "sleep"), patch.dict(
                "sys.modules", {"a2d_sdk.robot": fake_module}
            ), patch.object(nav, "read_hmi", return_value={"map_assignment_matches": True}), patch.object(nav, "read_controller", return_value={"map_selection_matches": True}), patch("builtins.print"):
                self.assertEqual(nav.main(["--station", "B"]), 0)
            path = next((root / "outputs" / "navigation_check").glob("*.json"))
            report = json.loads(path.read_text(encoding="utf-8"))
            self.assertTrue(report["requested_map_exists"])
            self.assertFalse(report["active_map_id_verified"])
            self.assertEqual(len(report["samples"]), 5)
            self.assertAlmostEqual(report["summary"]["yaw_error_deg"], 0.0)
            self.assertEqual(report["summary"]["position_error_m"], 0.0)
            with patch.object(nav, "PROJECT_ROOT", root), patch.object(nav.time, "sleep"), patch.dict(
                "sys.modules", {"a2d_sdk.robot": fake_module}
            ), patch.object(nav, "read_hmi", return_value={"map_assignment_matches": False}), patch.object(nav, "read_controller", return_value={"map_selection_matches": True}), patch("builtins.print"):
                self.assertEqual(nav.main(["--station", "B"]), 2)


if __name__ == "__main__":
    unittest.main()


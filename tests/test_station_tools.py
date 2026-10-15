from __future__ import annotations

import builtins
import importlib.util
import io
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType
import unittest
from contextlib import redirect_stdout
from unittest.mock import Mock, call, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_tool(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


teach_app = load_tool("teach_station")
map_app = load_tool("map_manager")


def sample(index, *, x=1.0, y=2.0, yaw=0.0, speed=0.0, status=8, confidence=80.0, map_id=6):
    return {
        "timestamp_ns": index,
        "agv_status": status,
        "position_conf": confidence,
        "agv_pos_x": x,
        "agv_pos_y": y,
        "agv_angle": yaw,
        "linear_speed": speed,
        "angular_speed": 0.0,
        "map_id": map_id,
        "chassis_error": 0,
        "hal_sha256": teach_app.HAL_SHA256,
    }


class StationToolTests(unittest.TestCase):
    def test_help_does_not_import_sdk_inside_import_guard(self):
        real_import = builtins.__import__

        def guarded_import(name, *args, **kwargs):
            if name in ("a2d_sdk", "a2d_sdk.robot"):
                raise AssertionError(f"SDK import happened during --help: {name}")
            return real_import(name, *args, **kwargs)

        for name in ("teach_station", "map_manager", "chassis_pause"):
            with self.subTest(tool=name), patch("builtins.__import__", side_effect=guarded_import), \
                 redirect_stdout(io.StringIO()):
                module = load_tool(name)
                with self.assertRaises(SystemExit) as raised:
                    module.main(["--help"])
            self.assertEqual(raised.exception.code, 0)

    def _config_root(self, temporary, config=None):
        root = Path(temporary)
        (root / "config").mkdir()
        (root / "config" / "transport.yaml").write_text(
            yaml.safe_dump(config or yaml.safe_load(
                (ROOT / "tests" / "fixtures" / "transport_config_test.yaml").read_text())),
            encoding="utf-8",
        )
        return root, root / "config" / "transport.yaml"

    def _run_teach(self, rows, *, station="A", expect_map_id=None,
                   hmi_effect=None, controller_effect=None, close_error=None):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root, config_path = self._config_root(temporary.name)
        before = config_path.read_bytes()
        native = Mock()
        if close_error is not None:
            native.close.side_effect = close_error
        args = ["--station", station, "--samples", str(len(rows))]
        if expect_map_id is not None:
            args += ["--expect-map-id", str(expect_map_id)]
        default_hmi = {"robot_config_map_id": 6}
        default_controller = {"driver_selected_map_id": 6}
        hmi_patch = patch.object(teach_app, "read_hmi", side_effect=hmi_effect or (lambda _config: default_hmi))
        controller_patch = patch.object(
            teach_app, "read_controller", side_effect=controller_effect or (lambda _config: default_controller))
        with patch.object(teach_app, "PROJECT_ROOT", root), \
             patch.object(teach_app, "NativeChassisFeedback", return_value=native), \
             patch.object(teach_app, "collect_samples", return_value=rows), \
             hmi_patch, controller_patch, patch("builtins.print"):
            code = teach_app.main(args)
        report_path = next((root / "outputs" / "station_teach").glob("*.json"))
        return code, json.loads(report_path.read_text(encoding="utf-8")), before, config_path.read_bytes(), native

    def _run_map(self, *, map_ids=(6,), map_names=("CUP",), error_code=0,
                 native_feedback=False, native_samples=None, close_error=None,
                 hmi_id=6, controller_id=6):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root, _ = self._config_root(temporary.name)
        slam = Mock()
        slam.get_map_list.return_value = type("MapList", (), {
            "error_code": error_code, "map_id": list(map_ids), "map_name": list(map_names)
        })()
        sdk = ModuleType("a2d_sdk.robot")
        sdk.Slam = Mock(return_value=slam)
        native = Mock()
        if close_error is not None:
            native.close.side_effect = close_error
        args = ["--list"]
        if native_feedback:
            args.append("--native-feedback")
        with patch.object(map_app, "PROJECT_ROOT", root), \
             patch.object(map_app, "NativeChassisFeedback", return_value=native), \
             patch.object(map_app, "collect_samples", return_value=native_samples or [
                 sample(1), sample(2), sample(3)]), \
             patch.object(map_app, "read_hmi", return_value={"robot_config_map_id": hmi_id}), \
             patch.object(map_app, "read_controller", return_value={"driver_selected_map_id": controller_id}), \
             patch("builtins.print"), patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
            code = map_app.main(args)
        report_path = next((root / "outputs" / "map_manager").glob("*_list.json"))
        return code, json.loads(report_path.read_text(encoding="utf-8")), slam, sdk.Slam, native

    def test_teach_station_writes_candidate_and_never_modifies_config(self):
        rows = [sample(i + 1, x=1.0 + i * 0.002, y=2.0 + i * 0.001, yaw=0.1 + i * 0.001)
                for i in range(5)]
        code, report, before, after, _ = self._run_teach(rows)
        self.assertEqual(code, 0)
        self.assertEqual(before, after)
        candidate = report["station_candidate"]
        self.assertAlmostEqual(candidate["x_m"], sum(row["agv_pos_x"] for row in rows) / len(rows))
        self.assertAlmostEqual(candidate["y_m"], sum(row["agv_pos_y"] for row in rows) / len(rows))
        self.assertAlmostEqual(candidate["yaw_rad"], sum(row["agv_angle"] for row in rows) / len(rows))
        self.assertEqual(report["planned_request_preview"], teach_app.navigation_request(candidate))
        self.assertEqual(report["note"], "candidate only; config/transport.yaml not modified")
        self.assertTrue(report["runtime_map"]["verified"])

    def test_teach_station_rejects_unstable_moving_or_non_idle(self):
        cases = (
            [sample(i + 1, x=1.0 + i * 0.01) for i in range(5)],
            [sample(i + 1, speed=0.02) for i in range(5)],
            [sample(i + 1, status=7) for i in range(5)],
        )
        for rows in cases:
            with self.subTest(rows=rows):
                code, report, _, _, _ = self._run_teach(rows)
                self.assertEqual(code, 2)
                self.assertEqual(report["status"], "STATION_TEACH_REJECTED")
                self.assertTrue(report["read_only"])

    def test_teach_station_rejects_inconsistent_map_ids(self):
        rows = [sample(1, map_id=6), sample(2, map_id=7), sample(3, map_id=6)]
        code, report, _, _, _ = self._run_teach(rows)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "STATION_TEACH_REJECTED")
        self.assertFalse(report["runtime_map"]["verified"])
        self.assertIn("runtime map verification", " ".join(report["rejection_reasons"]))

    def test_teach_station_marks_but_does_not_reject_consistent_other_map(self):
        rows = [sample(1, map_id=7), sample(2, map_id=7), sample(3, map_id=7)]
        code, report, _, _, _ = self._run_teach(rows)
        self.assertEqual(code, 0)
        self.assertTrue(report["runtime_map"]["verified"])
        self.assertFalse(report["map_matches_config"])

    def test_teach_station_rejects_nan_samples_and_keeps_them_in_report(self):
        rows = [sample(1), sample(2, x=math.nan), sample(3)]
        code, report, _, _, _ = self._run_teach(rows)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "STATION_TEACH_REJECTED")
        self.assertEqual(len(report["samples"]), 3)
        self.assertTrue(math.isnan(report["samples"][1]["agv_pos_x"]))
        self.assertIn("non-finite", " ".join(report["rejection_reasons"]))

    def test_station_candidate_uses_circular_mean_near_pi(self):
        rows = [sample(1, yaw=3.14), sample(2, yaw=-3.14),
                sample(3, yaw=3.13), sample(4, yaw=-3.13)]
        candidate = teach_app.station_candidate(rows)
        self.assertLess(abs(abs(candidate["yaw_rad"]) - math.pi), 0.02)

    def test_teach_samples_minimum_is_three(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root, _ = self._config_root(temporary.name)
        with patch.object(teach_app, "PROJECT_ROOT", root), \
             patch.object(teach_app, "NativeChassisFeedback", side_effect=AssertionError("blocked")), \
             patch("builtins.print"), self.assertRaises(SystemExit) as raised:
            teach_app.main(["--station", "A", "--samples", "2"])
        self.assertEqual(raised.exception.code, 2)
        rows = [sample(1), sample(2), sample(3)]
        code, report, _, _, _ = self._run_teach(rows)
        self.assertEqual(code, 0)
        self.assertEqual(len(report["samples"]), 3)

    def test_teach_hmi_and_controller_errors_keep_samples_in_report(self):
        rows = [sample(1), sample(2), sample(3)]
        code, report, _, _, _ = self._run_teach(
            rows,
            hmi_effect=RuntimeError("HMI offline"),
            controller_effect=TimeoutError("controller offline"),
        )
        self.assertEqual(code, 0)
        self.assertEqual(len(report["samples"]), 3)
        self.assertIn("HMI offline", report["hmi_error"])
        self.assertIn("controller offline", report["controller_error"])

    def test_teach_close_failure_does_not_replace_success(self):
        rows = [sample(1), sample(2), sample(3)]
        code, report, _, _, native = self._run_teach(rows, close_error=RuntimeError("close failed"))
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "STATION_TEACH_RECORDED")
        self.assertIn("close failed", report["close_error"])
        native.close.assert_called_once_with()

    def test_map_manager_reports_read_only_unverified_without_native_feedback(self):
        code, report, slam, _, _ = self._run_map()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "MAP_LIST_RECORDED")
        self.assertTrue(report["consistent"])
        self.assertIs(report["runtime_map_verified"], False)
        self.assertIn("runtime map verification omitted", report["note"])
        self.assertEqual(slam.mock_calls, [call.get_map_list()])

    def test_map_manager_detects_inconsistency_without_mutation(self):
        code, report, slam, _, _ = self._run_map(hmi_id=9)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "MAP_INCONSISTENT")
        self.assertEqual(report["commands_sent"], [])
        self.assertTrue(report["read_only"])
        self.assertFalse(report["consistent"])
        self.assertEqual(slam.mock_calls, [call.get_map_list()])

    def test_map_manager_rejects_map_id_name_length_mismatch(self):
        code, report, slam, _, _ = self._run_map(map_ids=(6, 7), map_names=("CUP",))
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertIn("different lengths", report["error"])
        self.assertEqual(slam.mock_calls, [call.get_map_list()])

    def test_map_manager_get_map_list_error_exits_1(self):
        code, report, slam, _, _ = self._run_map(error_code=5)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertIn("get_map_list failed", report["error"])
        self.assertEqual(slam.mock_calls, [call.get_map_list()])

    def test_map_manager_native_feedback_verifies_runtime_map(self):
        rows = [sample(1), sample(2), sample(3)]
        code, report, slam, _, _ = self._run_map(native_feedback=True, native_samples=rows)
        self.assertEqual(code, 0)
        self.assertTrue(report["runtime_map_verified"])
        self.assertTrue(report["runtime_map"]["verified"])
        self.assertEqual(slam.mock_calls, [call.get_map_list()])

    def test_map_manager_close_failure_does_not_replace_success(self):
        rows = [sample(1), sample(2), sample(3)]
        code, report, slam, _, native = self._run_map(
            native_feedback=True, native_samples=rows, close_error=RuntimeError("close failed"))
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "MAP_LIST_RECORDED")
        self.assertIn("close failed", report["close_error"])
        native.close.assert_called_once_with()
        self.assertEqual(slam.mock_calls, [call.get_map_list()])


if __name__ == "__main__":
    unittest.main()

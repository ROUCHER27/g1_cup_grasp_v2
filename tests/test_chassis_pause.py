from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from contextlib import nullcontext
from unittest.mock import Mock, call, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_tool(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


app = load_tool("chassis_pause")


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


def samples(count=5, *, speed=0.0, status=8, confidence=80.0, map_id=6, start=1):
    return [
        {
            "timestamp_ns": start + index,
            "agv_status": status,
            "position_conf": confidence,
            "agv_pos_x": 1.0,
            "agv_pos_y": 2.0,
            "agv_angle": 0.0,
            "linear_speed": speed,
            "angular_speed": 0.0,
            "map_id": map_id,
            "chassis_error": 0,
            "hal_sha256": "hal",
        }
        for index in range(count)
    ]


def hmi(*, paused=True, normal=True, scram=False, edged=False):
    return {"state": {"isPause": paused, "isNormal": normal, "isScram": scram, "isEdged": edged}}


class ChassisPauseTests(unittest.TestCase):
    def _config_root(self, temporary):
        root = Path(temporary)
        (root / "config").mkdir()
        (root / "config" / "transport.yaml").write_bytes(
            (ROOT / "tests" / "fixtures" / "transport_config_test.yaml").read_bytes())
        return root

    def _run_release(self, *, hmi_rows=None, source_motion=None, motion_at_read=None,
                     motion_status_at_read=None, source_exception=None,
                     native_feedback=False, close_error=False,
                     force_unconfirmed=False, before_samples=None):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = self._config_root(temporary.name)
        clock = FakeClock()
        slam = Mock()
        responses = [SimpleNamespace(success=True) for _ in range(10)]
        slam.pause_walk.side_effect = lambda enabled: responses.pop(0)
        read_count = [0]

        def get_position():
            read_count[0] += 1
            if source_exception is not None and read_count[0] == 1:
                raise source_exception
            moving = ((source_motion is not None and read_count[0] == 1) or
                      (motion_at_read is not None and read_count[0] == motion_at_read))
            state = {
                "linear_speed": source_motion if source_motion is not None and read_count[0] == 1
                else (0.02 if moving else 0.0),
                "angular_speed": 0.0,
                "agv_status": 7 if motion_status_at_read == read_count[0] else 8,
            }
            return state, 10 + read_count[0]

        slam.get_chassis_position.side_effect = get_position
        sdk = ModuleType("a2d_sdk.robot")
        sdk.Slam = Mock(return_value=slam)
        native = Mock()
        native.get_chassis_position.side_effect = get_position
        if close_error:
            native.close.side_effect = RuntimeError("close failed")
        sample_calls = [samples(start=1) if before_samples is None else before_samples]
        hmi_calls = list(hmi_rows or [hmi(), hmi(), hmi(paused=False)])

        def read_hmi(_config):
            if hmi_calls:
                return hmi_calls.pop(0)
            return (hmi_rows or [hmi(), hmi(), hmi(paused=False)])[-1]

        native_cls_patch = patch(
            "g1_cup_grasp.io.chassis_feedback.NativeChassisFeedback", return_value=native)
        confirmation_patch = nullcontext()
        if force_unconfirmed:
            def return_unconfirmed(target_slam, _last_stamp, feedback=None):
                response = target_slam.pause_walk(True)
                return {"stop_confirmed": False, "pause_acknowledged": response.success,
                        "pause_response": str(response), "feedback": []}

            confirmation_patch = patch.object(
                app.navigate_station, "pause_and_confirm", side_effect=return_unconfirmed)
        args = ["--release", "--confirm-path-clear"]
        if native_feedback:
            args.append("--native-feedback")
        with patch.object(app, "PROJECT_ROOT", root), \
             patch.object(app, "read_hmi", side_effect=read_hmi), \
             patch.object(app, "collect_samples", side_effect=lambda source, count=5: sample_calls.pop(0)), \
             patch.object(app.time, "monotonic", side_effect=clock.monotonic), \
             patch.object(app.time, "sleep", side_effect=clock.sleep), \
             native_cls_patch, confirmation_patch, patch("builtins.print"), \
             patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
            code = app.main(args)
        report_path = next((root / "outputs" / "navigation_check").glob("*_pause_release.json"))
        report = json.loads(report_path.read_text(encoding="utf-8"))
        return code, report, slam, sdk.Slam, read_count[0], native

    def test_no_action_flag_exits_without_sdk(self):
        sdk = ModuleType("a2d_sdk.robot")
        sdk.Slam = Mock()
        with patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
            with self.assertRaises(SystemExit) as raised:
                app.main([])
        self.assertEqual(raised.exception.code, 2)
        sdk.Slam.assert_not_called()

    def test_release_requires_both_flags(self):
        sdk = ModuleType("a2d_sdk.robot")
        sdk.Slam = Mock()
        with patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
            with self.assertRaises(SystemExit) as raised:
                app.main(["--release"])
        self.assertEqual(raised.exception.code, 2)
        sdk.Slam.assert_not_called()

    def test_release_refuses_each_unsafe_hmi_or_feedback_condition(self):
        cases = (
            (hmi(paused=False), samples()),
            (hmi(normal=False), samples()),
            (hmi(edged=True), samples()),
            (hmi(), samples(speed=0.02)),
            (hmi(), samples(confidence=49)),
        )
        for state, before in cases:
            with self.subTest(state=state, before=before[0]):
                code, report, slam, _, _, _ = self._run_release(
                    hmi_rows=[state], before_samples=before)
                self.assertEqual(code, 2)
                self.assertEqual(report["status"], "PAUSE_RELEASE_REJECTED")
                slam.pause_walk.assert_not_called()

    def test_release_refuses_scram(self):
        code, report, slam, _, _, _ = self._run_release(hmi_rows=[hmi(scram=True)])
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "PAUSE_RELEASE_REJECTED")
        slam.pause_walk.assert_not_called()

    def test_release_rechecks_hmi_pause_before_sending_continue(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi(paused=False)])
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "PAUSE_RELEASE_REJECTED")
        self.assertIn("changed before release", report["rejection_reason"])
        slam.pause_walk.assert_not_called()

    def test_release_calls_continue_once_and_repauses_on_motion(self):
        code, report, slam, _, _, _ = self._run_release(hmi_rows=[hmi(), hmi()], source_motion=0.02)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "RESUMED_MOTION_REPAUSED")
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])
        self.assertEqual(report["commands_sent"], ["pause_walk(False)", "pause_walk(True)"])

    def test_release_success_needs_hmi_unpaused_and_fresh_stationary_feedback(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi(), hmi(paused=False)])
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "PAUSE_RELEASED")
        self.assertEqual(slam.pause_walk.call_args_list, [call(False)])
        self.assertEqual(len(report["final_samples"]), 5)
        self.assertTrue(all(row["timestamp_ns"] > max(s["timestamp_ns"] for s in report["post_release_samples"])
                            for row in report["final_samples"]))
        self.assertEqual(report["hmi_after_release"]["state"]["isPause"], False)

    def test_exception_after_release_repauses_and_exits_1(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi()], source_exception=RuntimeError("feedback lost"))
        self.assertEqual(code, 1)
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])
        self.assertEqual(report["status"], "ERROR")
        self.assertEqual(report["commands_sent"], ["pause_walk(False)", "pause_walk(True)"])
        self.assertTrue(report["pre_release_samples"])
        self.assertIn("feedback lost", report["error"])

    def test_keyboard_interrupt_after_release_repauses(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi()], source_exception=KeyboardInterrupt())
        self.assertEqual(code, 1)
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])
        self.assertEqual(report["status"], "ERROR")
        self.assertEqual(report["commands_sent"], ["pause_walk(False)", "pause_walk(True)"])
        self.assertIn("KeyboardInterrupt", report["error"])

    def test_motion_during_hmi_wait_is_repaused(self):
        # The 5-second release monitor consumes 25 samples; sample 26 arrives while waiting for HMI.
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi(), hmi(paused=False)], motion_at_read=26)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "RESUMED_MOTION_REPAUSED")
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])
        self.assertEqual(report["commands_sent"], ["pause_walk(False)", "pause_walk(True)"])

    def test_walking_status_with_zero_speed_is_repaused(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi()], motion_status_at_read=1)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "RESUMED_MOTION_REPAUSED")
        self.assertEqual(report["post_release_samples"][0]["agv_status"], 7)
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])

    def test_pause_not_released_is_repaused_and_confirmed(self):
        code, report, slam, _, _, _ = self._run_release(hmi_rows=[hmi(), hmi()])
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "PAUSE_NOT_RELEASED_REPAUSED")
        self.assertTrue(report["stop_confirmation"]["stop_confirmed"])
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])

    def test_unconfirmed_repause_exits_1(self):
        code, report, slam, _, _, _ = self._run_release(
            hmi_rows=[hmi(), hmi()], force_unconfirmed=True)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "REPAUSE_UNCONFIRMED")
        self.assertEqual(slam.pause_walk.call_args_list, [call(False), call(True)])

    def test_final_sample_collector_discards_stale_feedback(self):
        clock = FakeClock()
        source = Mock()
        source.get_chassis_position.side_effect = [
            ({"linear_speed": 0.0}, 10), ({"linear_speed": 0.0}, 11),
            ({"linear_speed": 0.0}, 11), ({"linear_speed": 0.0}, 12),
            ({"linear_speed": 0.0}, 13),
        ]
        result = app.collect_fresh_samples(
            source, 10, 3, now=clock.monotonic, sleep=clock.sleep)
        self.assertEqual([row["timestamp_ns"] for row in result], [11, 12, 13])

    def test_pause_success_and_failure_are_reported(self):
        for success, expected_code, expected_status in (
                (True, 0, "PAUSE_REQUESTED"), (False, 2, "PAUSE_REQUEST_FAILED")):
            with self.subTest(success=success):
                temporary = TemporaryDirectory()
                self.addCleanup(temporary.cleanup)
                root = self._config_root(temporary.name)
                slam = Mock()
                slam.pause_walk.return_value = SimpleNamespace(success=success)
                sdk = ModuleType("a2d_sdk.robot")
                sdk.Slam = Mock(return_value=slam)
                with patch.object(app, "PROJECT_ROOT", root), patch("builtins.print"), \
                     patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
                    code = app.main(["--pause"])
                report_path = next((root / "outputs" / "navigation_check").glob("*_pause.json"))
                report = json.loads(report_path.read_text(encoding="utf-8"))
                self.assertEqual(code, expected_code)
                self.assertEqual(report["status"], expected_status)
                self.assertEqual(report["commands_sent"], ["pause_walk(True)"])
                self.assertEqual(report["pause_success"], success)
                slam.pause_walk.assert_called_once_with(True)

    def test_pause_rejects_native_feedback_flag(self):
        sdk = ModuleType("a2d_sdk.robot")
        sdk.Slam = Mock()
        with patch.dict("sys.modules", {"a2d_sdk.robot": sdk}):
            with self.assertRaises(SystemExit) as raised:
                app.main(["--pause", "--native-feedback"])
        self.assertEqual(raised.exception.code, 2)
        sdk.Slam.assert_not_called()

    def test_native_close_failure_does_not_replace_success(self):
        code, report, _, _, _, native = self._run_release(
            hmi_rows=[hmi(), hmi(), hmi(paused=False)], native_feedback=True, close_error=True)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "PAUSE_RELEASED")
        self.assertIn("close failed", report["close_error"])
        native.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()

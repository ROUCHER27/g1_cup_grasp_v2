#!/usr/bin/env python3
"""Safely release a paused G1 chassis after confirming the path is clear."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from check_navigation import FIELDS, STATE_NAMES, collect_samples, read_hmi
import navigate_station
from navigate_station import reject_if, validate_transport_config
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait


def _hmi_state(hmi: dict) -> dict:
    state = hmi.get("state", hmi)
    if not isinstance(state, dict):
        raise RuntimeError("HMI state is unavailable")
    return state


def _valid_speed(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def _stationary(sample: dict) -> bool:
    linear = sample.get("linear_speed")
    angular = sample.get("angular_speed")
    return (_valid_speed(linear) and abs(linear) <= 0.01 and
            _valid_speed(angular) and abs(angular) <= 0.02)


def _moving(sample: dict) -> bool:
    return sample.get("agv_status") == 7 or not _stationary(sample)


def _feedback_row(state: dict, stamp: int) -> dict:
    row = {key: state.get(key) for key in FIELDS}
    row["timestamp_ns"] = int(stamp)
    row["status_name"] = STATE_NAMES.get(state.get("agv_status"), "Unknown")
    return row


def _watch_once(source, last_stamp: int) -> tuple[int, dict | None]:
    """Read one feedback update and return it only when its timestamp advances."""
    feedback, stamp = source.get_chassis_position()
    stamp = int(stamp)
    if stamp <= last_stamp:
        return last_stamp, None
    return stamp, _feedback_row(feedback, stamp)


def collect_fresh_samples(source, last_stamp: int, count: int, *, timeout_s: float = 10.0,
                          now=None, sleep=None) -> list[dict]:
    """Collect only samples newer than the supplied timestamp."""
    now = time.monotonic if now is None else now
    sleep = time.sleep if sleep is None else sleep
    samples = []
    deadline = now() + timeout_s
    while len(samples) < count and now() < deadline:
        last_stamp, sample = _watch_once(source, last_stamp)
        if sample is not None:
            samples.append(sample)
        if len(samples) < count:
            remaining = deadline - now()
            if remaining > 0:
                sleep(min(0.5, remaining))
    if len(samples) < count:
        raise RuntimeError(f"Expected {count} fresh chassis samples, received {len(samples)}")
    return samples


def _record_confirmation(report: dict, confirmation: dict) -> bool:
    report["stop_confirmation"] = confirmation
    if "pause_response" in confirmation:
        report["repause_response"] = confirmation["pause_response"]
    if "pause_acknowledged" in confirmation:
        report["repause_success"] = confirmation["pause_acknowledged"] is True
    if confirmation.get("error"):
        report["repause_error"] = confirmation["error"]
    return confirmation.get("stop_confirmed") is True


def _repause_and_confirm(slam, source, last_stamp: int, report: dict,
                         *, success_status: str, rejection_reason: str) -> int:
    report["commands_sent"].append("pause_walk(True)")
    confirmation = navigate_station.pause_and_confirm(slam, last_stamp, feedback=source)
    if _record_confirmation(report, confirmation):
        report.update(status=success_status, rejection_reason=rejection_reason)
        return 2
    report.update(status="REPAUSE_UNCONFIRMED", origin_status=success_status,
                  rejection_reason=rejection_reason)
    return 1


def release_pause(slam, source, config, now=None, sleep=None, report: dict | None = None) -> dict:
    """Release pause, watching fresh feedback and restoring pause on motion."""
    now = time.monotonic if now is None else now
    sleep = time.sleep if sleep is None else sleep
    report = {} if report is None else report
    report.setdefault("read_only", False)
    report.setdefault("commands_sent", [])
    report.setdefault("pre_release_samples", [])
    report.setdefault("post_release_samples", [])
    report.setdefault("final_samples", [])
    release_issued = False
    repause_succeeded = False

    try:
        hmi = read_hmi(config)
        before = collect_samples(source, count=5)
        report.update(hmi=hmi, pre_release_samples=before)

        state = _hmi_state(hmi)
        try:
            reject_if(state.get("isPause") is not True,
                      "nothing to release: HMI isPause is not true")
            reject_if(state.get("isNormal") is not True, "HMI isNormal is not true")
            reject_if(any(state.get(key) is not False for key in ("isScram", "isEdged")),
                      "HMI scram or edge-touch state is not confirmed clear")
            reject_if(any(not _stationary(sample) for sample in before),
                      "pre-release chassis feedback is not stationary")
            confidences = [sample.get("position_conf") for sample in before]
            reject_if(any(not _valid_speed(value) for value in confidences) or min(confidences) < 50,
                      "position confidence is below 50 or invalid")
        except RuntimeError as exc:
            report.update(status="PAUSE_RELEASE_REJECTED", rejection_reason=str(exc))
            return report

        last_stamp = max(int(sample["timestamp_ns"]) for sample in before)

        # Recheck the operator-facing pause state immediately before sending continue.
        hmi_before_release = read_hmi(config)
        report["hmi_before_release"] = hmi_before_release
        try:
            reject_if(_hmi_state(hmi_before_release).get("isPause") is not True,
                      "HMI isPause changed before release")
        except RuntimeError as exc:
            report.update(status="PAUSE_RELEASE_REJECTED", rejection_reason=str(exc))
            return report

        report["commands_sent"].append("pause_walk(False)")
        release_issued = True
        response = slam.pause_walk(False)
        report["continue_response"] = str(response)
        report["continue_success"] = getattr(response, "success", None) is True
        if report["continue_success"] is not True:
            raise RuntimeError("pause_walk(False) did not report success")

        def watch_and_repause_if_moving() -> int | None:
            nonlocal last_stamp, repause_succeeded
            last_stamp, sample = _watch_once(source, last_stamp)
            if sample is None:
                return None
            report["post_release_samples"].append(sample)
            if not _moving(sample):
                return None
            code = _repause_and_confirm(
                slam, source, last_stamp, report,
                success_status="RESUMED_MOTION_REPAUSED",
                rejection_reason="motion detected after pause release",
            )
            repause_succeeded = report.get("repause_success") is True
            return code

        monitor_deadline = now() + 5.0
        while now() < monitor_deadline:
            code = watch_and_repause_if_moving()
            if code is not None:
                return report
            remaining = monitor_deadline - now()
            if remaining > 0:
                sleep(min(0.2, remaining))

        if not report["post_release_samples"]:
            raise RuntimeError("no new chassis feedback arrived during post-release monitoring")

        hmi_deadline = now() + 5.0
        while now() < hmi_deadline:
            hmi_after = read_hmi(config)
            report["hmi_after_release"] = hmi_after
            code = watch_and_repause_if_moving()
            if code is not None:
                return report
            if _hmi_state(hmi_after).get("isPause") is False:
                break
            remaining = hmi_deadline - now()
            if remaining > 0:
                sleep(min(0.2, remaining))
        else:
            _repause_and_confirm(
                slam, source, last_stamp, report,
                success_status="PAUSE_NOT_RELEASED_REPAUSED",
                rejection_reason="HMI still reports pause after 5 seconds",
            )
            repause_succeeded = report.get("repause_success") is True
            return report

        final_samples = collect_fresh_samples(source, last_stamp, count=5, now=now, sleep=sleep)
        report["final_samples"] = final_samples
        if any(_moving(sample) for sample in final_samples):
            last_stamp = max(int(sample["timestamp_ns"]) for sample in final_samples)
            _repause_and_confirm(
                slam, source, last_stamp, report,
                success_status="RESUMED_MOTION_REPAUSED",
                rejection_reason="final chassis feedback was not stationary",
            )
            repause_succeeded = report.get("repause_success") is True
            return report

        report["status"] = "PAUSE_RELEASED"
        return report
    except BaseException:
        if release_issued and not repause_succeeded:
            report["commands_sent"].append("pause_walk(True)")
            try:
                response = slam.pause_walk(True)
                report["repause_response"] = str(response)
                report["repause_success"] = getattr(response, "success", None) is True
                repause_succeeded = report["repause_success"]
            except BaseException as repause_exc:
                report["repause_error"] = f"{type(repause_exc).__name__}: {repause_exc}"
        raise


def _report_path(directory: Path, suffix: str) -> Path:
    now = datetime.now(timezone.utc)
    return directory / f"{now:%Y%m%d_%H%M%S_%f}_{suffix}.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--release", action="store_true", help="release an existing chassis pause")
    action.add_argument("--pause", action="store_true", help="send pause_walk(True) once")
    parser.add_argument("--confirm-path-clear", action="store_true",
                        help="confirm the physical path is clear before --release")
    parser.add_argument("--native-feedback", action="store_true",
                        help="use native chassis feedback, including the runtime map ID")
    args = parser.parse_args(argv)
    if not args.release and not args.pause:
        parser.error("choose --release or --pause")
    if args.release and not args.confirm_path_clear:
        parser.error("--release requires --confirm-path-clear")
    if args.pause and args.native_feedback:
        parser.error("--native-feedback applies only to --release")

    report = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "commands_sent": [],
        "pre_release_samples": [],
        "post_release_samples": [],
        "final_samples": [],
    }
    native = None
    code = 1
    try:
        config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8"))
        validate_transport_config(config)
        from a2d_sdk.robot import Slam

        slam = Slam()
        if args.release:
            source = slam
            if args.native_feedback:
                from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
                native = NativeChassisFeedback(config["native_feedback"])
                source = native
            release_pause(slam, source, config, now=time.monotonic, sleep=time.sleep, report=report)
            status = report.get("status")
            code = 0 if status == "PAUSE_RELEASED" else (
                1 if status in ("ERROR", "REPAUSE_UNCONFIRMED") else 2)
        else:
            report["commands_sent"].append("pause_walk(True)")
            response = slam.pause_walk(True)
            pause_success = getattr(response, "success", None) is True
            report.update(status="PAUSE_REQUESTED" if pause_success else "PAUSE_REQUEST_FAILED",
                          read_only=False, pause_response=str(response), pause_success=pause_success)
            code = 0 if pause_success else 2
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        code = 1
    finally:
        if native is not None:
            try:
                native.close()
            except BaseException as exc:
                report["close_error"] = f"{type(exc).__name__}: {exc}"

    suffix = "pause_release" if args.release else "pause"
    directory = PROJECT_ROOT / "outputs" / "navigation_check"
    directory.mkdir(parents=True, exist_ok=True)
    path = _report_path(directory, suffix)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

#!/usr/bin/env python3
"""Record a read-only candidate for a G1 chassis docking station."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from check_navigation import (collect_samples, read_controller, read_hmi, runtime_map_check,
                              summarize)
from navigate_station import HAL_SHA256, navigation_request, reject_if, validate_transport_config
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait


def station_candidate(samples: list[dict]) -> dict:
    reject_if(not samples, "no chassis samples to teach")
    sine = sum(math.sin(sample["agv_angle"]) for sample in samples) / len(samples)
    cosine = sum(math.cos(sample["agv_angle"]) for sample in samples) / len(samples)
    return {
        "x_m": sum(sample["agv_pos_x"] for sample in samples) / len(samples),
        "y_m": sum(sample["agv_pos_y"] for sample in samples) / len(samples),
        "yaw_rad": math.atan2(sine, cosine),
        "map_id": samples[-1].get("map_id"),
    }


def _finite(value) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _has_only_finite_numeric_fields(samples: list[dict]) -> bool:
    required_numeric_fields = (
        "timestamp_ns", "agv_status", "position_conf", "agv_pos_x", "agv_pos_y",
        "agv_angle", "linear_speed", "angular_speed", "map_id", "chassis_error",
    )
    for sample in samples:
        if any(not _finite(sample.get(key)) for key in required_numeric_fields):
            return False
        if any(isinstance(value, (int, float)) and not _finite(value)
               for value in sample.values()):
            return False
    return True


def teach_station(native, config: dict, station: str, count: int) -> dict:
    validate_transport_config(config)
    if station not in ("A", "B"):
        raise ValueError("station must be A or B")
    if type(count) is not int or count < 3:
        raise ValueError("sample count must be an integer of at least 3")

    samples = collect_samples(native, count=count)
    try:
        hmi = read_hmi(config)
        hmi_error = None
    except Exception as exc:
        hmi = None
        hmi_error = f"{type(exc).__name__}: {exc}"
    try:
        controller = read_controller(config)
        controller_error = None
    except Exception as exc:
        controller = None
        controller_error = f"{type(exc).__name__}: {exc}"

    samples_finite = _has_only_finite_numeric_fields(samples)
    if samples_finite:
        candidate = station_candidate(samples)
        stability = summarize(samples, candidate)
        runtime_map = runtime_map_check(samples, candidate["map_id"])
    else:
        candidate = None
        stability = None
        expected_map_id = samples[-1].get("map_id") if samples else config["map_id"]
        runtime_map = runtime_map_check(samples, expected_map_id)
    saved = config["stations"][station]
    map_matches_config = candidate is not None and candidate["map_id"] == config["map_id"]
    sample_hashes = [sample.get("hal_sha256") for sample in samples]

    reasons = []
    if not samples_finite:
        reasons.append("one or more chassis sample fields are missing, invalid, or non-finite")
    if stability is not None and (stability["x_range_m"] > 0.02 or stability["y_range_m"] > 0.02 or
            stability["yaw_range_deg"] > 2.0):
        reasons.append("station feedback is unstable")
    if stability is not None and (stability["max_abs_linear_speed_m_s"] > 0.01 or
            stability["max_abs_angular_speed_rad_s"] > 0.02):
        reasons.append("chassis feedback exceeds stationary speed thresholds")
    if any(sample.get("agv_status") != 8 for sample in samples):
        reasons.append("one or more chassis samples are not Idle")
    if stability is not None and stability["confidence_range"][0] < 50:
        reasons.append("position confidence is below 50")
    if not runtime_map["verified"]:
        reasons.append("runtime map verification failed")

    report = {
        "status": "STATION_TEACH_REJECTED" if reasons else "STATION_TEACH_RECORDED",
        "read_only": True,
        "station": station,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "samples": samples,
        "runtime_map": runtime_map,
        "hmi": hmi,
        "controller": controller,
        "station_candidate": candidate,
        "planned_request_preview": navigation_request(candidate) if candidate is not None else None,
        "current_config_station": dict(saved),
        "delta_vs_config": None,
        "stability": stability,
        "map_matches_config": map_matches_config,
        "hal_sha256_matches": bool(sample_hashes) and all(value == HAL_SHA256 for value in sample_hashes),
        "note": "candidate only; config/transport.yaml not modified",
    }
    if candidate is not None:
        report["delta_vs_config"] = {
            "dx_m": candidate["x_m"] - saved["x_m"],
            "dy_m": candidate["y_m"] - saved["y_m"],
            "dyaw_deg": math.degrees(math.atan2(
                math.sin(candidate["yaw_rad"] - saved["yaw_rad"]),
                math.cos(candidate["yaw_rad"] - saved["yaw_rad"]),
            )),
        }
    if hmi_error is not None:
        report["hmi_error"] = hmi_error
    if controller_error is not None:
        report["controller_error"] = controller_error
    if reasons:
        report["rejection_reasons"] = reasons
    return report


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def _sample_count(value: str) -> int:
    parsed = int(value)
    if parsed < 3:
        raise argparse.ArgumentTypeError("must be an integer of at least 3")
    return parsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", choices=("A", "B"), required=True)
    parser.add_argument("--samples", type=_sample_count, default=5)
    parser.add_argument("--expect-map-id", type=_positive_int)
    args = parser.parse_args(argv)

    report = {"status": "ERROR", "read_only": True, "station": args.station,
              "note": "candidate only; config/transport.yaml not modified"}
    native = None
    code = 1
    try:
        config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8"))
        validate_transport_config(config)
        native = NativeChassisFeedback(config["native_feedback"])
        report = teach_station(native, config, args.station, args.samples)
        if args.expect_map_id is not None:
            report["expected_map_id"] = args.expect_map_id
            candidate = report.get("station_candidate")
            if candidate is None or candidate["map_id"] != args.expect_map_id:
                report["status"] = "STATION_TEACH_REJECTED"
                report.setdefault("rejection_reasons", []).append(
                    f"runtime map ID does not match expected map {args.expect_map_id}")
        code = 2 if report["status"] == "STATION_TEACH_REJECTED" else 0
    except Exception as exc:
        report.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        code = 1
    finally:
        if native is not None:
            try:
                native.close()
            except BaseException as exc:
                report["close_error"] = f"{type(exc).__name__}: {exc}"

    directory = PROJECT_ROOT / "outputs" / "station_teach"
    directory.mkdir(parents=True, exist_ok=True)
    created = datetime.now(timezone.utc)
    path = directory / f"station_teach_{created:%Y%m%d_%H%M%S_%f}_{args.station}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

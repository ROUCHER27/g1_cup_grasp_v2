#!/usr/bin/env python3
"""Inspect a controller avoidance snapshot without changing any robot setting.

Live reads require --live. All other modes operate on local JSON files. Captures
produce review_required candidates; this tool cannot approve or apply a profile.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools")]
from g1_cup_grasp.avoidance_guard import build_manifest_candidate, validate_manifest, verify_snapshot
from g1_cup_grasp.io.controller_avoidance import read_avoidance_snapshot
from navigate_station import HAL_SHA256


def _invalid_constant(value):
    raise ValueError(f"non-finite JSON constant: {value}")


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("JSON number must be finite")
    return number


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=_invalid_constant,
                      parse_float=_finite_float)


def _envelope(path: Path, endpoint: str):
    value = _read_json(path)
    if (not isinstance(value, dict) or type(value.get("ErrorCode")) is not int
            or value["ErrorCode"] != 0 or value.get("Interface") != endpoint
            or not isinstance(value.get("Content"), dict)
            or not isinstance(value.get("TimeStamp"), str) or not value["TimeStamp"].strip()):
        raise ValueError(f"invalid saved response for {endpoint}")
    return value["Content"], value["TimeStamp"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--live", action="store_true", help="read current controller parameters; no write endpoints")
    source.add_argument("--snapshot", type=Path, help="saved snapshot or this tool's saved report")
    source.add_argument("--parameters-file", type=Path, help="saved Agv/GetParam response envelope")
    parser.add_argument("--device-file", type=Path, help="saved GetDeviceInfo response, paired with --parameters-file")
    parser.add_argument("--config", type=Path, default=ROOT / "config/transport.yaml")
    parser.add_argument("--manifest", type=Path, help="compare with an already reviewed manifest")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if bool(args.parameters_file) != bool(args.device_file):
        parser.error("--parameters-file and --device-file must be supplied together")
    report = {"read_only": True, "robot_commands_sent": False, "live_read": args.live,
              "created_at_utc": datetime.now(timezone.utc).isoformat(),
              "native_runtime_map_verified": False, "physical_3d_collision_verified": False,
              "sensor_coverage_verified": False, "route_clearance_verified": False}
    code = 0
    try:
        config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        if args.live:
            snapshot = read_avoidance_snapshot(config)
        elif args.snapshot:
            value = _read_json(args.snapshot)
            snapshot = value.get("snapshot", value)
        else:
            parameters, pt = _envelope(args.parameters_file, "/api/Config/Agv/GetParam")
            device, dt = _envelope(args.device_file, "/api/AMR/GetDeviceInfo")
            snapshot = {"device": device, "parameters": parameters, "received_monotonic": 0.0,
                        "query_duration_s": 0.0, "controller_timestamps": {"device": dt, "parameters": pt}}
        report["snapshot"] = snapshot
        if (type(snapshot["parameters"].get("mapId")) is not int
                or snapshot["parameters"]["mapId"] != config["map_id"]
                or type(snapshot["parameters"].get("id")) is not int
                or snapshot["parameters"]["id"] != config["robot_id"]):
            raise RuntimeError("snapshot robot or map does not match the selected config")
        if args.manifest:
            manifest = validate_manifest(_read_json(args.manifest),
                                         robot_id=config["robot_id"], map_id=config["map_id"],
                                         map_name=config["map_name"], hal_sha256=HAL_SHA256)
            report["verification"] = verify_snapshot(manifest, snapshot)
            report["status"] = "LIVE_CONFIGURATION_MATCH" if args.live else "OFFLINE_CONFIGURATION_MATCH"
        else:
            report["manifest_candidate"] = build_manifest_candidate(
                snapshot, map_name=config["map_name"], hal_sha256=HAL_SHA256)
            report["status"] = "MANIFEST_REVIEW_REQUIRED"
        report["live_freshness_verified"] = False
        if args.live:
            age = time.monotonic() - snapshot["received_monotonic"]
            if not 0 <= age <= 2.0 or not 0 <= snapshot["query_duration_s"] <= 2.0:
                raise RuntimeError("controller configuration readback is stale or slow")
            report["live_freshness_verified"] = True
        report["scope"] = "Configuration capture/match only; no profile approval, runtime-map proof, or physical safety certification."
    except Exception as exc:
        report.update(status="ERROR", error=f"{type(exc).__name__}: {exc}", live_freshness_verified=False)
        code = 1
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"report_path": str(args.output.resolve()), "status": report["status"],
                          "error": report.get("error"), "read_only": True}, ensure_ascii=False))
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())

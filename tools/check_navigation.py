#!/usr/bin/env python3
"""Read maps and chassis feedback; never send mode or motion commands."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import urllib.request

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait

STATE_NAMES = {
    0: "Resetting", 1: "EdgedTouch", 2: "Scram", 3: "Mapping",
    4: "NeedRelocate", 5: "Avoid", 6: "Charging", 7: "Walking",
    8: "Idle", 9: "NotReady",
}
FIELDS = (
    "agv_status", "position_conf", "agv_pos_x", "agv_pos_y",
    "agv_angle", "linear_speed", "angular_speed", "agv_task_state", "map_id", "chassis_error", "hal_sha256",
)


def angle_error_rad(actual: float, target: float) -> float:
    return math.atan2(math.sin(actual - target), math.cos(actual - target))


def collect_samples(slam, count: int = 5, timeout_s: float = 10.0) -> list[dict]:
    samples = []
    deadline = time.monotonic() + timeout_s
    previous_stamp = 0
    while len(samples) < count and time.monotonic() < deadline:
        state, stamp = slam.get_chassis_position()
        stamp = int(stamp)
        if stamp > previous_stamp:
            sample = {key: state.get(key) for key in FIELDS}
            sample["timestamp_ns"] = stamp
            sample["status_name"] = STATE_NAMES.get(state.get("agv_status"), "Unknown")
            samples.append(sample)
            previous_stamp = stamp
        time.sleep(0.5)
    if len(samples) < count:
        raise RuntimeError(f"Expected {count} updating feedback samples, received {len(samples)}")
    return samples


def summarize(samples: list[dict], target: dict) -> dict:
    last = samples[-1]
    dx = last["agv_pos_x"] - target["x_m"]
    dy = last["agv_pos_y"] - target["y_m"]
    yaw_offsets = [angle_error_rad(s["agv_angle"], samples[0]["agv_angle"]) for s in samples]
    return {
        "position_error_m": math.hypot(dx, dy),
        "dx_m": dx,
        "dy_m": dy,
        "yaw_error_deg": math.degrees(angle_error_rad(last["agv_angle"], target["yaw_rad"])),
        "x_range_m": max(s["agv_pos_x"] for s in samples) - min(s["agv_pos_x"] for s in samples),
        "y_range_m": max(s["agv_pos_y"] for s in samples) - min(s["agv_pos_y"] for s in samples),
        "yaw_range_deg": math.degrees(max(yaw_offsets) - min(yaw_offsets)),
        "confidence_range": [min(s["position_conf"] for s in samples), max(s["position_conf"] for s in samples)],
        "max_abs_linear_speed_m_s": max(abs(s["linear_speed"]) for s in samples),
        "max_abs_angular_speed_rad_s": max(abs(s["angular_speed"]) for s in samples),
    }


def read_hmi(config: dict) -> dict:
    def query(endpoint: str, payload: dict) -> dict:
        # These POST endpoints only query state/config; no command endpoint is used.
        request = urllib.request.Request(
            config["hmi_url"].rstrip("/") + endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(request, timeout=5.0) as response:
            result = json.load(response)
        if result["ErrorCode"] != 0:
            raise RuntimeError(f"{endpoint}: {result.get('ErrorMessage')}")
        return result["Content"]

    state = query("/api/AMR/GetState", {"id": config["robot_id"]})
    groups = query("/api/Config/GetAllParam", {})
    robot = next(item for item in groups["Agv"] if item["id"] == config["robot_id"])
    fields = (
        "id", "globalPose", "confidence", "protoStatus", "workStatus",
        "navigationStatus", "isHardAuto", "isSoftAuto", "isNormal",
        "isPause", "isScram", "isEdged", "batteryStateList", "exceptions", "updateTime",
        "isAvoid", "avoidStatus", "avoidSensorList",
    )
    return {
        "read_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_config_map_id": robot["mapId"],
        "map_assignment_matches": robot["mapId"] == config["map_id"],
        "state": {key: state.get(key) for key in fields},
        "global_pose_units": "x/y: mm, theta: degrees (HMI)",
    }


def runtime_map_check(samples: list[dict], expected_id: int) -> dict:
    ids = [sample.get("map_id") for sample in samples]
    available = bool(ids) and all(type(value) is int and value > 0 for value in ids)
    stamps = [sample.get("timestamp_ns") for sample in samples]
    updating = len(stamps) >= 3 and all(type(stamp) is int and stamp > 0 for stamp in stamps)
    updating = updating and all(second > first for first, second in zip(stamps, stamps[1:]))
    verified = available and updating and all(value == expected_id for value in ids)
    return {
        "source": "/hal/position.map_id",
        "available": available,
        "updating": updating,
        "map_ids": ids,
        "verified": verified,
        "note": ("updating position feedback reports the requested map"
                 if verified else "configured/driver-selected map cannot replace missing or mismatching runtime map feedback"),
    }


def read_controller(config: dict) -> dict:
    def query(endpoint: str) -> dict:
        request = urllib.request.Request(
            config["controller_url"].rstrip("/") + endpoint,
            data=json.dumps({"id": config["robot_id"]}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(request, timeout=5.0) as response:
            result = json.load(response)
        if result["ErrorCode"] != 0:
            raise RuntimeError(f"{endpoint}: {result.get('ErrorMessage')}")
        return result["Content"]

    device = query("/api/AMR/GetDeviceInfo")
    params = query("/api/Config/Agv/GetParam")
    return {
        "read_at_utc": datetime.now(timezone.utc).isoformat(),
        "serial_number": str(device["serialNumber"]),
        "software_version": device.get("softwareVersion"),
        "firmware_version": device.get("firmwareVersion"),
        "driver_selected_map_id": params["mapId"],
        "map_selection_matches": params["mapId"] == config["map_id"],
        "map_source": "/api/Config/Agv/GetParam.mapId (also used by HAL GetCurrentMap)",
        "independent_slam_map_verified": False,
        "auto_motion_parameters": params.get("moveControlParam", {}).get("autoMoveParam", {}),
        "chassis_shape": params.get("chassisParam", {}),
    }

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", choices=["A", "B"], required=True,
                        help="compare current feedback with this saved chassis stop")
    parser.add_argument("--native-feedback", action="store_true", help="read original ROS position over SSH")
    args = parser.parse_args(argv)
    config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8"))
    now = datetime.now(timezone.utc)
    report = {
        "checked_at_utc": now.isoformat(), "read_only": True,
        "station": args.station, "config": config,
        "active_map_id_verified": False,
        "active_map_note": "GDK 1.5 Position protobuf does not expose active map_id; map existence does not confirm the active map.",
        "physical_reach_and_clearance_verified": False,
    }
    code = 0
    native = None
    try:
        from a2d_sdk.robot import Slam
        slam = Slam()
        response = slam.get_map_list()
        report["map_list"] = {
            "error_code": int(response.error_code),
            "map_id": list(response.map_id), "map_name": list(response.map_name),
        }
        report["requested_map_exists"] = (
            response.error_code == 0 and config["map_id"] in response.map_id
        )
        if args.native_feedback:
            from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
            native = NativeChassisFeedback(config["native_feedback"])
        report["samples"] = collect_samples(native or slam)
        report["summary"] = summarize(report["samples"], config["stations"][args.station])
        report["hmi"] = read_hmi(config)
        report["controller"] = read_controller(config)
        report["runtime_map"] = runtime_map_check(report["samples"], config["map_id"])
        report["active_map_id_verified"] = report["runtime_map"]["verified"]
        report["active_map_note"] = report["runtime_map"]["note"]
        report["navigation_ready"] = False
        map_confirmed = (
            report["requested_map_exists"] and report["hmi"]["map_assignment_matches"]
            and report["controller"]["map_selection_matches"]
            and (not report["runtime_map"]["available"] or report["runtime_map"]["verified"])
        )
        report["status"] = "INSPECTION_RECORDED" if map_confirmed else "MAP_NOT_CONFIRMED"
        code = 0 if map_confirmed else 2
    except Exception as exc:
        report["status"] = "ERROR"
        report["error"] = f"{type(exc).__name__}: {exc}"
        code = 1
    if native is not None:
        native.close()
    directory = PROJECT_ROOT / "outputs" / "navigation_check"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{now:%Y%m%d_%H%M%S_%f}_{args.station}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())


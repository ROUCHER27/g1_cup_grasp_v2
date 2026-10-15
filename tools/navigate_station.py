#!/usr/bin/env python3
"""Inspect a saved stop or preview/execute the verified G1 navigation contract."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import yaml
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from check_navigation import collect_samples, read_controller, read_hmi, runtime_map_check, summarize
from g1_cup_grasp.config import load_config
from g1_cup_grasp.io.robot import GdkRightArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait

# Verified against this G1 HAL binary; Position feedback uses different units.
HAL_SHA256 = "1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309"
MIN_CONFIDENCE = 50.0


def reject_if(condition: bool, reason: str) -> None:
    if condition:
        raise RuntimeError(reason)


def navigation_request(target: dict) -> dict:
    values = [float(target[key]) for key in ("x_m", "y_m", "yaw_rad")]
    reject_if(not all(math.isfinite(value) for value in values), "non-finite navigation target")
    return {
        "x_mm": int(round(values[0] * 1000.0)),
        "y_mm": int(round(values[1] * 1000.0)),
        "yaw_deg": int(round(math.degrees(values[2]))) % 360,
        "frame": "active_map",
        "endpoint": "/hal/nav/navigate_to_pose",
        "verified_hal_sha256": HAL_SHA256,
    }


def validate_preflight(samples: list[dict], hmi: dict, controller: dict, config: dict) -> None:
    reject_if(len(samples) < 3, "insufficient updating preflight feedback")
    for sample in samples:
        keys = ("agv_pos_x", "agv_pos_y", "agv_angle", "position_conf", "linear_speed", "angular_speed")
        reject_if(not all(isinstance(sample.get(key), (int, float)) and
                         math.isfinite(sample[key]) for key in keys), "invalid preflight feedback")
    reject_if(not runtime_map_check(samples, config["map_id"])["verified"],
              "runtime map ID is missing or mismatches; controller configuration is insufficient")
    reject_if(not hmi["map_assignment_matches"] or not controller["map_selection_matches"],
              "configured/driver-selected map does not match")
    reject_if(controller["serial_number"] != str(config["robot_id"]), "controller robot identity mismatches")
    state = hmi["state"]
    batteries = state.get("batteryStateList")
    reject_if(not batteries or any(type(item.get("isCharging")) is not bool for item in batteries),
              "charging state is unknown")
    reject_if(any(item["isCharging"] for item in batteries), "robot is charging")
    reject_if(state.get("isNormal") is not True or any(
        state.get(key) is not False for key in ("isScram", "isEdged", "isPause")
    ), "HMI normal/emergency/touch/pause state is not confirmed safe")
    reject_if(state.get("isHardAuto") is not True or state.get("isSoftAuto") is not True,
              "automatic navigation mode is not confirmed")
    reject_if(any(sample["agv_status"] != 8 for sample in samples), "chassis is not Idle")
    initial = summarize(samples, config["stations"]["A"])
    reject_if(initial["confidence_range"][0] < MIN_CONFIDENCE, "localization confidence is below 50")
    reject_if(initial["position_error_m"] > 0.08 or abs(initial["yaw_error_deg"]) > 12.0,
              "robot is not localized near station A")
    reject_if(initial["x_range_m"] > 0.02 or initial["y_range_m"] > 0.02 or initial["yaw_range_deg"] > 2,
              "localization is unstable while stationary")
    reject_if(initial["max_abs_linear_speed_m_s"] > 0.01 or
              initial["max_abs_angular_speed_rad_s"] > 0.02, "chassis is not stationary")


class NavigationWatchdog:
    def __init__(self, destination: dict, map_id: int, initial: dict, now: float):
        self.destination = destination
        self.map_id = map_id
        self.last_stamp = int(initial["timestamp_ns"])
        self.last_feedback_time = now
        self.last_progress_time = now
        self.best_distance = math.inf
        self.best_yaw_error = math.inf
        self.progress_distance = math.inf
        self.progress_yaw_error = math.inf
        self.arrival_since = None
        self.arrival_count = 0

    def update(self, sample: dict, stamp: int, now: float) -> dict | None:
        reject_if(stamp < self.last_stamp, "chassis timestamp regressed")
        if stamp == self.last_stamp:
            reject_if(now - self.last_feedback_time > 2.0, "chassis feedback stale for over 2 seconds")
            return None
        self.last_stamp = stamp
        self.last_feedback_time = now
        keys = ("agv_pos_x", "agv_pos_y", "agv_angle", "position_conf", "linear_speed", "angular_speed")
        reject_if(not all(isinstance(sample.get(key), (int, float)) and
                         math.isfinite(sample[key]) for key in keys), "invalid chassis feedback")
        reject_if(sample.get("map_id") != self.map_id, "runtime map ID changed or disappeared")
        reject_if(sample.get("chassis_error") not in (None, 0), "chassis reports an error")
        reject_if(sample.get("agv_status") not in (7, 8), f"chassis protection/state {sample.get('agv_status')}")
        confidence = sample["position_conf"]
        reject_if(confidence < MIN_CONFIDENCE, "localization confidence is below 50")

        distance = math.hypot(sample["agv_pos_x"] - self.destination["x_m"],
                              sample["agv_pos_y"] - self.destination["y_m"])
        delta = sample["agv_angle"] - self.destination["yaw_rad"]
        yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
        self.best_distance = min(self.best_distance, distance)
        self.best_yaw_error = min(self.best_yaw_error, yaw_error)
        reject_if(self.best_distance <= 0.25 and distance > self.best_distance + 0.10,
                  "robot moved away after approaching the target")
        if (self.progress_distance - distance >= 0.02 or
                self.progress_yaw_error - yaw_error >= math.radians(5)):
            self.last_progress_time = now
            self.progress_distance = min(self.progress_distance, distance)
            self.progress_yaw_error = min(self.progress_yaw_error, yaw_error)
        reject_if(now - self.last_progress_time > 10.0, "navigation made no verified progress for 10 seconds")

        stopped = abs(sample["linear_speed"]) <= 0.01 and abs(sample["angular_speed"]) <= 0.02
        at_goal = sample["agv_status"] == 8 and distance <= 0.05 and yaw_error <= math.radians(5) and stopped
        if at_goal:
            if self.arrival_since is None:
                self.arrival_since = now
            self.arrival_count += 1
        else:
            self.arrival_since = None
            self.arrival_count = 0
        return {
            "distance_error_m": distance, "yaw_error_deg": math.degrees(yaw_error),
            "arrived": self.arrival_count >= 3 and now - self.arrival_since >= 1.0 if at_goal else False,
        }


def pause_and_confirm(slam, last_stamp: int, timeout_s: float = 5.0, feedback=None) -> dict:
    result = {"stop_confirmed": False, "feedback": []}
    try:
        response = slam.pause_walk(True)
        result["pause_response"] = str(response)
        result["pause_acknowledged"] = getattr(response, "success", None) is True
        deadline = time.monotonic() + timeout_s
        stopped_count, stopped_since = 0, None
        while time.monotonic() < deadline:
            state, stamp = (feedback or slam).get_chassis_position()
            stamp = int(stamp)
            if stamp > last_stamp:
                last_stamp = stamp
                speed = state.get("linear_speed")
                turn = state.get("angular_speed")
                stopped = (isinstance(speed, (float, int)) and math.isfinite(speed) and abs(speed) <= 0.01
                           and isinstance(turn, (float, int)) and math.isfinite(turn) and abs(turn) <= 0.02)
                result["feedback"].append({"timestamp_ns": stamp, "linear_speed": speed, "angular_speed": turn})
                if stopped:
                    stopped_count += 1
                    if stopped_since is None:
                        stopped_since = time.monotonic()
                    if stopped_count >= 3 and time.monotonic() - stopped_since >= 1.0:
                        result["stop_confirmed"] = True
                        break
                else:
                    stopped_count, stopped_since = 0, None
            time.sleep(0.2)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def enable_chassis_collision_avoidance(slam, config: dict, report: dict) -> None:
    level = config.get("agv_collision_level")
    reject_if(type(level) is not int or level not in (1, 2, 3),
              "agv_collision_level must be set to 1, 2, or 3")
    level_response = slam.set_agv_col_level(level)
    reject_if(getattr(level_response, "error_code", None) != 0,
              f"setting chassis collision level failed/unknown: {level_response}")
    report["chassis_collision_avoidance"] = {
        "level": level,
        "level_response": str(level_response),
        "enabled": False,
    }
    enabled_response = slam.set_agv_col_ctrl(True)
    reject_if(getattr(enabled_response, "success", None) is not True,
              f"enabling chassis collision avoidance failed/unknown: {enabled_response}")
    report["chassis_collision_avoidance"].update(
        enabled=True, enable_response=str(enabled_response)
    )


def execute_navigation(slam, source, arm, destination, config, initial, report, timeout_s=180.0):
    carry_pose = arm.current_end_pose()
    watchdog = NavigationWatchdog(destination, config["map_id"], initial, time.monotonic())
    request = report["planned_request"]
    enable_chassis_collision_avoidance(slam, config, report)
    report["request_attempted"], report["read_only"] = True, False
    response = slam.navigate_to_pose(request["x_mm"], request["y_mm"], request["yaw_deg"])
    report["navigation_response"] = str(response)
    reject_if(getattr(response, "error_code", None) != 0, f"navigation request rejected/unknown: {response}")
    report["request_accepted"] = True
    deadline = time.monotonic() + timeout_s
    last_progress_time = 0.0
    while time.monotonic() < deadline:
        state = arm.read_ready_state()
        arm.assert_execution_ready(state)
        reject_if(not 5.0 <= state.gripper_state[1] <= 90.0, "right gripper opened during navigation")
        pose = arm.current_end_pose()
        arm_error = math.dist(pose.position_m, carry_pose.position_m)
        rotation_error = math.degrees((Rotation.from_euler("xyz", carry_pose.rpy_rad).inv()
                                      * Rotation.from_euler("xyz", pose.rpy_rad)).magnitude())
        reject_if(arm_error > 0.02 or rotation_error > 7.0, "carry pose changed")
        position, stamp = source.get_chassis_position()
        current = {**position, "timestamp_ns": int(stamp),
                   "arm_position_error_mm": arm_error * 1000.0,
                   "arm_rotation_error_deg": rotation_error,
                   "right_gripper_feedback": state.gripper_state[1]}
        report.setdefault("navigation_feedback", []).append(current)
        progress = watchdog.update(position, int(stamp), time.monotonic())
        if progress is not None and time.monotonic() - last_progress_time >= 5.0:
            print(json.dumps({"progress": "NAVIGATING", "status": position["agv_status"],
                              **progress, "linear_speed_m_s": position["linear_speed"],
                              "angular_speed_rad_s": position["angular_speed"],
                              "arm_position_error_mm": arm_error * 1000.0}), flush=True)
            last_progress_time = time.monotonic()
        if progress is not None and progress["arrived"]:
            report["arrival"] = progress
            report["status"] = "ARRIVED"
            break
        time.sleep(0.2)
    else:
        raise RuntimeError("navigation timeout")

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", choices=["A", "B"], required=True)
    parser.add_argument("--preview", action="store_true", help="offline request conversion; no SDK or network")
    parser.add_argument("--dispatch", action="store_true", help="real motion, disabled after the incident")
    parser.add_argument("--carry-clearance-verified", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    args = parser.parse_args(argv)
    config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8-sig"))
    destination = config["stations"][args.station]
    now = datetime.now(timezone.utc)
    report = {
        "started_at_utc": now.isoformat(), "station": args.station,
        "dispatch_requested": args.dispatch, "request_attempted": False, "request_accepted": False,
        "read_only": True, "config": config, "runtime_map_id_verified": False,
        "arrival_scope": "fresh map pose, heading, Idle and near-zero speed; SDK task fields are not authoritative",
    }
    code, slam, watchdog = 0, None, None
    try:
        reject_if(args.preview and args.dispatch, "--preview and --dispatch are mutually exclusive")
        reject_if(not math.isfinite(args.timeout_s) or args.timeout_s <= 0, "invalid navigation timeout")
        report["planned_request"] = navigation_request(destination)
        if args.preview:
            report["status"] = "OFFLINE_REQUEST_PREVIEW"
        else:
            reject_if(args.dispatch and config.get("enable_real_navigation") is not True,
                      "real navigation remains disabled after the collision")
            from a2d_sdk.robot import Slam
            slam = Slam()
            maps = slam.get_map_list()
            reject_if(int(maps.error_code) != 0 or config["map_id"] not in maps.map_id,
                      "configured map is not available")
            samples = collect_samples(slam)
            hmi, controller = read_hmi(config), read_controller(config)
            report.update(initial_samples=samples, hmi=hmi, controller=controller)
            report["runtime_map"] = runtime_map_check(samples, config["map_id"])
            report["runtime_map_id_verified"] = report["runtime_map"]["verified"]
            report["initial_summary_vs_A"] = summarize(samples, config["stations"]["A"])
            if not args.dispatch:
                report["status"] = "INSPECTION_RECORDED"
            else:
                validate_preflight(samples, hmi, controller, config)
                reject_if(not args.carry_clearance_verified, "cup/arm route clearance is unverified")
                arm_config = load_config(PROJECT_ROOT / "config" / "grasp_v2.yaml")["robot"]
                arm = GdkRightArm(arm_config)
                state = arm.read_ready_state()
                arm.assert_execution_ready(state)
                reject_if(not 5.0 <= state.gripper_state[1] <= 90.0, "right gripper is no longer closed")
                execute_navigation(slam, slam, arm, destination, config, samples[-1], report, args.timeout_s)
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "ERROR"
        report["error"] = f"{type(exc).__name__}: {exc}"
        code = 1
        if report["request_attempted"] and slam is not None:
            report["stop"] = pause_and_confirm(slam, watchdog.last_stamp if watchdog is not None else 0)
            if not report["stop"]["stop_confirmed"]:
                report["status"] = "STOP_UNCONFIRMED"
    directory = PROJECT_ROOT / "outputs" / "navigation_check"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{now:%Y%m%d_%H%M%S_%f}_{args.station}_navigation.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())


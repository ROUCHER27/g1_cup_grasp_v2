#!/usr/bin/env python3
"""Inspect a saved stop or preview/execute the verified G1 navigation contract."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys
import time

import yaml
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from check_navigation import collect_samples, read_controller, read_hmi, runtime_map_check, summarize
from g1_cup_grasp.config import load_config
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.robot import GdkRightArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait

# Verified against this G1 HAL binary; Position feedback uses different units.
HAL_SHA256 = "1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309"
MIN_CONFIDENCE = 50.0
CHASSIS_INFO_KEYS = ("max_linear_speed", "max_angular_speed", "min_collision_distance")
DEFAULT_NAVIGATION = {
    "no_progress_timeout_s": 25.0,
    "timeout_base_s": 60.0,
    "timeout_per_metre_s": 60.0,
    "timeout_max_s": 300.0,
    "avoid_timeout_s": 30.0,
    "avoid_total_timeout_s": 60.0,
    "avoid_grace_s": 5.0,
    "avoid_max_episodes": 5,
    "avoid_max_departure_m": 1.0,
    "avoid_total_departure_m": 2.0,
    "treat_halted_walking_as_avoid": False,
    "detour": {
        "enabled": False,
        "lateral_offsets_m": [0.5, 0.7],
        "ahead_m": 0.6,
        "robot_radius_m": 0.35,
        "via_tolerance_m": 0.15,
        "occupied_threshold": 50,
    },
}
_DETOUR_KEYS = frozenset(DEFAULT_NAVIGATION["detour"])


def _finite_number(value) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError):
        return False


def _positive_finite(value) -> bool:
    return _finite_number(value) and value > 0


def navigation_settings(config: dict) -> dict:
    reject_if(not isinstance(config, dict), "transport config must be a mapping")
    supplied = config.get("navigation", {})
    reject_if(not isinstance(supplied, dict), "navigation must be a mapping")
    unknown = set(supplied) - set(DEFAULT_NAVIGATION)
    reject_if(unknown, f"unknown navigation keys: {sorted(unknown)}")

    settings = copy.deepcopy(DEFAULT_NAVIGATION)
    for key, value in supplied.items():
        if key == "detour":
            reject_if(not isinstance(value, dict), "navigation.detour must be a mapping")
            unknown_detour = set(value) - _DETOUR_KEYS
            reject_if(unknown_detour, f"unknown navigation.detour keys: {sorted(unknown_detour)}")
            settings["detour"].update(copy.deepcopy(value))
        else:
            settings[key] = value

    reject_if(type(settings["treat_halted_walking_as_avoid"]) is not bool,
              "treat_halted_walking_as_avoid must be bool")
    for key, value in settings.items():
        if key in ("detour", "treat_halted_walking_as_avoid"):
            continue
        reject_if(not _positive_finite(value), f"navigation.{key} must be finite and positive")
    reject_if(type(settings["avoid_max_episodes"]) is not int,
              "navigation.avoid_max_episodes must be an integer")
    reject_if(settings["avoid_timeout_s"] > settings["avoid_total_timeout_s"],
              "avoid_timeout_s must not exceed avoid_total_timeout_s")

    detour = settings["detour"]
    reject_if(type(detour["enabled"]) is not bool, "navigation.detour.enabled must be bool")
    reject_if(detour["enabled"], "navigation.detour is not implemented yet")
    offsets = detour["lateral_offsets_m"]
    reject_if(not isinstance(offsets, (list, tuple)) or not offsets,
              "navigation.detour.lateral_offsets_m must not be empty")
    reject_if(any(not _positive_finite(value) for value in offsets),
              "navigation.detour.lateral_offsets_m values must be finite and positive")
    for key in ("ahead_m", "robot_radius_m", "via_tolerance_m", "occupied_threshold"):
        reject_if(not _positive_finite(detour[key]), f"navigation.detour.{key} must be finite and positive")
    reject_if(detour["occupied_threshold"] > 100,
              "navigation.detour.occupied_threshold must be between 1 and 100")
    settings["detour"]["lateral_offsets_m"] = list(offsets)
    return settings


def leg_timeout_s(settings: dict, distance_m: float) -> float:
    reject_if(type(distance_m) not in (int, float) or not _finite_number(distance_m) or distance_m < 0,
              "navigation distance must be finite and non-negative")
    return min(settings["timeout_max_s"],
               settings["timeout_base_s"] + settings["timeout_per_metre_s"] * distance_m
               + settings["avoid_total_timeout_s"])


class NavigationAvoidTimeout(RuntimeError):
    pass


def _finite_chassis_info(value: dict) -> dict:
    reject_if(not isinstance(value, dict) or set(value) != set(CHASSIS_INFO_KEYS),
              "chassis_info must contain exactly max_linear_speed, max_angular_speed, and min_collision_distance")
    reject_if(any(not _finite_number(value[key]) for key in CHASSIS_INFO_KEYS),
              "chassis_info values must be finite numbers")
    return {key: float(value[key]) for key in CHASSIS_INFO_KEYS}


def _validated_chassis_info(value: dict) -> dict:
    values = _finite_chassis_info(value)
    reject_if(any(values[key] <= 0 for key in CHASSIS_INFO_KEYS),
              "chassis_info values must be finite and positive")
    return values


def validate_transport_config(config: dict) -> None:
    reject_if(not isinstance(config, dict), "transport config must be a mapping")
    map_id = config.get("map_id")
    reject_if(type(map_id) is not int or map_id <= 0, "map_id must be a positive integer")
    stations = config.get("stations")
    reject_if(not isinstance(stations, dict), "stations must contain A and B")
    for name in ("A", "B"):
        target = stations.get(name)
        reject_if(not isinstance(target, dict) or any(
            not _finite_number(target.get(key))
            for key in ("x_m", "y_m", "yaw_rad")
        ), f"stations.{name} must have finite x_m, y_m, and yaw_rad")
    level = config.get("agv_collision_level")
    reject_if(type(level) is not int or level not in (1, 2, 3),
              "agv_collision_level must be set to 1, 2, or 3")
    navigation_settings(config)
    if "chassis_info" in config:
        _validated_chassis_info(config["chassis_info"])
    place = config.get("place")
    if place is not None:
        base_pose = place.get("base_pose") if isinstance(place, dict) else None
        reject_if(not isinstance(base_pose, dict), "place.base_pose must include a map_id")
        reject_if(base_pose.get("map_id") != map_id,
                  "place base_pose was taught on another map; re-teach with tools/teach_place_pose.py")


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
    def __init__(self, destination: dict, map_id: int, initial: dict, now: float,
                 settings=None, arrival_tolerance_m=0.05,
                 arrival_tolerance_rad=math.radians(5)):
        self.destination = destination
        self.map_id = map_id
        self.settings = (copy.deepcopy(DEFAULT_NAVIGATION) if settings is None else
                         navigation_settings({"navigation": settings}))
        self.arrival_tolerance_m = arrival_tolerance_m
        self.arrival_tolerance_rad = arrival_tolerance_rad
        self.last_stamp = int(initial["timestamp_ns"])
        self.last_feedback_time = now
        self.last_update_time = now
        self.last_progress_time = now
        self.best_distance = math.inf
        self.best_yaw_error = math.inf
        self.progress_distance = math.inf
        self.progress_yaw_error = math.inf
        self.arrival_since = None
        self.arrival_count = 0
        self.avoid_since = None
        self.avoid_total_s = 0.0
        self.avoid_episodes = []
        self.avoid_grace_until = -math.inf
        self.rebaseline_pending = False
        self.rebaseline_count = 0

    def _pose(self, sample: dict) -> dict:
        return {"x": sample["agv_pos_x"], "y": sample["agv_pos_y"],
                "yaw": sample["agv_angle"]}

    def _start_avoid_episode(self, sample: dict, stamp: int, distance: float, now: float,
                             halted_walking: bool) -> None:
        maximum = self.settings["avoid_max_episodes"]
        if len(self.avoid_episodes) >= maximum:
            raise NavigationAvoidTimeout(f"chassis avoidance started more than {maximum} times")
        episode = {
            "kind": "halted_walking" if halted_walking else "agv_status_5",
            "start_ns": stamp, "end_ns": None, "duration_s": None,
            "start_pose": self._pose(sample), "end_pose": None,
            "start_distance_m": distance, "end_distance_m": None,
            "min_distance_m": distance, "max_distance_m": distance,
            "min_confidence": sample["position_conf"], "samples": 0,
        }
        self.avoid_episodes.append(episode)
        self.avoid_since = now
        self.rebaseline_pending = True

    def _close_avoid_episode(self, sample: dict, stamp: int, distance: float, now: float) -> None:
        episode = self.avoid_episodes[-1]
        duration = max(0.0, now - self.avoid_since)
        episode.update(end_ns=stamp, duration_s=duration, end_pose=self._pose(sample),
                       end_distance_m=distance)
        self.avoid_total_s += duration
        self.avoid_since = None
        self.avoid_grace_until = now + self.settings["avoid_grace_s"]

    def update(self, sample: dict, stamp: int, now: float) -> dict | None:
        reject_if(stamp < self.last_stamp, "chassis timestamp regressed")
        if stamp == self.last_stamp:
            reject_if(now - self.last_feedback_time > 2.0, "chassis feedback stale for over 2 seconds")
            return None
        self.last_stamp = stamp
        self.last_feedback_time = now
        self.last_update_time = now
        keys = ("agv_pos_x", "agv_pos_y", "agv_angle", "position_conf", "linear_speed", "angular_speed")
        reject_if(not all(isinstance(sample.get(key), (int, float)) and
                         math.isfinite(sample[key]) for key in keys), "invalid chassis feedback")
        reject_if(sample.get("map_id") != self.map_id, "runtime map ID changed or disappeared")
        reject_if(sample.get("chassis_error") not in (None, 0), "chassis reports an error")

        status = sample.get("agv_status")
        halted = abs(sample["linear_speed"]) <= 0.01 and abs(sample["angular_speed"]) <= 0.02
        halted_walking = (status == 7 and halted and
                          self.settings["treat_halted_walking_as_avoid"])
        avoiding = status == 5 or halted_walking
        if not avoiding:
            reject_if(status not in (7, 8), f"chassis protection/state {status}")
        confidence = sample["position_conf"]
        reject_if(confidence < MIN_CONFIDENCE, "localization confidence is below 50")

        distance = math.hypot(sample["agv_pos_x"] - self.destination["x_m"],
                              sample["agv_pos_y"] - self.destination["y_m"])
        delta = sample["agv_angle"] - self.destination["yaw_rad"]
        yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))

        if avoiding:
            if self.avoid_since is None:
                self._start_avoid_episode(sample, stamp, distance, now, halted_walking)
            episode = self.avoid_episodes[-1]
            episode["samples"] += 1
            episode["min_distance_m"] = min(episode["min_distance_m"], distance)
            episode["max_distance_m"] = max(episode["max_distance_m"], distance)
            episode["min_confidence"] = min(episode["min_confidence"], confidence)
            elapsed = max(0.0, now - self.avoid_since)
            if elapsed > self.settings["avoid_timeout_s"]:
                raise NavigationAvoidTimeout(
                    f"chassis avoidance exceeded {self.settings['avoid_timeout_s']:g} s")
            if self.avoid_total_s + elapsed > self.settings["avoid_total_timeout_s"]:
                raise NavigationAvoidTimeout(
                    f"cumulative chassis avoidance exceeded {self.settings['avoid_total_timeout_s']:g} s")
        else:
            if self.avoid_since is not None:
                self._close_avoid_episode(sample, stamp, distance, now)

        suspended = avoiding or now < self.avoid_grace_until
        if suspended:
            episode = self.avoid_episodes[-1]
            episode["max_distance_m"] = max(episode["max_distance_m"], distance)
            departure_limit = min(self.best_distance, episode["start_distance_m"])
            if distance > departure_limit + self.settings["avoid_max_departure_m"]:
                raise NavigationAvoidTimeout("chassis avoidance moved too far from the route")
            total_departure = sum(
                episode["max_distance_m"] - episode["start_distance_m"]
                for episode in self.avoid_episodes
            )
            if total_departure > self.settings["avoid_total_departure_m"]:
                raise NavigationAvoidTimeout("cumulative chassis avoidance departure exceeded limit")
        if not suspended:
            if self.rebaseline_pending:
                self.rebaseline_count += 1
                self.best_distance = self.progress_distance = distance
                self.best_yaw_error = self.progress_yaw_error = yaw_error
                self.last_progress_time = now
                self.rebaseline_pending = False
            self.best_distance = min(self.best_distance, distance)
            self.best_yaw_error = min(self.best_yaw_error, yaw_error)
            reject_if(self.best_distance <= 0.25 and distance > self.best_distance + 0.10,
                      "robot moved away after approaching the target")
            if (self.progress_distance - distance >= 0.02 or
                    self.progress_yaw_error - yaw_error >= math.radians(5)):
                self.last_progress_time = now
                self.progress_distance = min(self.progress_distance, distance)
                self.progress_yaw_error = min(self.progress_yaw_error, yaw_error)
            timeout = self.settings["no_progress_timeout_s"]
            reject_if(now - self.last_progress_time > timeout,
                      f"navigation made no verified progress for {timeout:g} seconds")

        stopped = halted
        at_goal = (status == 8 and distance <= self.arrival_tolerance_m and
                   yaw_error <= self.arrival_tolerance_rad and stopped)
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
            "avoiding": avoiding,
            "avoid_elapsed_s": max(0.0, now - self.avoid_since) if self.avoid_since is not None else 0.0,
            "avoid_total_s": self.avoid_total_s + (max(0.0, now - self.avoid_since)
                                                   if self.avoid_since is not None else 0.0),
        }

    def avoidance_summary(self) -> dict:
        episodes = copy.deepcopy(self.avoid_episodes)
        open_episode = None
        current_elapsed = 0.0
        if self.avoid_since is not None:
            current_elapsed = max(0.0, self.last_update_time - self.avoid_since)
            open_episode = copy.deepcopy(episodes[-1])
            open_episode["duration_s"] = current_elapsed
        durations = [episode["duration_s"] for episode in episodes if episode["duration_s"] is not None]
        if open_episode is not None:
            durations.append(current_elapsed)
        settings = {key: value for key, value in self.settings.items()
                    if key.startswith("avoid_") or key == "no_progress_timeout_s"}
        return {
            "episodes": episodes,
            "episode_count": len(episodes),
            "total_avoid_s": self.avoid_total_s + current_elapsed,
            "longest_episode_s": max(durations, default=0.0),
            "open_episode": open_episode,
            "rebaseline_count": self.rebaseline_count,
            "settings": settings,
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


def read_chassis_info(slam) -> dict:
    try:
        info, stamp = slam.get_chassis_info()
        if isinstance(info, dict):
            fields = {key: info.get(key) for key in CHASSIS_INFO_KEYS}
            raw_info = dict(info)
        elif isinstance(info, (list, tuple)) and len(info) == 3:
            fields = dict(zip(CHASSIS_INFO_KEYS, info))
            raw_info = list(info)
        else:
            fields = {key: getattr(info, key) for key in CHASSIS_INFO_KEYS}
            raw_info = vars(info).copy() if hasattr(info, "__dict__") else str(info)
        numbers = [fields[key] for key in CHASSIS_INFO_KEYS] + [stamp]
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in numbers):
            raise ValueError("chassis info values or timestamp are not finite numbers")
        values = {key: float(fields[key]) for key in CHASSIS_INFO_KEYS}
        return {"available": True, "values": values, "timestamp_ns": int(stamp),
                "raw": {"info": raw_info, "timestamp": stamp}}
    except Exception as exc:
        return {"available": False, "error": f"{type(exc).__name__}: {exc}"}


def apply_chassis_info(slam, requested: dict, before: dict, entry: dict, timeout_s=3.0) -> None:
    reject_if(not before.get("available"),
              "chassis_info read-back unavailable; refusing a blind set_chassis_info")
    requested_values = _validated_chassis_info(requested)
    previous_values = _finite_chassis_info(before.get("values"))
    reject_if(not _positive_finite(timeout_s), "chassis_info read-back timeout must be positive")
    slam.set_chassis_info(*(requested_values[key] for key in CHASSIS_INFO_KEYS))
    deadline = time.monotonic() + timeout_s
    while True:
        after = read_chassis_info(slam)
        matches = (after["available"] and after["timestamp_ns"] > before["timestamp_ns"] and
                   all(math.isclose(after["values"][key], requested_values[key],
                                    rel_tol=1e-6, abs_tol=1e-6)
                       for key in CHASSIS_INFO_KEYS))
        if matches:
            entry.update(applied=True, after=after)
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.2, remaining))
    try:
        slam.set_chassis_info(*(previous_values[key] for key in CHASSIS_INFO_KEYS))
        entry.update(applied=False, rollback=read_chassis_info(slam))
    except Exception as exc:
        entry.update(applied=False,
                     rollback_error=f"{type(exc).__name__}: {exc}")
    reject_if(True, "chassis_info read-back did not confirm the requested values; previous values republished")


def enable_chassis_collision_avoidance(slam, config: dict, report: dict) -> None:
    """Set collision sensitivity and request avoidance.

    HAL treats set_agv_col_ctrl(True) as a no-op, so the retained call is harmless.
    Never call set_agv_col_ctrl(False): HAL clears the avoidance region for about two seconds.
    """
    level = config.get("agv_collision_level")
    reject_if(type(level) is not int or level not in (1, 2, 3),
              "agv_collision_level must be set to 1, 2, or 3")
    entry = report["chassis_collision_avoidance"] = {
        "level": level,
        "enabled": False,
        "chassis_info_before": read_chassis_info(slam),
        "chassis_info_requested": None,
        "applied": None,
    }
    level_response = slam.set_agv_col_level(level)
    reject_if(getattr(level_response, "error_code", None) != 0,
              f"setting chassis collision level failed/unknown: {level_response}")
    entry["level_response"] = str(level_response)
    enabled_response = slam.set_agv_col_ctrl(True)
    reject_if(getattr(enabled_response, "success", None) is not True,
              f"enabling chassis collision avoidance failed/unknown: {enabled_response}")
    entry.update(enabled=True, enable_response=str(enabled_response))
    if "chassis_info" in config:
        requested = _validated_chassis_info(config["chassis_info"])
        entry["chassis_info_requested"] = requested
        apply_chassis_info(slam, requested, entry["chassis_info_before"], entry)
    entry["chassis_info_after"] = read_chassis_info(slam)


def execute_navigation(slam, source, arm, destination, config, initial, report, timeout_s=None,
                       *, request=None, leg="final", arrival_tolerance=None, no_cup=False):
    settings = navigation_settings(config)
    distance = math.hypot(initial["agv_pos_x"] - destination["x_m"],
                          initial["agv_pos_y"] - destination["y_m"])
    if timeout_s is None:
        timeout_s = leg_timeout_s(settings, distance)
    reject_if(not _positive_finite(timeout_s),
              "invalid navigation timeout")
    report["navigation_settings"] = settings
    report["navigation_timeout_s"] = float(timeout_s)
    report["no_cup"] = bool(no_cup)
    request = report["planned_request"] if request is None else request
    tolerance_m, tolerance_rad = arrival_tolerance or (0.05, math.radians(5))
    watchdog = NavigationWatchdog(destination, config["map_id"], initial, time.monotonic(), settings,
                                  tolerance_m, tolerance_rad)
    try:
        carry_pose = None if no_cup else arm.current_end_pose()
        enable_chassis_collision_avoidance(slam, config, report)
        report["request_attempted"], report["read_only"] = True, False
        response = slam.navigate_to_pose(request["x_mm"], request["y_mm"], request["yaw_deg"])
        report["navigation_response"] = str(response)
        reject_if(getattr(response, "error_code", None) != 0,
                  f"navigation request rejected/unknown: {response}")
        report["request_accepted"] = True
        deadline = time.monotonic() + timeout_s
        last_progress_time = 0.0
        while time.monotonic() < deadline:
            state = arm.read_ready_state()
            arm.assert_execution_ready(state)
            if no_cup:
                arm_error = rotation_error = None
            else:
                reject_if(not 5.0 <= state.gripper_state[1] <= 90.0,
                          "right gripper opened during navigation")
                pose = arm.current_end_pose()
                arm_error = math.dist(pose.position_m, carry_pose.position_m)
                rotation_error = math.degrees((Rotation.from_euler("xyz", carry_pose.rpy_rad).inv()
                                              * Rotation.from_euler("xyz", pose.rpy_rad)).magnitude())
                reject_if(arm_error > 0.02 or rotation_error > 7.0, "carry pose changed")
            position, stamp = source.get_chassis_position()
            current = {**position, "timestamp_ns": int(stamp), "leg": leg,
                       "arm_position_error_mm": None if arm_error is None else arm_error * 1000.0,
                       "arm_rotation_error_deg": rotation_error,
                       "right_gripper_feedback": state.gripper_state[1]}
            report.setdefault("navigation_feedback", []).append(current)
            progress = watchdog.update(position, int(stamp), time.monotonic())
            if progress is not None and time.monotonic() - last_progress_time >= 5.0:
                print(json.dumps({"progress": "NAVIGATING", "status": position["agv_status"],
                                  "leg": leg, "avoiding": progress["avoiding"],
                                  "avoid_elapsed_s": progress["avoid_elapsed_s"],
                                  **progress, "linear_speed_m_s": position["linear_speed"],
                                  "angular_speed_rad_s": position["angular_speed"],
                                  "arm_position_error_mm": None if arm_error is None else arm_error * 1000.0}),
                      flush=True)
                last_progress_time = time.monotonic()
            if progress is not None and progress["arrived"]:
                report["arrival"] = progress
                report["status"] = "ARRIVED"
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("navigation timeout")
    finally:
        try:
            report["avoidance"] = watchdog.avoidance_summary()
        except Exception as exc:
            report["avoidance"] = {"error": f"{type(exc).__name__}: {exc}"}

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", choices=["A", "B"], required=True)
    parser.add_argument("--preview", action="store_true", help="offline request conversion; no SDK or network")
    parser.add_argument("--dispatch", action="store_true", help="real motion, disabled after the incident")
    parser.add_argument("--native-feedback", action="store_true",
                        help="use native chassis feedback with map IDs")
    parser.add_argument("--no-cup", action="store_true",
                        help="skip only cup gripper-range and carry-pose drift checks")
    parser.add_argument("--label", help="safe report filename suffix, such as obstacle")
    parser.add_argument("--carry-clearance-verified", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=None)
    args = parser.parse_args(argv)
    config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8-sig"))
    now = datetime.now(timezone.utc)
    report = {
        "started_at_utc": now.isoformat(), "station": args.station,
        "dispatch_requested": args.dispatch, "request_attempted": False, "request_accepted": False,
        "native_feedback_requested": args.native_feedback, "no_cup": args.no_cup,
        "read_only": True, "config": config,
        "runtime_map_id_verified": False,
        "arrival_scope": "fresh map pose, heading, Idle and near-zero speed; SDK task fields are not authoritative",
    }
    code, slam, native, label_suffix = 0, None, None, ""
    try:
        validate_transport_config(config)
        reject_if(args.preview and args.dispatch, "--preview and --dispatch are mutually exclusive")
        if args.label is not None:
            reject_if(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,39}", args.label) is None,
                      "--label must be 1-40 letters, digits, underscores, or hyphens")
            label_suffix = f"_{args.label}"
        reject_if(args.dispatch and not args.native_feedback,
                  "--dispatch requires --native-feedback")
        reject_if(args.timeout_s is not None and
                  (not _positive_finite(args.timeout_s)),
                  "invalid navigation timeout")
        destination = config["stations"][args.station]
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
            source = slam
            if args.native_feedback:
                native = NativeChassisFeedback(config["native_feedback"])
                source = native
            samples = collect_samples(source)
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
                reject_if(not args.no_cup and not 5.0 <= state.gripper_state[1] <= 90.0,
                          "right gripper is no longer closed")
                execute_navigation(slam, source, arm, destination, config, samples[-1], report,
                                   args.timeout_s, no_cup=args.no_cup)
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "ERROR"
        report["error"] = f"{type(exc).__name__}: {exc}"
        code = 1
        if report["request_attempted"] and slam is not None:
            last_stamp = report.get("navigation_feedback", [{}])[-1].get("timestamp_ns", 0)
            report["stop"] = pause_and_confirm(slam, last_stamp, feedback=native)
            if not report["stop"]["stop_confirmed"]:
                report["status"] = "STOP_UNCONFIRMED"
    directory = PROJECT_ROOT / "outputs" / "navigation_check"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{now:%Y%m%d_%H%M%S_%f}_{args.station}_navigation{label_suffix}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    if native is not None:
        native.close()
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

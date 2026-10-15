"""Offline geometric and braking-distance checks for near-table docking."""

from __future__ import annotations

import math
from typing import Any


_LASER_SENSOR_X_M = 0.223
_LASER_REGION1_NEAR_X_M = 0.050
_LASER_REGION1_FAR_X_M = 0.221
_CHASSIS_FRONT_X_M = 0.300
_VISUAL_REGION6_FRONT_EXTENT_M = 0.440


class DockingClearanceError(ValueError):
    """Raised when a supplied clearance input is malformed or non-finite."""


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise DockingClearanceError(f"{path} must be a mapping")
    return value


def _number(
    value: Any,
    path: str,
    *,
    allow_none: bool = False,
    minimum: float | None = None,
    strict_minimum: bool = False,
) -> float | None:
    if value is None and allow_none:
        return None
    if type(value) not in (int, float):
        raise DockingClearanceError(f"{path} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise DockingClearanceError(f"{path} must be a finite number") from None
    if not math.isfinite(result):
        raise DockingClearanceError(f"{path} must be a finite number")
    if minimum is not None:
        failed = result <= minimum if strict_minimum else result < minimum
        if failed:
            comparator = "greater than" if strict_minimum else "at least"
            raise DockingClearanceError(f"{path} must be {comparator} {minimum}")
    return result


def _finite_result(value: float, path: str) -> float:
    if not math.isfinite(value):
        raise DockingClearanceError(f"derived {path} must be finite")
    return value


def _front_stop_regions() -> dict[str, dict[str, float | str]]:
    laser_front_boundary = _finite_result(
        _LASER_SENSOR_X_M + _LASER_REGION1_FAR_X_M,
        "laser region 1 front boundary",
    )
    return {
        "laser_region1": {
            "sensor_x_from_body_center_m": _LASER_SENSOR_X_M,
            "near_x_from_sensor_m": _LASER_REGION1_NEAR_X_M,
            "far_x_from_sensor_m": _LASER_REGION1_FAR_X_M,
            "front_boundary_from_body_center_m": laser_front_boundary,
            "front_extent_beyond_chassis_m": _finite_result(
                laser_front_boundary - _CHASSIS_FRONT_X_M,
                "laser region 1 front extent",
            ),
            "binding_evidence": "laser_region1 is bound to the current front stop",
            "reference": "sensor x=0.223 m; region x=0.050..0.221 m; chassis front x=0.300 m",
        },
        "visual_region6": {
            "front_extent_beyond_chassis_m": _VISUAL_REGION6_FRONT_EXTENT_M,
            "binding_evidence": "included in forward avoidance regions [1, 6]; stationary trigger source is unconfirmed",
            "reference": "known forward visual region 6 extent beyond chassis front",
        },
    }


def _required_inputs(config: dict[str, Any]) -> tuple[
    dict[str, float | None],
    dict[str, dict[str, float | None]],
    dict[str, float],
    list[str] | None,
    str | None,
]:
    motion_config = _mapping(config.get("motion"), "motion")
    expected_motion = {
        "speed_m_s",
        "reaction_delay_s",
        "minimum_braking_deceleration_m_s2",
        "error_margin_m",
    }
    if set(motion_config) != expected_motion:
        raise DockingClearanceError(
            "motion must contain exactly " + ", ".join(sorted(expected_motion))
        )
    motion = {
        "speed_m_s": _number(
            motion_config["speed_m_s"], "motion.speed_m_s", allow_none=True,
            minimum=0.0, strict_minimum=True,
        ),
        "reaction_delay_s": _number(
            motion_config["reaction_delay_s"], "motion.reaction_delay_s", allow_none=True,
            minimum=0.0, strict_minimum=True,
        ),
        "minimum_braking_deceleration_m_s2": _number(
            motion_config["minimum_braking_deceleration_m_s2"],
            "motion.minimum_braking_deceleration_m_s2",
            allow_none=True,
            minimum=0.0,
            strict_minimum=True,
        ),
        "error_margin_m": _number(
            motion_config["error_margin_m"], "motion.error_margin_m", allow_none=True,
            minimum=0.0, strict_minimum=True,
        ),
    }

    stations_config = _mapping(config.get("stations"), "stations")
    if set(stations_config) != {"A", "B"}:
        raise DockingClearanceError("stations must contain exactly A and B")
    station_measurements: dict[str, dict[str, float | None]] = {}
    for name in ("A", "B"):
        station = _mapping(stations_config[name], f"stations.{name}")
        expected_station = {
            "approach_start_clearance_to_nearest_support_m",
            "intended_stop_gap_to_nearest_support_m",
            "table_edge_gap_at_intended_stop_m",
        }
        if set(station) != expected_station:
            raise DockingClearanceError(
                f"stations.{name} must contain exactly "
                + ", ".join(sorted(expected_station))
            )
        station_measurements[name] = {
            "approach_start_clearance_to_nearest_support_m": _number(
                station["approach_start_clearance_to_nearest_support_m"],
                f"stations.{name}.approach_start_clearance_to_nearest_support_m",
                allow_none=True,
                minimum=0.0,
                strict_minimum=True,
            ),
            "intended_stop_gap_to_nearest_support_m": _number(
                station["intended_stop_gap_to_nearest_support_m"],
                f"stations.{name}.intended_stop_gap_to_nearest_support_m",
                allow_none=True,
                minimum=0.0,
                strict_minimum=True,
            ),
            "table_edge_gap_at_intended_stop_m": _number(
                station["table_edge_gap_at_intended_stop_m"],
                f"stations.{name}.table_edge_gap_at_intended_stop_m",
                allow_none=True,
                minimum=0.0,
                strict_minimum=True,
            ),
        }
    current_config = _mapping(config.get("current_pose_observation"), "current_pose_observation")
    if set(current_config) != {
        "table_edge_to_chassis_front_gap_m",
        "two_table_legs_to_chassis_front_gap_m",
    }:
        raise DockingClearanceError(
            "current_pose_observation must contain exactly the table-edge and two-leg gaps"
        )
    current_observation = {
        "table_edge_to_chassis_front_gap_m": _number(
            current_config["table_edge_to_chassis_front_gap_m"],
            "current_pose_observation.table_edge_to_chassis_front_gap_m",
            minimum=0.0,
            strict_minimum=True,
        ),
        "two_table_legs_to_chassis_front_gap_m": _number(
            current_config["two_table_legs_to_chassis_front_gap_m"],
            "current_pose_observation.two_table_legs_to_chassis_front_gap_m",
            minimum=0.0,
            strict_minimum=True,
        ),
    }
    trigger_config = _mapping(
        config.get("stationary_trigger_evidence"), "stationary_trigger_evidence"
    )
    if set(trigger_config) != {"stationary_trigger_regions", "evidence_note"}:
        raise DockingClearanceError(
            "stationary_trigger_evidence must contain exactly regions and evidence_note"
        )
    trigger_regions = trigger_config["stationary_trigger_regions"]
    evidence_note = trigger_config["evidence_note"]
    if trigger_regions is not None:
        if type(trigger_regions) is not list or not trigger_regions:
            raise DockingClearanceError(
                "stationary_trigger_regions must be null or a non-empty list"
            )
        if any(type(region) is not str for region in trigger_regions):
            raise DockingClearanceError("stationary_trigger_regions must contain region names")
        if len(set(trigger_regions)) != len(trigger_regions):
            raise DockingClearanceError("stationary_trigger_regions contains duplicates")
        if set(trigger_regions) - {"laser_region1", "visual_region6"}:
            raise DockingClearanceError("stationary_trigger_regions contains an unknown region")
        if "laser_region1" not in trigger_regions:
            raise DockingClearanceError("stationary trigger evidence must include laser_region1")
        if type(evidence_note) is not str or not evidence_note.strip():
            raise DockingClearanceError(
                "stationary_trigger_evidence.evidence_note is required when sources are confirmed"
            )
    elif evidence_note is not None:
        raise DockingClearanceError(
            "stationary_trigger_evidence.evidence_note must be null while sources are unknown"
        )
    return motion, station_measurements, current_observation, trigger_regions, evidence_note


def _missing_inputs(
    motion: dict[str, float | None],
    stations: dict[str, dict[str, float | None]],
    stationary_trigger_regions: list[str] | None,
) -> list[str]:
    missing = [f"motion.{key}" for key, value in motion.items() if value is None]
    for name, values in stations.items():
        for key, value in values.items():
            if value is None:
                missing.append(f"stations.{name}.{key}")
    if stationary_trigger_regions is None:
        missing.append("stationary_trigger_evidence.stationary_trigger_regions")
        missing.append("stationary_trigger_evidence.evidence_note")
    elif "visual_region6" in stationary_trigger_regions:
        for name, values in stations.items():
            if values["table_edge_gap_at_intended_stop_m"] is None:
                missing.append(f"stations.{name}.table_edge_gap_at_intended_stop_m")
    return missing


def _station_result(
    name: str,
    measurements: dict[str, float | None],
    motion: dict[str, float | None],
    static_regions: dict[str, dict[str, Any]],
    stationary_trigger_regions: list[str] | None,
) -> dict[str, Any]:
    start_gap = measurements["approach_start_clearance_to_nearest_support_m"]
    stop_gap = measurements["intended_stop_gap_to_nearest_support_m"]
    table_edge_gap = measurements["table_edge_gap_at_intended_stop_m"]
    margin = motion["error_margin_m"]
    speed = motion["speed_m_s"]
    reaction = motion["reaction_delay_s"]
    deceleration = motion["minimum_braking_deceleration_m_s2"]

    laser1_required_gap = None
    laser1_no_trigger = None
    if stop_gap is not None and margin is not None:
        laser1_required_gap = _finite_result(
            static_regions["laser_region1"]["front_extent_beyond_chassis_m"] + margin,
            f"stations.{name} laser region 1 no-trigger gap",
        )
        # A structure at the polygon boundary may trigger; require strict clearance.
        laser1_no_trigger = stop_gap > laser1_required_gap

    visual6_required_gap = None
    visual6_no_trigger = None
    if table_edge_gap is not None and margin is not None:
        visual6_required_gap = _finite_result(
            static_regions["visual_region6"]["front_extent_beyond_chassis_m"] + margin,
            f"stations.{name} visual region 6 projected gap",
        )
        visual6_no_trigger = table_edge_gap > visual6_required_gap
    visual6_overlap = None
    if table_edge_gap is not None:
        visual6_overlap = max(
            0.0,
            static_regions["visual_region6"]["front_extent_beyond_chassis_m"]
            - table_edge_gap,
        )
    forward_region6_may_overlap = (
        None
        if table_edge_gap is None or margin is None
        else table_edge_gap <= visual6_required_gap
    )

    available_distance = None
    if start_gap is not None and stop_gap is not None:
        available_distance = _finite_result(
            start_gap - stop_gap, f"stations.{name} approach run-up"
        )

    required_distance = None
    passed_braking = None
    if all(value is not None for value in (speed, reaction, deceleration, margin)):
        required_distance = _finite_result(
            speed * reaction + speed**2 / (2 * deceleration) + margin,
            f"stations.{name} braking distance",
        )
    if available_distance is not None and required_distance is not None:
        # Equality passes exactly: the configured margin is already in the formula.
        passed_braking = available_distance >= required_distance

    missing = []
    if start_gap is None:
        missing.append(f"stations.{name}.approach_start_clearance_to_nearest_support_m")
    if stop_gap is None:
        missing.append(f"stations.{name}.intended_stop_gap_to_nearest_support_m")
    if table_edge_gap is None:
        missing.append(f"stations.{name}.table_edge_gap_at_intended_stop_m")
    for key, value in motion.items():
        if value is None:
            missing.append(f"motion.{key}")
    if stationary_trigger_regions is None:
        missing.append("stationary_trigger_evidence.stationary_trigger_regions")
        missing.append("stationary_trigger_evidence.evidence_note")

    confirmed_static_checks: list[bool | None] = []
    if stationary_trigger_regions is not None:
        if "laser_region1" in stationary_trigger_regions:
            confirmed_static_checks.append(laser1_no_trigger)
        if "visual_region6" in stationary_trigger_regions:
            confirmed_static_checks.append(visual6_no_trigger)
    static_checks_passed = (
        None
        if not confirmed_static_checks or any(value is None for value in confirmed_static_checks)
        else all(value is True for value in confirmed_static_checks)
    )
    known_braking_and_laser1_failed = (
        passed_braking is False or laser1_no_trigger is False
    )
    unresolved_visual_projection = forward_region6_may_overlap is True
    if known_braking_and_laser1_failed:
        passed = False
    elif missing or static_checks_passed is None or unresolved_visual_projection:
        passed = None
    else:
        passed = static_checks_passed and passed_braking is True
    if missing:
        status = "BLOCKED_MISSING_MEASUREMENTS"
    elif known_braking_and_laser1_failed:
        status = "GEOMETRIC_CONDITIONS_FAIL"
    elif unresolved_visual_projection:
        status = "BLOCKED_VISUAL_HEIGHT_OR_TRIGGER_UNVERIFIED"
    elif static_checks_passed is None:
        status = "BLOCKED_STOP_TRIGGER_SOURCE_UNCONFIRMED"
    elif passed:
        status = "GEOMETRIC_CONDITIONS_PASS"
    else:
        status = "GEOMETRIC_CONDITIONS_FAIL"

    return {
        "status": status,
        "missing_inputs": missing,
        "approach_start_clearance_to_nearest_support_m": start_gap,
        "intended_stop_gap_to_nearest_support_m": stop_gap,
        "table_edge_gap_at_intended_stop_m": table_edge_gap,
        "available_braking_runup_m": available_distance,
        "laser1_no_trigger_required_gap_m": laser1_required_gap,
        "laser_region1_no_trigger": laser1_no_trigger,
        "visual_region6_required_table_edge_gap_m": visual6_required_gap,
        "visual_region6_no_trigger": (
            visual6_no_trigger
            if stationary_trigger_regions is not None
            and "visual_region6" in stationary_trigger_regions
            else None
        ),
        "visual_region6_projected_overlap_m": visual6_overlap,
        "forward_region6_may_overlap": forward_region6_may_overlap,
        "no_static_stop_zone_intersection": static_checks_passed,
        "stationary_trigger_regions_evaluated": stationary_trigger_regions,
        "braking": {
            "formula": "speed_m_s * reaction_delay_s + speed_m_s^2 / (2 * minimum_braking_deceleration_m_s2) + error_margin_m",
            "speed_m_s": speed,
            "reaction_delay_s": reaction,
            "minimum_braking_deceleration_m_s2": deceleration,
            "error_margin_m": margin,
            "available_distance_m": available_distance,
            "required_distance_m": required_distance,
            "passed": passed_braking,
        },
        "geometry_conditions_passed": passed,
        "laser1_and_braking_conditions_passed": (
            None
            if laser1_no_trigger is None or passed_braking is None
            else laser1_no_trigger and passed_braking
        ),
        "physical_safety_verified": False,
        "manipulation_reach_verified": False,
        "arm_lateral_and_height_envelope_verified": False,
    }


def evaluate_docking_clearance(config: dict[str, Any]) -> dict[str, Any]:
    """Evaluate only the stated near-table geometry and stopping-distance model.

    The result never authorizes robot motion. Unknown measurements remain null
    and make the station/global assessment blocked.
    """
    config = _mapping(config, "config")
    expected_config_keys = {
        "motion",
        "stations",
        "current_pose_observation",
        "stationary_trigger_evidence",
    }
    if set(config) != expected_config_keys:
        raise DockingClearanceError(
            "config must contain exactly " + ", ".join(sorted(expected_config_keys))
        )
    motion, stations, current_observation, stationary_regions, trigger_evidence_note = (
        _required_inputs(config)
    )
    static_regions = _front_stop_regions()
    station_results = {
        name: _station_result(name, stations[name], motion, static_regions, stationary_regions)
        for name in ("A", "B")
    }
    missing = _missing_inputs(motion, stations, stationary_regions)
    known_failures = any(
        result["laser_region1_no_trigger"] is False
        or result["braking"]["passed"] is False
        or (
            stationary_regions is not None
            and "visual_region6" in stationary_regions
            and result["visual_region6_no_trigger"] is False
        )
        for result in station_results.values()
    )
    visual_projection_overlap = any(
        result["forward_region6_may_overlap"] is True
        for result in station_results.values()
    )
    if missing:
        status = "BLOCKED_MISSING_MEASUREMENTS"
        geometry_passed = None
    elif known_failures:
        status = "GEOMETRIC_CONDITIONS_FAIL"
        geometry_passed = False
    elif visual_projection_overlap:
        status = "BLOCKED_VISUAL_HEIGHT_OR_TRIGGER_UNVERIFIED"
        geometry_passed = None
    elif stationary_regions is None:
        status = "BLOCKED_STOP_TRIGGER_SOURCE_UNCONFIRMED"
        geometry_passed = None
    else:
        geometry_passed = all(
            result["geometry_conditions_passed"] is True
            for result in station_results.values()
        )
        status = (
            "GEOMETRIC_CONDITIONS_PASS"
            if geometry_passed
            else "GEOMETRIC_CONDITIONS_FAIL"
        )

    blockers = []
    if missing:
        blockers.append({
            "code": "required_measurements_missing",
            "inputs": missing,
        })
    if stationary_regions is None:
        blockers.append({
            "code": "stationary_stop_trigger_source_unconfirmed",
            "known_binding": "laser_region1 is bound to front stop",
            "unresolved_binding": "forward avoidance also includes visual_region6; stationary trigger behavior is unconfirmed",
        })
    for name, result in station_results.items():
        if result["laser_region1_no_trigger"] is False:
            blockers.append({
                "code": "low_support_gap_intersects_laser_region1_projection",
                "station": name,
                "required_gap_m": result["laser1_no_trigger_required_gap_m"],
                "planned_gap_m": result["intended_stop_gap_to_nearest_support_m"],
            })
        if result["forward_region6_may_overlap"] is True:
            blockers.append({
                "code": "table_edge_in_visual_region6_forward_projection",
                "station": name,
                "overlap_m": result["visual_region6_projected_overlap_m"],
                "interpretation": "projection only; sensor height and stop-trigger semantics are unverified",
            })
        if (
            stationary_regions is not None
            and "visual_region6" in stationary_regions
            and result["visual_region6_no_trigger"] is False
        ):
            blockers.append({
                "code": "table_edge_gap_intersects_confirmed_visual_region6",
                "station": name,
                "required_gap_m": result["visual_region6_required_table_edge_gap_m"],
                "planned_gap_m": result["table_edge_gap_at_intended_stop_m"],
            })
        if result["braking"]["passed"] is False:
            blockers.append({
                "code": "approach_runup_below_braking_requirement",
                "station": name,
                "available_distance_m": result["braking"]["available_distance_m"],
                "required_distance_m": result["braking"]["required_distance_m"],
            })

    laser_extent = static_regions["laser_region1"]["front_extent_beyond_chassis_m"]
    visual_extent = static_regions["visual_region6"]["front_extent_beyond_chassis_m"]
    observed_legs_gap = current_observation["two_table_legs_to_chassis_front_gap_m"]
    observed_edge_gap = current_observation["table_edge_to_chassis_front_gap_m"]
    current_pose_note = {
        **current_observation,
        "source": "user-reported approximate measurement",
        "station_assignment": "current_pose_only_not_A_or_B",
        "laser1_low_support_x_projection_difference_m": _finite_result(
            observed_legs_gap - laser_extent,
            "current-pose low support versus laser1 projection",
        ),
        "visual6_table_edge_x_projected_overlap_m": _finite_result(
            max(0.0, visual_extent - observed_edge_gap),
            "current-pose table edge versus visual6 projection",
        ),
        "layer_interpretation": (
            "The 0.20 m table-edge gap is an arm/visual-height observation; the 0.40 m two-leg gap is a low chassis-level observation. They are separate obstacles and are not interchangeable clearances."
        ),
        "trigger_interpretation": (
            "The visual6 value is a forward x-projection only; height coverage and whether it causes a stationary stop are unconfirmed."
        ),
    }
    if current_pose_note["visual6_table_edge_x_projected_overlap_m"] > 0:
        blockers.append({
            "code": "current_pose_table_edge_in_visual_region6_forward_projection",
            "overlap_m": current_pose_note["visual6_table_edge_x_projected_overlap_m"],
            "station_assignment": "current_pose_only_not_A_or_B",
            "interpretation": "projection only; height and trigger behavior are unverified",
        })

    return {
        "schema_version": 1,
        "status": status,
        "geometry_conditions_passed": geometry_passed,
        "physical_safety_verified": False,
        "real_motion_permitted": False,
        "model": "offline_front_stop_zone_and_stopping_distance_check",
        "measurement_reference": (
            "For each station, approach-start clearance and intended stop gap refer to the same nearest low table leg/crossbar along the same straight approach. Table-edge gap is a separate higher-layer measurement."
        ),
        "comparison_rules": {
            "laser1_low_support_projection": "intended_stop_gap_to_nearest_support_m > laser_region1.front_extent_beyond_chassis_m + error_margin_m",
            "visual6_table_edge_projection": "table_edge_gap_at_intended_stop_m > visual_region6.front_extent_beyond_chassis_m + error_margin_m; a projected overlap is not proof of a sensor trigger",
            "braking_requirement": "approach_start_clearance_to_nearest_support_m - intended_stop_gap_to_nearest_support_m >= speed_m_s * reaction_delay_s + speed_m_s^2 / (2 * minimum_braking_deceleration_m_s2) + error_margin_m",
            "boundary_policy": "strict greater-than for region separation; exact braking-distance equality passes; no tolerance is added to either comparison",
        },
        "front_stop_regions": static_regions,
        "stationary_trigger_source_evidence_supplied": stationary_regions is not None,
        "stationary_trigger_regions": stationary_regions,
        "stationary_trigger_evidence_note": trigger_evidence_note,
        "current_pose_observation": current_pose_note,
        "motion_inputs": motion,
        "stations": station_results,
        "missing_inputs": missing,
        "blockers": blockers,
        "unverified_requirements": [
            "This calculation does not authorize motion or recommend clearing/overriding any front stop region.",
            "Right-arm/cup reach at the measured stopping gap has not been verified.",
            "Left/right arm lateral sweep, table legs/crossbars at arm heights, and sensor height coverage require separate measurement.",
            "Actual reaction delay and minimum braking deceleration must be measured under representative load and surface conditions.",
            "The static-zone check uses forward distance only; it does not prove full 2-D or 3-D collision clearance.",
        ],
    }

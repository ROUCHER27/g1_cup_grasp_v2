"""Create an offline-only proposal for the A-to-B tabletop docking sequence."""

from __future__ import annotations

import math
from typing import Any


class DockingPlanError(ValueError):
    """Raised when the offline plan inputs are malformed or inconsistent."""


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DockingPlanError(f"{name} must be a mapping")
    return value


def _number(value: Any, name: str, *, allow_none: bool = False) -> float | None:
    if value is None and allow_none:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DockingPlanError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise DockingPlanError(f"{name} must be a finite number") from None
    if not math.isfinite(result):
        raise DockingPlanError(f"{name} must be a finite number")
    return result


def _positive(value: Any, name: str, *, allow_none: bool = False) -> float | None:
    result = _number(value, name, allow_none=allow_none)
    if result is not None and result <= 0:
        raise DockingPlanError(f"{name} must be greater than zero")
    return result


def _pose(station: dict[str, Any], name: str) -> dict[str, float]:
    return {
        "x_m": _number(station.get("x_m"), f"stations.{name}.x_m"),
        "y_m": _number(station.get("y_m"), f"stations.{name}.y_m"),
        "yaw_rad": _number(station.get("yaw_rad"), f"stations.{name}.yaw_rad"),
    }


def _retreat_pose(
    station: dict[str, float], distance_m: float
) -> dict[str, float]:
    yaw = station["yaw_rad"]
    return {
        "x_m": station["x_m"] - distance_m * math.cos(yaw),
        "y_m": station["y_m"] - distance_m * math.sin(yaw),
        "yaw_rad": yaw,
    }


def _shortest_angle_deg(start_rad: float, end_rad: float) -> float:
    delta = end_rad - start_rad
    return math.degrees(math.atan2(math.sin(delta), math.cos(delta)))


def _blocker(code: str, message: str) -> dict[str, str]:
    return {"code": code, "message": message}


def _require_finite_tree(value: Any, name: str = "plan") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise DockingPlanError(f"derived value {name} must be finite")
    if isinstance(value, dict):
        for key, child in value.items():
            _require_finite_tree(child, f"{name}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_tree(child, f"{name}[{index}]")


def create_docking_plan(
    docking_config: dict[str, Any], transport_config: dict[str, Any]
) -> dict[str, Any]:
    """Return a proposal-only plan; this function has no robot or network I/O."""
    docking = _mapping(docking_config, "docking config")
    transport = _mapping(transport_config, "transport config")

    requested_map = _mapping(docking.get("map"), "map")
    transport_map_id = transport.get("map_id")
    requested_map_id = requested_map.get("id")
    if (
        isinstance(transport_map_id, bool)
        or not isinstance(transport_map_id, int)
        or transport_map_id <= 0
        or isinstance(requested_map_id, bool)
        or not isinstance(requested_map_id, int)
        or requested_map_id <= 0
    ):
        raise DockingPlanError("map id values must be positive integers")
    transport_map_name = transport.get("map_name")
    requested_map_name = requested_map.get("name")
    if (
        not isinstance(transport_map_name, str)
        or not transport_map_name.strip()
        or not isinstance(requested_map_name, str)
        or not requested_map_name.strip()
    ):
        raise DockingPlanError("map names must be non-empty strings")
    if (requested_map_id, requested_map_name) != (
        transport_map_id,
        transport_map_name,
    ):
        raise DockingPlanError("docking map must match transport map id and name")

    stations_config = _mapping(transport.get("stations"), "stations")
    station_a = _pose(_mapping(stations_config.get("A"), "stations.A"), "A")
    station_b = _pose(_mapping(stations_config.get("B"), "stations.B"), "B")

    distances = _mapping(docking.get("distances"), "distances")
    departure_distance = _positive(
        distances.get("a_departure_assumption_m"),
        "distances.a_departure_assumption_m",
    )
    approach_distance = _positive(
        distances.get("b_approach_assumption_m"),
        "distances.b_approach_assumption_m",
    )

    geometry_config = _mapping(
        docking.get("geometry_estimate"), "geometry_estimate"
    )
    chassis_length = _positive(
        geometry_config.get("chassis_length_m"),
        "geometry_estimate.chassis_length_m",
    )
    chassis_width = _positive(
        geometry_config.get("chassis_width_m"),
        "geometry_estimate.chassis_width_m",
    )
    side_extension = _positive(
        geometry_config.get("arm_side_extension_each_m"),
        "geometry_estimate.arm_side_extension_each_m",
    )
    front_extension = _positive(
        geometry_config.get("arm_front_extension_m"),
        "geometry_estimate.arm_front_extension_m",
    )
    rotation_margin = _positive(
        geometry_config.get("rotation_margin_m"),
        "geometry_estimate.rotation_margin_m",
    )
    straight_side_margin = _positive(
        geometry_config.get("straight_side_margin_m"),
        "geometry_estimate.straight_side_margin_m",
    )
    envelope_front = chassis_length / 2 + front_extension
    envelope_rear = chassis_length / 2
    envelope_half_width = chassis_width / 2 + side_extension
    envelope_width = 2 * envelope_half_width
    # Round away sub-picometer binary noise so the proposal's 1.45 m stays 1.45 m.
    derived_minimum_corridor_width = round(
        envelope_width + 2 * straight_side_margin, 12
    )
    _require_finite_tree(
        {"derived_minimum_corridor_width_m": derived_minimum_corridor_width}
    )
    required_rotation_diameter = 2 * (
        math.hypot(max(envelope_front, envelope_rear), envelope_half_width)
        + rotation_margin
    )

    clearance = _mapping(docking.get("clearance"), "clearance")
    minimum_corridor_width = _positive(
        clearance.get("minimum_straight_corridor_width_m"),
        "clearance.minimum_straight_corridor_width_m",
    )
    if minimum_corridor_width < derived_minimum_corridor_width:
        raise DockingPlanError(
            "clearance.minimum_straight_corridor_width_m must be at least the derived minimum "
            f"({derived_minimum_corridor_width:.6f} m from full envelope and side margins)"
        )
    measured_corridor_width = _positive(
        clearance.get("measured_corridor_width_m"),
        "clearance.measured_corridor_width_m",
        allow_none=True,
    )
    measured_rotation_diameter = _positive(
        clearance.get("measured_rotation_clear_diameter_m"),
        "clearance.measured_rotation_clear_diameter_m",
        allow_none=True,
    )

    station_dx = station_b["x_m"] - station_a["x_m"]
    station_dy = station_b["y_m"] - station_a["y_m"]
    station_distance = math.hypot(station_dx, station_dy)
    if not math.isfinite(station_distance) or station_distance <= 0:
        raise DockingPlanError("A and B station positions must be distinct")

    a_departure = _retreat_pose(station_a, departure_distance)
    b_approach = _retreat_pose(station_b, approach_distance)
    candidate_transfer_distance = math.hypot(
        b_approach["x_m"] - a_departure["x_m"],
        b_approach["y_m"] - a_departure["y_m"],
    )
    heading_change = _shortest_angle_deg(
        station_a["yaw_rad"], station_b["yaw_rad"]
    )

    axis_x = station_dx / station_distance
    axis_y = station_dy / station_distance
    a_departure_progress = (
        (a_departure["x_m"] - station_a["x_m"]) * axis_x
        + (a_departure["y_m"] - station_a["y_m"]) * axis_y
    )
    b_approach_progress = (
        (b_approach["x_m"] - station_a["x_m"]) * axis_x
        + (b_approach["y_m"] - station_a["y_m"]) * axis_y
    )

    blockers = [
        _blocker(
            "arm_and_cup_swept_envelope_unverified",
            "The fixed left arm and right arm/cup swept envelope has not been measured through departure, travel, turn, approach, and manipulation.",
        ),
        _blocker(
            "constrained_straight_motion_unavailable",
            "The current navigate_to_pose(x_mm, y_mm, yaw_deg) interface only specifies an endpoint; it does not guarantee straight reverse/forward motion or a verified stop.",
        ),
        _blocker(
            "stationary_motion_inhibition_unverified",
            "A verified chassis lock during the A pick and B placement has not been established.",
        ),
        _blocker(
            "three_dimensional_collision_scene_missing",
            "No complete 3-D collision model or measured table-leg, wall, frame, cable, sensor-height, and stopping-distance evidence is supplied.",
        ),
        _blocker(
            "route_not_collision_checked",
            "These candidate poses do not include a collision-checked route between A and B.",
        ),
    ]
    if measured_corridor_width is None:
        blockers.append(
            _blocker(
                "site_corridor_unmeasured",
                "A/B route corridor width has not been measured, so the required straight-path clearance cannot be checked.",
            )
        )
    if measured_rotation_diameter is None:
        blockers.append(
            _blocker(
                "site_rotation_clearance_unmeasured",
                "The available turning clearance has not been measured; the estimated 2-D rotation requirement is not a site check.",
            )
        )
    if a_departure_progress >= b_approach_progress:
        blockers.append(
            _blocker(
                "candidate_points_projected_order_inverted",
                "Along the A-to-B station axis, the A departure candidate has passed the B approach candidate (progress %.3f m vs %.3f m; point separation %.3f m). These assumed offsets leave no ordered transit segment to verify."
                % (
                    a_departure_progress,
                    b_approach_progress,
                    candidate_transfer_distance,
                ),
            )
        )
    if (
        measured_corridor_width is not None
        and measured_corridor_width < minimum_corridor_width
    ):
        blockers.append(
            _blocker(
                "measured_corridor_below_requirement",
                "The supplied measured corridor width is below the stated minimum requirement.",
            )
        )
    if (
        measured_rotation_diameter is not None
        and measured_rotation_diameter < required_rotation_diameter
    ):
        blockers.append(
            _blocker(
                "measured_rotation_clearance_below_requirement",
                "The supplied measured rotation clearance is below the conservative requirement.",
            )
        )
    measured_thresholds_met = (
        None
        if measured_corridor_width is None or measured_rotation_diameter is None
        else measured_corridor_width >= minimum_corridor_width
        and measured_rotation_diameter >= required_rotation_diameter
    )

    plan = {
        "schema_version": 1,
        "status": "OFFLINE_PLAN_NOT_EXECUTABLE",
        "execution": {
            "mode": "read_only",
            "proposal_only": True,
            "executable": False,
        },
        "map": {
            "id": requested_map_id,
            "name": requested_map_name,
            "matches_transport_config": True,
        },
        "arm_intent": {
            "left_arm": "fixed_during_transport",
            "right_arm": "pick_at_A_and_place_at_B",
        },
        "clearance": {
            "minimum_straight_corridor_width_m": minimum_corridor_width,
            "derived_minimum_corridor_width_m": derived_minimum_corridor_width,
            "straight_side_margin_each_m": straight_side_margin,
            "conservative_rotation_clear_diameter_m": required_rotation_diameter,
            "measured_corridor_width_m": measured_corridor_width,
            "measured_rotation_clear_diameter_m": measured_rotation_diameter,
            "measured_minimum_thresholds_met": measured_thresholds_met,
            "site_clearance_verified": False,
        },
        "geometry_estimate": {
            "chassis_length_m": chassis_length,
            "chassis_width_m": chassis_width,
            "arm_side_extension_each_m": side_extension,
            "arm_front_extension_m": front_extension,
            "rotation_margin_m": rotation_margin,
            "combined_envelope_front_m": envelope_front,
            "combined_envelope_rear_m": envelope_rear,
            "combined_envelope_half_width_m": envelope_half_width,
            "combined_envelope_width_m": envelope_width,
            "straight_side_margin_each_m": straight_side_margin,
            "source_status": "approximate_proposal_input_not_measured",
        },
        "station_geometry": {
            "a_to_b_distance_m": station_distance,
            "candidate_a_departure_to_b_approach_distance_m": candidate_transfer_distance,
            "a_departure_progress_along_a_to_b_m": a_departure_progress,
            "b_approach_progress_along_a_to_b_m": b_approach_progress,
            "candidate_points_projected_ordered": a_departure_progress < b_approach_progress,
        },
        "phases": [
            {
                "phase": "A_STATIONARY_PICK",
                "pose": station_a,
                "motion_requirement": "stationary_chassis_and_verified_right_arm_pick",
            },
            {
                "phase": "A_STRAIGHT_DEPARTURE_CANDIDATE",
                "candidate_pose": a_departure,
                "motion_requirement": "verified_low_speed_reverse_along_A_heading",
                "distance_m": departure_distance,
                "distance_basis": "assumption_only_not_measured",
            },
            {
                "phase": "ROTATE_AND_TRANSPORT_CANDIDATE",
                "from_pose": a_departure,
                "to_pose": b_approach,
                "candidate_distance_m": candidate_transfer_distance,
                "heading_change_deg": heading_change,
                "required_rotation_clear_diameter_m": required_rotation_diameter,
                "motion_requirement": "collision_checked_turn_and_transport_path",
            },
            {
                "phase": "B_STRAIGHT_APPROACH_CANDIDATE",
                "candidate_pose": b_approach,
                "motion_requirement": "verified_low_speed_forward_along_B_heading",
                "distance_m": approach_distance,
                "distance_basis": "assumption_only_not_measured",
            },
            {
                "phase": "B_STATIONARY_PLACE",
                "pose": station_b,
                "motion_requirement": "stationary_chassis_and_verified_right_arm_place",
            },
        ],
        "blockers": blockers,
    }
    _require_finite_tree(plan)
    return plan

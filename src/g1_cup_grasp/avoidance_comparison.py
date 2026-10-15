"""Offline comparison of a saved controller snapshot and a proposed profile.

Controller coordinates are treated as millimeters/degrees from the saved
snapshot. This module never contacts a controller or creates an approved
manifest; sensor geometry is not evidence of field-of-view or height coverage.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from .avoidance_profile import AvoidanceProfile, build_proposal, transform_polygon_to_sensor_frame


_TIERS = ("stop", "warn2", "warn1")
_TIER_TO_PROFILE_LEVEL = {"stop": 1, "warn2": 2, "warn1": 3}
_SENSOR_GROUPS = ("laserList", "cameraList", "ultrasonicList")
_SUPPORTED_SHAPES = {0, 2}
_POINT_TOLERANCE_MM = 1e-6


def _reject_json_constant(value: str):
    raise ValueError(f"snapshot contains non-finite JSON value {value}")


def _parse_finite_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("snapshot contains a non-finite JSON number")
    return result


def load_snapshot_report(path: str | Path) -> dict:
    """Read a saved report with ``report.snapshot``; never performs live I/O."""
    try:
        value = json.loads(
            Path(path).read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
            parse_float=_parse_finite_float,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"cannot read saved controller snapshot: {exc}") from exc
    if type(value) is not dict or type(value.get("snapshot")) is not dict:
        raise ValueError("snapshot report must contain an object at report.snapshot")
    return value


def _mapping(value: Any, name: str) -> dict:
    if type(value) is not dict:
        raise ValueError(f"{name} must be an object")
    return value


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_int(value: Any, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _finite_result(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"derived {name} is not finite")
    return value


def _clean_mm(value: float) -> float:
    return round(_finite_result(value, "coordinate"), 6)


def _parse_sensor_map(parameters: dict) -> dict[int, dict]:
    sensors_param = _mapping(parameters.get("sensorsParam"), "snapshot.parameters.sensorsParam")
    result: dict[int, dict] = {}
    for group in _SENSOR_GROUPS:
        raw_list = sensors_param.get(group)
        if type(raw_list) is not list:
            raise ValueError(f"snapshot.parameters.sensorsParam.{group} must be a list")
        for index, raw_sensor in enumerate(raw_list):
            sensor = _mapping(raw_sensor, f"sensorsParam.{group}[{index}]")
            sensor_id = _positive_int(sensor.get("id"), f"sensorsParam.{group}[{index}].id")
            if sensor_id in result:
                raise ValueError(f"sensor id {sensor_id} is duplicated")
            result[sensor_id] = {
                "id": sensor_id,
                "name": sensor.get("name", ""),
                "group": group,
                "x_mm": _finite_number(sensor.get("x"), f"sensor {sensor_id}.x"),
                "y_mm": _finite_number(sensor.get("y"), f"sensor {sensor_id}.y"),
                "z_raw": _finite_number(sensor.get("z"), f"sensor {sensor_id}.z"),
                "theta_deg": _finite_number(sensor.get("theta"), f"sensor {sensor_id}.theta"),
            }
    return result


def _parse_polygon(value: Any, name: str) -> list[tuple[float, float]]:
    if type(value) is not list or len(value) < 3:
        raise ValueError(f"{name} must contain at least three vertices")
    points = []
    for index, raw_point in enumerate(value):
        point = _mapping(raw_point, f"{name}[{index}]")
        points.append((
            _finite_number(point.get("x"), f"{name}[{index}].x"),
            _finite_number(point.get("y"), f"{name}[{index}].y"),
        ))
    if points[0] == points[-1]:
        points.pop()
    if len(set(points)) < 3:
        raise ValueError(f"{name} must contain at least three unique vertices")
    return points


def _signed_area2(polygon: Sequence[tuple[float, float]]) -> float:
    try:
        area = math.fsum(
            polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
            - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
            for index in range(len(polygon))
        )
    except (OverflowError, ValueError) as exc:
        raise ValueError("polygon area is not finite") from exc
    return _finite_result(area, "polygon area")


def _cross(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return _finite_result(
        (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]),
        "polygon cross product",
    )


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> int:
    value = _cross(a, b, c)
    if value > _POINT_TOLERANCE_MM:
        return 1
    if value < -_POINT_TOLERANCE_MM:
        return -1
    return 0


def _segments_intersect(
    a: tuple[float, float], b: tuple[float, float],
    c: tuple[float, float], d: tuple[float, float],
) -> bool:
    o1, o2 = _orientation(a, b, c), _orientation(a, b, d)
    o3, o4 = _orientation(c, d, a), _orientation(c, d, b)
    if o1 != o2 and o3 != o4:
        return True

    def on_segment(p, q, r):
        return (
            min(p[0], r[0]) - _POINT_TOLERANCE_MM <= q[0] <= max(p[0], r[0]) + _POINT_TOLERANCE_MM
            and min(p[1], r[1]) - _POINT_TOLERANCE_MM <= q[1] <= max(p[1], r[1]) + _POINT_TOLERANCE_MM
        )

    return (
        (o1 == 0 and on_segment(a, c, b))
        or (o2 == 0 and on_segment(a, d, b))
        or (o3 == 0 and on_segment(c, a, d))
        or (o4 == 0 and on_segment(c, b, d))
    )


def _is_simple_convex_polygon(polygon: Sequence[tuple[float, float]]) -> bool:
    count = len(polygon)
    if count < 3 or len(set(polygon)) < 3:
        return False
    try:
        if abs(_signed_area2(polygon)) <= _POINT_TOLERANCE_MM:
            return False
        signs = {
            _orientation(polygon[i], polygon[(i + 1) % count], polygon[(i + 2) % count])
            for i in range(count)
        } - {0}
        if len(signs) != 1:
            return False
        for first in range(count):
            a, b = polygon[first], polygon[(first + 1) % count]
            for second in range(first + 1, count):
                if second == first or second == (first + 1) % count or (second + 1) % count == first:
                    continue
                c, d = polygon[second], polygon[(second + 1) % count]
                if _segments_intersect(a, b, c, d):
                    return False
    except ValueError:
        return False
    return True


def _point_in_convex_polygon(
    point: tuple[float, float], polygon: Sequence[tuple[float, float]]
) -> bool:
    orientation = 1 if _signed_area2(polygon) > 0 else -1
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        cross = _cross(start, end, point)
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        tolerance = _POINT_TOLERANCE_MM * max(1.0, edge_length)
        if orientation * cross < -tolerance:
            return False
    return True


def _polygon_contains(
    container: Sequence[tuple[float, float]], target: Sequence[tuple[float, float]]
) -> bool:
    if not _is_simple_convex_polygon(container) or not _is_simple_convex_polygon(target):
        raise ValueError("polygon containment requires simple convex polygons")
    return all(_point_in_convex_polygon(point, container) for point in target)


def _polygon_bounds(polygon: Sequence[tuple[float, float]]) -> dict[str, float]:
    return {
        "x_min": _clean_mm(min(point[0] for point in polygon)),
        "x_max": _clean_mm(max(point[0] for point in polygon)),
        "y_min": _clean_mm(min(point[1] for point in polygon)),
        "y_max": _clean_mm(max(point[1] for point in polygon)),
    }


def _sensor_to_body(
    polygon: Sequence[tuple[float, float]], sensor: dict
) -> list[tuple[float, float]]:
    yaw_deg = sensor["theta_deg"] % 360.0
    yaw = math.radians(yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = []
    for x_sensor, y_sensor in polygon:
        x_body = _finite_result(
            sensor["x_mm"] + cosine * x_sensor - sine * y_sensor,
            "body-frame x coordinate",
        )
        y_body = _finite_result(
            sensor["y_mm"] + sine * x_sensor + cosine * y_sensor,
            "body-frame y coordinate",
        )
        result.append((x_body, y_body))
    return result


def _strategy_usage(parameters: dict, range_ids: set[int]) -> tuple[dict[int, list[dict]], list[int]]:
    strategy = parameters.get("avoidStrategyParam")
    if type(strategy) is not dict:
        return {range_id: [] for range_id in range_ids}, []
    usage = {range_id: [] for range_id in range_ids}
    missing: set[int] = set()
    for group in ("globalStrategy", "localStrtegy"):
        values = strategy.get(group, [])
        if type(values) is not list:
            continue
        for index, value in enumerate(values):
            if type(value) is not dict or type(value.get("rangeList")) is not list:
                continue
            for referenced in value["rangeList"]:
                if type(referenced) is not int or referenced <= 0:
                    continue
                if referenced not in range_ids:
                    missing.add(referenced)
                    continue
                usage[referenced].append({
                    "group": group,
                    "strategy_id": value.get("id"),
                    "strategy_name": value.get("name", ""),
                })
    return usage, sorted(missing)


def _region_role(region: dict, sensor: dict, strategy_usage: list[dict]) -> str:
    labels = [str(region.get("name", "")), str(sensor.get("name", ""))]
    labels.extend(str(item.get("strategy_name", "")) for item in strategy_usage)
    normalized = " ".join(labels).lower()
    if "旋" in normalized or "rotat" in normalized:
        return "UNSUPPORTED_ROTATION_SWEEP"
    yaw = math.radians(sensor["theta_deg"] % 360.0)
    forward_component = math.cos(yaw)
    if forward_component < -0.5 or "后" in normalized or "rear" in normalized or "backward" in normalized:
        return "NOT_DESIGNED_FOR_REAR_SENSOR"
    if forward_component >= 0.5:
        return "FORWARD_PROFILE_COMPARISON"
    return "UNSUPPORTED_SENSOR_ORIENTATION"


def _tier_ranges(region: dict, range_path: str) -> dict[str, dict]:
    result = {}
    for tier in _TIERS:
        tier_value = _mapping(region.get(tier), f"{range_path}.{tier}")
        polygon = _parse_polygon(tier_value.get("positions"), f"{range_path}.{tier}.positions")
        z_range = _mapping(tier_value.get("zRange"), f"{range_path}.{tier}.zRange")
        raw_z_min, raw_z_max = z_range.get("min"), z_range.get("max")
        z_min = _finite_number(raw_z_min, f"{range_path}.{tier}.zRange.min")
        z_max = _finite_number(raw_z_max, f"{range_path}.{tier}.zRange.max")
        if z_min > z_max:
            raise ValueError(f"{range_path}.{tier}.zRange.min must not exceed max")
        result[tier] = {"polygon": polygon, "z_range_raw": {"min": raw_z_min, "max": raw_z_max}}
    return result


def _check_containment(
    actual_polygon: Sequence[tuple[float, float]],
    target_polygon: Sequence[tuple[float, float]],
    *,
    applicable: bool,
    polygon_supported: bool,
) -> dict:
    base = {
        "actual_polygon_supported": polygon_supported,
        "actual_polygon_bounds_mm": _polygon_bounds(actual_polygon),
        "target_polygon_bounds_mm": _polygon_bounds(target_polygon),
        "target_polygon_body_frame_mm": [
            [_clean_mm(x), _clean_mm(y)] for x, y in target_polygon
        ],
    }
    if not applicable:
        return {**base, "status": "NOT_DESIGNED_FOR_THIS_REGION", "contained": None}
    if not polygon_supported:
        return {**base, "status": "UNSUPPORTED_NON_CONVEX_ACTUAL_POLYGON", "contained": None}
    return {
        **base,
        "status": "CHECKED_CONVEX_POLYGON_CONTAINMENT",
        "contained": _polygon_contains(actual_polygon, target_polygon),
        "method": "all target polygon vertices checked against actual convex polygon edges; no bbox containment",
    }


def _candidate_for_tier(
    target_polygon_body_mm: Sequence[tuple[float, float]],
    sensor: dict,
    current_polygon_sensor_mm: Sequence[tuple[float, float]],
    *,
    polygon_supported: bool,
    z_range: dict,
) -> dict:
    sensor_pose = {
        "x_m": sensor["x_mm"] / 1000.0,
        "y_m": sensor["y_mm"] / 1000.0,
        "yaw_rad": math.radians(sensor["theta_deg"] % 360.0),
    }
    candidate_sensor_m = transform_polygon_to_sensor_frame(
        [(x / 1000.0, y / 1000.0) for x, y in target_polygon_body_mm], sensor_pose
    )
    candidate_sensor = [(x * 1000.0, y * 1000.0) for x, y in candidate_sensor_m]
    candidate_body = list(target_polygon_body_mm)
    local_bounds = _polygon_bounds(candidate_sensor)
    body_bounds = _polygon_bounds(candidate_body)
    blockers = ["sensor_coverage_unverified", "height_zrange_unverified"]
    if local_bounds["x_min"] < 0:
        blockers.append("crosses_sensor_local_rear")
    if not polygon_supported:
        blockers.append("current_region_polygon_unsupported")

    candidate_contains_current = None
    current_contains_candidate = None
    if polygon_supported:
        candidate_contains_current = _polygon_contains(candidate_sensor, current_polygon_sensor_mm)
        current_contains_candidate = _polygon_contains(current_polygon_sensor_mm, candidate_sensor)
    current_bounds = _polygon_bounds(current_polygon_sensor_mm)
    delta = {
        key: _clean_mm(local_bounds[key] - current_bounds[key])
        for key in ("x_min", "x_max", "y_min", "y_max")
    }
    return {
        "status": "BLOCKED_NOT_FOR_APPLICATION" if blockers else "REVIEW_REQUIRED",
        "target_profile_level": None,
        "body_frame_polygon_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in candidate_body],
        "body_frame_bounds_mm": body_bounds,
        "sensor_local_polygon_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in candidate_sensor],
        "sensor_local_bounds_mm": local_bounds,
        "z_range_preserved_from_current": dict(z_range),
        "candidate_contains_current_polygon": candidate_contains_current,
        "current_contains_candidate_polygon": current_contains_candidate,
        "axis_aligned_bounds_delta_mm_descriptive_only": delta,
        "containment_method": "convex polygon edge test; bounds are descriptive only",
        "blocker_codes": blockers,
        "applied": False,
    }


def compare_snapshot_to_profile(
    snapshot_report: Mapping[str, Any], profile: AvoidanceProfile
) -> dict:
    """Compare a saved controller report with a proposal, without live I/O."""
    report = _mapping(snapshot_report, "snapshot report")
    snapshot = _mapping(report.get("snapshot"), "snapshot report.snapshot")
    device = _mapping(snapshot.get("device"), "snapshot.device")
    parameters = _mapping(snapshot.get("parameters"), "snapshot.parameters")
    serial = device.get("serialNumber")
    if not isinstance(serial, str) or not serial.strip():
        raise ValueError("snapshot.device.serialNumber must be non-empty text")
    robot_id = _positive_int(parameters.get("id"), "snapshot.parameters.id")
    map_id = _positive_int(parameters.get("mapId"), "snapshot.parameters.mapId")

    sensors = _parse_sensor_map(parameters)
    range_container = _mapping(parameters.get("avoidRangesParam"), "snapshot.parameters.avoidRangesParam")
    raw_regions = range_container.get("avoidRangeList")
    if type(raw_regions) is not list or not raw_regions:
        raise ValueError("snapshot.parameters.avoidRangesParam.avoidRangeList must be a non-empty list")
    regions_by_id = {}
    for index, raw_region in enumerate(raw_regions):
        region = _mapping(raw_region, f"avoidRangeList[{index}]")
        range_id = _positive_int(region.get("id"), f"avoidRangeList[{index}].id")
        if range_id in regions_by_id:
            raise ValueError(f"avoidance range id {range_id} is duplicated")
        sensor_id = _positive_int(region.get("sensorId"), f"avoidance range {range_id}.sensorId")
        parsed_tiers = _tier_ranges(region, f"avoidance range {range_id}")
        shape = region.get("shape")
        if type(shape) is not int:
            raise ValueError(f"avoidance range {range_id}.shape must be an integer")
        regions_by_id[range_id] = {
            "id": range_id,
            "name": str(region.get("name", "")),
            "sensor_id": sensor_id,
            "shape": shape,
            "tiers": parsed_tiers,
        }

    strategy_usage, missing_range_ids = _strategy_usage(parameters, set(regions_by_id))
    design = build_proposal(profile)
    body_polygon = [tuple(point * 1000.0 for point in pair)
                    for pair in design["body_envelope_polygon_body_frame_m"]]
    levels = {
        zone["level"]: [tuple(point * 1000.0 for point in pair)
                        for pair in zone["polygon_body_frame_m"]]
        for zone in design["travel_zones"]
    }

    profile_data = {
        "geometry_frame": design["geometry_frame"],
        "body_frame_envelope_mm": {
            key: _clean_mm(value * 1000.0)
            for key, value in design["body_frame_envelope_m"].items()
        },
        "full_body_envelope_polygon_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in body_polygon],
        "travel_zones": [
            {
                "level": level,
                "name": zone["name"],
                "polygon_body_frame_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in levels[level]],
                "bounds_mm": _polygon_bounds(levels[level]),
            }
            for zone in design["travel_zones"]
            for level in [zone["level"]]
        ],
    }

    parsed_sensors = []
    for sensor in sensors.values():
        theta_normalized = (sensor["theta_deg"] % 360.0 + 180.0) % 360.0 - 180.0
        parsed_sensors.append({
            "id": sensor["id"],
            "name": sensor["name"],
            "group": sensor["group"],
            "pose_controller_raw": {
                "x_mm": sensor["x_mm"], "y_mm": sensor["y_mm"],
                "z_raw": sensor["z_raw"], "theta_deg": sensor["theta_deg"],
            },
            "pose_normalized": {
                "x_mm": _clean_mm(sensor["x_mm"]),
                "y_mm": _clean_mm(sensor["y_mm"]),
                "yaw_deg": _clean_mm(theta_normalized),
            },
            "pose_reference": "sensor origin expressed in chassis/body frame; x/y mm, theta degrees",
        })

    global_blockers = [
        {
            "code": "saved_snapshot_not_currently_reverified",
            "message": "This is an offline comparison of a saved capture; no current controller read or freshness check is performed.",
        },
        {
            "code": "sensor_field_of_view_unverified",
            "message": "Polygon coordinates do not prove sensor field of view, range, occlusion, or coverage at arm/table heights.",
        },
        {
            "code": "zrange_height_semantics_unverified",
            "message": "Controller zRange values are retained verbatim; their reference and physical height coverage are unverified.",
        },
        {
            "code": "rearward_protection_not_designed",
            "message": "The supplied profile defines forward travel zones only; no rearward margin profile was provided.",
        },
        {
            "code": "rotation_swept_circle_not_modeled",
            "message": "The profile's conservative circular turn requirement cannot be represented by a straight rectangular region comparison.",
        },
        {
            "code": "runtime_strategy_selection_not_verified",
            "message": "Snapshot strategy references are shown for context; runtime strategy selection and activation are not verified.",
        },
    ]
    if report.get("read_only") is not True or report.get("robot_commands_sent") is not False:
        global_blockers.append({
            "code": "source_report_command_provenance_unverified",
            "message": "The source report does not affirm a read-only capture with no robot commands.",
        })
    if missing_range_ids:
        global_blockers.append({
            "code": "strategy_references_missing_region_ids",
            "message": f"Saved strategy lists reference absent region IDs: {missing_range_ids}.",
        })

    output_regions = []
    for range_id in sorted(regions_by_id):
        region = regions_by_id[range_id]
        sensor = sensors.get(region["sensor_id"])
        region_usage = strategy_usage[range_id]
        if sensor is None:
            output_regions.append({
                "id": range_id,
                "name": region["name"],
                "sensor_id": region["sensor_id"],
                "design_applicability": "UNSUPPORTED_SENSOR_POSE_MISSING",
                "candidate_generation": "PRESERVE_CURRENT_UNSUPPORTED",
                "strategy_usage": region_usage,
                "blocker_codes": ["sensor_pose_missing"],
                "tiers": {},
            })
            continue

        role = _region_role(region, sensor, region_usage)
        polygon_supported_by_shape = region["shape"] in _SUPPORTED_SHAPES
        region_blockers = []
        if role == "UNSUPPORTED_ROTATION_SWEEP":
            region_blockers.append("rotation_requires_swept_circle")
        elif role == "NOT_DESIGNED_FOR_REAR_SENSOR":
            region_blockers.append("forward_profile_not_applicable_to_rear_sensor")
        elif role != "FORWARD_PROFILE_COMPARISON":
            region_blockers.append("sensor_orientation_not_designed")
        if not polygon_supported_by_shape:
            region_blockers.append("unsupported_controller_shape_code")

        output_tiers = {}
        for tier in _TIERS:
            tier_value = region["tiers"][tier]
            sensor_polygon = tier_value["polygon"]
            body_frame_polygon = _sensor_to_body(sensor_polygon, sensor)
            convex = _is_simple_convex_polygon(body_frame_polygon)
            polygon_supported = polygon_supported_by_shape and convex
            z_range = tier_value["z_range_raw"]
            containment = {
                "full_body_envelope": _check_containment(
                    body_frame_polygon, body_polygon,
                    applicable=role == "FORWARD_PROFILE_COMPARISON",
                    polygon_supported=polygon_supported,
                ),
            }
            for level in (1, 2, 3):
                containment[f"travel_level_{level}"] = _check_containment(
                    body_frame_polygon, levels[level],
                    applicable=role == "FORWARD_PROFILE_COMPARISON",
                    polygon_supported=polygon_supported,
                )

            candidate = None
            if role == "FORWARD_PROFILE_COMPARISON" and polygon_supported:
                target_level = _TIER_TO_PROFILE_LEVEL[tier]
                candidate = _candidate_for_tier(
                    levels[target_level], sensor, sensor_polygon,
                    polygon_supported=polygon_supported, z_range=z_range,
                )
                candidate["target_profile_level"] = target_level
                if "crosses_sensor_local_rear" in candidate["blocker_codes"]:
                    region_blockers.append("profile_candidate_crosses_sensor_local_rear")
            else:
                containment_unavailable = any(
                    check["status"] == "UNSUPPORTED_NON_CONVEX_ACTUAL_POLYGON"
                    for check in containment.values()
                )
                if containment_unavailable and "non_convex_actual_polygon_unsupported" not in region_blockers:
                    region_blockers.append("non_convex_actual_polygon_unsupported")

            output_tiers[tier] = {
                "sensor_local_polygon_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in sensor_polygon],
                "body_frame_polygon_mm": [[_clean_mm(x), _clean_mm(y)] for x, y in body_frame_polygon],
                "body_frame_bounds_mm": _polygon_bounds(body_frame_polygon),
                "actual_polygon_supported_for_containment": polygon_supported,
                "z_range_raw": z_range,
                "height_coverage_verified": False,
                "containment_checks": containment,
                "suggested_candidate": candidate,
            }
        output_regions.append({
            "id": range_id,
            "name": region["name"],
            "sensor_id": region["sensor_id"],
            "sensor_name": sensor["name"],
            "sensor_group": sensor["group"],
            "sensor_pose_controller_raw": {
                "x_mm": sensor["x_mm"], "y_mm": sensor["y_mm"],
                "z_raw": sensor["z_raw"], "theta_deg": sensor["theta_deg"],
            },
            "shape": region["shape"],
            "design_applicability": role,
            "candidate_generation": (
                "FORWARD_PROFILE_CANDIDATES_BLOCKED" if role == "FORWARD_PROFILE_COMPARISON"
                else "PRESERVE_CURRENT_UNSUPPORTED"
            ),
            "strategy_usage": region_usage,
            "blocker_codes": sorted(set(region_blockers + ["zrange_height_coverage_unverified"])),
            "tiers": output_tiers,
        })

    return {
        "schema_version": 1,
        "status": "PROPOSAL_NOT_APPLIED",
        "proposal_only": True,
        "read_only": True,
        "controller_contacted": False,
        "controller_configuration_changed": False,
        "robot_commands_sent": False,
        "approved_manifest_created": False,
        "api_requests_made": False,
        "live_comparison": False,
        "measurements_verified": False,
        "sensor_coverage_verified": False,
        "physical_3d_collision_verified": False,
        "route_clearance_verified": False,
        "native_runtime_map_verified": False,
        "source_snapshot": {
            "serial_number": serial,
            "software_version": device.get("softwareVersion"),
            "firmware_version": device.get("firmwareVersion"),
            "map_id": map_id,
            "created_at_utc": report.get("created_at_utc"),
            "was_live_capture": report.get("live_read") is True,
            "current_freshness_verified": False,
            "read_only_capture": report.get("read_only") is True,
            "robot_commands_sent": report.get("robot_commands_sent"),
        },
        "units": {
            "controller_xy": "mm",
            "controller_theta": "degrees",
            "profile_geometry": "m converted to mm for comparison",
            "z_range": "controller-native raw values; reference/units unverified",
        },
        "body_frame": "chassis/body-centered XY; +x forward, +y left; no world/map coordinates",
        "sensor_transform": "body_xy_mm = sensor_origin_body_xy_mm + R(theta_deg) * sensor_local_xy_mm",
        "sensor_pose_calibration": "controller x/y/theta interpreted in chassis/body frame; physical mounting calibration is not independently verified",
        "candidate_tier_mapping_assumption": {
            "stop": 1,
            "warn2": 2,
            "warn1": 3,
            "note": "Comparison proposal only; this tier correspondence is not a controller approval.",
        },
        "profile_geometry": profile_data,
        "sensors": parsed_sensors,
        "regions": output_regions,
        "global_blockers": global_blockers,
        "limitations": [
            "The current region polygons are copied from a saved snapshot and are not re-read from the controller.",
            "Containment is checked on complete convex polygons, not bounding boxes.",
            "Non-convex or unsupported controller shapes are marked unsupported rather than approximated by a rectangle.",
            "A forward profile is not assigned to the rear-facing sensor, and rotation zones are not substituted with straight rectangles.",
            "Negative sensor-local x candidates are reported as blocked; they do not imply the sensor can see behind itself.",
            "No proposal is approved or applied and no controller write API is called.",
        ],
    }

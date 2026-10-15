"""Offline geometry for a proposal-only, three-level avoidance profile.

This module performs no robot, controller, network, or sensor I/O. Sensor-frame
polygons are geometric coordinate transforms and do not imply sensor coverage.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


Point2 = tuple[float, float]
Polygon2 = tuple[Point2, ...]


@dataclass(frozen=True)
class Body:
    length_m: float
    width_m: float
    front_extension_m: float
    side_extension_m: float


@dataclass(frozen=True)
class TravelZone:
    level: int
    name: str
    forward_m: float
    side_m: float


@dataclass(frozen=True)
class SensorPose:
    sensor_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class AvoidanceProfile:
    body: Body
    rotation_extra_m: float
    travel_zones: tuple[TravelZone, ...]
    sensors: tuple[SensorPose, ...]


def _mapping(value: object, name: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _finite_number(value: object, name: str, *, positive: bool = False,
                   nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    if positive and number <= 0:
        raise ValueError(f"{name} must be positive")
    if nonnegative and number < 0:
        raise ValueError(f"{name} must be nonnegative")
    return number


def _finite_result(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must remain finite")
    return value


def _checked_add(left: float, right: float, name: str) -> float:
    return _finite_result(left + right, name)


def _checked_multiply(left: float, right: float, name: str) -> float:
    return _finite_result(left * right, name)


def validate_profile(data: object) -> AvoidanceProfile:
    """Validate the small YAML schema and return an immutable geometry profile."""
    root = _mapping(data, "profile")
    version = root.get("schema_version")
    if isinstance(version, bool) or not isinstance(version, int) or version != 1:
        raise ValueError("schema_version must be 1")
    if root.get("status") != "PROPOSAL_NOT_APPLIED":
        raise ValueError("status must be PROPOSAL_NOT_APPLIED")
    if root.get("proposal_only") is not True or root.get("read_only") is not True:
        raise ValueError("profile must remain proposal_only and read_only")
    if root.get("measurements_verified", False) is not False:
        raise ValueError("measurements_verified must remain false for this proposal")
    if root.get("sensor_coverage_verified", False) is not False:
        raise ValueError("sensor_coverage_verified must remain false for this proposal")

    body_data = _mapping(root.get("body"), "body")
    extensions = _mapping(body_data.get("extensions_m"), "body.extensions_m")
    body = Body(
        length_m=_finite_number(body_data.get("length_m"), "body.length_m", positive=True),
        width_m=_finite_number(body_data.get("width_m"), "body.width_m", positive=True),
        front_extension_m=_finite_number(
            extensions.get("front"), "body.extensions_m.front", nonnegative=True
        ),
        side_extension_m=_finite_number(
            extensions.get("side"), "body.extensions_m.side", nonnegative=True
        ),
    )

    margins = _mapping(root.get("margins"), "margins")
    rotation_extra_m = _finite_number(
        margins.get("rotation_extra_m"), "margins.rotation_extra_m", positive=True
    )
    zones_data = margins.get("travel_zones")
    if not isinstance(zones_data, list) or len(zones_data) != 3:
        raise ValueError("margins.travel_zones must contain exactly three nested levels")
    zones: list[TravelZone] = []
    for index, raw_zone in enumerate(zones_data, 1):
        zone = _mapping(raw_zone, f"margins.travel_zones[{index - 1}]")
        level = zone.get("level")
        if isinstance(level, bool) or not isinstance(level, int) or level != index:
            raise ValueError("travel zone levels must be ordered 1, 2, 3")
        name = zone.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"travel zone {level} name must be a nonempty string")
        zones.append(TravelZone(
            level=level,
            name=name,
            forward_m=_finite_number(
                zone.get("forward_m"), f"travel zone {level} forward_m", positive=True
            ),
            side_m=_finite_number(
                zone.get("side_m"), f"travel zone {level} side_m", positive=True
            ),
        ))
    for inner, outer in zip(zones, zones[1:]):
        if outer.forward_m <= inner.forward_m or outer.side_m <= inner.side_m:
            raise ValueError("travel zones must be strictly nested in forward and side margins")

    sensors_data = root.get("sensors", [])
    if not isinstance(sensors_data, list):
        raise ValueError("sensors must be a list")
    sensors: list[SensorPose] = []
    sensor_ids: set[str] = set()
    for index, raw_sensor in enumerate(sensors_data):
        sensor = _mapping(raw_sensor, f"sensors[{index}]")
        sensor_id = sensor.get("id")
        if not isinstance(sensor_id, str) or not sensor_id.strip():
            raise ValueError(f"sensors[{index}].id must be a nonempty string")
        if sensor_id in sensor_ids:
            raise ValueError(f"sensor id {sensor_id!r} must be unique")
        sensor_ids.add(sensor_id)
        sensors.append(SensorPose(
            sensor_id=sensor_id,
            x_m=_finite_number(sensor.get("x_m"), f"sensor {sensor_id}.x_m"),
            y_m=_finite_number(sensor.get("y_m"), f"sensor {sensor_id}.y_m"),
            yaw_rad=_finite_number(sensor.get("yaw_rad"), f"sensor {sensor_id}.yaw_rad"),
        ))

    return AvoidanceProfile(
        body=body,
        rotation_extra_m=rotation_extra_m,
        travel_zones=tuple(zones),
        sensors=tuple(sensors),
    )


def load_profile(path: str | Path) -> AvoidanceProfile:
    """Read and validate a local YAML profile; no external I/O is performed."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return validate_profile(data)


def body_envelope(profile: AvoidanceProfile) -> tuple[float, float, float, float]:
    """Return ``(x_min, x_max, y_min, y_max)`` in the body-centered frame."""
    rear = _finite_result(profile.body.length_m / 2, "body rear extent")
    front = _checked_add(rear, profile.body.front_extension_m, "body front extent")
    half_width = _checked_add(
        profile.body.width_m / 2, profile.body.side_extension_m, "body half-width"
    )
    return -rear, front, -half_width, half_width


def rectangle_polygon(x_min: float, x_max: float, half_width: float) -> Polygon2:
    """Return a counterclockwise rectangle in the body frame."""
    return (
        (x_min, -half_width),
        (x_max, -half_width),
        (x_max, half_width),
        (x_min, half_width),
    )


def transform_polygon_to_sensor_frame(
    polygon_xy: Sequence[Sequence[float]], sensor_pose: Mapping[str, float]
) -> list[Point2]:
    """Transform body-frame polygon points into a sensor's local XY frame.

    ``sensor_pose`` is ``x_m``, ``y_m``, and ``yaw_rad`` expressed in the body frame;
    the inverse rigid transform is applied to each polygon point.
    """
    pose = _mapping(sensor_pose, "sensor_pose")
    sensor_x = _finite_number(pose.get("x_m"), "sensor_pose.x_m")
    sensor_y = _finite_number(pose.get("y_m"), "sensor_pose.y_m")
    yaw = _finite_number(pose.get("yaw_rad"), "sensor_pose.yaw_rad")
    c, s = math.cos(yaw), math.sin(yaw)
    if not isinstance(polygon_xy, Sequence) or isinstance(polygon_xy, (str, bytes)):
        raise ValueError("polygon_xy must be a sequence of points")
    if len(polygon_xy) < 3:
        raise ValueError("polygon_xy must contain at least three points")
    result: list[Point2] = []
    for index, point in enumerate(polygon_xy):
        if not isinstance(point, Sequence) or len(point) != 2:
            raise ValueError(f"polygon point {index} must have two coordinates")
        x = _finite_number(point[0], f"polygon point {index}.x")
        y = _finite_number(point[1], f"polygon point {index}.y")
        dx = _finite_result(x - sensor_x, f"polygon point {index} x offset")
        dy = _finite_result(y - sensor_y, f"polygon point {index} y offset")
        local_x = _finite_result(c * dx + s * dy, f"polygon point {index} sensor x")
        local_y = _finite_result(-s * dx + c * dy, f"polygon point {index} sensor y")
        result.append((local_x, local_y))
    return result


def _rounded(value: float) -> float:
    # Keep JSON easy to inspect while retaining sufficient precision for meters.
    return round(_finite_result(value, "reported geometry"), 9)


def build_proposal(profile: AvoidanceProfile) -> dict:
    """Build an offline geometry proposal with explicit unverified status."""
    x_min, x_max, y_min, y_max = body_envelope(profile)
    half_width = y_max
    envelope_polygon = rectangle_polygon(x_min, x_max, half_width)
    envelope_radius = _finite_result(
        max(math.hypot(x, y) for x, y in envelope_polygon), "rotation envelope radius"
    )
    conservative_radius = _checked_add(
        envelope_radius, profile.rotation_extra_m, "conservative rotation radius"
    )
    clear_diameter = _checked_multiply(
        2.0, conservative_radius, "conservative rotation diameter"
    )
    sensors = []
    for sensor in profile.sensors:
        pose = {"x_m": sensor.x_m, "y_m": sensor.y_m, "yaw_rad": sensor.yaw_rad}
        sensors.append({
            "id": sensor.sensor_id,
            "pose_body_frame": pose,
            "coverage_verified": False,
            "coverage_note": "Coordinate transform does not establish field of view, range, or height coverage.",
        })

    travel_zones = []
    for zone in profile.travel_zones:
        front_boundary = _checked_add(x_max, zone.forward_m, f"zone {zone.level} front boundary")
        zone_half_width = _checked_add(half_width, zone.side_m, f"zone {zone.level} half-width")
        polygon = rectangle_polygon(x_min, front_boundary, zone_half_width)
        zone_result = {
            "level": zone.level,
            "name": zone.name,
            "coordinates_frame": "body_frame_xy",
            "forward_margin_m": zone.forward_m,
            "side_margin_m": zone.side_m,
            "rear_boundary_m": _rounded(x_min),
            "rear_clearance_assumed_m": 0.0,
            "front_boundary_m": _rounded(front_boundary),
            "half_width_m": _rounded(zone_half_width),
            "reference": "full_robot_envelope",
            "polygon_body_frame_m": [[_rounded(x), _rounded(y)] for x, y in polygon],
            "sensor_polygons": [],
        }
        for sensor in profile.sensors:
            local_polygon = transform_polygon_to_sensor_frame(
                polygon,
                {"x_m": sensor.x_m, "y_m": sensor.y_m, "yaw_rad": sensor.yaw_rad},
            )
            zone_result["sensor_polygons"].append({
                "sensor_id": sensor.sensor_id,
                "coordinates_frame": "sensor_local_xy",
                "polygon_m": [[_rounded(x), _rounded(y)] for x, y in local_polygon],
                "coverage_verified": False,
            })
        travel_zones.append(zone_result)

    return {
        "schema_version": 1,
        "status": "PROPOSAL_NOT_APPLIED",
        "proposal_only": True,
        "read_only": True,
        "measurements_verified": False,
        "sensor_coverage_verified": False,
        "geometry_frame": "body_centered_xy_m; +x forward, +y left, yaw counterclockwise",
        "units": {"position": "m", "angle": "rad"},
        "sensor_pose_reference": "x_m/y_m in body frame; yaw_rad counterclockwise from body +x",
        "body_frame_envelope_m": {
            "x_min": _rounded(x_min),
            "x_max": _rounded(x_max),
            "y_min": _rounded(y_min),
            "y_max": _rounded(y_max),
        },
        "body_envelope_polygon_body_frame_m": [
            [_rounded(x), _rounded(y)] for x, y in envelope_polygon
        ],
        "rotation": {
            "envelope_radius_m": _rounded(envelope_radius),
            "extra_margin_m": profile.rotation_extra_m,
            "conservative_radius_m": _rounded(conservative_radius),
            "clear_diameter_m": _rounded(clear_diameter),
            "reference": "maximum body-centered corner distance plus configured rotation extra margin",
        },
        "travel_zones": travel_zones,
        "sensors": sensors,
        "limitations": [
            "Dimensions and proposed clearances are unverified estimates.",
            "The rear clearance was not specified; zones extend rearward only to the full robot envelope.",
            "Sensor-frame polygons are coordinate transforms, not claims of actual sensor coverage.",
            "Sensor field of view, range, occlusion, and height coverage are unverified.",
            "This proposal is not applied to a controller and emits no control request.",
        ],
    }

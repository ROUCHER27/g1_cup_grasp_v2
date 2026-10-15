"""Strict, read-only verification of the controller avoidance configuration."""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re
import threading
import time
from typing import Any


_KIND = "g1_controller_avoidance_manifest"
_SCHEMA_VERSION = 1
_APPROVED_STATUS = "approved_controller_snapshot"
_REVIEW_STATUS = "review_required"
_HAL_HASH_RE = re.compile(r"^[0-9a-f]{64}$")
_MANIFEST_KEYS = {
    "kind",
    "schema_version",
    "status",
    "robot_id",
    "agv_type_id",
    "map_id",
    "map_name",
    "hal_sha256",
    "expected",
    "provenance",
    "claims",
    "canonical_sha256",
}
_EXPECTED_KEYS = {"device", "parameters"}
_DEVICE_KEYS = {"serialNumber", "softwareVersion", "firmwareVersion"}
_PARAMETER_KEYS = {
    "id",
    "agvTypeId",
    "mapId",
    "chassisParam",
    "sensorsParam",
    "avoidRangesParam",
    "avoidStrategyParam",
    "scheduleParam",
    "moveControlParam",
}
_MONITORED_CLAIMS = {
    "configuration_manifest_match_only": True,
    "physical_3d_collision_verified": False,
    "sensor_coverage_verified": False,
    "route_clearance_verified": False,
    "straight_reverse_verified": False,
    "speed_policy_verified": False,
}
_SENSOR_LIST_KEYS = ("laserList", "cameraList", "ultrasonicList")
_SENSOR_KEYS = {
    "id",
    "brand",
    "mark",
    "name",
    "transferMode",
    "x",
    "y",
    "z",
    "theta",
    "direction",
    "frequency",
    "resolution",
    "scanAngleRange",
    "detectAngleRange",
    "offset",
    "param1",
    "param2",
}
_RANGE_KEYS = {
    "id",
    "mark",
    "name",
    "sensorId",
    "shape",
    "uuid",
    "stop",
    "warn1",
    "warn2",
}
_STRATEGY_KEYS = {"id", "name", "rangeList", "uuid"}
_MISSING = object()


def _fail(condition: bool, message: str) -> None:
    if condition:
        raise ValueError(message)


def _is_number(value: Any) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError):
        return False


def _validate_json_value(value: Any, path: str = "$") -> None:
    if value is None or type(value) in (str, bool, int):
        return
    if type(value) is float:
        _fail(not math.isfinite(value), f"{path} must contain only finite JSON numbers")
        return
    if type(value) is list:
        for index, item in enumerate(value):
            _validate_json_value(item, f"{path}[{index}]")
        return
    if type(value) is dict:
        for key, item in value.items():
            _fail(type(key) is not str, f"{path} object keys must be strings")
            _validate_json_value(item, f"{path}.{key}")
        return
    raise ValueError(f"{path} is not a JSON value")


def _canonical_bytes(value: Any) -> bytes:
    _validate_json_value(value)
    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"value cannot be represented as canonical JSON: {exc}") from exc


def canonical_digest(manifest: dict) -> str:
    """Hash canonical JSON for a manifest, excluding its self-digest field.

    This detects accidental edits; it does not establish who approved the file.
    """
    _fail(type(manifest) is not dict, "manifest must be a mapping")
    payload = copy.deepcopy(manifest)
    payload.pop("canonical_sha256", None)
    return hashlib.sha256(_canonical_bytes(payload)).hexdigest()


def _require_keys(value: Any, required: set[str], path: str) -> dict:
    _fail(type(value) is not dict, f"{path} must be a mapping")
    missing = required - set(value)
    _fail(missing, f"{path} is missing required fields: {sorted(missing)}")
    return value


def _require_exact_keys(value: Any, expected: set[str], path: str) -> dict:
    value = _require_keys(value, expected, path)
    extra = set(value) - expected
    _fail(extra, f"{path} has unknown fields: {sorted(extra)}")
    return value


def _require_int(value: Any, path: str, *, minimum: int | None = None) -> int:
    _fail(type(value) is not int, f"{path} must be an integer")
    if minimum is not None:
        _fail(value < minimum, f"{path} must be at least {minimum}")
    return value


def _require_text(value: Any, path: str, *, allow_empty: bool = False) -> str:
    _fail(type(value) is not str, f"{path} must be text")
    _fail(not allow_empty and not value, f"{path} must not be empty")
    return value


def _require_finite_number(value: Any, path: str, *, minimum: float | None = None) -> None:
    _fail(not _is_number(value), f"{path} must be a finite number")
    if minimum is not None:
        _fail(value < minimum, f"{path} must be at least {minimum}")


def _validate_hal_hash(value: Any, path: str) -> str:
    _require_text(value, path)
    _fail(_HAL_HASH_RE.fullmatch(value) is None, f"{path} must be a lowercase SHA-256 hex digest")
    return value


def _validate_device(device: Any, robot_id: int, path: str) -> dict:
    device = _require_exact_keys(device, _DEVICE_KEYS, path)
    serial = _require_text(device["serialNumber"], f"{path}.serialNumber")
    _fail(serial != str(robot_id), f"{path}.serialNumber does not match robot_id")
    _require_text(device["softwareVersion"], f"{path}.softwareVersion")
    _require_text(device["firmwareVersion"], f"{path}.firmwareVersion")
    return device


def _validate_sensor_param(sensor_param: Any, path: str) -> set[int]:
    sensor_param = _require_keys(
        sensor_param, {"id", "name", "uuid", *_SENSOR_LIST_KEYS}, path
    )
    _require_int(sensor_param["id"], f"{path}.id", minimum=0)
    _require_text(sensor_param["name"], f"{path}.name", allow_empty=True)
    _require_text(sensor_param["uuid"], f"{path}.uuid", allow_empty=True)
    sensor_ids: set[int] = set()
    for list_key in _SENSOR_LIST_KEYS:
        sensors = sensor_param[list_key]
        _fail(type(sensors) is not list, f"{path}.{list_key} must be a list")
        for index, sensor in enumerate(sensors):
            item_path = f"{path}.{list_key}[{index}]"
            sensor = _require_keys(sensor, _SENSOR_KEYS, item_path)
            sensor_id = _require_int(sensor["id"], f"{item_path}.id", minimum=1)
            _fail(sensor_id in sensor_ids, f"{item_path}.id is duplicated across sensor lists")
            sensor_ids.add(sensor_id)
            for key in ("brand", "transferMode", "direction"):
                _require_int(sensor[key], f"{item_path}.{key}", minimum=0)
            _require_text(sensor["mark"], f"{item_path}.mark", allow_empty=True)
            _require_text(sensor["name"], f"{item_path}.name", allow_empty=True)
            for key in ("x", "y", "z", "theta", "frequency", "resolution", "offset", "param1", "param2"):
                _require_finite_number(sensor[key], f"{item_path}.{key}")
            for key in ("scanAngleRange", "detectAngleRange"):
                value_range = _require_exact_keys(
                    sensor[key], {"min", "max"}, f"{item_path}.{key}"
                )
                _require_finite_number(value_range["min"], f"{item_path}.{key}.min")
                _require_finite_number(value_range["max"], f"{item_path}.{key}.max")
                _fail(
                    value_range["min"] > value_range["max"],
                    f"{item_path}.{key}.min must not exceed max",
                )
    return sensor_ids


def _validate_avoid_ranges(value: Any, sensor_ids: set[int], path: str) -> set[int]:
    value = _require_keys(value, {"id", "name", "uuid", "avoidRangeList"}, path)
    _require_int(value["id"], f"{path}.id", minimum=0)
    _require_text(value["name"], f"{path}.name", allow_empty=True)
    _require_text(value["uuid"], f"{path}.uuid", allow_empty=True)
    ranges = value["avoidRangeList"]
    _fail(type(ranges) is not list or not ranges, f"{path}.avoidRangeList must be a non-empty list")
    range_ids: set[int] = set()
    for index, avoid_range in enumerate(ranges):
        item_path = f"{path}.avoidRangeList[{index}]"
        avoid_range = _require_keys(avoid_range, _RANGE_KEYS, item_path)
        range_id = _require_int(avoid_range["id"], f"{item_path}.id", minimum=1)
        _fail(range_id in range_ids, f"{item_path}.id is duplicated")
        range_ids.add(range_id)
        _require_int(avoid_range["sensorId"], f"{item_path}.sensorId", minimum=1)
        _fail(
            avoid_range["sensorId"] not in sensor_ids,
            f"{item_path}.sensorId does not identify a configured sensor",
        )
        shape = _require_int(avoid_range["shape"], f"{item_path}.shape", minimum=0)
        _fail(shape not in (0, 2), f"{item_path}.shape is unsupported; expected a locally observed shape 0 or 2")
        _require_int(avoid_range["mark"], f"{item_path}.mark")
        _require_text(avoid_range["name"], f"{item_path}.name", allow_empty=True)
        _require_text(avoid_range["uuid"], f"{item_path}.uuid", allow_empty=True)
        for tier in ("stop", "warn1", "warn2"):
            tier_path = f"{item_path}.{tier}"
            tier_value = _require_exact_keys(
                avoid_range[tier], {"positions", "zRange"}, tier_path
            )
            positions = tier_value["positions"]
            _fail(
                type(positions) is not list or len(positions) < 3,
                f"{tier_path}.positions must contain at least three points",
            )
            for point_index, point in enumerate(positions):
                point_path = f"{tier_path}.positions[{point_index}]"
                point = _require_exact_keys(point, {"x", "y"}, point_path)
                _require_finite_number(point["x"], f"{point_path}.x")
                _require_finite_number(point["y"], f"{point_path}.y")
            unique_points = {(point["x"], point["y"]) for point in positions}
            _fail(
                len(unique_points) < 3,
                f"{tier_path}.positions must contain at least three unique vertices",
            )
            try:
                doubled_area = math.fsum(
                    point["x"] * positions[(point_index + 1) % len(positions)]["y"]
                    - positions[(point_index + 1) % len(positions)]["x"] * point["y"]
                    for point_index, point in enumerate(positions)
                )
            except (OverflowError, ValueError) as exc:
                raise ValueError(f"{tier_path}.positions area is not finite") from exc
            _fail(not math.isfinite(doubled_area), f"{tier_path}.positions area is not finite")
            _fail(doubled_area == 0.0, f"{tier_path}.positions has zero area")
            z_range = _require_exact_keys(tier_value["zRange"], {"min", "max"}, f"{tier_path}.zRange")
            _require_finite_number(z_range["min"], f"{tier_path}.zRange.min")
            _require_finite_number(z_range["max"], f"{tier_path}.zRange.max")
            _fail(z_range["min"] > z_range["max"], f"{tier_path}.zRange.min must not exceed max")
    return range_ids


def _validate_avoid_strategy(value: Any, range_ids: set[int], path: str) -> None:
    value = _require_keys(value, {"id", "name", "uuid", "globalStrategy", "localStrtegy"}, path)
    _require_int(value["id"], f"{path}.id", minimum=0)
    _require_text(value["name"], f"{path}.name", allow_empty=True)
    _require_text(value["uuid"], f"{path}.uuid", allow_empty=True)
    for key in ("globalStrategy", "localStrtegy"):
        strategies = value[key]
        _fail(type(strategies) is not list, f"{path}.{key} must be a list")
        ids: set[int] = set()
        for index, strategy in enumerate(strategies):
            item_path = f"{path}.{key}[{index}]"
            strategy = _require_keys(strategy, _STRATEGY_KEYS, item_path)
            strategy_id = _require_int(strategy["id"], f"{item_path}.id", minimum=1)
            _fail(strategy_id in ids, f"{item_path}.id is duplicated")
            ids.add(strategy_id)
            _require_text(strategy["name"], f"{item_path}.name", allow_empty=True)
            _require_text(strategy["uuid"], f"{item_path}.uuid", allow_empty=True)
            range_list = strategy["rangeList"]
            _fail(type(range_list) is not list, f"{item_path}.rangeList must be a list")
            referenced_ids: set[int] = set()
            for range_index, range_id in enumerate(range_list):
                _require_int(range_id, f"{item_path}.rangeList[{range_index}]", minimum=1)
                _fail(range_id in referenced_ids, f"{item_path}.rangeList contains a duplicated range ID")
                referenced_ids.add(range_id)
                _fail(
                    range_id not in range_ids,
                    f"{item_path}.rangeList references ID {range_id} absent from avoidRangeList",
                )


def _validate_parameters(parameters: Any, robot_id: int, agv_type_id: int, map_id: int, path: str) -> dict:
    parameters = _require_exact_keys(parameters, _PARAMETER_KEYS, path)
    _fail(_require_int(parameters["id"], f"{path}.id", minimum=1) != robot_id,
          f"{path}.id does not match robot_id")
    _fail(_require_int(parameters["agvTypeId"], f"{path}.agvTypeId", minimum=1) != agv_type_id,
          f"{path}.agvTypeId does not match agv_type_id")
    _fail(_require_int(parameters["mapId"], f"{path}.mapId", minimum=1) != map_id,
          f"{path}.mapId does not match map_id")

    chassis = _require_keys(
        parameters["chassisParam"],
        {"id", "mode", "name", "width", "head", "tail", "wheelAngle", "wheelDiameter", "wheelDistance", "wheelSpace", "uuid"},
        f"{path}.chassisParam",
    )
    _require_int(chassis["id"], f"{path}.chassisParam.id", minimum=1)
    _require_int(chassis["mode"], f"{path}.chassisParam.mode", minimum=0)
    _require_text(chassis["name"], f"{path}.chassisParam.name", allow_empty=True)
    for key in ("width", "head", "tail", "wheelAngle", "wheelDiameter", "wheelDistance", "wheelSpace"):
        _require_finite_number(chassis[key], f"{path}.chassisParam.{key}", minimum=0.0)
    _require_text(chassis["uuid"], f"{path}.chassisParam.uuid", allow_empty=True)

    sensor_ids = _validate_sensor_param(parameters["sensorsParam"], f"{path}.sensorsParam")
    range_ids = _validate_avoid_ranges(
        parameters["avoidRangesParam"], sensor_ids, f"{path}.avoidRangesParam"
    )
    _validate_avoid_strategy(
        parameters["avoidStrategyParam"], range_ids, f"{path}.avoidStrategyParam"
    )

    schedule = _require_keys(
        parameters["scheduleParam"],
        {"maxSpeed", "maxFullVectorSpeed", "maxEmptyVectorSpeed", "maxCurveSpeed", "isCanBackward", "isCanShift", "length", "width", "rotateRadius", "avoidRange", "safeDistance"},
        f"{path}.scheduleParam",
    )
    for key in ("isCanBackward", "isCanShift"):
        _fail(type(schedule[key]) is not bool, f"{path}.scheduleParam.{key} must be bool")
    for key in ("maxSpeed", "maxFullVectorSpeed", "maxEmptyVectorSpeed", "maxCurveSpeed", "length", "width", "rotateRadius", "avoidRange", "safeDistance"):
        _require_finite_number(schedule[key], f"{path}.scheduleParam.{key}", minimum=0.0)

    move = _require_exact_keys(
        parameters["moveControlParam"], {"autoMoveParam"}, f"{path}.moveControlParam"
    )
    auto_move = _require_keys(
        move["autoMoveParam"],
        {"maxSpeed", "acc", "dec", "rotateAcc", "rotateDec", "maxRotateSpeed"},
        f"{path}.moveControlParam.autoMoveParam",
    )
    for key in ("maxSpeed", "acc", "dec", "rotateAcc", "rotateDec", "maxRotateSpeed"):
        _require_finite_number(auto_move[key], f"{path}.moveControlParam.autoMoveParam.{key}", minimum=0.0)
    return parameters


def _validate_provenance(value: Any, path: str) -> dict:
    value = _require_exact_keys(value, {"controller_timestamps", "query_duration_s"}, path)
    stamps = _require_exact_keys(
        value["controller_timestamps"], {"device", "parameters"}, f"{path}.controller_timestamps"
    )
    for name, stamp in stamps.items():
        _require_text(stamp, f"{path}.controller_timestamps.{name}")
    _require_finite_number(value["query_duration_s"], f"{path}.query_duration_s", minimum=0.0)
    return value


def _validate_manifest_structure(data: Any, *, allowed_statuses: set[str]) -> dict:
    data = _require_exact_keys(data, _MANIFEST_KEYS, "manifest")
    _validate_json_value(data)
    _fail(data["kind"] != _KIND, f"manifest.kind must be {_KIND}")
    _fail(type(data["schema_version"]) is not int or data["schema_version"] != _SCHEMA_VERSION,
          f"manifest.schema_version must be {_SCHEMA_VERSION}")
    _fail(data["status"] not in allowed_statuses,
          "manifest.status must be " + " or ".join(sorted(allowed_statuses)))
    robot_id = _require_int(data["robot_id"], "manifest.robot_id", minimum=1)
    agv_type_id = _require_int(data["agv_type_id"], "manifest.agv_type_id", minimum=1)
    map_id = _require_int(data["map_id"], "manifest.map_id", minimum=1)
    _require_text(data["map_name"], "manifest.map_name")
    _validate_hal_hash(data["hal_sha256"], "manifest.hal_sha256")

    expected = _require_exact_keys(data["expected"], _EXPECTED_KEYS, "manifest.expected")
    _validate_device(expected["device"], robot_id, "manifest.expected.device")
    _validate_parameters(
        expected["parameters"], robot_id, agv_type_id, map_id, "manifest.expected.parameters"
    )
    _validate_provenance(data["provenance"], "manifest.provenance")
    claims = _require_exact_keys(data["claims"], set(_MONITORED_CLAIMS), "manifest.claims")
    _fail(
        _canonical_bytes(claims) != _canonical_bytes(_MONITORED_CLAIMS),
        "manifest.claims must not assert physical or route verification",
    )
    digest = _validate_hal_hash(data["canonical_sha256"], "manifest.canonical_sha256")
    _fail(digest != canonical_digest(data), "manifest canonical_sha256 does not match its contents")
    return data


def validate_manifest(
    data: dict,
    *,
    robot_id: int,
    map_id: int,
    map_name: str,
    hal_sha256: str,
) -> dict:
    """Validate an explicitly approved manifest against caller expectations."""
    robot_id = _require_int(robot_id, "robot_id", minimum=1)
    map_id = _require_int(map_id, "map_id", minimum=1)
    _require_text(map_name, "map_name")
    _validate_hal_hash(hal_sha256, "hal_sha256")
    manifest = _validate_manifest_structure(data, allowed_statuses={_APPROVED_STATUS})
    checks = (
        (manifest["robot_id"] != robot_id, "manifest robot_id does not match"),
        (manifest["map_id"] != map_id, "manifest map_id does not match"),
        (manifest["map_name"] != map_name, "manifest map_name does not match"),
        (manifest["hal_sha256"] != hal_sha256, "manifest hal_sha256 does not match"),
    )
    for failed, message in checks:
        _fail(failed, message)
    return copy.deepcopy(manifest)


def build_manifest_candidate(snapshot: dict, *, map_name: str, hal_sha256: str) -> dict:
    """Build a review-only manifest candidate from one captured readback."""
    _require_text(map_name, "map_name")
    _validate_hal_hash(hal_sha256, "hal_sha256")
    snapshot = _require_exact_keys(
        snapshot,
        {"device", "parameters", "received_monotonic", "query_duration_s", "controller_timestamps"},
        "snapshot",
    )
    _validate_json_value(snapshot)
    _require_finite_number(snapshot["received_monotonic"], "snapshot.received_monotonic", minimum=0.0)
    _require_finite_number(snapshot["query_duration_s"], "snapshot.query_duration_s", minimum=0.0)
    stamps = _validate_provenance(
        {"controller_timestamps": snapshot["controller_timestamps"],
         "query_duration_s": snapshot["query_duration_s"]},
        "snapshot",
    )
    parameters = snapshot["parameters"]
    parameters = _require_keys(parameters, _PARAMETER_KEYS, "snapshot.parameters")
    robot_id = _require_int(parameters["id"], "snapshot.parameters.id", minimum=1)
    agv_type_id = _require_int(parameters["agvTypeId"], "snapshot.parameters.agvTypeId", minimum=1)
    map_id = _require_int(parameters["mapId"], "snapshot.parameters.mapId", minimum=1)
    _validate_device(snapshot["device"], robot_id, "snapshot.device")
    move_control = _require_keys(
        parameters["moveControlParam"],
        {"autoMoveParam"},
        "snapshot.parameters.moveControlParam",
    )
    selected_parameters = {
        "id": parameters["id"],
        "agvTypeId": parameters["agvTypeId"],
        "mapId": parameters["mapId"],
        "chassisParam": copy.deepcopy(parameters["chassisParam"]),
        "sensorsParam": copy.deepcopy(parameters["sensorsParam"]),
        "avoidRangesParam": copy.deepcopy(parameters["avoidRangesParam"]),
        "avoidStrategyParam": copy.deepcopy(parameters["avoidStrategyParam"]),
        "scheduleParam": copy.deepcopy(parameters["scheduleParam"]),
        "moveControlParam": {
            "autoMoveParam": copy.deepcopy(move_control["autoMoveParam"])
        },
    }
    manifest = {
        "kind": _KIND,
        "schema_version": _SCHEMA_VERSION,
        "status": _REVIEW_STATUS,
        "robot_id": robot_id,
        "agv_type_id": agv_type_id,
        "map_id": map_id,
        "map_name": map_name,
        "hal_sha256": hal_sha256,
        "expected": {
            "device": {
                "serialNumber": snapshot["device"]["serialNumber"],
                "softwareVersion": snapshot["device"]["softwareVersion"],
                "firmwareVersion": snapshot["device"]["firmwareVersion"],
            },
            "parameters": selected_parameters,
        },
        "provenance": {
            "controller_timestamps": copy.deepcopy(stamps["controller_timestamps"]),
            "query_duration_s": stamps["query_duration_s"],
        },
        "claims": copy.deepcopy(_MONITORED_CLAIMS),
    }
    manifest["canonical_sha256"] = canonical_digest(manifest)
    _validate_manifest_structure(manifest, allowed_statuses={_REVIEW_STATUS})
    return manifest


def _diff_values(expected: Any, actual: Any, path: str) -> list[str]:
    if expected is _MISSING:
        return [f"{path}: unexpected field"]
    if actual is _MISSING:
        return [f"{path}: missing field"]
    if type(expected) is not type(actual):
        return [f"{path}: JSON type differs"]
    if type(expected) is dict:
        differences: list[str] = []
        for key in sorted(set(expected) | set(actual)):
            child_path = f"{path}.{key}" if path else key
            differences.extend(
                _diff_values(expected.get(key, _MISSING), actual.get(key, _MISSING), child_path)
            )
        return differences
    if type(expected) is list:
        differences = []
        if len(expected) != len(actual):
            differences.append(f"{path}: array length differs")
        for index in range(min(len(expected), len(actual))):
            differences.extend(_diff_values(expected[index], actual[index], f"{path}[{index}]"))
        return differences
    if expected != actual:
        return [f"{path}: value differs"]
    return []


def verify_snapshot(manifest: dict, snapshot: dict) -> dict:
    """Compare one snapshot with an approved manifest; this is not a freshness check."""
    manifest = validate_manifest(
        manifest,
        robot_id=manifest.get("robot_id") if type(manifest) is dict else 0,
        map_id=manifest.get("map_id") if type(manifest) is dict else 0,
        map_name=manifest.get("map_name") if type(manifest) is dict else "",
        hal_sha256=manifest.get("hal_sha256") if type(manifest) is dict else "",
    )
    differences: list[str] = []
    if type(snapshot) is not dict:
        differences.append("snapshot: must be a mapping")
        snapshot = {}
    for key in ("device", "parameters"):
        if key not in snapshot:
            differences.append(f"snapshot.{key}: missing field")

    expected = manifest["expected"]
    actual_device = snapshot.get("device", _MISSING)
    actual_parameters = snapshot.get("parameters", _MISSING)
    for field in _DEVICE_KEYS:
        actual_value = actual_device.get(field, _MISSING) if type(actual_device) is dict else _MISSING
        differences.extend(
            _diff_values(expected["device"][field], actual_value, f"device.{field}")
        )
    for field in _PARAMETER_KEYS:
        expected_value = expected["parameters"][field]
        if field == "moveControlParam":
            actual_value = (
                actual_parameters.get("moveControlParam", {}).get("autoMoveParam", _MISSING)
                if type(actual_parameters) is dict
                and type(actual_parameters.get("moveControlParam")) is dict
                else _MISSING
            )
            expected_value = expected_value["autoMoveParam"]
            path = "parameters.moveControlParam.autoMoveParam"
        else:
            actual_value = actual_parameters.get(field, _MISSING) if type(actual_parameters) is dict else _MISSING
            path = f"parameters.{field}"
        differences.extend(_diff_values(expected_value, actual_value, path))

    # Keep the report concise while retaining all distinct mismatching paths.
    differences = sorted(set(differences))
    matches = not differences
    return {
        "status": "CONFIGURATION_MANIFEST_MATCH" if matches else "CONFIGURATION_MANIFEST_MISMATCH",
        "configuration_manifest_match": matches,
        "live_freshness_verified": False,
        "freshness_scope": "local_request_response_receipt_only",
        "server_state_freshness_verified": False,
        "physical_3d_collision_verified": False,
        "sensor_coverage_verified": False,
        "route_clearance_verified": False,
        "straight_reverse_verified": False,
        "speed_policy_verified": False,
        "robot_id": manifest["robot_id"],
        "map_id": manifest["map_id"],
        "expected_sha256": manifest["canonical_sha256"],
        "differences": differences,
    }


class AvoidanceMonitor:
    """Poll an injected read-only snapshot source and latch the first failure."""

    def __init__(
        self,
        manifest: dict,
        read_snapshot,
        *,
        poll_interval_s: float = 1.0,
        max_age_s: float = 2.0,
        clock=time.monotonic,
    ):
        _fail(not callable(read_snapshot), "read_snapshot must be callable")
        _fail(not callable(clock), "clock must be callable")
        _require_finite_number(poll_interval_s, "poll_interval_s", minimum=0.0)
        _require_finite_number(max_age_s, "max_age_s", minimum=0.0)
        _fail(poll_interval_s <= 0, "poll_interval_s must be positive")
        _fail(max_age_s <= 0, "max_age_s must be positive")
        _fail(type(manifest) is not dict, "manifest must be a mapping")
        self._manifest = validate_manifest(
            manifest,
            robot_id=manifest.get("robot_id"),
            map_id=manifest.get("map_id"),
            map_name=manifest.get("map_name"),
            hal_sha256=manifest.get("hal_sha256"),
        )
        self._read_snapshot = read_snapshot
        self._poll_interval_s = float(poll_interval_s)
        self._max_age_s = float(max_age_s)
        self._clock = clock
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._thread_started = False
        self._started = False
        self._closed = False
        self._failure: str | None = None
        self._last_received_monotonic: float | None = None
        self._last_result: dict | None = None
        self._last_query_duration_s: float | None = None
        self._poll_count = 0

    @staticmethod
    def _clock_value(clock, label: str) -> float:
        value = clock()
        _require_finite_number(value, label, minimum=0.0)
        return float(value)

    def _capture_and_verify(self) -> dict | None:
        started_at = self._clock_value(self._clock, "monitor clock before read")
        snapshot = self._read_snapshot()
        finished_at = self._clock_value(self._clock, "monitor clock after read")
        if self._stop_event.is_set():
            return None
        _fail(finished_at < started_at, "monitor clock regressed during snapshot read")
        elapsed = finished_at - started_at
        _fail(elapsed > self._max_age_s, f"total local read duration exceeds max_age_s ({elapsed:.3f}s)")
        _fail(type(snapshot) is not dict, "read_snapshot must return a mapping")
        snapshot = _require_exact_keys(
            snapshot,
            {"device", "parameters", "received_monotonic", "query_duration_s", "controller_timestamps"},
            "snapshot",
        )
        _validate_json_value(snapshot)
        _require_finite_number(
            snapshot["received_monotonic"], "snapshot.received_monotonic", minimum=0.0
        )
        _require_finite_number(
            snapshot["query_duration_s"], "snapshot.query_duration_s", minimum=0.0
        )
        received = float(snapshot["received_monotonic"])
        duration = float(snapshot["query_duration_s"])
        _validate_provenance(
            {"controller_timestamps": snapshot["controller_timestamps"],
             "query_duration_s": snapshot["query_duration_s"]},
            "snapshot",
        )
        _fail(received < started_at, "snapshot receipt predates this read; stale cached snapshot")
        _fail(received > finished_at, "snapshot receipt is in the future")
        age = finished_at - received
        _fail(age > self._max_age_s, f"snapshot is stale (age {age:.3f}s)")
        _fail(duration > self._max_age_s, f"snapshot query duration exceeds max_age_s ({duration:.3f}s)")
        with self._lock:
            previous = self._last_received_monotonic
        _fail(
            previous is not None and received <= previous,
            "snapshot receipt did not advance; stale or reused snapshot",
        )

        result = verify_snapshot(self._manifest, snapshot)
        _fail(
            not result["configuration_manifest_match"],
            "controller configuration mismatch: " + "; ".join(result["differences"][:8]),
        )
        if self._stop_event.is_set():
            return None
        result["live_freshness_verified"] = True
        result["freshness_scope"] = "local_request_response_receipt_only"
        result["server_state_freshness_verified"] = False
        result["snapshot_age_s"] = age
        result["query_duration_s"] = duration
        result["measured_read_duration_s"] = elapsed
        result["received_monotonic"] = received
        result["verified_at_monotonic"] = finished_at
        with self._lock:
            if self._closed or self._stop_event.is_set():
                return None
            self._last_received_monotonic = received
            self._last_result = result
            self._last_query_duration_s = duration
            self._poll_count += 1
        return copy.deepcopy(result)

    def _latch_failure(self, error: Exception) -> None:
        message = f"{type(error).__name__}: {error}"
        with self._lock:
            if self._failure is None:
                self._failure = message
                self._stop_event.set()

    def start(self) -> dict:
        with self._lock:
            _fail(self._closed, "monitor is closed; create a new monitor")
            _fail(self._started, "monitor has already been started")
            _fail(self._failure is not None, f"monitor is failed: {self._failure}")
        try:
            first = self._capture_and_verify()
            _fail(first is None, "monitor was closed before its initial verification completed")
        except Exception as exc:
            self._latch_failure(exc)
            raise RuntimeError(f"avoidance monitor initial verification failed: {exc}") from exc

        with self._lock:
            _fail(self._closed, "monitor was closed during startup")
            self._started = True
            self._thread = threading.Thread(
                target=self._poll,
                name="avoidance-manifest-monitor",
                daemon=True,
            )
            thread = self._thread
        try:
            thread.start()
        except Exception as exc:
            self._latch_failure(exc)
            raise RuntimeError(f"avoidance monitor could not start: {exc}") from exc
        with self._lock:
            if self._thread is thread:
                self._thread_started = True
        return self.check()

    def _poll(self) -> None:
        while not self._stop_event.wait(self._poll_interval_s):
            try:
                result = self._capture_and_verify()
                if result is None:
                    return
            except Exception as exc:
                self._latch_failure(exc)
                return

    def _current_age(self) -> float | None:
        with self._lock:
            received = self._last_received_monotonic
        if received is None:
            return None
        now = self._clock_value(self._clock, "monitor clock")
        return now - received

    def check(self) -> dict:
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"avoidance monitor failed: {self._failure}")
            _fail(not self._started, "avoidance monitor has not been started")
            _fail(self._closed, "avoidance monitor is closed")
            thread = self._thread
        if thread is None or not thread.is_alive():
            error = RuntimeError("avoidance monitor background reader stopped unexpectedly")
            self._latch_failure(error)
            raise RuntimeError(f"avoidance monitor failed: {error}") from error
        result = self.summary()
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"avoidance monitor failed: {self._failure}")
            if self._closed:
                raise RuntimeError("avoidance monitor is closed")
            if (
                result.get("status") != "MONITORING"
                or result.get("configuration_manifest_match") is not True
                or result.get("live_freshness_verified") is not True
            ):
                error = RuntimeError("avoidance monitor summary is not healthy")
                self._latch_failure(error)
                raise RuntimeError(f"avoidance monitor failed: {error}") from error
            thread = self._thread
            if thread is None or not thread.is_alive():
                error = RuntimeError("avoidance monitor background reader stopped unexpectedly")
                self._latch_failure(error)
                raise RuntimeError(f"avoidance monitor failed: {error}") from error
            if self._last_received_monotonic is None or self._last_result is None:
                error = RuntimeError("avoidance monitor has no successful verification")
                self._latch_failure(error)
                raise RuntimeError(f"avoidance monitor failed: {error}") from error
            try:
                now = self._clock_value(self._clock, "monitor clock")
                age = now - self._last_received_monotonic
                _fail(age < 0, "last snapshot receipt is in the future")
                _fail(age > self._max_age_s, f"last configuration readback is stale (age {age:.3f}s)")
            except Exception as exc:
                self._latch_failure(exc)
                raise RuntimeError(f"avoidance monitor failed: {exc}") from exc
            current = copy.deepcopy(self._last_result)
            current.update(
                status="MONITORING",
                configuration_manifest_match=True,
                live_freshness_verified=True,
                snapshot_age_s=age,
            )
            return current

    def summary(self) -> dict:
        with self._lock:
            started = self._started
            closed = self._closed
            failure = self._failure
            previous = copy.deepcopy(self._last_result)
            thread = self._thread
        if previous is None:
            result = {
                "configuration_manifest_match": False,
                "live_freshness_verified": False,
                "freshness_scope": "local_request_response_receipt_only",
                "server_state_freshness_verified": False,
                "physical_3d_collision_verified": False,
                "sensor_coverage_verified": False,
                "route_clearance_verified": False,
                "straight_reverse_verified": False,
                "speed_policy_verified": False,
                "robot_id": self._manifest["robot_id"],
                "map_id": self._manifest["map_id"],
                "expected_sha256": self._manifest["canonical_sha256"],
                "differences": [],
            }
        else:
            result = previous

        age: float | None = None
        if previous is not None:
            try:
                age = self._current_age()
            except Exception as exc:
                if failure is None:
                    self._latch_failure(exc)
                    failure = f"{type(exc).__name__}: {exc}"
        if failure is not None:
            result.update(
                status="ERROR",
                configuration_manifest_match=False,
                live_freshness_verified=False,
                failure=failure,
            )
        elif closed:
            result.update(
                status="CLOSED",
                configuration_manifest_match=False,
                live_freshness_verified=False,
            )
        elif not started:
            result.update(
                status="NOT_STARTED",
                configuration_manifest_match=False,
                live_freshness_verified=False,
            )
        elif thread is None or not thread.is_alive():
            error = RuntimeError("avoidance monitor background reader stopped unexpectedly")
            self._latch_failure(error)
            result.update(
                status="ERROR",
                configuration_manifest_match=False,
                live_freshness_verified=False,
                failure=f"{type(error).__name__}: {error}",
            )
        elif age is None or age < 0 or age > self._max_age_s:
            error = RuntimeError(f"last configuration readback is stale (age {age})")
            self._latch_failure(error)
            result.update(
                status="ERROR",
                configuration_manifest_match=False,
                live_freshness_verified=False,
                snapshot_age_s=age,
                failure=f"{type(error).__name__}: {error}",
            )
        else:
            result.update(
                status="MONITORING",
                configuration_manifest_match=True,
                live_freshness_verified=True,
                snapshot_age_s=age,
            )
        return result

    def close(self) -> None:
        self._stop_event.set()
        with self._lock:
            self._closed = True
            thread = self._thread
            thread_started = self._thread_started
        if thread_started and thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

"""Bounded read-only controller queries for avoidance configuration checks."""
from __future__ import annotations

import json
import math
import time
import urllib.request
from urllib.parse import urlsplit


MAX_RESPONSE_BYTES = 1024 * 1024
QUERY_TIMEOUT_S = 0.5


def _reject_json_constant(value):
    raise ValueError(f"non-finite JSON value: {value}")


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("controller JSON number must be finite")
    return number


def _query(base_url: str, endpoint: str, robot_id: int) -> tuple[dict, str]:
    request = urllib.request.Request(
        base_url.rstrip("/") + endpoint,
        data=json.dumps({"id": robot_id}).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=QUERY_TIMEOUT_S) as response:
            raw = response.read(MAX_RESPONSE_BYTES + 1)
        if len(raw) > MAX_RESPONSE_BYTES:
            raise ValueError("controller response exceeds size limit")
        envelope = json.loads(raw, parse_constant=_reject_json_constant, parse_float=_finite_float)
        if not isinstance(envelope, dict):
            raise ValueError("response must be an object")
        if type(envelope.get("ErrorCode")) is not int or envelope["ErrorCode"] != 0:
            raise ValueError("controller query did not return ErrorCode 0")
        if envelope.get("Interface") != endpoint:
            raise ValueError("response Interface does not match the query")
        timestamp = envelope.get("TimeStamp")
        if not isinstance(timestamp, str) or not timestamp.strip():
            raise ValueError("controller TimeStamp is missing")
        content = envelope.get("Content")
        if not isinstance(content, dict) or not content:
            raise ValueError("controller Content is missing")
        return content, timestamp
    except Exception as exc:
        raise RuntimeError(f"avoidance read-back failed at {endpoint}: {exc}") from exc


def read_avoidance_snapshot(config: dict) -> dict:
    """Read configuration; never send a robot command or change a setting.

    Controller timestamps are retained for audit, not compared with the local
    wall clock. The monitor uses local monotonic receipt time and request duration.
    A socket timeout applies per query; consumers must also enforce maximum age.
    """
    robot_id, map_id = config.get("robot_id"), config.get("map_id")
    if type(robot_id) is not int or robot_id <= 0 or type(map_id) is not int or map_id <= 0:
        raise RuntimeError("avoidance read-back requires positive robot_id and map_id")
    url = config.get("controller_url")
    if not isinstance(url, str) or urlsplit(url).scheme not in ("http", "https") or not urlsplit(url).hostname:
        raise RuntimeError("avoidance read-back requires a valid controller_url")
    started = time.monotonic()
    device, device_timestamp = _query(url, "/api/AMR/GetDeviceInfo", robot_id)
    if device.get("serialNumber") != str(robot_id):
        raise RuntimeError("avoidance controller serial number mismatches robot_id")
    parameters, parameters_timestamp = _query(url, "/api/Config/Agv/GetParam", robot_id)
    if type(parameters.get("id")) is not int or parameters["id"] != robot_id:
        raise RuntimeError("avoidance controller id mismatches robot_id")
    if type(parameters.get("mapId")) is not int or parameters["mapId"] != map_id:
        raise RuntimeError("avoidance controller mapId mismatches configured map")
    received = time.monotonic()
    return {
        "device": device,
        "parameters": parameters,
        "received_monotonic": received,
        "query_duration_s": received - started,
        "controller_timestamps": {"device": device_timestamp, "parameters": parameters_timestamp},
    }

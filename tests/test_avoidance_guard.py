from __future__ import annotations

import copy
from pathlib import Path
import sys
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.avoidance_guard import (
    AvoidanceMonitor,
    build_manifest_candidate,
    canonical_digest,
    validate_manifest,
    verify_snapshot,
)


def snapshot() -> dict:
    return {
        "device": {
            "serialNumber": "424",
            "softwareVersion": "7.2.0.23",
            "firmwareVersion": "2.0.0.7",
        },
        "parameters": {
            "id": 424,
            "agvTypeId": 1,
            "mapId": 7,
            "chassisParam": {
                "id": 1,
                "mode": 0,
                "name": "底盘",
                "uuid": "{00000000-0000-0000-0000-000000000000}",
                "width": 450,
                "head": 300,
                "tail": 300,
                "wheelAngle": 24.3,
                "wheelDiameter": 152,
                "wheelDistance": 375,
                "wheelSpace": 375,
            },
            "sensorsParam": {
                "id": 0,
                "name": "",
                "uuid": "{7fab0920-ea76-11ee-b21c-693dbabae4ca}",
                "laserList": [
                    {
                        "id": 6,
                        "brand": 3,
                        "mark": "",
                        "name": "前激光",
                        "transferMode": 3,
                        "x": 223,
                        "y": 0,
                        "z": 0,
                        "theta": 0,
                        "direction": 0,
                        "frequency": 15,
                        "resolution": 0.333333,
                        "scanAngleRange": {"min": -135, "max": 135},
                        "detectAngleRange": {"min": 10, "max": 50000},
                        "offset": 0,
                        "param1": 0,
                        "param2": 0,
                    }
                ],
                "cameraList": [
                    {
                        "id": 12,
                        "brand": 4,
                        "mark": "CH26C20006Y",
                        "name": "前视觉避障相机",
                        "transferMode": 5,
                        "x": 300,
                        "y": 0,
                        "z": 272,
                        "theta": 0,
                        "direction": 0,
                        "frequency": 0,
                        "resolution": 0,
                        "scanAngleRange": {"min": 0, "max": 0},
                        "detectAngleRange": {"min": 0, "max": 0},
                        "offset": 0,
                        "param1": 0,
                        "param2": 0,
                    }
                ],
                "ultrasonicList": [],
            },
            "avoidRangesParam": {
                "id": 0,
                "name": "",
                "uuid": "{00000000-0000-0000-0000-000000000000}",
                "avoidRangeList": [
                    {
                        "id": 1,
                        "mark": 0,
                        "name": "前激光避障",
                        "sensorId": 6,
                        "shape": 0,
                        "uuid": "{00000000-0000-0000-0000-000000000000}",
                        "stop": {
                            "positions": [
                                {"x": 50, "y": -272},
                                {"x": 50, "y": 260},
                                {"x": 221, "y": 260},
                                {"x": 221, "y": -272},
                            ],
                            "zRange": {"max": 0, "min": 0},
                        },
                        "warn1": {
                            "positions": [
                                {"x": 50, "y": -420},
                                {"x": 50, "y": 420},
                                {"x": 1240, "y": 420},
                                {"x": 1240, "y": -420},
                            ],
                            "zRange": {"max": 0, "min": 0},
                        },
                        "warn2": {
                            "positions": [
                                {"x": 50, "y": -340},
                                {"x": 50, "y": 340},
                                {"x": 840, "y": 340},
                                {"x": 840, "y": -340},
                            ],
                            "zRange": {"max": 0, "min": 0},
                        },
                    }
                ],
            },
            "avoidStrategyParam": {
                "id": 0,
                "name": "",
                "uuid": "{0dcd3a11-81c5-11f1-9067-3706eca90d25}",
                "globalStrategy": [
                    {"id": 1, "name": "前进", "rangeList": [], "uuid": "forward"},
                    {"id": 2, "name": "后退", "rangeList": [], "uuid": "backward"},
                    {"id": 7, "name": "前停车", "rangeList": [1], "uuid": "stop-front"},
                ],
                "localStrtegy": [
                    {"id": 11, "name": "左关", "rangeList": [], "uuid": "local-left-off"}
                ],
            },
            "scheduleParam": {
                "id": 0,
                "maxSpeed": 1000,
                "maxFullVectorSpeed": 20,
                "maxEmptyVectorSpeed": 20,
                "maxCurveSpeed": 200,
                "isCanBackward": False,
                "isCanShift": False,
                "length": 600,
                "width": 450,
                "rotateRadius": 375,
                "avoidRange": 200,
                "safeDistance": 100,
            },
            "moveControlParam": {
                "id": 1,
                "defaultMode": 1,
                "autoMoveParam": {
                    "id": 0,
                    "maxSpeed": 200,
                    "acc": 600,
                    "dec": 600,
                    "rotateAcc": 200,
                    "rotateDec": 200,
                    "maxRotateSpeed": 200,
                },
            },
        },
        "received_monotonic": 100.0,
        "query_duration_s": 0.1,
        "controller_timestamps": {
            "device": "2026-10-14 13:16:14",
            "parameters": "2026-10-14 13:16:15",
        },
    }


def approved_manifest(source: dict | None = None) -> dict:
    manifest = build_manifest_candidate(
        snapshot() if source is None else source,
        map_name="cup_new",
        hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
    )
    manifest["status"] = "approved_controller_snapshot"
    manifest["canonical_sha256"] = canonical_digest(manifest)
    return manifest


def test_candidate_is_review_required_and_cannot_be_used_as_approved_manifest():
    candidate = build_manifest_candidate(
        snapshot(),
        map_name="cup_new",
        hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
    )

    assert candidate["kind"] == "g1_controller_avoidance_manifest"
    assert candidate["schema_version"] == 1
    assert candidate["status"] == "review_required"
    assert candidate["expected"]["parameters"]["mapId"] == 7
    assert candidate["expected"]["parameters"]["avoidStrategyParam"]["localStrtegy"]
    assert candidate["claims"] == {
        "configuration_manifest_match_only": True,
        "physical_3d_collision_verified": False,
        "sensor_coverage_verified": False,
        "route_clearance_verified": False,
        "straight_reverse_verified": False,
        "speed_policy_verified": False,
    }
    assert candidate["canonical_sha256"] == canonical_digest(candidate)

    with pytest.raises(ValueError, match="approved_controller_snapshot"):
        validate_manifest(
            candidate,
            robot_id=424,
            map_id=7,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_approved_manifest_rejects_identity_digest_and_claim_edits():
    manifest = approved_manifest()
    with pytest.raises(ValueError, match="map_id"):
        validate_manifest(
            manifest,
            robot_id=424,
            map_id=6,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )

    tampered = copy.deepcopy(manifest)
    tampered["expected"]["parameters"]["scheduleParam"]["maxSpeed"] += 1
    with pytest.raises(ValueError, match="canonical_sha256"):
        validate_manifest(
            tampered,
            robot_id=424,
            map_id=7,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )

    overclaim = copy.deepcopy(manifest)
    overclaim["claims"]["physical_3d_collision_verified"] = True
    overclaim["canonical_sha256"] = canonical_digest(overclaim)
    with pytest.raises(ValueError, match="claims"):
        validate_manifest(
            overclaim,
            robot_id=424,
            map_id=7,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_approved_manifest_matches_configuration_without_claiming_live_freshness():
    source = snapshot()
    manifest = validate_manifest(
        approved_manifest(source),
        robot_id=424,
        map_id=7,
        map_name="cup_new",
        hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
    )

    result = verify_snapshot(manifest, source)

    assert result["status"] == "CONFIGURATION_MANIFEST_MATCH"
    assert result["configuration_manifest_match"] is True
    assert result["live_freshness_verified"] is False
    assert result["freshness_scope"] == "local_request_response_receipt_only"
    assert result["server_state_freshness_verified"] is False
    assert result["physical_3d_collision_verified"] is False
    assert result["sensor_coverage_verified"] is False
    assert result["route_clearance_verified"] is False


@pytest.mark.parametrize(
    ("mutate", "path"),
    [
        (lambda value: value["parameters"].update(mapId=6), "parameters.mapId"),
        (
            lambda value: value["parameters"]["avoidRangesParam"]["avoidRangeList"][0]["stop"]["positions"][0].update(x=51),
            "parameters.avoidRangesParam",
        ),
        (
            lambda value: value["parameters"]["avoidStrategyParam"]["localStrtegy"][0]["rangeList"].append(3),
            "parameters.avoidStrategyParam",
        ),
        (
            lambda value: value["parameters"]["sensorsParam"]["cameraList"][0].update(z=273),
            "parameters.sensorsParam",
        ),
        (
            lambda value: value["parameters"]["scheduleParam"].update(maxSpeed=999),
            "parameters.scheduleParam",
        ),
        (
            lambda value: value["parameters"]["moveControlParam"]["autoMoveParam"].update(maxSpeed=199),
            "parameters.moveControlParam.autoMoveParam",
        ),
        (
            lambda value: value["parameters"]["chassisParam"].update(width=450.0),
            "parameters.chassisParam.width",
        ),
        (
            lambda value: value["parameters"]["avoidRangesParam"]["avoidRangeList"][0]["stop"]["positions"].reverse(),
            "parameters.avoidRangesParam",
        ),
    ],
)
def test_any_manifested_configuration_change_fails_closed(mutate, path):
    source = snapshot()
    manifest = validate_manifest(
        approved_manifest(source),
        robot_id=424,
        map_id=7,
        map_name="cup_new",
        hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
    )
    changed = copy.deepcopy(source)
    mutate(changed)

    result = verify_snapshot(manifest, changed)

    assert result["status"] == "CONFIGURATION_MANIFEST_MISMATCH"
    assert result["configuration_manifest_match"] is False
    assert any(path in difference for difference in result["differences"])


@pytest.mark.parametrize(
        "mutate",
        [
        lambda data: data.pop("sensorsParam"),
        lambda data: data["avoidRangesParam"]["avoidRangeList"][0].pop("stop"),
        lambda data: data["avoidStrategyParam"].pop("localStrtegy"),
        lambda data: data["moveControlParam"].pop("autoMoveParam"),
        ],
)
def test_candidate_rejects_missing_configuration_fields(mutate):
    source = snapshot()
    mutate(source["parameters"])

    with pytest.raises(ValueError):
        build_manifest_candidate(
            source,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_candidate_rejects_strategy_reference_to_an_unknown_range():
    source = snapshot()
    source["parameters"]["avoidStrategyParam"]["globalStrategy"][2]["rangeList"] = [99]

    with pytest.raises(ValueError, match="avoidRangeList"):
        build_manifest_candidate(
            source,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_candidate_rejects_duplicate_strategy_range_references():
    source = snapshot()
    source["parameters"]["avoidStrategyParam"]["globalStrategy"][2]["rangeList"] = [1, 1]

    with pytest.raises(ValueError, match="duplicated"):
        build_manifest_candidate(
            source,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


@pytest.mark.parametrize(
    "positions",
    [
        [{"x": 50, "y": 0}, {"x": 50, "y": 0}, {"x": 50, "y": 0}],
        [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 2, "y": 0}, {"x": 3, "y": 0}],
    ],
)
def test_candidate_rejects_degenerate_avoidance_polygons(positions):
    source = snapshot()
    source["parameters"]["avoidRangesParam"]["avoidRangeList"][0]["stop"]["positions"] = positions

    with pytest.raises(ValueError, match="(three unique|zero area)"):
        build_manifest_candidate(
            source,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_candidate_rejects_unverified_avoidance_shape():
    source = snapshot()
    source["parameters"]["avoidRangesParam"]["avoidRangeList"][0]["shape"] = 1

    with pytest.raises(ValueError, match="shape"):
        build_manifest_candidate(
            source,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


def test_validator_rejects_design_proposal_as_manifest():
    proposal = {
        "proposal_only": True,
        "combined_envelope": {"front_m": 0.8, "half_width_m": 0.525},
        "travel_regions": [],
    }

    with pytest.raises(ValueError):
        validate_manifest(
            proposal,
            robot_id=424,
            map_id=7,
            map_name="cup_new",
            hal_sha256="1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309",
        )


class FakeClock:
    def __init__(self, value: float = 100.0):
        self.value = value
        self.lock = threading.Lock()

    def __call__(self) -> float:
        with self.lock:
            return self.value

    def advance(self, seconds: float) -> float:
        with self.lock:
            self.value += seconds
            return self.value


def fresh_reader(clock: FakeClock, mutate=None):
    calls = 0
    lock = threading.Lock()
    called = threading.Event()

    def read():
        nonlocal calls
        with lock:
            calls += 1
            call_number = calls
        received = clock.advance(0.01)
        value = snapshot()
        value["received_monotonic"] = received
        if mutate is not None:
            mutate(value, call_number)
        called.set()
        return value

    return read, called, lambda: calls


def test_monitor_requires_a_fresh_initial_match_and_check_is_nonblocking():
    clock = FakeClock()
    reader, _, _ = fresh_reader(clock)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    try:
        started = monitor.start()
        checked = monitor.check()

        assert started["configuration_manifest_match"] is True
        assert started["live_freshness_verified"] is True
        assert started["freshness_scope"] == "local_request_response_receipt_only"
        assert started["server_state_freshness_verified"] is False
        assert checked["status"] == "MONITORING"
        assert checked["live_freshness_verified"] is True
    finally:
        monitor.close()


def test_check_does_not_mask_a_background_failure_reported_during_summary():
    clock = FakeClock()
    reader, _, _ = fresh_reader(clock)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    monitor.start()
    ordinary_summary = monitor.summary

    def racing_summary():
        result = ordinary_summary()
        result.update(
            status="ERROR",
            configuration_manifest_match=False,
            live_freshness_verified=False,
            failure="RuntimeError: background configuration mismatch",
        )
        return result

    monitor.summary = racing_summary

    with pytest.raises(RuntimeError, match="summary is not healthy"):
        monitor.check()
    with pytest.raises(RuntimeError, match="summary is not healthy"):
        monitor.check()
    monitor.close()


def test_monitor_latches_staleness_until_closed():
    clock = FakeClock()
    reader, _, _ = fresh_reader(clock)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    monitor.start()
    clock.advance(3.0)

    with pytest.raises(RuntimeError, match="stale"):
        monitor.check()
    assert monitor.summary()["status"] == "ERROR"
    assert monitor.summary()["live_freshness_verified"] is False
    monitor.close()


def test_summary_observing_expired_readback_latches_failure():
    clock = FakeClock()
    reader, _, _ = fresh_reader(clock)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    monitor.start()
    clock.advance(3.0)

    assert monitor.summary()["status"] == "ERROR"
    with pytest.raises(RuntimeError, match="stale"):
        monitor.check()
    monitor.close()


def test_monitor_latches_a_background_mismatch_and_does_not_recover_silently():
    clock = FakeClock()

    def change_second_snapshot(value, call_number):
        if call_number == 2:
            value["parameters"]["avoidRangesParam"]["avoidRangeList"][0]["stop"]["positions"][0]["x"] += 1

    reader, _, call_count = fresh_reader(clock, change_second_snapshot)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=0.05, max_age_s=2.0, clock=clock
    )
    monitor.start()
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if call_count() >= 2:
            break
        time.sleep(0.005)

    assert call_count() >= 2
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            monitor.check()
        except RuntimeError as exc:
            assert "mismatch" in str(exc).lower()
            break
        time.sleep(0.005)
    else:
        pytest.fail("background configuration mismatch was not surfaced")

    count_after_failure = call_count()
    time.sleep(0.04)
    assert call_count() == count_after_failure
    with pytest.raises(RuntimeError, match="mismatch"):
        monitor.check()
    monitor.close()


def test_monitor_rejects_reused_snapshot_without_renewing_its_age():
    clock = FakeClock()
    old = snapshot()

    def reused_reader():
        return copy.deepcopy(old)

    monitor = AvoidanceMonitor(
        approved_manifest(), reused_reader, poll_interval_s=0.01, max_age_s=2.0, clock=clock
    )
    monitor.start()
    clock.advance(3.0)
    with pytest.raises(RuntimeError, match="stale|reused"):
        monitor.check()
    monitor.close()


def test_monitor_rejects_a_query_that_took_longer_than_the_age_budget():
    clock = FakeClock()

    def slow_reader():
        value = snapshot()
        value["received_monotonic"] = clock()
        value["query_duration_s"] = 2.1
        return value

    monitor = AvoidanceMonitor(
        approved_manifest(), slow_reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    with pytest.raises(RuntimeError, match="duration"):
        monitor.start()
    with pytest.raises(RuntimeError, match="duration"):
        monitor.check()
    monitor.close()


def test_monitor_measures_full_local_read_duration_even_if_reader_underreports_it():
    clock = FakeClock()

    def slow_but_underreported_reader():
        received = clock.advance(2.1)
        value = snapshot()
        value["received_monotonic"] = received
        value["query_duration_s"] = 0.01
        return value

    monitor = AvoidanceMonitor(
        approved_manifest(),
        slow_but_underreported_reader,
        poll_interval_s=10.0,
        max_age_s=2.0,
        clock=clock,
    )
    with pytest.raises(RuntimeError, match="duration"):
        monitor.start()
    monitor.close()


def test_close_returns_within_its_bound_when_background_read_is_stuck():
    clock = FakeClock()
    second_read_started = threading.Event()
    release_read = threading.Event()
    calls = 0

    def reader():
        nonlocal calls
        calls += 1
        received = clock.advance(0.01)
        if calls == 2:
            second_read_started.set()
            release_read.wait(3.0)
        value = snapshot()
        value["received_monotonic"] = received
        return value

    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=0.01, max_age_s=2.0, clock=clock
    )
    monitor.start()
    try:
        assert second_read_started.wait(1.0)
        started_at = time.monotonic()
        monitor.close()
        assert time.monotonic() - started_at < 1.5
    finally:
        release_read.set()


def test_close_during_thread_start_does_not_join_an_unstarted_thread(monkeypatch):
    clock = FakeClock()
    reader, _, _ = fresh_reader(clock)
    monitor = AvoidanceMonitor(
        approved_manifest(), reader, poll_interval_s=10.0, max_age_s=2.0, clock=clock
    )
    actual_start = threading.Thread.start
    start_entered = threading.Event()
    allow_start = threading.Event()
    close_errors = []
    start_errors = []

    def gated_thread_start(thread):
        if thread.name == "avoidance-manifest-monitor":
            start_entered.set()
            if not allow_start.wait(1.0):
                raise RuntimeError("test did not release monitor start")
        return actual_start(thread)

    monkeypatch.setattr(threading.Thread, "start", gated_thread_start)

    def start_monitor():
        try:
            monitor.start()
        except Exception as exc:
            start_errors.append(exc)

    starter = threading.Thread(target=start_monitor)
    starter.start()
    try:
        assert start_entered.wait(1.0)
        try:
            monitor.close()
        except Exception as exc:
            close_errors.append(exc)
    finally:
        allow_start.set()
        starter.join(timeout=1.0)

    assert not starter.is_alive()
    assert close_errors == []
    assert start_errors

from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.avoidance_comparison import (
    compare_snapshot_to_profile,
    load_snapshot_report,
)
from g1_cup_grasp.avoidance_profile import load_profile


# Frozen copy of the 2026-10-14 read-only capture; do not depend on live outputs.
SNAPSHOT_PATH = ROOT / "tests/fixtures/controller_avoidance_snapshot_20261014.json"
PROFILE_PATH = ROOT / "config/avoidance_profile.example.yaml"


def real_comparison() -> dict:
    return compare_snapshot_to_profile(
        load_snapshot_report(SNAPSHOT_PATH), load_profile(PROFILE_PATH)
    )


def region_by_id(report: dict, range_id: int) -> dict:
    return next(region for region in report["regions"] if region["id"] == range_id)


def test_real_front_regions_preserve_snapshot_boundaries_and_body_transform():
    comparison = real_comparison()
    laser_stop = region_by_id(comparison, 1)["tiers"]["stop"]
    vision_stop = region_by_id(comparison, 6)["tiers"]["stop"]

    assert comparison["status"] == "PROPOSAL_NOT_APPLIED"
    assert comparison["controller_contacted"] is False
    assert comparison["sensor_coverage_verified"] is False
    assert comparison["physical_3d_collision_verified"] is False
    assert comparison["route_clearance_verified"] is False
    assert comparison["read_only"] is True
    assert laser_stop["sensor_local_polygon_mm"] == [
        [50.0, -272.0], [50.0, 260.0], [221.0, 260.0], [221.0, -272.0]
    ]
    assert laser_stop["body_frame_polygon_mm"] == [
        [273.0, -272.0], [273.0, 260.0], [444.0, 260.0], [444.0, -272.0]
    ]
    assert vision_stop["body_frame_bounds_mm"]["x_max"] == pytest.approx(740.0)
    assert vision_stop["body_frame_bounds_mm"]["y_min"] == pytest.approx(-260.0)
    assert vision_stop["body_frame_bounds_mm"]["y_max"] == pytest.approx(260.0)


def test_front_candidates_show_the_sensor_rearward_crossing_and_keep_current_values():
    comparison = real_comparison()
    laser_stop = region_by_id(comparison, 1)["tiers"]["stop"]
    candidate = laser_stop["suggested_candidate"]

    assert candidate["target_profile_level"] == 1
    assert candidate["body_frame_bounds_mm"]["x_min"] == pytest.approx(-300.0)
    assert candidate["body_frame_bounds_mm"]["x_max"] == pytest.approx(1000.0)
    assert candidate["sensor_local_bounds_mm"]["x_min"] == pytest.approx(-523.0)
    assert candidate["sensor_local_bounds_mm"]["x_max"] == pytest.approx(777.0)
    assert candidate["status"] == "BLOCKED_NOT_FOR_APPLICATION"
    assert "crosses_sensor_local_rear" in candidate["blocker_codes"]
    assert laser_stop["sensor_local_polygon_mm"][0][0] == pytest.approx(50.0)
    assert laser_stop["containment_checks"]["full_body_envelope"]["contained"] is False
    assert candidate["candidate_contains_current_polygon"] is True
    assert candidate["current_contains_candidate_polygon"] is False
    assert candidate["axis_aligned_bounds_delta_mm_descriptive_only"]["x_min"] == pytest.approx(-573.0)


def test_rear_sensor_is_not_assigned_the_forward_profile_and_keeps_its_polygon():
    comparison = real_comparison()
    rear = region_by_id(comparison, 7)
    stop = rear["tiers"]["stop"]

    assert rear["design_applicability"] == "NOT_DESIGNED_FOR_REAR_SENSOR"
    assert rear["candidate_generation"] == "PRESERVE_CURRENT_UNSUPPORTED"
    assert stop["sensor_local_polygon_mm"][0][0] == pytest.approx(50.0)
    assert stop["body_frame_bounds_mm"]["x_min"] == pytest.approx(-740.0)
    assert stop["body_frame_bounds_mm"]["x_max"] == pytest.approx(-350.0)
    assert "forward_profile_not_applicable_to_rear_sensor" in rear["blocker_codes"]


def test_rotation_regions_are_not_replaced_by_straight_rectangle_candidates():
    comparison = real_comparison()

    for range_id in (3, 5):
        region = region_by_id(comparison, range_id)
        assert region["design_applicability"] == "UNSUPPORTED_ROTATION_SWEEP"
        assert region["candidate_generation"] == "PRESERVE_CURRENT_UNSUPPORTED"
        assert all(tier["suggested_candidate"] is None for tier in region["tiers"].values())
        assert "rotation_requires_swept_circle" in region["blocker_codes"]


def test_z_ranges_are_kept_raw_and_never_marked_as_height_verified():
    comparison = real_comparison()
    laser_stop = region_by_id(comparison, 1)["tiers"]["stop"]
    vision_stop = region_by_id(comparison, 6)["tiers"]["stop"]
    blocker_codes = {blocker["code"] for blocker in comparison["global_blockers"]}

    assert laser_stop["z_range_raw"] == {"min": 0, "max": 0}
    assert vision_stop["z_range_raw"] == {"min": -180, "max": 180}
    assert all(
        tier["height_coverage_verified"] is False
        for region in comparison["regions"]
        for tier in region["tiers"].values()
    )
    assert "zrange_height_semantics_unverified" in blocker_codes
    assert "rearward_protection_not_designed" in blocker_codes
    assert "rotation_swept_circle_not_modeled" in blocker_codes


def test_convex_polygon_containment_uses_vertices_not_bounding_box():
    report = load_snapshot_report(SNAPSHOT_PATH)
    changed = copy.deepcopy(report)
    parameters = changed["snapshot"]["parameters"]
    laser = parameters["sensorsParam"]["laserList"][0]
    laser.update(x=0, y=0, theta=0)
    stop = next(r for r in parameters["avoidRangesParam"]["avoidRangeList"] if r["id"] == 1)["stop"]
    stop["positions"] = [
        {"x": -300, "y": 0},
        {"x": 250, "y": 525},
        {"x": 800, "y": 0},
        {"x": 250, "y": -525},
    ]

    comparison = compare_snapshot_to_profile(changed, load_profile(PROFILE_PATH))
    stop_result = region_by_id(comparison, 1)["tiers"]["stop"]
    check = stop_result["containment_checks"]["full_body_envelope"]

    assert check["actual_polygon_supported"] is True
    assert check["actual_polygon_bounds_mm"] == {
        "x_min": pytest.approx(-300.0),
        "x_max": pytest.approx(800.0),
        "y_min": pytest.approx(-525.0),
        "y_max": pytest.approx(525.0),
    }
    assert check["contained"] is False


def test_non_convex_region_is_reported_unsupported_instead_of_bbox_containment():
    report = load_snapshot_report(SNAPSHOT_PATH)
    changed = copy.deepcopy(report)
    parameters = changed["snapshot"]["parameters"]
    laser = parameters["sensorsParam"]["laserList"][0]
    laser.update(x=0, y=0, theta=0)
    stop = next(r for r in parameters["avoidRangesParam"]["avoidRangeList"] if r["id"] == 1)["stop"]
    stop["positions"] = [
        {"x": -300, "y": -525},
        {"x": 800, "y": -525},
        {"x": 100, "y": 0},
        {"x": 800, "y": 525},
        {"x": -300, "y": 525},
    ]

    comparison = compare_snapshot_to_profile(changed, load_profile(PROFILE_PATH))
    check = region_by_id(comparison, 1)["tiers"]["stop"]["containment_checks"]["full_body_envelope"]

    assert check["actual_polygon_supported"] is False
    assert check["status"] == "UNSUPPORTED_NON_CONVEX_ACTUAL_POLYGON"
    assert check["contained"] is None


def test_cli_writes_offline_comparison_without_controller_access(capsys):
    from tools.compare_avoidance import main

    with TemporaryDirectory() as directory:
        output = Path(directory) / "comparison.json"
        result = main([
            "--snapshot", str(SNAPSHOT_PATH),
            "--profile", str(PROFILE_PATH),
            "--output", str(output),
        ])
        comparison = json.loads(output.read_text(encoding="utf-8"))
        summary = json.loads(capsys.readouterr().out)

    assert result == 0
    assert comparison["controller_contacted"] is False
    assert comparison["controller_configuration_changed"] is False
    assert comparison["approved_manifest_created"] is False
    assert comparison["status"] == "PROPOSAL_NOT_APPLIED"
    assert comparison["global_blockers"]
    assert summary["blocker_count"] == (
        summary["global_blocker_count"]
        + summary["region_blocker_count"]
        + summary["candidate_blocker_count"]
    )

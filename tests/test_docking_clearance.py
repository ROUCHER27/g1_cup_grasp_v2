import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.docking_clearance import evaluate_docking_clearance


def measured_config():
    return {
        "motion": {
            "speed_m_s": 0.5,
            "reaction_delay_s": 0.2,
            "minimum_braking_deceleration_m_s2": 1.0,
            "error_margin_m": 0.05,
        },
        "stations": {
            "A": {
                "approach_start_clearance_to_nearest_support_m": 1.0,
                "intended_stop_gap_to_nearest_support_m": 0.6,
                "table_edge_gap_at_intended_stop_m": 0.6,
            },
            "B": {
                "approach_start_clearance_to_nearest_support_m": 1.2,
                "intended_stop_gap_to_nearest_support_m": 0.6,
                "table_edge_gap_at_intended_stop_m": 0.7,
            },
        },
        "current_pose_observation": {
            "table_edge_to_chassis_front_gap_m": 0.2,
            "two_table_legs_to_chassis_front_gap_m": 0.4,
        },
        "stationary_trigger_evidence": {
            "stationary_trigger_regions": ["laser_region1"],
            "evidence_note": "synthetic test fixture verification",
        },
    }


def set_value(config, path, value):
    current = config
    for key in path[:-1]:
        current = current[key]
    current[path[-1]] = value


def test_known_inputs_can_pass_geometric_checks_without_claiming_physical_safety():
    report = evaluate_docking_clearance(measured_config())

    assert report["status"] == "GEOMETRIC_CONDITIONS_PASS"
    assert report["geometry_conditions_passed"] is True
    assert report["physical_safety_verified"] is False
    assert report["real_motion_permitted"] is False
    assert report["front_stop_regions"]["laser_region1"]["front_extent_beyond_chassis_m"] == pytest.approx(0.144)
    assert report["front_stop_regions"]["visual_region6"]["front_extent_beyond_chassis_m"] == pytest.approx(0.440)
    assert report["stations"]["A"]["braking"]["required_distance_m"] == pytest.approx(0.275)
    assert report["stations"]["A"]["laser_region1_no_trigger"] is True
    assert report["stations"]["A"]["no_static_stop_zone_intersection"] is True
    assert report["current_pose_observation"]["station_assignment"] == "current_pose_only_not_A_or_B"
    assert report["current_pose_observation"]["visual6_table_edge_x_projected_overlap_m"] == pytest.approx(0.24)
    assert report["stationary_trigger_source_evidence_supplied"] is True
    json.dumps(report, allow_nan=False)


def test_example_null_measurements_remain_blocked_without_safety_defaults():
    config = yaml.safe_load(
        (ROOT / "config/docking_clearance.example.yaml").read_text(encoding="utf-8")
    )

    report = evaluate_docking_clearance(config)

    assert report["status"] == "BLOCKED_MISSING_MEASUREMENTS"
    assert report["geometry_conditions_passed"] is None
    assert report["physical_safety_verified"] is False
    assert report["real_motion_permitted"] is False
    assert {
        "motion.speed_m_s",
        "motion.reaction_delay_s",
        "motion.minimum_braking_deceleration_m_s2",
        "motion.error_margin_m",
        "stations.A.approach_start_clearance_to_nearest_support_m",
        "stations.A.intended_stop_gap_to_nearest_support_m",
        "stations.A.table_edge_gap_at_intended_stop_m",
        "stations.B.approach_start_clearance_to_nearest_support_m",
        "stations.B.intended_stop_gap_to_nearest_support_m",
        "stations.B.table_edge_gap_at_intended_stop_m",
        "stationary_trigger_evidence.stationary_trigger_regions",
    }.issubset(set(report["missing_inputs"]))
    assert report["stations"]["A"]["braking"]["required_distance_m"] is None
    assert any(
        blocker["code"] == "current_pose_table_edge_in_visual_region6_forward_projection"
        for blocker in report["blockers"]
    )


def test_insufficient_approach_runup_fails_the_braking_requirement():
    config = measured_config()
    config["stations"]["A"]["approach_start_clearance_to_nearest_support_m"] = 0.8

    report = evaluate_docking_clearance(config)

    assert report["stations"]["A"]["braking"]["available_distance_m"] == pytest.approx(0.2)
    assert report["stations"]["A"]["braking"]["required_distance_m"] == pytest.approx(0.275)
    assert report["stations"]["A"]["braking"]["passed"] is False
    assert report["stations"]["B"]["braking"]["passed"] is True
    assert report["status"] == "GEOMETRIC_CONDITIONS_FAIL"


def test_stop_gap_at_static_zone_boundary_fails_without_tolerance_relaxation():
    config = measured_config()
    config["stations"]["A"]["intended_stop_gap_to_nearest_support_m"] = 0.194

    report = evaluate_docking_clearance(config)

    assert report["stations"]["A"]["laser1_no_trigger_required_gap_m"] == pytest.approx(0.194)
    assert report["stations"]["A"]["laser_region1_no_trigger"] is False
    assert report["stations"]["A"]["no_static_stop_zone_intersection"] is False
    assert report["status"] == "GEOMETRIC_CONDITIONS_FAIL"


def test_braking_distance_exact_boundary_passes_and_any_shortfall_fails():
    config = measured_config()
    config["stations"]["A"]["approach_start_clearance_to_nearest_support_m"] = 0.875

    at_boundary = evaluate_docking_clearance(config)
    assert at_boundary["stations"]["A"]["braking"]["available_distance_m"] == pytest.approx(0.275)
    assert at_boundary["stations"]["A"]["braking"]["passed"] is True

    config["stations"]["A"]["approach_start_clearance_to_nearest_support_m"] = 0.874999999
    below_boundary = evaluate_docking_clearance(config)
    assert below_boundary["stations"]["A"]["braking"]["passed"] is False


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("motion", "speed_m_s"), True),
        (("motion", "reaction_delay_s"), float("nan")),
        (("motion", "minimum_braking_deceleration_m_s2"), float("inf")),
        (("motion", "error_margin_m"), -0.01),
        (("motion", "reaction_delay_s"), 0),
        (("motion", "error_margin_m"), 0),
        (("stations", "A", "intended_stop_gap_to_nearest_support_m"), False),
        (("stations", "B", "approach_start_clearance_to_nearest_support_m"), float("nan")),
    ],
)
def test_boolean_non_finite_and_negative_inputs_are_rejected(path, value):
    config = measured_config()
    set_value(config, path, value)

    with pytest.raises(ValueError):
        evaluate_docking_clearance(config)


def test_each_station_must_supply_both_geometry_measurements():
    config = measured_config()
    config["stations"]["B"]["intended_stop_gap_to_nearest_support_m"] = None

    report = evaluate_docking_clearance(config)

    assert report["status"] == "BLOCKED_MISSING_MEASUREMENTS"
    assert report["geometry_conditions_passed"] is None
    assert "stations.B.intended_stop_gap_to_nearest_support_m" in report["missing_inputs"]


def test_visual_region_projection_uses_table_edge_layer_not_low_leg_gap():
    config = measured_config()
    config["stations"]["A"]["table_edge_gap_at_intended_stop_m"] = 0.2
    config["stationary_trigger_evidence"]["stationary_trigger_regions"] = [
        "laser_region1",
        "visual_region6",
    ]

    report = evaluate_docking_clearance(config)

    assert report["stations"]["A"]["laser_region1_no_trigger"] is True
    assert report["stations"]["A"]["visual_region6_projected_overlap_m"] == pytest.approx(0.24)
    assert report["stations"]["A"]["visual_region6_no_trigger"] is False
    assert report["stations"]["A"]["no_static_stop_zone_intersection"] is False
    assert report["stations"]["A"]["approach_start_clearance_to_nearest_support_m"] == 1.0


def test_cli_writes_the_offline_example_report_to_requested_path(tmp_path):
    config_path = ROOT / "config/docking_clearance.example.yaml"
    output_path = tmp_path / "clearance-check.json"
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/check_docking_clearance.py"),
            "--config",
            str(config_path),
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(output_path.read_text(encoding="utf-8"))

    assert report["status"] == "BLOCKED_MISSING_MEASUREMENTS"
    assert report["real_motion_permitted"] is False

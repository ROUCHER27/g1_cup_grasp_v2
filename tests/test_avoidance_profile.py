from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.avoidance_profile import (
    build_proposal,
    load_profile,
    transform_polygon_to_sensor_frame,
    validate_profile,
)


EXAMPLE_CONFIG = ROOT / "config" / "avoidance_profile.example.yaml"


def example_data() -> dict:
    return yaml.safe_load(EXAMPLE_CONFIG.read_text(encoding="utf-8"))


def test_example_proposal_contains_full_envelope_and_conservative_turn_radius():
    proposal = build_proposal(load_profile(EXAMPLE_CONFIG))

    assert proposal["status"] == "PROPOSAL_NOT_APPLIED"
    assert proposal["proposal_only"] is True
    assert proposal["read_only"] is True
    assert proposal["measurements_verified"] is False
    assert proposal["sensor_coverage_verified"] is False
    assert proposal["units"] == {"position": "m", "angle": "rad"}
    assert proposal["geometry_frame"].startswith("body_centered_xy_m")
    assert proposal["body_frame_envelope_m"] == {
        "x_min": pytest.approx(-0.30),
        "x_max": pytest.approx(0.80),
        "y_min": pytest.approx(-0.525),
        "y_max": pytest.approx(0.525),
    }
    assert proposal["rotation"]["envelope_radius_m"] == pytest.approx(
        math.hypot(0.80, 0.525)
    )
    assert proposal["rotation"]["conservative_radius_m"] == pytest.approx(
        math.hypot(0.80, 0.525) + 0.20
    )
    assert proposal["rotation"]["clear_diameter_m"] == pytest.approx(
        2 * (math.hypot(0.80, 0.525) + 0.20)
    )


def test_three_travel_zones_expand_forward_and_side_from_full_robot_envelope():
    proposal = build_proposal(load_profile(EXAMPLE_CONFIG))
    zones = proposal["travel_zones"]

    assert [zone["level"] for zone in zones] == [1, 2, 3]
    assert [zone["front_boundary_m"] for zone in zones] == pytest.approx([1.0, 1.2, 1.5])
    assert [zone["half_width_m"] for zone in zones] == pytest.approx([0.725, 0.825, 0.925])
    assert all(zone["rear_boundary_m"] == pytest.approx(-0.30) for zone in zones)
    assert all(zone["reference"] == "full_robot_envelope" for zone in zones)
    assert all(zone["coordinates_frame"] == "body_frame_xy" for zone in zones)


def test_sensor_transform_uses_inverse_sensor_pose_rotation_and_translation():
    polygon = [(2.0, 3.0), (2.0, 4.0), (1.0, 4.0)]
    local = transform_polygon_to_sensor_frame(
        polygon, {"x_m": 1.0, "y_m": 2.0, "yaw_rad": math.pi / 2}
    )

    expected = [(1.0, -1.0), (2.0, -1.0), (2.0, 0.0)]
    assert len(local) == len(expected)
    for actual_point, expected_point in zip(local, expected):
        assert actual_point == pytest.approx(expected_point)


def test_supplied_sensor_pose_produces_local_polygons_without_claiming_coverage():
    data = example_data()
    data["sensors"] = [{"id": "test_sensor", "x_m": 0.0, "y_m": 0.0, "yaw_rad": math.pi / 2}]

    proposal = build_proposal(validate_profile(data))
    sensor = proposal["sensors"][0]
    local_polygon = proposal["travel_zones"][0]["sensor_polygons"][0]["polygon_m"]

    assert sensor["pose_body_frame"] == {
        "x_m": 0.0,
        "y_m": 0.0,
        "yaw_rad": math.pi / 2,
    }
    assert sensor["coverage_verified"] is False
    assert proposal["travel_zones"][0]["sensor_polygons"][0]["coordinates_frame"] == "sensor_local_xy"
    expected = [(-0.725, 0.30), (-0.725, -1.0), (0.725, -1.0), (0.725, 0.30)]
    for actual_point, expected_point in zip(local_polygon, expected):
        assert actual_point == pytest.approx(expected_point)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda data: data["body"].update(length_m=float("nan")), "finite"),
        (lambda data: data["body"].update(length_m=True), "finite"),
        (lambda data: data["body"].update(width_m=0), "positive"),
        (lambda data: data["margins"]["travel_zones"][0].update(forward_m=0), "positive"),
        (lambda data: data["margins"]["travel_zones"][1].update(side_m=0.1), "nested"),
    ],
)
def test_invalid_sizes_margins_and_non_nested_zones_are_rejected(mutate, message):
    data = example_data()
    mutate(data)

    with pytest.raises(ValueError, match=message):
        validate_profile(data)


def test_non_finite_sensor_pose_is_rejected():
    with pytest.raises(ValueError, match="finite"):
        transform_polygon_to_sensor_frame(
            [(0.0, 0.0)], {"x_m": 0.0, "y_m": 0.0, "yaw_rad": float("inf")}
        )


def test_integer_too_large_for_float_is_rejected_as_invalid_number():
    data = example_data()
    data["body"]["length_m"] = 10**10000

    with pytest.raises(ValueError, match="finite"):
        validate_profile(data)


def test_finite_dimensions_that_overflow_derived_envelope_are_rejected():
    data = example_data()
    data["body"]["length_m"] = 1.6e308
    data["body"]["extensions_m"]["front"] = 1.0e308

    with pytest.raises(ValueError, match="finite"):
        build_proposal(validate_profile(data))


def test_finite_dimensions_that_overflow_rotation_radius_are_rejected():
    data = example_data()
    data["body"]["width_m"] = 1.7e308
    data["body"]["extensions_m"]["front"] = 1.7e308

    with pytest.raises(ValueError, match="finite"):
        build_proposal(validate_profile(data))


def test_finite_rotation_margin_that_overflows_clear_diameter_is_rejected():
    data = example_data()
    data["margins"]["rotation_extra_m"] = 1.7e308

    with pytest.raises(ValueError, match="finite"):
        build_proposal(validate_profile(data))


def test_finite_polygon_and_sensor_coordinates_that_overflow_transform_are_rejected():
    polygon = [(1.7e308, 0.0), (1.6e308, 1.0), (1.6e308, -1.0)]

    with pytest.raises(ValueError, match="finite"):
        transform_polygon_to_sensor_frame(
            polygon, {"x_m": -1.7e308, "y_m": 0.0, "yaw_rad": 0.0}
        )


def test_cli_does_not_write_or_print_non_finite_json_for_overflowing_geometry(capsys):
    from tools.plan_avoidance import main

    data = example_data()
    data["body"]["length_m"] = 1.6e308
    data["body"]["extensions_m"]["front"] = 1.0e308
    with TemporaryDirectory() as directory:
        config = Path(directory) / "overflow.yaml"
        output = Path(directory) / "proposal.json"
        config.write_text(yaml.safe_dump(data), encoding="utf-8")

        with pytest.raises(SystemExit) as error:
            main(["--config", str(config), "--output", str(output)])

        assert error.value.code == 2
        assert not output.exists()
        assert capsys.readouterr().out == ""


def test_cli_writes_offline_proposal_without_claiming_sensor_coverage():
    from tools.plan_avoidance import main

    with TemporaryDirectory() as directory:
        output = Path(directory) / "proposal.json"
        result = main(["--config", str(EXAMPLE_CONFIG), "--output", str(output)])
        proposal = json.loads(output.read_text(encoding="utf-8"))

    assert result == 0
    assert proposal["status"] == "PROPOSAL_NOT_APPLIED"
    assert proposal["proposal_only"] is True
    assert proposal["sensor_coverage_verified"] is False
    assert proposal["sensors"] == []

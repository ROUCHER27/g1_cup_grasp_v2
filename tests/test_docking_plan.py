import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1_cup_grasp.docking_plan import create_docking_plan


def sample_inputs():
    transport = {
        "map_id": 7,
        "map_name": "cup_new",
        "stations": {
            "A": {
                "x_m": 0.04899999871850014,
                "y_m": 0.07199999690055847,
                "yaw_rad": -0.1850386778461329,
            },
            "B": {
                "x_m": -1.5700000524520874,
                "y_m": 0.3959999978542328,
                "yaw_rad": 2.976620960236659,
            },
        },
    }
    docking = {
        "map": {"id": 7, "name": "cup_new"},
        "distances": {
            "a_departure_assumption_m": 0.9,
            "b_approach_assumption_m": 0.9,
        },
        "geometry_estimate": {
            "chassis_length_m": 0.6,
            "chassis_width_m": 0.45,
            "arm_side_extension_each_m": 0.3,
            "arm_front_extension_m": 0.5,
            "rotation_margin_m": 0.2,
            "straight_side_margin_m": 0.2,
        },
        "clearance": {
            "minimum_straight_corridor_width_m": 1.45,
            "measured_corridor_width_m": None,
            "measured_rotation_clear_diameter_m": None,
        },
    }
    return docking, transport


class DockingPlanTests(unittest.TestCase):
    def test_plan_is_read_only_and_orders_candidate_docking_phases(self):
        docking, transport = sample_inputs()

        plan = create_docking_plan(docking, transport)

        self.assertEqual(plan["status"], "OFFLINE_PLAN_NOT_EXECUTABLE")
        self.assertEqual(plan["execution"], {
            "mode": "read_only",
            "proposal_only": True,
            "executable": False,
        })
        self.assertEqual(
            [phase["phase"] for phase in plan["phases"]],
            [
                "A_STATIONARY_PICK",
                "A_STRAIGHT_DEPARTURE_CANDIDATE",
                "ROTATE_AND_TRANSPORT_CANDIDATE",
                "B_STRAIGHT_APPROACH_CANDIDATE",
                "B_STATIONARY_PLACE",
            ],
        )
        self.assertAlmostEqual(
            plan["phases"][1]["candidate_pose"]["x_m"], -0.835636223, places=8
        )
        self.assertAlmostEqual(
            plan["phases"][1]["candidate_pose"]["y_m"], 0.237586093, places=8
        )
        self.assertAlmostEqual(
            plan["phases"][3]["candidate_pose"]["x_m"], -0.682219349, places=8
        )
        self.assertAlmostEqual(
            plan["phases"][3]["candidate_pose"]["y_m"], 0.248198030, places=8
        )
        self.assertAlmostEqual(
            plan["phases"][2]["heading_change_deg"], -178.850246481, places=7
        )
        self.assertAlmostEqual(
            plan["clearance"]["conservative_rotation_clear_diameter_m"],
            2.313765920900464,
            places=12,
        )
        self.assertAlmostEqual(
            plan["geometry_estimate"]["combined_envelope_half_width_m"],
            0.525,
            places=12,
        )
        self.assertAlmostEqual(
            plan["station_geometry"]["candidate_a_departure_to_b_approach_distance_m"],
            0.153783454,
            places=8,
        )
        self.assertFalse(
            plan["station_geometry"]["candidate_points_projected_ordered"]
        )
        self.assertTrue(plan["blockers"])
        blocker_codes = {blocker["code"] for blocker in plan["blockers"]}
        self.assertIn("candidate_points_projected_order_inverted", blocker_codes)
        self.assertIn("constrained_straight_motion_unavailable", blocker_codes)
        json.dumps(plan, allow_nan=False)
        self.assertNotIn("commands", plan)
        self.assertNotIn("commands", plan["phases"][0])

    def test_departure_and_approach_candidates_are_behind_their_station_headings(self):
        docking, transport = sample_inputs()
        plan = create_docking_plan(docking, transport)
        a, b = transport["stations"]["A"], transport["stations"]["B"]
        a_departure = plan["phases"][1]["candidate_pose"]
        b_approach = plan["phases"][3]["candidate_pose"]

        a_projection = (
            (a_departure["x_m"] - a["x_m"]) * math.cos(a["yaw_rad"])
            + (a_departure["y_m"] - a["y_m"]) * math.sin(a["yaw_rad"])
        )
        b_projection = (
            (b_approach["x_m"] - b["x_m"]) * math.cos(b["yaw_rad"])
            + (b_approach["y_m"] - b["y_m"]) * math.sin(b["yaw_rad"])
        )

        self.assertAlmostEqual(a_projection, -0.9, places=10)
        self.assertAlmostEqual(b_projection, -0.9, places=10)
        self.assertEqual(
            plan["phases"][3]["motion_requirement"],
            "verified_low_speed_forward_along_B_heading",
        )

    def test_boolean_values_are_not_accepted_as_distances(self):
        docking, transport = sample_inputs()
        docking["distances"]["a_departure_assumption_m"] = True

        with self.assertRaisesRegex(ValueError, "finite number"):
            create_docking_plan(docking, transport)

    def test_non_finite_values_are_rejected(self):
        docking, transport = sample_inputs()
        docking["clearance"]["measured_corridor_width_m"] = float("nan")

        with self.assertRaisesRegex(ValueError, "finite number"):
            create_docking_plan(docking, transport)

    def test_finite_inputs_that_overflow_derived_clearance_are_rejected(self):
        docking, transport = sample_inputs()
        docking["geometry_estimate"]["chassis_length_m"] = 1e308
        docking["geometry_estimate"]["rotation_margin_m"] = 1e308

        with self.assertRaisesRegex(ValueError, "finite"):
            create_docking_plan(docking, transport)

    def test_map_id_and_name_must_match_transport_configuration(self):
        docking, transport = sample_inputs()
        docking["map"]["id"] = 8

        with self.assertRaisesRegex(ValueError, "must match transport map"):
            create_docking_plan(docking, transport)

    def test_map_ids_must_be_positive_integers(self):
        docking, transport = sample_inputs()
        docking["map"]["id"] = 0

        with self.assertRaisesRegex(ValueError, "positive integers"):
            create_docking_plan(docking, transport)

        docking, transport = sample_inputs()
        transport["map_id"] = 0
        with self.assertRaisesRegex(ValueError, "positive integers"):
            create_docking_plan(docking, transport)

    def test_wrong_numeric_types_are_rejected(self):
        docking, transport = sample_inputs()
        transport["stations"]["A"]["x_m"] = "0.049"

        with self.assertRaisesRegex(ValueError, "finite number"):
            create_docking_plan(docking, transport)

    def test_zero_or_negative_candidate_distances_are_rejected(self):
        docking, transport = sample_inputs()
        docking["distances"]["b_approach_assumption_m"] = 0

        with self.assertRaisesRegex(ValueError, "greater than zero"):
            create_docking_plan(docking, transport)

    def test_measured_thresholds_remove_unmeasured_clearance_blockers_only(self):
        docking, transport = sample_inputs()
        docking["clearance"]["measured_corridor_width_m"] = 1.45
        docking["clearance"]["measured_rotation_clear_diameter_m"] = 2.313765920900464

        plan = create_docking_plan(docking, transport)

        codes = {blocker["code"] for blocker in plan["blockers"]}
        self.assertNotIn("site_corridor_unmeasured", codes)
        self.assertNotIn("site_rotation_clearance_unmeasured", codes)
        self.assertIn("route_not_collision_checked", codes)
        self.assertFalse(plan["clearance"]["site_clearance_verified"])

    def test_minimum_corridor_width_includes_full_envelope_and_side_margins(self):
        docking, transport = sample_inputs()
        plan = create_docking_plan(docking, transport)

        self.assertAlmostEqual(
            plan["geometry_estimate"]["combined_envelope_width_m"], 1.05, places=12
        )
        self.assertAlmostEqual(
            plan["clearance"]["straight_side_margin_each_m"], 0.2, places=12
        )
        self.assertAlmostEqual(
            plan["clearance"]["derived_minimum_corridor_width_m"], 1.45, places=12
        )

        docking["clearance"]["minimum_straight_corridor_width_m"] = 1.05
        with self.assertRaisesRegex(ValueError, "at least the derived minimum"):
            create_docking_plan(docking, transport)

    def test_measured_width_cannot_pass_by_omitting_occupied_envelope_margins(self):
        docking, transport = sample_inputs()
        docking["clearance"]["measured_corridor_width_m"] = 1.05
        docking["clearance"]["measured_rotation_clear_diameter_m"] = 2.313765920900464

        plan = create_docking_plan(docking, transport)

        self.assertFalse(plan["clearance"]["measured_minimum_thresholds_met"])
        blocker_codes = {blocker["code"] for blocker in plan["blockers"]}
        self.assertIn("measured_corridor_below_requirement", blocker_codes)

    def test_cli_reads_example_and_transport_config_and_writes_json_plan(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "docking-plan.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(root / "tools/plan_docking.py"),
                    "--output",
                    str(output),
                ],
                cwd=root,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            plan = json.loads(output.read_text(encoding="utf-8"))

        self.assertEqual(plan["status"], "OFFLINE_PLAN_NOT_EXECUTABLE")
        self.assertEqual(plan["map"]["id"], 7)
        self.assertEqual(plan["map"]["name"], "cup_new")
        self.assertEqual(plan["execution"]["executable"], False)


if __name__ == "__main__":
    unittest.main()

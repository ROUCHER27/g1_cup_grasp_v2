import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1_cup_grasp.config import load_config
from g1_cup_grasp.domain import EndPose, Target3D
from g1_cup_grasp.grasp_planner import plan_side_grasp, result_to_dict
from g1_cup_grasp.kinematics import IkResult, UrdfRightArm


class FakeModel:
    def __init__(self, grasp_failure=False):
        self.grasp_failure = grasp_failure

    def solve_with_restarts(self, pose, seed, waist, stage):
        success = not (stage == "clearance" and pose.position_m[2] > 0.67)
        success = success and not (self.grasp_failure and stage == "grasp")
        reason = "ok" if success else "position error 40.0 mm"
        result = IkResult(np.asarray(seed) + 0.01, 0 if success else 0.04, 0, success, reason)
        return result, [{"success": success, "reason": reason}]


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(load_config(Path(__file__).resolve().parents[1] / "config/grasp_v2.yaml")["grasp"])
        self.config["yaw_offsets_deg"] = [0]
        self.target = Target3D((0, 0), 0.5, np.zeros(3), np.array([0.75, -0.25, 0.60]), 0.9, 30)

    def test_alternative_route_and_failure_details(self):
        plan = plan_side_grasp(self.target, np.zeros(7), np.zeros(2), FakeModel(), self.config)
        self.assertEqual(plan.status, "PLAN_READY")
        self.assertEqual(len(plan.candidates), 3)
        first = plan.candidates[0]
        self.assertEqual(first.failure_stage, "clearance")
        self.assertEqual(first.stage_checks[0]["status"], "passed")
        self.assertEqual(first.stage_checks[-1]["status"], "not_checked")
        self.assertEqual(plan.selected.clearance_height_m, 0.10)
        self.assertEqual(len(plan.selected.waypoints), 6)
        json.dumps(result_to_dict(plan))

    def test_transport_does_not_reject_grasp_for_unused_retreat(self):
        class RetreatBlocked(FakeModel):
            def solve_with_restarts(self,pose,seed,waist,stage):
                if stage=="retreat":
                    return IkResult(np.asarray(seed),0,0,False,"solution is too close to a joint limit"),[]
                return super().solve_with_restarts(pose,seed,waist,stage)
        args=(self.target,np.zeros(7),np.zeros(2),RetreatBlocked(),self.config)
        self.assertEqual(plan_side_grasp(*args).status,"NO_FEASIBLE_GRASP")
        plan=plan_side_grasp(*args,include_return=False)
        self.assertEqual(plan.status,"PLAN_READY")
        self.assertEqual([w.stage for w in plan.selected.waypoints],["clearance","pregrasp","grasp","lift"])

    def test_grasp_failure_is_distinguished(self):
        plan = plan_side_grasp(self.target, np.zeros(7), np.zeros(2), FakeModel(True), self.config)
        self.assertEqual(plan.status, "NO_FEASIBLE_GRASP")
        self.assertEqual(plan.candidates[1].failure_stage, "grasp")
        self.assertEqual(plan.candidates[1].stage_checks[0]["status"], "no_solution_found")

    def test_failed_direct_grasp_check_does_not_reject_route_solution(self):
        class RouteSeedModel(FakeModel):
            def solve_with_restarts(self, pose, seed, waist, stage):
                self.grasp_failure = stage == "grasp" and np.allclose(seed, 0)
                return super().solve_with_restarts(pose, seed, waist, stage)
        plan = plan_side_grasp(self.target, np.zeros(7), np.zeros(2), RouteSeedModel(), self.config)
        self.assertEqual(plan.status, "PLAN_READY")
        self.assertEqual(plan.selected.stage_checks[0]["status"], "no_solution_found")

    def test_safety_gap_is_not_relaxed(self):
        self.config["route_options"] = [{"pregrasp_distance_m": 0.06, "clearance_height_m": 0.10}]
        plan = plan_side_grasp(self.target, np.zeros(7), np.zeros(2), FakeModel(), self.config)
        self.assertEqual(plan.status, "NO_FEASIBLE_GRASP")
        self.assertIn("safety gap", plan.candidates[0].reason)

    def test_workspace_and_legacy_config(self):
        self.config.pop("route_options")
        plan = plan_side_grasp(self.target, np.zeros(7), np.zeros(2), FakeModel(), self.config)
        self.assertEqual(len(plan.candidates), 1)
        target = Target3D((0, 0), 0.5, np.zeros(3), np.array([2, 0, 0.6]), 0.9, 30)
        self.assertEqual(plan_side_grasp(target, np.zeros(7), np.zeros(2), FakeModel(), self.config).status,
                         "NAVIGATION_REQUIRED")

    def test_ik_restarts_stop_at_success_and_preserve_failures(self):
        class RestartModel:
            lower = np.full(7, -2.0)
            upper = np.full(7, 2.0)
            joint_names = [str(i) for i in range(7)]
            limit_margin = 0.03
            position_tolerance_m = 0.01
            orientation_tolerance_deg = 5
            stage_position_tolerance_m = {}
            stage_orientation_tolerance_deg = {}
            calls = 0
            succeed = True

            def solve(self, target, seed, waist, stage):
                self.calls += 1
                ok = self.succeed and self.calls == 2
                return IkResult(np.zeros(7), 0 if ok else .04, 0, ok, "ok" if ok else "position error")
        model = RestartModel()
        pose = EndPose(np.zeros(3), np.zeros(3))
        result, attempts = UrdfRightArm.solve_with_restarts(model, pose, np.zeros(7), [], "grasp")
        self.assertTrue(result.success)
        self.assertEqual(len(attempts), 2)
        self.assertEqual(attempts[0]["position_tolerance_mm"], 10)
        model.calls = 0
        model.succeed = False
        result, attempts = UrdfRightArm.solve_with_restarts(model, pose, np.zeros(7), [], "grasp")
        self.assertFalse(result.success)
        self.assertEqual(len(attempts), 3)


if __name__ == "__main__":
    unittest.main()

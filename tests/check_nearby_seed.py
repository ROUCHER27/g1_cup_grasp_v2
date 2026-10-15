"""Offline assert-based check; never connects to the robot."""
import copy
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from g1_cup_grasp.config import load_config
from g1_cup_grasp.domain import Target3D
from g1_cup_grasp.grasp_planner import load_nearby_seed, plan_side_grasp, result_to_dict, seed_context
from g1_cup_grasp.kinematics import IkResult


class FakeModel:
    def __init__(self, warm_failure=False):
        self.warm_failure = warm_failure
        self.warm_calls = 0
        self.fallback_calls = 0

    def solve(self, pose, seed, waist, stage):
        self.warm_calls += 1
        return IkResult(np.asarray(seed), 0, 0, not self.warm_failure, "mock")

    def solve_with_restarts(self, pose, seed, waist, stage):
        self.fallback_calls += 1
        return IkResult(np.asarray(seed), 0, 0, True, "mock"), []


def check():
    config = load_config(ROOT / "config/grasp_v2.yaml")
    grasp = copy.deepcopy(config["grasp"])
    grasp["yaw_offsets_deg"] = [0]
    grasp["route_options"] = [{"pregrasp_distance_m": .08, "clearance_height_m": .10}]
    target = Target3D((0, 0), .5, np.zeros(3), np.array([.75, -.25, .60]), .9, 30)
    waist = np.zeros(2)
    plan = plan_side_grasp(target, np.zeros(7), waist, FakeModel(), grasp)
    assert plan.status == "PLAN_READY"
    candidate = result_to_dict(plan)["candidates"][0]
    cache = {"context": seed_context(config), "anchor_m": target.point_base_m.tolist(),
             "waist_urdf": waist.tolist(), "candidate": candidate}
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "seed.json"
        path.write_text(json.dumps(cache))
        assert load_nearby_seed(path, target, waist, config) is not None
        far = Target3D((0, 0), .5, np.zeros(3), target.point_base_m + [.051, 0, 0], .9, 30)
        assert load_nearby_seed(path, far, waist, config) is None
        assert load_nearby_seed(path, target, waist + .002, config) is None
        changed = copy.deepcopy(config)
        changed["grasp"]["base_rpy_rad"][0] += .01
        assert load_nearby_seed(path, target, waist, changed) is None
        path.write_text("{}")
        assert load_nearby_seed(path, target, waist, config) is None
    for failure in (False, True):
        model = FakeModel(failure)
        result = plan_side_grasp(target, np.zeros(7), waist, model, grasp, nearby_seed=candidate)
        assert result.status == "PLAN_READY" and model.warm_calls == 7
        assert model.fallback_calls == (7 if failure else 0)
    print("PASS: nearby seed eligibility and fallback; no hardware commands")


if __name__ == "__main__":
    check()

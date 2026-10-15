from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .domain import EndPose, Target3D
from .kinematics import UrdfRightArm


@dataclass(frozen=True)
class PlannedWaypoint:
    stage: str
    pose: EndPose
    joints_rad: np.ndarray
    position_error_m: float
    orientation_error_deg: float


@dataclass(frozen=True)
class GraspCandidate:
    yaw_offset_deg: float
    feasible: bool
    score: float | None
    reason: str
    waypoints: tuple[PlannedWaypoint, ...]
    pregrasp_distance_m: float = 0.0
    clearance_height_m: float = 0.0
    failure_stage: str | None = None
    stage_checks: tuple[dict[str, Any], ...] = tuple()


@dataclass(frozen=True)
class GraspPlanningResult:
    status: str
    target: Target3D
    candidates: tuple[GraspCandidate, ...]
    selected_index: int | None

    @property
    def selected(self) -> GraspCandidate | None:
        return None if self.selected_index is None else self.candidates[self.selected_index]


def _positive(config: dict[str, Any], key: str) -> float:
    value = float(config[key])
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"grasp.{key} must be positive and finite")
    return value


def _inside_workspace(point: np.ndarray, workspace: dict[str, Any]) -> bool:
    x, y, z = point
    return (
        float(workspace["x_min_m"]) <= x <= float(workspace["x_max_m"])
        and float(workspace["y_min_m"]) <= y <= float(workspace["y_max_m"])
        and float(workspace["z_min_m"]) <= z <= float(workspace["z_max_m"])
    )


def _candidate_poses(anchor: np.ndarray, yaw_deg: float, config: dict[str, Any]) -> list[tuple[str, EndPose]]:
    cup_height = _positive(config, "cup_height_m")
    cup_diameter = _positive(config, "cup_diameter_m")
    pregrasp_distance = _positive(config, "pregrasp_distance_m")
    surface_gap = _positive(config, "min_surface_gap_m")
    clearance_height = _positive(config, "clearance_height_m")
    lift_height = _positive(config, "trial_lift_m")
    if pregrasp_distance < cup_diameter / 2 + surface_gap:
        raise ValueError("pregrasp distance is inside the cup safety gap")

    base_rotation = Rotation.from_euler("xyz", config["base_rpy_rad"])
    rotation = Rotation.from_euler("z", yaw_deg, degrees=True) * base_rotation
    rpy = rotation.as_euler("xyz")
    forward = rotation.apply([0.0, 0.0, 1.0])
    horizontal = float(np.linalg.norm(forward[:2]))
    if horizontal < 1e-6 or abs(float(forward[2])) > 0.35:
        raise ValueError("candidate gripper +Z axis is not a side approach")
    approach = np.array([forward[0] / horizontal, forward[1] / horizontal, 0.0])

    midpoint = anchor - np.array([0.0, 0.0, cup_height / 2.0])
    effective_offset = np.asarray(config["anchor_to_link7_offset_m"], dtype=float).reshape(3)
    grasp = midpoint - rotation.apply(effective_offset)
    pregrasp = grasp - pregrasp_distance * approach
    clearance = pregrasp + np.array([0.0, 0.0, clearance_height])
    lift = grasp + np.array([0.0, 0.0, lift_height])
    return [
        ("clearance", EndPose(clearance, rpy, "base_link")),
        ("pregrasp", EndPose(pregrasp, rpy, "base_link")),
        ("grasp", EndPose(grasp, rpy, "base_link")),
        ("lift", EndPose(lift, rpy, "base_link")),
        ("return", EndPose(grasp.copy(), rpy.copy(), "base_link")),
        ("retreat", EndPose(pregrasp.copy(), rpy.copy(), "base_link")),
    ]


def seed_context(config: dict[str, Any]) -> dict[str, Any]:
    return {key: config[key] for key in ("robot", "camera", "calibration", "grasp", "ik")}


def load_nearby_seed(path: Path, target: Target3D, waist: Any,
                     config: dict[str, Any]) -> dict[str, Any] | None:
    try:
        cache = json.loads(path.read_text(encoding="utf-8"))
        anchor = np.asarray(cache["anchor_m"], dtype=float)
        cached_waist = np.asarray(cache["waist_urdf"], dtype=float)
        if (cache["context"] != seed_context(config) or anchor.shape != (3,)
                or not np.all(np.isfinite(anchor))
                or cached_waist.shape != np.asarray(waist).shape
                or not np.all(np.isfinite(cached_waist))
                or np.linalg.norm(cached_waist - waist) > 0.001
                or np.linalg.norm(anchor - target.point_base_m) > 0.05):
            return None
        candidate = cache["candidate"]
        if not candidate["feasible"] or not np.isfinite(candidate["yaw_offset_deg"]):
            return None
        stages = {item["stage"] for item in candidate["waypoints"]}
        if stages != {"clearance", "pregrasp", "grasp", "lift", "return", "retreat"}:
            return None
        for item in candidate["waypoints"]:
            joints = np.asarray(item["right_arm_joint_rad"], dtype=float)
            if joints.shape != (7,) or not np.all(np.isfinite(joints)):
                return None
        return candidate
    except (OSError, ValueError, TypeError, KeyError):
        return None


def save_ik_seed(path: Path, plan: GraspPlanningResult, waist: Any,
                 config: dict[str, Any]) -> None:
    if plan.selected is None:
        return
    data = result_to_dict(plan)
    cache = {"context": seed_context(config), "anchor_m": plan.target.point_base_m.tolist(),
             "waist_urdf": np.asarray(waist).tolist(),
             "candidate": data["candidates"][plan.selected_index]}
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(cache, indent=2), encoding="utf-8")
    temporary.replace(path)


def plan_side_grasp(
    target: Target3D,
    current_right_joints: Any,
    waist_urdf: Any,
    model: UrdfRightArm,
    config: dict[str, Any],
    nearby_seed: dict[str, Any] | None = None,
    include_return: bool = True,
) -> GraspPlanningResult:
    anchor = np.asarray(target.point_base_m, dtype=float).reshape(3)
    workspace = config["workspace"]
    if not _inside_workspace(anchor, workspace):
        return GraspPlanningResult("NAVIGATION_REQUIRED", target, tuple(), None)

    current = np.asarray(current_right_joints, dtype=float).reshape(7)
    candidates: list[GraspCandidate] = []
    routes = config.get("route_options", [{
        "pregrasp_distance_m": config["pregrasp_distance_m"],
        "clearance_height_m": config["clearance_height_m"],
    }])
    if not routes:
        raise ValueError("grasp.route_options must not be empty")
    for yaw in config["yaw_offsets_deg"]:
        yaw_deg = float(yaw)
        preferred = {}
        if nearby_seed is not None and abs(yaw_deg - nearby_seed["yaw_offset_deg"]) < 1e-6:
            preferred = {item["stage"]: np.asarray(item["right_arm_joint_rad"], dtype=float)
                         for item in nearby_seed["waypoints"]}

        def solve_pose(pose: EndPose, seed: np.ndarray, stage: str):
            if stage in preferred:
                result = model.solve(pose, preferred[stage], waist_urdf, stage=stage)
                warm_attempt = {"source": "nearby_plan_seed", "success": result.success,
                                "reason": result.reason,
                                "position_error_mm": result.position_error_m * 1000.0,
                                "orientation_error_deg": result.orientation_error_deg}
                if result.success:
                    return result, [warm_attempt]
                result, attempts = model.solve_with_restarts(pose, seed, waist_urdf, stage)
                return result, [warm_attempt, *attempts]
            return model.solve_with_restarts(pose, seed, waist_urdf, stage)
        # Grasp feasibility is independent of the chosen intermediate route.
        try:
            poses = _candidate_poses(anchor, yaw_deg, config)
            grasp_pose = dict(poses)["grasp"]
            if not _inside_workspace(grasp_pose.position_m, workspace):
                candidates.append(GraspCandidate(yaw_deg, False, None,
                    "grasp: waypoint outside workspace", tuple(), failure_stage="grasp",
                    stage_checks=({"stage": "grasp", "phase": "grasp_reachability",
                                   "status": "outside_workspace"},)))
                continue
            grasp_result, attempts = solve_pose(grasp_pose, current, "grasp")
            grasp_check = {"stage": "grasp", "phase": "grasp_reachability",
                           "status": "passed" if grasp_result.success else "no_solution_found",
                           "position_m": grasp_pose.position_m.tolist(),
                           "rpy_rad": grasp_pose.rpy_rad.tolist(),
                           "attempts": attempts}
            # A failed solve from current joints does not rule out a route-seeded solution.
        except (KeyError, TypeError, ValueError) as exc:
            candidates.append(GraspCandidate(yaw_deg, False, None, str(exc), tuple(),
                                             failure_stage="configuration"))
            continue
        for route in routes:
            route_config = dict(config, pregrasp_distance_m=route["pregrasp_distance_m"],
                                clearance_height_m=route["clearance_height_m"])
            distance = _positive(route_config, "pregrasp_distance_m")
            height = _positive(route_config, "clearance_height_m")
            try:
                poses = _candidate_poses(anchor, yaw_deg, route_config)
                if not include_return:
                    poses = poses[:4]
            except (KeyError, TypeError, ValueError) as exc:
                candidates.append(GraspCandidate(yaw_deg, False, None, str(exc), tuple(),
                                                 distance, height, "configuration", (grasp_check,)))
                continue
            seed = current.copy()
            solved: list[PlannedWaypoint] = []
            failure = None
            failure_stage = None
            checks = [grasp_check]
            for stage, pose in poses:
                check = {"stage": stage, "phase": "route",
                         "position_m": pose.position_m.tolist(), "rpy_rad": pose.rpy_rad.tolist()}
                if failure is not None:
                    checks.append(dict(check, status="not_checked", reason="earlier stage failed"))
                    continue
                if not _inside_workspace(pose.position_m, workspace):
                    failure = f"{stage}: waypoint outside workspace"
                    failure_stage = stage
                    checks.append(dict(check, status="outside_workspace"))
                    continue
                result, attempts = solve_pose(pose, seed, stage)
                checks.append(dict(check, status="passed" if result.success else "failed", attempts=attempts))
                if not result.success:
                    failure = f"{stage}: {result.reason}"
                    failure_stage = stage
                    continue
                solved.append(PlannedWaypoint(
                    stage, pose, result.joints_rad,
                    result.position_error_m, result.orientation_error_deg,
                ))
                seed = result.joints_rad
            if failure is not None:
                candidates.append(GraspCandidate(yaw_deg, False, None, failure, tuple(solved),
                                                 distance, height, failure_stage, tuple(checks)))
                continue

            movement = sum(float(np.linalg.norm(
                waypoint.joints_rad - (current if index == 0 else solved[index - 1].joints_rad)
            )) for index, waypoint in enumerate(solved))
            score = movement + abs(yaw_deg) / 180.0
            candidates.append(GraspCandidate(yaw_deg, True, score, "ok", tuple(solved),
                                             distance, height, None, tuple(checks)))

    feasible = [(index, candidate) for index, candidate in enumerate(candidates) if candidate.feasible]
    if not feasible:
        return GraspPlanningResult("NO_FEASIBLE_GRASP", target, tuple(candidates), None)
    selected_index = min(feasible, key=lambda item: float(item[1].score))[0]
    return GraspPlanningResult("PLAN_READY", target, tuple(candidates), selected_index)


def result_to_dict(result: GraspPlanningResult) -> dict[str, Any]:
    def waypoint_data(waypoint: PlannedWaypoint) -> dict[str, Any]:
        return {
            "stage": waypoint.stage,
            "position_m": waypoint.pose.position_m.tolist(),
            "rpy_rad": waypoint.pose.rpy_rad.tolist(),
            "right_arm_joint_rad": waypoint.joints_rad.tolist(),
            "ik_position_error_mm": waypoint.position_error_m * 1000.0,
            "ik_orientation_error_deg": waypoint.orientation_error_deg,
        }

    return {
        "status": result.status,
        "selected_index": result.selected_index,
        "validation_scope": "waypoint_ik_only; interpolated paths and platform collisions not checked",
        "candidates": [
            {
                "yaw_offset_deg": candidate.yaw_offset_deg,
                "feasible": candidate.feasible,
                "score": candidate.score,
                "reason": candidate.reason,
                "failure_stage": candidate.failure_stage,
                "pregrasp_distance_m": candidate.pregrasp_distance_m,
                "clearance_height_m": candidate.clearance_height_m,
                "stage_checks": list(candidate.stage_checks),
                "waypoints": [waypoint_data(item) for item in candidate.waypoints],
            }
            for candidate in result.candidates
        ],
    }

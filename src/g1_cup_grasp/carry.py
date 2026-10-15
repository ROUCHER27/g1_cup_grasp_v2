"""Kinematic checks for upright cup transport; no robot commands."""
from __future__ import annotations

import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .domain import EndPose


def pose_data(pose):
    return {"position_m": pose.position_m.tolist(), "rpy_rad": pose.rpy_rad.tolist()}


def pose_from_data(data):
    return EndPose(np.asarray(data["position_m"], dtype=float), np.asarray(data["rpy_rad"], dtype=float))


def check_path(model, start, targets, seed, waist, cup_up_in_tip=None):
    q = np.asarray(seed, dtype=float)
    current = start
    rows = []
    for stage, target in targets:
        rotations = Rotation.from_euler("xyz", [current.rpy_rad, target.rpy_rad])
        slerp = Slerp([0, 1], rotations)
        distance = np.linalg.norm(target.position_m - current.position_m)
        rotation = (rotations[0].inv() * rotations[1]).magnitude()
        count = max(3, int(math.ceil(distance / .01)), int(math.ceil(rotation / math.radians(3))))
        for u in np.linspace(0, 1, count + 1):
            pose = EndPose((1-u)*current.position_m + u*target.position_m,
                           slerp([u])[0].as_euler("xyz"))
            result = model.solve(pose, q, waist, stage=stage)
            if not result.success:
                raise RuntimeError(f"{stage} path at {u:.3f}: {result.reason}")
            step = float(np.max(np.abs(result.joints_rad - q)))
            if step > .35:
                raise RuntimeError(f"{stage} IK branch changed by {step:.3f} rad")
            q = result.joints_rad
            margin = float(np.min(np.minimum(q-model.lower, model.upper-q)))
            tilt = 0.0
            if cup_up_in_tip is not None:
                up = Rotation.from_euler("xyz", pose.rpy_rad).apply(cup_up_in_tip)
                tilt = math.degrees(math.acos(float(np.clip(up[2], -1, 1))))
                if tilt > 5.0:
                    raise RuntimeError(f"{stage} predicted cup tilt {tilt:.2f} degrees")
            rows.append({"stage":stage, "fraction":float(u), **pose_data(pose),
                         "right_arm_joint_rad":q.tolist(), "joint_margin_rad":margin,
                         "position_error_mm":result.position_error_m*1000,
                         "orientation_error_deg":result.orientation_error_deg,
                         "cup_tilt_deg":tilt})
        current = target
    return rows


def plan_carry(model, lifted_pose, seed, waist, config, grasp_pose=None, cup_top=None, cup_height=.105):
    xyz = lifted_pose.position_m.copy()
    xyz[2] = max(xyz[2], float(config["lift_height_m"]))
    raised = EndPose(xyz.copy(), lifted_pose.rpy_rad.copy())
    if "rpy_rad" in config:
        rpy = np.asarray(config["rpy_rad"], dtype=float)
    else:
        rpy = lifted_pose.rpy_rad.copy()
        rpy[2] = float(config["yaw_rad"])
    final = EndPose(np.asarray(config["position_m"],dtype=float),rpy)
    cup_up = Rotation.from_euler("xyz", lifted_pose.rpy_rad).inv().apply([0,0,1])
    targets = [("carry",raised)]
    if "rpy_rad" not in config:
        xyz[1] = float(config["side_y_m"])
        targets.append(("carry",EndPose(xyz.copy(), lifted_pose.rpy_rad.copy())))
    targets.append(("carry",final))
    rows = check_path(model,lifted_pose,targets,seed,waist,cup_up)
    result = {"waypoints":[pose_data(pose) for _,pose in targets], "samples":rows,
              "kinematics_verified":True, "physical_tracking_verified":False,
              "collision_model_verified":False, "final_pose":pose_data(final)}
    if cup_top is not None and grasp_pose is not None:
        center = np.asarray(cup_top) - np.array([0,0,cup_height/2])
        offset = Rotation.from_euler("xyz",grasp_pose.rpy_rad).inv().apply(center-grasp_pose.position_m)
        result["cup_center_in_tip_m"] = offset.tolist()
        result["final_cup_center_m"] = (final.position_m + Rotation.from_euler("xyz",final.rpy_rad).apply(offset)).tolist()
    return result


def footprint(model, left_model, right_q, left_q, waist, carry_plan, cup_radius=.04):
    right = model.link_positions(right_q,waist)[:,:2]
    left = left_model.link_positions(left_q,waist)[:,:2]
    points = np.vstack([right,left])
    radius = float(np.max(np.linalg.norm(points,axis=1))) + .07
    cup = carry_plan.get("final_cup_center_m")
    if cup is not None:
        radius = max(radius,float(np.linalg.norm(cup[:2]))+cup_radius+.05)
    return {"approximate_turning_radius_m":radius, "minimum_turning_clear_diameter_m":2*radius,
            "scope":"link-center estimate with 70mm padding; meshes, scene and planner footprint not verified",
            "left_tip_m":left_model.forward(left_q,waist)[:3,3].tolist(),
            "right_tip_m":model.forward(right_q,waist)[:3,3].tolist()}


def require_grip(state, config):
    values = state.gripper_state
    low, high = config["gripper_feedback_range"]
    if len(values)!=2 or not math.isfinite(values[1]) or not low <= values[1] <= high:
        raise RuntimeError(f"right gripper is open/empty or outside validated cup range: {values}")
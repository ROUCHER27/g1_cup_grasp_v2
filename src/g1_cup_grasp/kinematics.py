from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .domain import EndPose
from .geometry.transforms import compose_transform, rpy_to_matrix


@dataclass(frozen=True)
class JointSpec:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass(frozen=True)
class IkResult:
    joints_rad: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    success: bool
    reason: str


def _vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    return np.fromstring(text, sep=" ", dtype=float) if text else np.asarray(default, dtype=float)


def _motion_transform(joint: JointSpec, value: float) -> np.ndarray:
    result = np.eye(4, dtype=float)
    if joint.kind in {"revolute", "continuous"}:
        result[:3, :3] = Rotation.from_rotvec(joint.axis * value).as_matrix()
    elif joint.kind == "prismatic":
        result[:3, 3] = joint.axis * value
    return result


class UrdfRightArm:
    """Minimal URDF FK/IK for the fixed-waist G1 right arm."""

    def __init__(self, path: str | Path, base_link: str, tip_link: str, config: dict[str, Any], arm_joint_prefix: str = "idx6"):
        urdf_path = Path(path)
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        root = ET.parse(urdf_path).getroot()
        by_child: dict[str, JointSpec] = {}
        for element in root.findall("joint"):
            parent = element.find("parent").attrib["link"]
            child = element.find("child").attrib["link"]
            origin_element = element.find("origin")
            xyz = _vector(None if origin_element is None else origin_element.attrib.get("xyz"), (0, 0, 0))
            rpy = _vector(None if origin_element is None else origin_element.attrib.get("rpy"), (0, 0, 0))
            axis_element = element.find("axis")
            axis = _vector(None if axis_element is None else axis_element.attrib.get("xyz"), (0, 0, 1))
            norm = float(np.linalg.norm(axis))
            if norm > 0:
                axis = axis / norm
            limit = element.find("limit")
            kind = element.attrib["type"]
            lower = float(limit.attrib.get("lower", -np.pi)) if limit is not None else 0.0
            upper = float(limit.attrib.get("upper", np.pi)) if limit is not None else 0.0
            if kind == "continuous":
                lower, upper = -np.pi, np.pi
            by_child[child] = JointSpec(
                name=element.attrib["name"], kind=kind, parent=parent, child=child,
                origin=compose_transform(rpy_to_matrix(rpy), xyz), axis=axis,
                lower=lower, upper=upper,
            )

        chain: list[JointSpec] = []
        link = tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"No URDF chain from {base_link!r} to {tip_link!r}")
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        self.chain = list(reversed(chain))
        self.arm_joints = [joint for joint in self.chain if joint.name.startswith(arm_joint_prefix)]
        if len(self.arm_joints) != 7:
            raise ValueError(f"Expected 7 right-arm joints, found {[j.name for j in self.arm_joints]}")
        self.waist_joints = [joint for joint in self.chain if joint.name.startswith("idx0")]
        self.lower = np.array([joint.lower for joint in self.arm_joints], dtype=float)
        self.upper = np.array([joint.upper for joint in self.arm_joints], dtype=float)
        self.base_alignment = compose_transform(
            np.eye(3), config.get("urdf_to_gdk_translation_m", [0.0, 0.0, 0.0])
        )
        self.max_nfev = int(config.get("max_nfev", 500))
        self.position_tolerance_m = float(config.get("position_tolerance_m", 0.015))
        self.orientation_tolerance_deg = float(config.get("orientation_tolerance_deg", 5.0))
        self.stage_position_tolerance_m = dict(config.get("stage_position_tolerance_m", {}))
        self.stage_orientation_tolerance_deg = dict(config.get("stage_orientation_tolerance_deg", {}))
        self.seed_regularization = float(config.get("seed_regularization", 0.02))
        self.limit_margin = float(config.get("joint_limit_margin_rad", 0.03))

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.arm_joints]

    def forward(self, arm_joints: Any, waist_urdf: Any) -> np.ndarray:
        arm = np.asarray(arm_joints, dtype=float).reshape(7)
        waist = np.asarray(waist_urdf, dtype=float).reshape(len(self.waist_joints))
        values = {joint.name: float(value) for joint, value in zip(self.arm_joints, arm)}
        values.update({joint.name: float(value) for joint, value in zip(self.waist_joints, waist)})
        transform = self.base_alignment.copy()
        for joint in self.chain:
            transform = transform @ joint.origin
            transform = transform @ _motion_transform(joint, values.get(joint.name, 0.0))
        return transform

    def link_positions(self, arm_joints: Any, waist_urdf: Any) -> np.ndarray:
        values = {joint.name: float(value) for joint, value in zip(self.arm_joints, arm_joints)}
        values.update({joint.name: float(value) for joint, value in zip(self.waist_joints, waist_urdf)})
        transform = self.base_alignment.copy()
        points = []
        for joint in self.chain:
            transform = transform @ joint.origin @ _motion_transform(joint, values.get(joint.name, 0.0))
            points.append(transform[:3, 3].copy())
        return np.asarray(points)
    @staticmethod
    def pose_errors(transform: np.ndarray, target: EndPose) -> tuple[float, float]:
        position_error = float(np.linalg.norm(transform[:3, 3] - target.position_m))
        wanted = rpy_to_matrix(target.rpy_rad)
        rotation_error = Rotation.from_matrix(wanted @ transform[:3, :3].T).magnitude()
        return position_error, float(np.rad2deg(rotation_error))

    def solve(self, target: EndPose, seed_arm: Any, waist_urdf: Any, stage: str | None = None) -> IkResult:
        seed = np.asarray(seed_arm, dtype=float).reshape(7)
        if not np.all(np.isfinite(seed)):
            raise ValueError("IK seed contains non-finite values")
        seed = np.clip(seed, self.lower + 1e-6, self.upper - 1e-6)
        wanted_rotation = rpy_to_matrix(target.rpy_rad)
        position_tolerance = float(self.stage_position_tolerance_m.get(stage, self.position_tolerance_m))
        orientation_tolerance = float(self.stage_orientation_tolerance_deg.get(stage, self.orientation_tolerance_deg))
        position_scale = max(position_tolerance, 1e-4)
        rotation_scale = max(np.deg2rad(orientation_tolerance), 1e-4)

        def residual(q: np.ndarray) -> np.ndarray:
            current = self.forward(q, waist_urdf)
            position = (current[:3, 3] - target.position_m) / position_scale
            rotation = Rotation.from_matrix(wanted_rotation @ current[:3, :3].T).as_rotvec()
            regularization = np.sqrt(self.seed_regularization) * (q - seed)
            return np.concatenate([position, rotation / rotation_scale, regularization])

        result = least_squares(
            residual, seed, bounds=(self.lower, self.upper),
            max_nfev=self.max_nfev, xtol=1e-9, ftol=1e-9, gtol=1e-9,
        )
        q = np.asarray(result.x, dtype=float)
        position_error, orientation_error = self.pose_errors(self.forward(q, waist_urdf), target)
        within_limits = bool(np.all(q >= self.lower + self.limit_margin) and
                             np.all(q <= self.upper - self.limit_margin))
        success = bool(
            result.success
            and position_error <= position_tolerance
            and orientation_error <= orientation_tolerance
            and within_limits
        )
        if not result.success:
            reason = f"optimizer failed: {result.message}"
        elif position_error > position_tolerance:
            reason = f"position error {position_error * 1000:.1f} mm"
        elif orientation_error > orientation_tolerance:
            reason = f"orientation error {orientation_error:.1f} deg"
        elif not within_limits:
            reason = "solution is too close to a joint limit"
        else:
            reason = "ok"
        return IkResult(q, position_error, orientation_error, success, reason)

    def solve_with_restarts(self, target: EndPose, seed_arm: Any, waist_urdf: Any,
                            stage: str) -> tuple[IkResult, list[dict[str, Any]]]:
        seed = np.asarray(seed_arm, dtype=float).reshape(7)
        if not np.all(np.isfinite(seed)):
            raise ValueError("IK seed contains non-finite values")
        midpoint = (self.lower + self.upper) / 2.0
        seeds = [seed, 0.75 * seed + 0.25 * midpoint,
                 0.50 * seed + 0.50 * midpoint]
        seeds[1] = seeds[1] + np.array([0.35, 0, -0.45, 0.35, 0, 0, 0])
        seeds[2] = seeds[2] + np.array([-0.35, 0, 0.45, -0.35, 0, 0, 0])
        attempts: list[dict[str, Any]] = []
        results: list[IkResult] = []
        for index, initial in enumerate(seeds):
            initial = np.clip(initial, self.lower + 1e-6, self.upper - 1e-6)
            result = self.solve(target, initial, waist_urdf, stage=stage)
            margin = np.minimum(result.joints_rad - self.lower, self.upper - result.joints_rad)
            closest = int(np.argmin(margin))
            attempts.append({
                "seed_index": index, "seed_joint_rad": initial.tolist(),
                "success": result.success, "reason": result.reason,
                "position_error_mm": result.position_error_m * 1000.0,
                "orientation_error_deg": result.orientation_error_deg,
                "position_tolerance_mm": 1000.0 * self.stage_position_tolerance_m.get(stage, self.position_tolerance_m),
                "orientation_tolerance_deg": self.stage_orientation_tolerance_deg.get(stage, self.orientation_tolerance_deg),
                "right_arm_joint_rad": result.joints_rad.tolist(),
                "closest_limit_joint": self.joint_names[closest],
                "joint_limit_margin_rad": float(margin[closest]),
                "required_joint_limit_margin_rad": self.limit_margin,
            })
            results.append(result)
            if result.success:
                return result, attempts
        position_tol = self.stage_position_tolerance_m.get(stage, self.position_tolerance_m)
        orientation_tol = self.stage_orientation_tolerance_deg.get(stage, self.orientation_tolerance_deg)
        best = min(results, key=lambda item: max(
            item.position_error_m / position_tol, item.orientation_error_deg / orientation_tol
        ))
        return best, attempts

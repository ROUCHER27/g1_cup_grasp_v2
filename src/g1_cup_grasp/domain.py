from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


def vector3(value: Any, name: str = "vector") -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size != 3 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite numbers")
    return result


def vector6(value: Any, name: str = "pose") -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size != 6 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite numbers")
    return result


@dataclass(frozen=True)
class ImageFrame:
    image: np.ndarray
    timestamp_ns: int
    camera_name: str


@dataclass(frozen=True)
class RgbdFrame:
    color: ImageFrame
    depth: ImageFrame


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: np.ndarray

    @property
    def center_uv(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))


@dataclass(frozen=True)
class Target3D:
    pixel_uv: tuple[float, float]
    depth_m: float
    point_camera_m: np.ndarray
    point_base_m: np.ndarray
    confidence: float
    valid_depth_count: int


@dataclass(frozen=True)
class EndPose:
    """Cartesian pose represented as position xyz and roll/pitch/yaw."""

    position_m: np.ndarray
    rpy_rad: np.ndarray
    frame: str = "base_link"

    def as_vector6(self) -> np.ndarray:
        return np.concatenate([vector3(self.position_m), vector3(self.rpy_rad)])


@dataclass(frozen=True)
class MotionState:
    timestamp_ns: int
    head_joint_rad: list[float]
    waist_state: list[float]
    arm_joint_rad: list[float]
    gripper_state: list[float]
    raw: Any = None


@dataclass(frozen=True)
class PoseTarget:
    name: str
    pose: EndPose


@dataclass(frozen=True)
class GraspPlan:
    hover: PoseTarget
    grasp: PoseTarget
    lift: PoseTarget
    hover_deltas: list[np.ndarray]
    approach_deltas: list[np.ndarray]
    lift_deltas: list[np.ndarray]
    gripper_open: float
    gripper_closed: float


from __future__ import annotations

from typing import Any

import numpy as np


def _as_float_vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size != size or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def rpy_to_matrix(rpy_rad: Any) -> np.ndarray:
    roll, pitch, yaw = _as_float_vector(rpy_rad, 3, "rpy_rad")
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def matrix_to_rpy(rotation: Any) -> np.ndarray:
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(r)):
        raise ValueError("rotation contains non-finite values")
    sy = float(np.hypot(r[0, 0], r[1, 0]))
    if sy > 1e-9:
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = np.arctan2(r[1, 0], r[0, 0])
    else:
        roll = np.arctan2(-r[1, 2], r[1, 1])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


def quaternion_xyzw_to_matrix(quaternion_xyzw: Any) -> np.ndarray:
    x, y, z, w = _as_float_vector(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def compose_transform(rotation: Any, translation: Any) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    result[:3, 3] = _as_float_vector(translation, 3, "translation")
    return result


def invert_transform(transform: Any) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float).reshape(4, 4)
    result = np.eye(4, dtype=float)
    rotation = matrix[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ matrix[:3, 3]
    return result


def transform_point(transform: Any, point: Any) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float).reshape(4, 4)
    xyz = _as_float_vector(point, 3, "point")
    return matrix[:3, :3] @ xyz + matrix[:3, 3]


def load_transform(value: dict[str, Any]) -> np.ndarray:
    if "matrix" in value:
        matrix = np.asarray(value["matrix"], dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(matrix)):
            raise ValueError("transform matrix contains non-finite values")
        if "translation_m" in value:
            translation = _as_float_vector(value["translation_m"], 3, "translation_m")
            if not np.allclose(matrix[:3, 3], translation, atol=1e-6):
                raise ValueError("transform matrix translation disagrees with translation_m")
        return matrix
    translation = value.get("translation_m", [0.0, 0.0, 0.0])
    if "quaternion_xyzw" in value:
        rotation = quaternion_xyzw_to_matrix(value["quaternion_xyzw"])
    elif "rpy_rad" in value:
        rotation = rpy_to_matrix(value["rpy_rad"])
    else:
        rotation = np.eye(3)
    return compose_transform(rotation, translation)


def pose_delta(current_pose: Any, target_pose: Any) -> np.ndarray:
    current = np.asarray(current_pose, dtype=float).reshape(6)
    target = np.asarray(target_pose, dtype=float).reshape(6)
    delta = target - current
    delta[3:] = (delta[3:] + np.pi) % (2 * np.pi) - np.pi
    return delta


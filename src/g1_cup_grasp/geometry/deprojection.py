from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..domain import Detection, Target3D
from .transforms import transform_point


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int | None = None
    height: int | None = None

    @classmethod
    def from_config(cls, value: dict[str, Any]) -> "CameraIntrinsics":
        return cls(
            fx=float(value["fx"]),
            fy=float(value["fy"]),
            cx=float(value["cx"]),
            cy=float(value["cy"]),
            width=int(value["width"]) if value.get("width") is not None else None,
            height=int(value["height"]) if value.get("height") is not None else None,
        )


def _valid_depth(depth: np.ndarray, mask: np.ndarray, scale: float) -> np.ndarray:
    if depth.ndim != 2:
        raise ValueError("depth must be a two-dimensional array")
    if mask.shape != depth.shape:
        raise ValueError(f"mask shape {mask.shape} does not match depth shape {depth.shape}")
    values = np.asarray(depth, dtype=float)[np.asarray(mask, dtype=bool)] * float(scale)
    values = values[np.isfinite(values) & (values > 0.05) & (values < 10.0)]
    return values


def sample_depth_m(
    depth: np.ndarray,
    mask: np.ndarray,
    scale: float,
    min_valid: int = 20,
) -> tuple[float, int]:
    values = _valid_depth(depth, mask, scale)
    if values.size < min_valid:
        raise ValueError(f"not enough valid depth pixels: {values.size} < {min_valid}")
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if mad > 1e-6:
        keep = np.abs(values - median) <= max(3.0 * mad, 0.01)
        filtered = values[keep]
        if filtered.size >= min_valid:
            values = filtered
    return float(np.median(values)), int(values.size)


def _mask_centroid(mask: np.ndarray) -> tuple[float, float]:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        raise ValueError("segmentation mask is empty")
    return float(np.mean(xs)), float(np.mean(ys))


def _target_uv(detection: Detection, anchor: str) -> tuple[float, float]:
    if anchor == "mask_centroid":
        return _mask_centroid(detection.mask)
    if anchor == "bbox_top_center":
        x1, y1, x2, y2 = detection.bbox_xyxy
        if not np.all(np.isfinite([x1, y1, x2, y2])) or x2 <= x1 or y2 <= y1:
            raise ValueError("detection bounding box must be finite and nonempty")
        return 0.5 * (x1 + x2), float(y1)
    raise ValueError(f"unsupported target anchor: {anchor!r}")


def _resize_mask(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    if mask.shape == shape:
        return np.asarray(mask, dtype=bool)
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - requirements include cv2
        raise RuntimeError("opencv-python is required to resize a mask") from exc
    height, width = shape
    resized = cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)
    return resized.astype(bool)


def estimate_target_from_mask(
    detection: Detection,
    depth_image: np.ndarray,
    color_intrinsics: CameraIntrinsics,
    depth_scale: float,
    T_base_camera: np.ndarray,
    depth_intrinsics: CameraIntrinsics | None = None,
    depth_registered_to_color: bool = True,
    min_valid_depth: int = 20,
    target_anchor: str = "mask_centroid",
) -> Target3D:
    """Estimate one robust 3D point from a mask.

    For registered depth, the mask centroid and intrinsics are in the color
    image coordinate system.  For unregistered streams we deliberately refuse
    to silently pretend that the images are aligned; an explicit depth
    intrinsics/mapping implementation must be validated before real motion.
    """
    if not depth_registered_to_color:
        raise ValueError(
            "depth is not configured as registered to color; validate RGB-D alignment "
            "and set camera.depth_registered_to_color=true before localization"
        )
    if depth_image.ndim != 2:
        raise ValueError("depth must be a two-dimensional array")
    if (color_intrinsics.width is not None and color_intrinsics.height is not None
            and depth_image.shape != (color_intrinsics.height, color_intrinsics.width)):
        raise ValueError("Registered depth size does not match calibrated color size")
    mask = _resize_mask(detection.mask, depth_image.shape)
    anchor_u, anchor_v = _target_uv(detection, target_anchor)
    if not np.all(np.isfinite([anchor_u, anchor_v])):
        raise ValueError("Target anchor must be finite")
    # Keep the depth on the same local cup surface as the requested anchor.
    yy, xx = np.ogrid[:mask.shape[0], :mask.shape[1]]
    distance_sq = (xx - anchor_u) ** 2 + (yy - anchor_v) ** 2
    local_mask = mask & (distance_sq <= 12 ** 2)
    try:
        median_depth, valid_count = sample_depth_m(
            depth_image, local_mask, depth_scale, min_valid_depth
        )
    except ValueError as exc:
        raise ValueError(f"No reliable cup depth within 12 px of target anchor: {exc}") from exc
    depth_m_image = np.asarray(depth_image, dtype=float) * depth_scale
    valid = local_mask & np.isfinite(depth_m_image) & (depth_m_image > 0.05) & (depth_m_image < 10.0)
    ys, xs = np.nonzero(valid)
    depths = depth_m_image[ys, xs]
    # Choose a real pixel closest to the local median; its depth and ray stay paired.
    index = np.lexsort((distance_sq[ys, xs], np.abs(depths - median_depth)))[0]
    u, v = float(xs[index]), float(ys[index])
    depth_m = float(depths[index])
    intrinsics = color_intrinsics if depth_registered_to_color else depth_intrinsics
    if intrinsics is None:
        raise ValueError("depth intrinsics are required for unregistered depth")
    point_camera = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx * depth_m,
            (v - intrinsics.cy) / intrinsics.fy * depth_m,
            depth_m,
        ],
        dtype=float,
    )
    point_base = transform_point(T_base_camera, point_camera)
    return Target3D(
        pixel_uv=(u, v),
        depth_m=depth_m,
        point_camera_m=point_camera,
        point_base_m=point_base,
        confidence=float(detection.confidence),
        valid_depth_count=valid_count,
    )


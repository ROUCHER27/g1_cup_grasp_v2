from __future__ import annotations

import time
from pathlib import Path

import cv2
from typing import Any

import numpy as np

from ..domain import ImageFrame, RgbdFrame


def _unpack_image(result: Any, fallback_timestamp_ns: int) -> tuple[np.ndarray | None, int]:
    """Accept both documented GDK returns and small SDK patch differences."""
    if result is None:
        return None, fallback_timestamp_ns
    if isinstance(result, tuple) and len(result) >= 2:
        image, timestamp = result[0], result[1]
        if image is None:
            return None, int(timestamp or fallback_timestamp_ns)
        return np.asarray(image), int(timestamp or fallback_timestamp_ns)
    return np.asarray(result), fallback_timestamp_ns


class GdkRgbdSource:
    """Live RGB-D source using the official CosineCamera API."""

    def __init__(
        self,
        camera_group: list[str],
        color_name: str = "head",
        depth_name: str = "head_depth",
        max_attempts: int = 20,
    ) -> None:
        try:
            from a2d_sdk.robot import CosineCamera
        except ImportError as exc:  # pragma: no cover - requires robot runtime
            raise RuntimeError(
                "Cannot import a2d_sdk. Run on the Ubuntu GDK environment and source env.sh."
            ) from exc
        self.color_name = color_name
        self.depth_name = depth_name
        self.max_attempts = int(max_attempts)
        self._camera = CosineCamera(camera_group)

    def read(self) -> RgbdFrame:
        for _ in range(self.max_attempts):
            now_ns = time.time_ns()
            color_result = self._camera.get_latest_image(self.color_name)
            color, color_timestamp = _unpack_image(color_result, now_ns)
            # The official guide notes that the first latest-image result may
            # be None, so keep polling rather than treating it as a hard error.
            if color is None:
                time.sleep(0.01)
                continue
            # G1 head frames are RGB; Ultralytics ndarray input and OpenCV use BGR.
            color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            depth_result = self._camera.get_image_nearest(self.depth_name, color_timestamp)
            depth, depth_timestamp = _unpack_image(depth_result, color_timestamp)
            if depth is None:
                time.sleep(0.01)
                continue
            if depth.ndim == 3 and depth.shape[2] == 1:
                depth = depth[:, :, 0]
            if depth.ndim != 2:
                raise ValueError(f"GDK depth frame must be HxW or HxWx1, got {depth.shape}")
            return RgbdFrame(
                color=ImageFrame(color_bgr, color_timestamp, self.color_name),
                depth=ImageFrame(depth, depth_timestamp, self.depth_name),
            )
        raise TimeoutError("GDK camera did not return a valid synchronized RGB-D frame")

    def close(self) -> None:
        close = getattr(self._camera, "close", None)
        if callable(close):
            close()


class FileRgbdSource:
    """One-shot RGB-D source for offline geometry/model tests."""

    def __init__(self, color_path: str | Path, depth_path: str | Path) -> None:
        self.color_path = Path(color_path)
        self.depth_path = Path(depth_path)
        if not self.color_path.exists():
            raise FileNotFoundError(self.color_path)
        if not self.depth_path.exists():
            raise FileNotFoundError(self.depth_path)
        self._frame: RgbdFrame | None = None

    def read(self) -> RgbdFrame:
        import cv2

        if self._frame is not None:
            return self._frame
        color = cv2.imread(str(self.color_path), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(self.depth_path), cv2.IMREAD_UNCHANGED)
        if color is None:
            raise ValueError(f"Cannot decode color image: {self.color_path}")
        if depth is None:
            raise ValueError(f"Cannot decode depth image: {self.depth_path}")
        timestamp = time.time_ns()
        self._frame = RgbdFrame(
            color=ImageFrame(color, timestamp, "file_color"),
            depth=ImageFrame(depth, timestamp, "file_depth"),
        )
        return self._frame

    def close(self) -> None:
        return None

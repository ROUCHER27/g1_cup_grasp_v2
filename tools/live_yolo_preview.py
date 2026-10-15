#!/usr/bin/env python3
"""Read the G1 head camera and preview YOLO cup segmentation.

This tool is intentionally perception-only. It imports ``CosineCamera`` but
never creates a robot controller, reads motion state, or sends arm/gripper
commands.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any


LOGGER = logging.getLogger("live_yolo_preview")
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
DEFAULT_MODEL = PROJECT_ROOT / "models" / "cup_yolo11s_seg.pt"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preview G1 head-camera YOLO11 cup segmentation without robot motion."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--camera", default="head", help="GDK color camera name")
    parser.add_argument(
        "--camera-group",
        action="append",
        default=None,
        help="GDK camera group; repeat for multiple groups (default: color and depth)",
    )
    parser.add_argument("--device", default="0", help="Ultralytics device, e.g. 0 or cpu")
    parser.add_argument(
        "--input-color",
        choices=["rgb", "bgr"],
        default="rgb",
        help="channel order returned by the GDK camera (G1 head camera: rgb)",
    )
    parser.add_argument(
        "--mask-style",
        choices=["contour", "fill"],
        default="contour",
        help="display masks as green contours or the Ultralytics filled overlay",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="minimum confidence; 0.25 matches the first G1 field-image test",
    )
    parser.add_argument(
        "--class-name", default="cup", help="model class name to retain"
    )
    parser.add_argument(
        "--class-id",
        type=int,
        default=None,
        help="optional explicit class id; otherwise resolved from --class-name",
    )
    parser.add_argument(
        "--save-dir", type=Path, default=PROJECT_ROOT / "outputs" / "live_preview"
    )
    parser.add_argument(
        "--save-every",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="automatically save snapshots at this interval; 0 disables",
    )
    parser.add_argument(
        "--max-empty-frames",
        type=int,
        default=100,
        help="exit after this many consecutive empty/error camera reads",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after N valid frames; 0 runs until Q, Esc, or Ctrl-C",
    )
    parser.add_argument(
        "--display-scale", type=float, default=1.0, help="preview-window scale"
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="run without a GUI; use Ctrl-C to stop and --save-every to save",
    )
    parser.add_argument("--log-every", type=int, default=30)
    parser.add_argument(
        "--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], default="INFO"
    )
    return parser


def unpack_camera_frame(value: Any) -> tuple[Any | None, int | None]:
    """Handle both ndarray and documented ``(image, timestamp_ns)`` returns."""
    if value is None:
        return None, None
    if isinstance(value, tuple):
        image = value[0] if value else None
        timestamp = value[1] if len(value) > 1 else None
        return image, int(timestamp) if timestamp is not None else None
    return value, None


def model_names(model: Any) -> dict[int, str]:
    names = getattr(model, "names", {})
    if isinstance(names, dict):
        return {int(key): str(name) for key, name in names.items()}
    return {index: str(name) for index, name in enumerate(names)}


def resolve_class_id(model: Any, class_id: int | None, class_name: str) -> int:
    names = model_names(model)
    if class_id is not None:
        if class_id not in names:
            raise ValueError(f"class id {class_id} is absent from model classes: {names}")
        return class_id

    wanted = class_name.casefold()
    matches = [index for index, name in names.items() if name.casefold() == wanted]
    if len(matches) != 1:
        raise ValueError(
            f"could not uniquely resolve class name {class_name!r}; "
            f"model classes are {names}. Pass --class-id explicitly."
        )
    return matches[0]


def detection_summary(result: Any) -> tuple[int, float | None]:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return 0, None
    scores = boxes.conf.detach().cpu().tolist()
    return len(scores), max(float(score) for score in scores)


def _label_for_class(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    try:
        return str(names[class_id])
    except (IndexError, KeyError, TypeError):
        return str(class_id)


def render_detection_overlay(raw_image: Any, result: Any, mask_style: str) -> Any:
    """Render detections while keeping the camera's real object colours visible."""
    import cv2
    import numpy as np

    if mask_style == "fill":
        return result.plot(labels=True, conf=True, boxes=True, masks=True)

    # A contour shows the segmentation boundary without turning a yellow cup
    # blue. This also makes mask leakage onto a hand easy to see and diagnose.
    annotated = raw_image.copy()
    green = (0, 255, 0)
    masks = getattr(result, "masks", None)
    for polygon in getattr(masks, "xy", []) if masks is not None else []:
        contour = np.asarray(polygon, dtype=np.int32)
        if contour.size >= 6:
            cv2.polylines(annotated, [contour.reshape(-1, 1, 2)], True, green, 2, cv2.LINE_AA)

    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return annotated
    names = getattr(result, "names", {})
    for coordinates, score, class_value in zip(
        boxes.xyxy.detach().cpu().tolist(),
        boxes.conf.detach().cpu().tolist(),
        boxes.cls.detach().cpu().tolist(),
    ):
        x1, y1, x2, y2 = (int(round(value)) for value in coordinates)
        class_id = int(class_value)
        label = f"{_label_for_class(names, class_id)} {float(score):.2f}"
        cv2.rectangle(annotated, (x1, y1), (x2, y2), green, 2, cv2.LINE_AA)
        (label_width, label_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
        )
        label_y = max(y1, label_height + baseline + 4)
        cv2.rectangle(
            annotated,
            (x1, label_y - label_height - baseline - 4),
            (x1 + label_width + 8, label_y + 2),
            green,
            -1,
        )
        cv2.putText(
            annotated,
            label,
            (x1 + 4, label_y - baseline),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated


def draw_status(
    image: Any,
    fps: float,
    inference_ms: float,
    detections: int,
    best_confidence: float | None,
) -> None:
    import cv2

    confidence_text = "--" if best_confidence is None else f"{best_confidence:.3f}"
    lines = [
        f"FPS {fps:.1f} | inference {inference_ms:.1f} ms",
        f"cups {detections} | best confidence {confidence_text}",
        "S: save snapshot | Q/Esc: quit",
    ]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.62
    thickness = 2
    line_height = 27
    widths = [cv2.getTextSize(line, font, font_scale, thickness)[0][0] for line in lines]
    overlay = image.copy()
    cv2.rectangle(overlay, (8, 8), (max(widths) + 28, 18 + line_height * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, image, 0.42, 0.0, image)
    for index, line in enumerate(lines):
        color = (80, 255, 80) if detections else (80, 190, 255)
        cv2.putText(
            image,
            line,
            (18, 33 + index * line_height),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


def save_snapshot(
    raw_image: Any,
    annotated_image: Any,
    result: Any,
    save_dir: Path,
    camera_timestamp_ns: int | None,
    target_data: dict | None = None,
) -> tuple[Path, Path, Path]:
    import cv2

    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    stem = f"head_{stamp}"
    raw_path = save_dir / f"{stem}_raw.jpg"
    annotated_path = save_dir / f"{stem}_annotated.jpg"
    metadata_path = save_dir / f"{stem}.json"
    if not cv2.imwrite(str(raw_path), raw_image):
        raise OSError(f"failed to save raw image: {raw_path}")
    if not cv2.imwrite(str(annotated_path), annotated_image):
        raise OSError(f"failed to save annotated image: {annotated_path}")

    detections: list[dict[str, Any]] = []
    boxes = getattr(result, "boxes", None)
    names = getattr(result, "names", {})
    if boxes is not None:
        xyxy = boxes.xyxy.detach().cpu().tolist()
        scores = boxes.conf.detach().cpu().tolist()
        classes = boxes.cls.detach().cpu().tolist()
        for coordinates, score, class_value in zip(xyxy, scores, classes):
            class_index = int(class_value)
            label = names.get(class_index, str(class_index)) if isinstance(names, dict) else names[class_index]
            detections.append(
                {
                    "class_id": class_index,
                    "label": str(label),
                    "confidence": float(score),
                    "bbox_xyxy": [float(value) for value in coordinates],
                }
            )
    metadata = {
        "saved_at": datetime.now().astimezone().isoformat(),
        "camera_timestamp_ns": camera_timestamp_ns,
        "detections": detections,
        "target": target_data,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return raw_path, annotated_path, metadata_path


def draw_target_overlay(image: Any, result: Any, depth: Any, config: dict,
                        color_timestamp_ns: int | None,
                        depth_timestamp_ns: int | None) -> dict | None:
    import cv2
    import numpy as np
    from g1_cup_grasp.domain import Detection
    from g1_cup_grasp.geometry.deprojection import CameraIntrinsics, estimate_target_from_mask
    from g1_cup_grasp.geometry.transforms import load_transform

    target_data = None
    lines = []
    try:
        boxes, masks = result.boxes, result.masks
        if boxes is None or len(boxes) == 0 or masks is None:
            raise ValueError("No segmented cup target")
        camera_cfg = config["camera"]
        if not camera_cfg.get("intrinsics_verified", False):
            raise ValueError("Camera intrinsics are not verified")
        if color_timestamp_ns is None or depth_timestamp_ns is None:
            raise ValueError("RGB-D timestamps unavailable")
        sync_ms = abs(depth_timestamp_ns - color_timestamp_ns) / 1e6
        if sync_ms > float(camera_cfg.get("max_sync_delta_ms", 50.0)):
            raise ValueError(f"RGB-D timing mismatch: {sync_ms:.1f} ms")
        if depth is None:
            raise ValueError("Depth frame unavailable")
        depth = np.asarray(depth)
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]
        if depth.shape != image.shape[:2]:
            raise ValueError("Registered RGB and depth sizes differ")
        scores = boxes.conf.detach().cpu().numpy()
        index = int(np.argmax(scores))
        if scores[index] < float(config["runtime"].get("max_detection_confidence", 0.65)):
            raise ValueError("Cup confidence below grasp threshold")
        mask = masks.data[index].detach().cpu().numpy().astype(bool)
        detection = Detection(
            int(boxes.cls[index].item()), "cup", float(scores[index]),
            tuple(boxes.xyxy[index].detach().cpu().tolist()), mask,
        )
        target = estimate_target_from_mask(
            detection, depth, CameraIntrinsics.from_config(camera_cfg["color_intrinsics"]),
            float(camera_cfg.get("depth_scale", 0.001)),
            load_transform(config["calibration"]["T_base_camera"]),
            depth_registered_to_color=bool(camera_cfg.get("depth_registered_to_color", False)),
            min_valid_depth=int(camera_cfg.get("min_valid_depth_pixels", 20)),
            target_anchor=config["grasp"].get("target_anchor", "mask_centroid"),
        )
        u, v = map(int, target.pixel_uv)
        cv2.drawMarker(image, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(image, (u, v), 5, (0, 0, 255), 2)
        lines = [
            f"TARGET uv=({u}, {v}) depth={target.depth_m:.4f} m | sync={sync_ms:.1f} ms",
            "CAM xyz m: " + ", ".join(f"{value:.4f}" for value in target.point_camera_m),
            "BASE xyz m: " + ", ".join(f"{value:.4f}" for value in target.point_base_m),
        ]
        target_data = {"pixel_uv": list(target.pixel_uv), "depth_m": target.depth_m,
                       "point_camera_m": target.point_camera_m.tolist(),
                       "point_base_m": target.point_base_m.tolist(),
                       "valid_depth_count": target.valid_depth_count,
                       "depth_timestamp_ns": depth_timestamp_ns, "sync_delta_ms": sync_ms}
    except ValueError as exc:
        lines = [f"TARGET unavailable: {exc}"]
    y0 = image.shape[0] - 28 * len(lines) - 12
    cv2.rectangle(image, (0, y0 - 20), (image.shape[1], image.shape[0]), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(image, line, (12, y0 + index * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (80, 255, 255) if target_data else (80, 190, 255), 2, cv2.LINE_AA)
    return target_data


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 <= args.conf <= 1.0:
        raise ValueError("--conf must be between 0 and 1")
    if args.imgsz <= 0:
        raise ValueError("--imgsz must be positive")
    if args.max_empty_frames <= 0:
        raise ValueError("--max-empty-frames must be positive")
    if args.display_scale <= 0:
        raise ValueError("--display-scale must be positive")
    if args.save_every < 0:
        raise ValueError("--save-every cannot be negative")


def run(args: argparse.Namespace) -> int:
    import cv2
    import numpy as np

    validate_args(args)
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("Ultralytics is not installed in this Python environment") from exc
    try:
        from a2d_sdk.robot import CosineCamera
    except ImportError as exc:
        raise RuntimeError(
            "GDK import failed. Activate the project environment, source "
            "~/a2d_sdk/env.sh, and install the GDK dependencies such as ruckig."
        ) from exc

    from g1_cup_grasp.config import load_config
    config = load_config(PROJECT_ROOT / "config/grasp_v2.yaml")
    if args.camera != config["camera"]["color_name"]:
        raise ValueError("Target overlay requires the calibrated color camera")
    model = YOLO(str(model_path))
    cup_class_id = resolve_class_id(model, args.class_id, args.class_name)
    groups = args.camera_group or [args.camera, config["camera"]["depth_name"]]
    camera = CosineCamera(groups)
    stop_requested = False

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        LOGGER.info("received signal %s; stopping", signum)
        stop_requested = True

    for signal_name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signal_name, request_stop)

    show_window = not args.no_window
    if show_window and sys.platform != "win32" and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        LOGGER.warning("no graphical display detected; continuing without a window")
        show_window = False

    LOGGER.info("model: %s", model_path)
    LOGGER.info("camera: %s; groups: %s", args.camera, groups)
    LOGGER.info("camera input color order: %s", args.input_color.upper())
    LOGGER.info("mask display style: %s", args.mask_style)
    LOGGER.info("cup class: %d (%s); confidence: %.2f", cup_class_id, args.class_name, args.conf)
    LOGGER.info("save directory: %s", args.save_dir.expanduser().resolve())
    if show_window:
        LOGGER.info("controls: S saves a snapshot; Q or Esc exits")

    frame_durations_ms: deque[float] = deque(maxlen=30)
    consecutive_empty = 0
    valid_frames = 0
    last_auto_save = time.monotonic()
    window_name = "G1 head camera - YOLO cup segmentation"

    try:
        while not stop_requested:
            try:
                camera_value = camera.get_latest_image(args.camera)
                image, camera_timestamp_ns = unpack_camera_frame(camera_value)
            except Exception as exc:
                consecutive_empty += 1
                LOGGER.warning(
                    "camera read failed (%d/%d): %s",
                    consecutive_empty,
                    args.max_empty_frames,
                    exc,
                )
                if consecutive_empty >= args.max_empty_frames:
                    raise RuntimeError("camera disconnected or stopped returning frames") from exc
                time.sleep(0.05)
                continue

            if image is None:
                consecutive_empty += 1
                if consecutive_empty == 1 or consecutive_empty % 20 == 0:
                    LOGGER.warning(
                        "empty camera frame (%d/%d)",
                        consecutive_empty,
                        args.max_empty_frames,
                    )
                if consecutive_empty >= args.max_empty_frames:
                    raise RuntimeError("camera stopped returning valid frames")
                time.sleep(0.05)
                continue

            depth_image, depth_timestamp_ns = None, None
            if camera_timestamp_ns is not None:
                try:
                    depth_image, depth_timestamp_ns = unpack_camera_frame(
                        camera.get_image_nearest(config["camera"]["depth_name"], camera_timestamp_ns)
                    )
                except Exception as exc:
                    LOGGER.warning("depth read failed: %s", exc)
            consecutive_empty = 0
            camera_image = np.asarray(image)
            if camera_image.ndim != 3 or camera_image.shape[2] != 3:
                raise ValueError(f"expected an HxWx3 color image, received {camera_image.shape}")
            if camera_image.dtype != np.uint8:
                raise ValueError(f"expected uint8 color data, received {camera_image.dtype}")

            # G1's head camera returns RGB. OpenCV and an ndarray source fed
            # to Ultralytics use BGR, so convert exactly once here. Without
            # this, a physical yellow object (R+G) is displayed as cyan (B+G).
            raw_image = (
                cv2.cvtColor(camera_image, cv2.COLOR_RGB2BGR)
                if args.input_color == "rgb"
                else np.ascontiguousarray(camera_image)
            )

            frame_started = time.perf_counter()
            results = model.predict(
                source=raw_image,
                imgsz=args.imgsz,
                conf=args.conf,
                classes=[cup_class_id],
                device=args.device,
                retina_masks=True,
                verbose=False,
            )
            if not results:
                raise RuntimeError("YOLO returned no Results object")
            result = results[0]
            annotated = render_detection_overlay(raw_image, result, args.mask_style)
            frame_ms = (time.perf_counter() - frame_started) * 1000.0
            frame_durations_ms.append(frame_ms)
            fps = 1000.0 / (sum(frame_durations_ms) / len(frame_durations_ms))
            inference_ms = float(getattr(result, "speed", {}).get("inference", frame_ms))
            count, best_confidence = detection_summary(result)
            draw_status(annotated, fps, inference_ms, count, best_confidence)
            target_data = draw_target_overlay(
                annotated, result, depth_image, config, camera_timestamp_ns, depth_timestamp_ns
            )

            valid_frames += 1
            if args.log_every > 0 and valid_frames % args.log_every == 0:
                LOGGER.info(
                    "frame=%d fps=%.1f inference=%.1fms cups=%d best_conf=%s",
                    valid_frames,
                    fps,
                    inference_ms,
                    count,
                    "--" if best_confidence is None else f"{best_confidence:.3f}",
                )

            save_now = False
            if args.save_every > 0 and time.monotonic() - last_auto_save >= args.save_every:
                save_now = True
                last_auto_save = time.monotonic()

            if show_window:
                display = annotated
                if args.display_scale != 1.0:
                    display = cv2.resize(
                        annotated,
                        None,
                        fx=args.display_scale,
                        fy=args.display_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                try:
                    cv2.imshow(window_name, display)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error as exc:
                    LOGGER.warning("OpenCV window unavailable; continuing headless: %s", exc)
                    show_window = False
                    key = 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("s"), ord("S")):
                    save_now = True

            if save_now:
                paths = save_snapshot(
                    raw_image,
                    annotated,
                    result,
                    args.save_dir.expanduser().resolve(),
                    camera_timestamp_ns,
                    target_data,
                )
                LOGGER.info("saved raw=%s annotated=%s metadata=%s", *paths)

            if args.max_frames > 0 and valid_frames >= args.max_frames:
                break
    finally:
        try:
            camera.close()
        finally:
            if show_window:
                cv2.destroyAllWindows()

    LOGGER.info("stopped after %d valid frames", valid_frames)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        return run(args)
    except KeyboardInterrupt:
        LOGGER.info("interrupted by user")
        return 130
    except Exception as exc:
        LOGGER.error("%s: %s", type(exc).__name__, exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Inspect synchronized G1 head RGB-D frames without any robot motion.

The script intentionally uses only ``CosineCamera``.  It never constructs a
robot controller, reads joint state, or sends arm/gripper commands.  It is a
diagnostic tool for answering four questions before 3-D localisation:

1. Does the head colour stream arrive?
2. Does the head depth stream arrive and contain valid samples?
3. Is the depth frame nearest to the colour timestamp sufficiently close?
4. Do the colour and depth image sizes permit a later registration check?

It does *not* claim that same-sized RGB and depth images are registered, does
not guess depth units, and does not calculate a cup's 3-D position.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any


LOGGER = logging.getLogger("check_rgbd")
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect G1 head RGB-D synchronization without robot motion."
    )
    parser.add_argument("--color-name", default="head", help="GDK colour camera name")
    parser.add_argument("--depth-name", default="head_depth", help="GDK depth camera name")
    parser.add_argument(
        "--camera-group",
        action="append",
        default=None,
        help="GDK camera group; repeat if required (default: head and head_depth)",
    )
    parser.add_argument(
        "--max-sync-delta-ms",
        type=float,
        default=50.0,
        help="diagnostic pass threshold for |depth_ts - colour_ts| (default: 50)",
    )
    parser.add_argument(
        "--save-dir", type=Path, default=PROJECT_ROOT / "outputs" / "rgbd_check"
    )
    parser.add_argument(
        "--save-every",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="save a capture set automatically at this interval; 0 disables",
    )
    parser.add_argument(
        "--max-empty-frames",
        type=int,
        default=100,
        help="exit after this many consecutive missing colour/depth frames",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after N valid RGB-D pairs; 0 runs until Q, Esc, or Ctrl-C",
    )
    parser.add_argument(
        "--display-scale", type=float, default=0.75, help="preview-window scale"
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
    """Support both GDK ``ndarray`` and ``(image, timestamp_ns)`` returns."""
    if value is None:
        return None, None
    if isinstance(value, tuple):
        image = value[0] if value else None
        timestamp = value[1] if len(value) > 1 else None
        return image, int(timestamp) if timestamp is not None else None
    return value, None


def timestamp_delta_ms(color_timestamp_ns: int | None, depth_timestamp_ns: int | None) -> float | None:
    if color_timestamp_ns is None or depth_timestamp_ns is None:
        return None
    return (depth_timestamp_ns - color_timestamp_ns) / 1_000_000.0


def ensure_color_bgr(image: Any) -> Any:
    """Convert the known G1 RGB head image to OpenCV BGR exactly once."""
    import cv2
    import numpy as np

    colour = np.asarray(image)
    if colour.ndim != 3 or colour.shape[2] != 3:
        raise ValueError(f"expected HxWx3 colour image, received {colour.shape}")
    if colour.dtype != np.uint8:
        raise ValueError(f"expected uint8 colour data, received {colour.dtype}")
    return cv2.cvtColor(colour, cv2.COLOR_RGB2BGR)


def ensure_depth_2d(image: Any) -> Any:
    """Accept 2-D depth data or an HxWx1 representation without altering values."""
    import numpy as np

    depth = np.asarray(image)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        raise ValueError(
            f"expected 2-D depth image (or HxWx1), received shape {depth.shape}"
        )
    if depth.dtype.kind not in "uif":
        raise ValueError(f"expected numeric depth data, received {depth.dtype}")
    return depth


def depth_statistics(depth: Any) -> dict[str, float | int | None | str]:
    """Describe raw depth values only; unit conversion is intentionally deferred."""
    import numpy as np

    values = np.asarray(depth)
    finite = np.isfinite(values)
    positive = finite & (values > 0)
    valid = values[positive]
    result: dict[str, float | int | None | str] = {
        "dtype": str(values.dtype),
        "height": int(values.shape[0]),
        "width": int(values.shape[1]),
        "total_pixels": int(values.size),
        "valid_positive_pixels": int(valid.size),
        "valid_positive_ratio": float(valid.size / values.size) if values.size else 0.0,
        "raw_min": None,
        "raw_median": None,
        "raw_max": None,
        "raw_p02": None,
        "raw_p98": None,
    }
    if valid.size:
        result.update(
            {
                "raw_min": float(valid.min()),
                "raw_median": float(np.median(valid)),
                "raw_max": float(valid.max()),
                "raw_p02": float(np.percentile(valid, 2)),
                "raw_p98": float(np.percentile(valid, 98)),
            }
        )
    return result


def colorize_depth(depth: Any, statistics: dict[str, float | int | None | str]) -> Any:
    """Create a view-only false-colour image while preserving raw depth elsewhere."""
    import cv2
    import numpy as np

    raw = np.asarray(depth)
    preview = np.zeros(raw.shape, dtype=np.uint8)
    valid = np.isfinite(raw) & (raw > 0)
    low = statistics["raw_p02"]
    high = statistics["raw_p98"]
    if valid.any() and low is not None and high is not None:
        low_value, high_value = float(low), float(high)
        if high_value <= low_value:
            high_value = low_value + 1.0
        scaled = (raw.astype(np.float32) - low_value) * 255.0 / (high_value - low_value)
        preview[valid] = np.clip(scaled[valid], 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured


def add_text(image: Any, lines: list[tuple[str, tuple[int, int, int]]]) -> Any:
    import cv2

    result = image.copy()
    overlay = result.copy()
    line_height = 25
    cv2.rectangle(overlay, (8, 8), (650, 16 + line_height * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, result, 0.42, 0.0, result)
    for index, (line, colour) in enumerate(lines):
        cv2.putText(
            result,
            line,
            (18, 33 + index * line_height),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            colour,
            2,
            cv2.LINE_AA,
        )
    return result


def make_panel(
    color_bgr: Any,
    depth_preview: Any,
    sync_delta_ms: float | None,
    max_sync_delta_ms: float,
    statistics: dict[str, float | int | None | str],
) -> tuple[Any, Any | None, bool]:
    """Make a diagnostic panel and an optional same-size visual overlay.

    An overlay is only generated when both images have the same dimensions.
    This is a visual aid, *not* proof of depth registration.
    """
    import cv2

    color_height, color_width = color_bgr.shape[:2]
    depth_height, depth_width = depth_preview.shape[:2]
    same_shape = (color_height, color_width) == (depth_height, depth_width)
    if sync_delta_ms is None:
        sync_text = "sync: timestamp unavailable"
        sync_colour = (0, 190, 255)
    else:
        passed = abs(sync_delta_ms) <= max_sync_delta_ms
        sync_text = (
            f"sync delta: {sync_delta_ms:+.2f} ms "
            f"({'PASS' if passed else 'CHECK'}; limit {max_sync_delta_ms:.1f} ms)"
        )
        sync_colour = (80, 255, 80) if passed else (0, 170, 255)

    valid_ratio = float(statistics["valid_positive_ratio"] or 0.0) * 100.0
    raw_range = f"raw depth p02-p98: {statistics['raw_p02']} - {statistics['raw_p98']}"
    colour_view = add_text(
        color_bgr,
        [
            ("RGB: G1 head colour (displayed BGR after RGB conversion)", (255, 255, 255)),
            (sync_text, sync_colour),
            ("S: save set | Q/Esc: quit", (80, 255, 80)),
        ],
    )
    depth_view = add_text(
        depth_preview,
        [
            (f"Depth raw: {statistics['dtype']} {depth_width}x{depth_height}", (255, 255, 255)),
            (f"valid positive samples: {valid_ratio:.1f}%", (255, 255, 255)),
            (raw_range, (255, 255, 255)),
            ("Colours are only a visual preview; raw values are saved in .npy", (0, 170, 255)),
        ],
    )

    overlay: Any | None = None
    if same_shape:
        overlay = cv2.addWeighted(color_bgr, 0.70, depth_preview, 0.30, 0.0)
        overlay = add_text(
            overlay,
            [
                ("RGB + depth preview (same size only; NOT registration proof)", (0, 170, 255)),
                ("Compare object edges manually before enabling depth registration", (0, 170, 255)),
            ],
        )
        panel = cv2.hconcat([colour_view, depth_view, overlay])
    else:
        target_height = color_height
        depth_for_panel = cv2.resize(
            depth_view,
            (int(depth_width * target_height / depth_height), target_height),
            interpolation=cv2.INTER_NEAREST,
        )
        panel = cv2.hconcat([colour_view, depth_for_panel])
    return panel, overlay, same_shape


def save_capture(
    color_bgr: Any,
    depth: Any,
    depth_preview: Any,
    panel: Any,
    overlay: Any | None,
    metadata: dict[str, Any],
    save_dir: Path,
) -> Path:
    """Save a self-contained diagnostic set; ``depth_raw.npy`` is authoritative."""
    import cv2
    import numpy as np

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    capture_dir = save_dir / f"rgbd_{stamp}"
    capture_dir.mkdir(parents=True, exist_ok=False)
    paths = {
        "color_bgr.jpg": color_bgr,
        "depth_preview.png": depth_preview,
        "diagnostic_panel.jpg": panel,
    }
    if overlay is not None:
        paths["same_size_overlay.jpg"] = overlay
    for filename, image in paths.items():
        if not cv2.imwrite(str(capture_dir / filename), image):
            raise OSError(f"failed to save {filename}")

    depth_array = np.asarray(depth)
    np.save(capture_dir / "depth_raw.npy", depth_array)
    # PNG is an additional convenient view of true raw integers only.  The
    # .npy file above remains the lossless source for all numeric depth types.
    if depth_array.dtype in (np.dtype("uint8"), np.dtype("uint16")):
        if not cv2.imwrite(str(capture_dir / "depth_raw.png"), depth_array):
            raise OSError("failed to save depth_raw.png")
        metadata["depth_raw_png_saved"] = True
    else:
        metadata["depth_raw_png_saved"] = False

    (capture_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return capture_dir


def validate_args(args: argparse.Namespace) -> None:
    if args.max_sync_delta_ms < 0:
        raise ValueError("--max-sync-delta-ms cannot be negative")
    if args.save_every < 0:
        raise ValueError("--save-every cannot be negative")
    if args.max_empty_frames <= 0:
        raise ValueError("--max-empty-frames must be positive")
    if args.max_frames < 0:
        raise ValueError("--max-frames cannot be negative")
    if args.display_scale <= 0:
        raise ValueError("--display-scale must be positive")


def run(args: argparse.Namespace) -> int:
    import cv2
    import numpy as np

    validate_args(args)
    try:
        from a2d_sdk.robot import CosineCamera
    except ImportError as exc:
        raise RuntimeError(
            "GDK import failed. Activate the project environment, source "
            "~/a2d_sdk/env.sh, and ensure GDK dependencies are installed."
        ) from exc

    groups = args.camera_group or [args.color_name, args.depth_name]
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

    LOGGER.info("colour stream: %s; depth stream: %s; groups: %s", args.color_name, args.depth_name, groups)
    LOGGER.info("sync diagnostic threshold: %.1f ms", args.max_sync_delta_ms)
    LOGGER.info("captures will be saved in: %s", args.save_dir.expanduser().resolve())
    LOGGER.info("this tool never issues robot motion or gripper commands")
    if show_window:
        LOGGER.info("controls: S saves an RGB-D capture set; Q or Esc exits")

    valid_frames = 0
    consecutive_empty = 0
    last_auto_save = time.monotonic()
    window_name = "G1 RGB-D diagnostic (no robot motion)"

    try:
        while not stop_requested:
            try:
                colour_value = camera.get_latest_image(args.color_name)
                colour_image, colour_timestamp_ns = unpack_camera_frame(colour_value)
                if colour_image is None:
                    raise RuntimeError("colour frame is empty")
                # Current G1 SDK returns (image, timestamp_ns).  A fallback
                # keeps the tool usable with an SDK patch that returns only
                # an ndarray, but the result is then marked as unsynchronised
                # rather than silently treated as timestamp-validated.
                nearest_timestamp_ns = colour_timestamp_ns or time.time_ns()
                depth_value = camera.get_image_nearest(args.depth_name, nearest_timestamp_ns)
                depth_image, depth_timestamp_ns = unpack_camera_frame(depth_value)
                if depth_image is None:
                    raise RuntimeError("nearest depth frame is empty")
            except Exception as exc:
                consecutive_empty += 1
                if consecutive_empty == 1 or consecutive_empty % 20 == 0:
                    LOGGER.warning(
                        "RGB-D read failed (%d/%d): %s",
                        consecutive_empty,
                        args.max_empty_frames,
                        exc,
                    )
                if consecutive_empty >= args.max_empty_frames:
                    raise RuntimeError("camera stopped returning valid synchronized RGB-D frames") from exc
                time.sleep(0.05)
                continue

            consecutive_empty = 0
            color_bgr = ensure_color_bgr(colour_image)
            depth = ensure_depth_2d(depth_image)
            statistics = depth_statistics(depth)
            depth_preview = colorize_depth(depth, statistics)
            sync_delta_ms = timestamp_delta_ms(colour_timestamp_ns, depth_timestamp_ns)
            panel, overlay, same_shape = make_panel(
                color_bgr,
                depth_preview,
                sync_delta_ms,
                args.max_sync_delta_ms,
                statistics,
            )
            valid_frames += 1

            sync_pass = (
                None
                if sync_delta_ms is None
                else abs(sync_delta_ms) <= args.max_sync_delta_ms
            )
            if args.log_every > 0 and valid_frames % args.log_every == 0:
                LOGGER.info(
                    "pair=%d color=%s depth=%s raw=%s sync_delta_ms=%s sync_pass=%s same_shape=%s valid_depth=%.1f%%",
                    valid_frames,
                    tuple(color_bgr.shape),
                    tuple(depth.shape),
                    depth.dtype,
                    "--" if sync_delta_ms is None else f"{sync_delta_ms:+.2f}",
                    sync_pass,
                    same_shape,
                    float(statistics["valid_positive_ratio"] or 0.0) * 100.0,
                )

            metadata: dict[str, Any] = {
                "saved_at": datetime.now().astimezone().isoformat(),
                "tool": "tools/check_rgbd.py",
                "colour_stream": args.color_name,
                "depth_stream": args.depth_name,
                "colour_timestamp_ns": colour_timestamp_ns,
                "depth_timestamp_ns": depth_timestamp_ns,
                "depth_minus_colour_ms": sync_delta_ms,
                "max_sync_delta_ms": args.max_sync_delta_ms,
                "sync_within_threshold": sync_pass,
                "color_shape_hwc": [int(value) for value in color_bgr.shape],
                "depth_shape_hw": [int(value) for value in depth.shape],
                "same_image_size": same_shape,
                "depth_statistics_raw": statistics,
                "interpretation": [
                    "depth_raw.npy preserves the exact numeric depth frame.",
                    "No depth unit conversion has been applied.",
                    "same_image_size is not proof that RGB and depth are registered.",
                    "Do not set depth_registered_to_color=true until a physical alignment test passes.",
                ],
            }

            save_now = False
            if args.save_every > 0 and time.monotonic() - last_auto_save >= args.save_every:
                save_now = True
                last_auto_save = time.monotonic()

            if show_window:
                display = panel
                if args.display_scale != 1.0:
                    display = cv2.resize(
                        panel,
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
                capture_dir = save_capture(
                    color_bgr,
                    depth,
                    depth_preview,
                    panel,
                    overlay,
                    metadata,
                    args.save_dir.expanduser().resolve(),
                )
                LOGGER.info("saved RGB-D diagnostic set: %s", capture_dir)

            if args.max_frames > 0 and valid_frames >= args.max_frames:
                break
    finally:
        try:
            camera.close()
        finally:
            if show_window:
                cv2.destroyAllWindows()

    LOGGER.info("stopped after %d valid RGB-D pairs", valid_frames)
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

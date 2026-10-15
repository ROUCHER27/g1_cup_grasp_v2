#!/usr/bin/env python3
"""Record local G1 head RGB-D evidence without sending robot commands."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from g1_cup_grasp.io.camera import GdkRgbdSource
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait


def json_safe(value, invalid, path="$"):
    """Preserve unavailable numeric feedback as null with explicit provenance."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        invalid.append(path)
        return None
    if isinstance(value, dict):
        return {key: json_safe(item, invalid, path + "." + key) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item, invalid, path + f"[{index}]") for index, item in enumerate(value)]
    return value


def _state(dds, slam):
    result = {}
    for name, method in (("head", "head_joint_states"), ("waist", "waist_joint_states"), ("arms", "arm_joint_states")):
        try:
            values, stamp = getattr(dds, method)()
            result[name] = {"values": [float(v) for v in values], "timestamp_ns": int(stamp)}
        except Exception as exc:
            result[name] = {"error": str(exc)}
    try:
        pose, stamp = slam.get_chassis_position()
        result["chassis"] = {"feedback": dict(pose), "timestamp_ns": int(stamp),
                             "runtime_map_id_verified": False}
    except Exception as exc:
        result["chassis"] = {"error": str(exc)}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not np.isfinite(args.duration_s) or not 1 <= args.duration_s <= 30:
        parser.error("duration must be between 1 and 30 seconds")
    if not np.isfinite(args.fps) or not 1 <= args.fps <= 15:
        parser.error("fps must be between 1 and 15")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rgb_dir, depth_dir = args.output_dir / "rgb", args.output_dir / "depth"
    rgb_dir.mkdir(); depth_dir.mkdir()
    config = yaml.safe_load((ROOT / "config/grasp_v2.yaml").read_text())
    camera_config = config["camera"]
    report = {"status": "RECORDING", "read_only": True, "robot_commands_sent": False,
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "camera_config": camera_config, "calibration": config["calibration"],
              "calibration_live_pose_verified": False, "requested_duration_s": args.duration_s,
              "requested_fps": args.fps, "frames": [], "dropped_unsynchronized": 0,
              "camera_read_timeouts": 0, "runtime_map_id_verified": False}
    source = None
    code = 1
    try:
        print("Opening read-only head color/depth subscribers...", flush=True)
        source = GdkRgbdSource(camera_config["group"], camera_config["color_name"], camera_config["depth_name"])
        from a2d_sdk.robot import RobotDds, Slam
        dds, slam = RobotDds(), Slam()  # Subscriptions/clients only; no service or motion calls.
        print("Waiting for updating synchronized RGB-D frames...", flush=True)
        startup_deadline = time.monotonic() + 20
        started = None; previous_stamp = None; last_saved = 0.; last_progress = 0.
        while time.monotonic() < (startup_deadline if started is None else started + args.duration_s):
            try:
                frame = source.read()
            except TimeoutError:
                report["camera_read_timeouts"] += 1
                continue
            now = time.monotonic()
            stamp = frame.color.timestamp_ns
            if stamp == previous_stamp or (previous_stamp is not None and stamp < previous_stamp):
                time.sleep(.01); continue
            sync_ms = abs(stamp - frame.depth.timestamp_ns) / 1e6
            if sync_ms > camera_config["max_sync_delta_ms"]:
                report["dropped_unsynchronized"] += 1
                time.sleep(.01); continue
            if now - last_saved < 1. / args.fps:
                time.sleep(.005); continue
            if frame.depth.image.dtype != np.uint16:
                raise RuntimeError(f"unexpected raw depth dtype: {frame.depth.image.dtype}")
            if frame.depth.image.shape != frame.color.image.shape[:2]:
                raise RuntimeError("registered depth and color shapes differ")
            if started is None:
                started = now
                report["state_before"] = _state(dds, slam)
            index = len(report["frames"])
            rgb = rgb_dir / f"frame_{index:05d}.jpg"
            depth = depth_dir / f"frame_{index:05d}.png"
            if not cv2.imwrite(str(rgb), frame.color.image, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                raise RuntimeError("failed to save color frame")
            if not cv2.imwrite(str(depth), frame.depth.image):
                raise RuntimeError("failed to save raw depth frame")
            values = frame.depth.image[frame.depth.image > 0].astype(float) * camera_config["depth_scale"]
            report["frames"].append({"index": index, "color_path": str(rgb), "depth_path": str(depth),
                                     "color_timestamp_ns": stamp, "depth_timestamp_ns": frame.depth.timestamp_ns,
                                     "sync_delta_ms": sync_ms, "elapsed_s": now - started,
                                     "valid_depth_fraction": float(np.count_nonzero(frame.depth.image) / frame.depth.image.size),
                                     "depth_percentiles_m": np.percentile(values, [5, 50, 95]).tolist() if values.size else None})
            with (args.output_dir / "frames.jsonl").open("a") as frames_file:
                frames_file.write(json.dumps(report["frames"][-1], allow_nan=False) + "\n")
            previous_stamp, last_saved = stamp, now
            if now - last_progress >= 2:
                print(f"Captured {index + 1} frames; elapsed {now - started:.1f}s", flush=True)
                last_progress = now
        if not report["frames"]:
            raise RuntimeError("no synchronized RGB-D frames arrived before startup deadline")
        report["state_after"] = _state(dds, slam)
        report["image_shape_hw"] = list(frame.depth.image.shape)
        report["frame_count"] = len(report["frames"])
        report["capture_span_s"] = report["frames"][-1]["elapsed_s"]
        report["status"] = "RGBD_CAPTURED"
        report["depth_storage"] = "uint16 PNG, lossless; multiply raw values by camera_config.depth_scale for meters"
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
    finally:
        path = args.output_dir / "capture_report.json"
        invalid = []
        report = json_safe(report, invalid)
        report["invalid_feedback_fields"] = invalid
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        if source is not None:
            source.close()
        print(json.dumps({"report_path": str(path), "status": report["status"], "frames": len(report["frames"]),
                          "error": report.get("error"), "read_only": True}, ensure_ascii=False), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

#!/usr/bin/env python3
"""Capture a V2 side-grasp pose without commanding the robot."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from g1_cup_grasp.config import load_config
from g1_cup_grasp.io.camera import GdkRgbdSource
from g1_cup_grasp.io.robot import GdkRightArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait
from g1_cup_grasp.perception.yolo_segmenter import YoloCupSegmenter
from grasp_v2 import capture_stable_target, target_to_dict


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config" / "grasp_v2.yaml")
    parser.add_argument("--device", default=None)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "outputs" / "side_teach")
    return parser


def run(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    camera_cfg = config["camera"]
    model_cfg = config["model"]
    robot_cfg = config["robot"]
    robot = GdkRightArm(robot_cfg)
    camera = GdkRgbdSource(
        list(camera_cfg["group"]), str(camera_cfg["color_name"]), str(camera_cfg["depth_name"])
    )
    try:
        state = robot.read_ready_state()
        head_error = abs(float(state.head_joint_rad[1]) - float(robot_cfg["head_pitch_reference_deg"]))
        waist_error = float(np.linalg.norm(
            np.asarray(state.waist_state) - np.asarray(robot_cfg["waist_reference"])
        ))
        if head_error > float(robot_cfg["head_pitch_tolerance_deg"]):
            raise RuntimeError("Head pitch differs from the hand-eye calibration pose")
        if waist_error > float(robot_cfg["waist_tolerance"]):
            raise RuntimeError("Waist pose differs from the hand-eye calibration pose")

        segmenter = YoloCupSegmenter(
            args.model or PROJECT_ROOT / model_cfg["path"],
            class_id=int(model_cfg["class_id"]), class_name=str(model_cfg["class_name"]),
            imgsz=int(model_cfg["imgsz"]), device=args.device or model_cfg["device"],
        )
        target, _, _, stability = capture_stable_target(camera, segmenter, config)
        link7 = robot.current_end_pose()
        midpoint = target.point_base_m - np.array(
            [0.0, 0.0, float(config["grasp"]["cup_height_m"]) / 2.0]
        )
        rotation = Rotation.from_euler("xyz", link7.rpy_rad)
        offset = rotation.inv().apply(midpoint - link7.position_m)
        report = {
            "status": "SIDE_TEACH_CAPTURED",
            "note": "Candidate only; move the cup and validate plan-only before setting grasp.calibrated=true.",
            "target": target_to_dict(target),
            "stability": stability,
            "link7_pose": {
                "position_m": link7.position_m.tolist(),
                "rpy_rad": link7.rpy_rad.tolist(),
            },
            "grasp_config_candidate": {
                "base_rpy_rad": link7.rpy_rad.tolist(),
                "anchor_to_link7_offset_m": offset.tolist(),
                "calibrated": False,
            },
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        path = args.output_dir / f"side_teach_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        report["report_path"] = str(path)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    finally:
        # shortcut: GDK close can block while holding the GIL; process exit releases these resources.
        pass


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(json.dumps({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"},
                         ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

#!/usr/bin/env python3
"""Plan and optionally execute a V2 right-arm side grasp for one upright cup."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from g1_cup_grasp.config import load_config
from g1_cup_grasp.domain import Detection, EndPose, Target3D
from g1_cup_grasp.geometry.deprojection import CameraIntrinsics, estimate_target_from_mask
from g1_cup_grasp.geometry.transforms import invert_transform, load_transform, matrix_to_rpy, transform_point
from g1_cup_grasp.grasp_planner import GraspPlanningResult, plan_side_grasp, result_to_dict, load_nearby_seed, save_ik_seed
from g1_cup_grasp.io.camera import GdkRgbdSource
from g1_cup_grasp.io.robot import GdkRightArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait
from g1_cup_grasp.kinematics import UrdfRightArm
from g1_cup_grasp.perception.yolo_segmenter import YoloCupSegmenter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config" / "grasp_v2.yaml")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--execute", action="store_true", help="execute the selected route")
    parser.add_argument("--enable-real", action="store_true", help="second required real-motion flag")
    parser.add_argument(
        "--stop-after", choices=["clearance", "pregrasp", "grasp", "lift", "return", "retreat"],
        default=None, help="stop after one named execution stage",
    )
    parser.add_argument("--repeat", action="store_true", help="keep resources loaded; press Enter for each new round, q to quit")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--save-dir", type=Path, default=PROJECT_ROOT / "outputs" / "plans")
    return parser


def target_to_dict(target: Target3D) -> dict[str, Any]:
    return {
        "pixel_uv": list(target.pixel_uv),
        "depth_m": target.depth_m,
        "point_camera_m": target.point_camera_m.tolist(),
        "point_base_m": target.point_base_m.tolist(),
        "confidence": target.confidence,
        "valid_depth_count": target.valid_depth_count,
    }


def capture_stable_target(camera: GdkRgbdSource, segmenter: YoloCupSegmenter,
                          config: dict[str, Any]) -> tuple[Target3D, Any, Detection, dict[str, Any]]:
    runtime = config["runtime"]
    camera_cfg = config["camera"]
    grasp_cfg = config["grasp"]
    required = int(runtime.get("stable_frames", 3))
    if required <= 0:
        raise ValueError("runtime.stable_frames must be positive")
    threshold = float(runtime.get("stable_position_std_m", 0.008))
    intrinsics = CameraIntrinsics.from_config(camera_cfg["color_intrinsics"])
    base_from_camera = load_transform(config["calibration"]["T_base_camera"])
    samples: list[tuple[Target3D, Any, Detection, float]] = []
    last_error = "no frame"

    for _ in range(max(20, required * 10)):
        frame = camera.read()
        sync_ms = abs(frame.depth.timestamp_ns - frame.color.timestamp_ns) / 1e6
        if sync_ms > float(camera_cfg["max_sync_delta_ms"]):
            last_error = f"RGB-D sync delta {sync_ms:.1f} ms"
            time.sleep(0.1)
            continue
        detections = segmenter.predict(
            frame.color.image, confidence=float(runtime.get("max_detection_confidence", 0.65))
        )
        if not detections:
            last_error = "no cup detection"
            continue
        detection = detections[0]
        try:
            target = estimate_target_from_mask(
                detection=detection,
                depth_image=frame.depth.image,
                color_intrinsics=intrinsics,
                depth_scale=float(camera_cfg["depth_scale"]),
                T_base_camera=base_from_camera,
                depth_registered_to_color=bool(camera_cfg["depth_registered_to_color"]),
                min_valid_depth=int(camera_cfg.get("min_valid_depth_pixels", 20)),
                target_anchor=str(grasp_cfg.get("target_anchor", "bbox_top_center")),
            )
        except ValueError as exc:
            last_error = str(exc)
            continue
        samples.append((target, frame, detection, sync_ms))
        if len(samples) < required:
            continue
        recent = samples[-required:]
        points = np.stack([item[0].point_base_m for item in recent])
        median = np.median(points, axis=0)
        spread = float(np.max(np.linalg.norm(points - median, axis=1)))
        if spread > threshold:
            last_error = f"target spread {spread * 1000:.1f} mm"
            continue
        index = int(np.argmin(np.linalg.norm(points - median, axis=1)))
        target, frame, detection, sync_ms = recent[index]
        return target, frame, detection, {
            "frames": required,
            "max_spread_mm": spread * 1000.0,
            "sync_delta_ms": sync_ms,
        }
    raise RuntimeError(f"No stable cup target: {last_error}")


def validate_robot_model(model: UrdfRightArm, robot: GdkRightArm, state: Any,
                         config: dict[str, Any]) -> dict[str, Any]:
    joints = robot.right_arm_joints(state)
    waist = robot.waist_for_urdf(state)
    transform = model.forward(joints, waist)
    predicted = EndPose(transform[:3, 3], matrix_to_rpy(transform[:3, :3]), "base_link")
    actual = robot.current_end_pose()
    position_error = float(np.linalg.norm(predicted.position_m - actual.position_m))
    orientation_error = float(np.rad2deg(
        (Rotation.from_euler("xyz", predicted.rpy_rad).inv()
         * Rotation.from_euler("xyz", actual.rpy_rad)).magnitude()
    ))
    limits = config["model_validation"]
    passed = (
        position_error <= float(limits["position_tolerance_m"])
        and orientation_error <= float(limits["orientation_tolerance_deg"])
    )
    return {
        "passed": bool(passed),
        "position_error_mm": position_error * 1000.0,
        "orientation_error_deg": orientation_error,
        "predicted_position_m": predicted.position_m.tolist(),
        "actual_position_m": actual.position_m.tolist(),
        "right_arm_joint_rad": joints.tolist(),
        "waist_urdf": waist.tolist(),
    }


def _project(point_base: np.ndarray, camera_from_base: np.ndarray,
             intrinsics: CameraIntrinsics) -> tuple[int, int] | None:
    point = transform_point(camera_from_base, point_base)
    if point[2] <= 0.05:
        return None
    u = int(round(intrinsics.fx * point[0] / point[2] + intrinsics.cx))
    v = int(round(intrinsics.fy * point[1] / point[2] + intrinsics.cy))
    if intrinsics.width is not None and not 0 <= u < intrinsics.width:
        return None
    if intrinsics.height is not None and not 0 <= v < intrinsics.height:
        return None
    return u, v


def render_preview(frame: Any, detection: Detection, target: Target3D,
                   plan: GraspPlanningResult | None, config: dict[str, Any],
                   model_validation: dict[str, Any]) -> np.ndarray:
    image = np.asarray(frame.color.image).copy()
    mask = np.asarray(detection.mask, dtype=bool)
    if mask.shape != image.shape[:2]:
        mask = cv2.resize(mask.astype(np.uint8), (image.shape[1], image.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    tint = image.copy()
    tint[mask] = (40, 210, 40)
    image = cv2.addWeighted(tint, 0.35, image, 0.65, 0.0)
    x1, y1, x2, y2 = (int(round(value)) for value in detection.bbox_xyxy)
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
    u, v = (int(round(value)) for value in target.pixel_uv)
    cv2.drawMarker(image, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)

    if plan is not None and plan.selected is not None:
        intrinsics = CameraIntrinsics.from_config(config["camera"]["color_intrinsics"])
        camera_from_base = invert_transform(load_transform(config["calibration"]["T_base_camera"]))
        pixels = [_project(item.pose.position_m, camera_from_base, intrinsics)
                  for item in plan.selected.waypoints]
        visible = [pixel for pixel in pixels if pixel is not None]
        if len(visible) > 1:
            cv2.polylines(image, [np.asarray(visible, dtype=np.int32)], False, (255, 0, 255), 2)
        for index, pixel in enumerate(pixels):
            if pixel is None:
                continue
            cv2.circle(image, pixel, 5, (255, 0, 255), -1)
            cv2.putText(image, str(index + 1), (pixel[0] + 6, pixel[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

    status = "MODEL_OK" if model_validation["passed"] else "MODEL_MISMATCH"
    plan_status = "NOT_PLANNED" if plan is None else plan.status
    lines = [
        f"target uv=({u},{v}) depth={target.depth_m:.3f}m",
        "base xyz=" + ", ".join(f"{value:.3f}" for value in target.point_base_m),
        f"model={status} pos={model_validation['position_error_mm']:.1f}mm "
        f"rot={model_validation['orientation_error_deg']:.1f}deg",
        f"plan={plan_status}",
    ]
    if plan is not None and plan.selected is not None:
        lines.append(f"selected yaw={plan.selected.yaw_offset_deg:+.0f}deg score={plan.selected.score:.3f}")
        lines.append(f"pregrasp={plan.selected.pregrasp_distance_m * 1000:.0f}mm "
                     f"clearance={plan.selected.clearance_height_m * 1000:.0f}mm")
    elif plan is not None:
        for candidate in plan.candidates[:3]:
            lines.append(f"yaw={candidate.yaw_offset_deg:+.0f} "
                         f"d={candidate.pregrasp_distance_m * 1000:.0f} "
                         f"h={candidate.clearance_height_m * 1000:.0f}mm: {candidate.reason}")
    overlay = image.copy()
    cv2.rectangle(overlay, (8, 8), (790, 26 + 27 * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, image, 0.40, 0.0, image)
    for index, line in enumerate(lines):
        cv2.putText(image, line, (18, 35 + index * 27), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (80, 255, 80), 2, cv2.LINE_AA)
    return image


def execute_plan(robot: GdkRightArm, plan: GraspPlanningResult,
                 config: dict[str, Any], stop_after: str | None) -> list[dict[str, Any]]:
    selected = plan.selected
    if selected is None:
        raise RuntimeError("No selected grasp candidate")
    robot_cfg = config["robot"]
    execution_cfg = config["execution"]
    results: list[dict[str, Any]] = []
    robot.move_gripper(float(robot_cfg["gripper_open"]))
    time.sleep(1.0)
    for waypoint in selected.waypoints:
        result = robot.move_to_pose(
            waypoint.pose,
            duration_s=float(execution_cfg["durations_s"][waypoint.stage]),
            sample_hz=float(robot_cfg["trajectory_sample_hz"]),
            position_tolerance_m=float(execution_cfg["stage_position_tolerance_m"][waypoint.stage]),
            orientation_tolerance_deg=float(execution_cfg["stage_orientation_tolerance_deg"][waypoint.stage]),
        )
        result["stage"] = waypoint.stage
        if waypoint.stage == "grasp":
            robot.move_gripper(float(robot_cfg["gripper_closed"]))
            time.sleep(1.0)
            result["gripper"] = "closed"
        elif waypoint.stage == "return":
            robot.move_gripper(float(robot_cfg["gripper_open"]))
            time.sleep(1.0)
            result["gripper"] = "opened"
        results.append(result)
        if stop_after == waypoint.stage:
            break
    return results


def save_outputs(save_dir: Path, report: dict[str, Any], preview: np.ndarray) -> tuple[Path, Path]:
    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    report_path = save_dir / f"grasp_v2_{stamp}.json"
    preview_path = save_dir / f"grasp_v2_{stamp}_preview.jpg"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if not cv2.imwrite(str(preview_path), preview):
        raise OSError(f"Could not save preview: {preview_path}")
    return report_path, preview_path


def run(args: argparse.Namespace) -> int:
    if args.execute != args.enable_real:
        raise ValueError("Real execution requires both --execute and --enable-real")
    config = load_config(args.config)
    model_cfg = config["model"]
    robot_cfg = config["robot"]
    camera_cfg = config["camera"]
    urdf_path = PROJECT_ROOT / robot_cfg["urdf_path"]
    model = UrdfRightArm(
        urdf_path, robot_cfg["urdf_base_link"], robot_cfg["urdf_tip_link"], config["ik"]
    )
    robot = GdkRightArm(robot_cfg, pose_model=model)
    motion_started = False
    try:
        segmenter = YoloCupSegmenter(
            args.model or PROJECT_ROOT / model_cfg["path"],
            class_id=int(model_cfg["class_id"]), class_name=str(model_cfg["class_name"]),
            imgsz=int(model_cfg["imgsz"]), device=args.device or model_cfg["device"],
        )
        camera = GdkRgbdSource(
            list(camera_cfg["group"]), str(camera_cfg["color_name"]), str(camera_cfg["depth_name"])
        )
        seed_path = PROJECT_ROOT / "outputs" / "ik_seed_cache.json"
        while True:
            if args.repeat:
                prompt = ("Confirm cup and route clear; Enter to plan and EXECUTE, q to quit: "
                          if args.execute else "Enter to plan only (no motion), q to quit: ")
                try:
                    answer = input(prompt).strip().lower()
                except EOFError:
                    return 0
                if answer == "q":
                    return 0
                if answer:
                    continue
            target, frame, detection, stability = capture_stable_target(camera, segmenter, config)
            state = robot.read_ready_state()
            model_validation = validate_robot_model(model, robot, state, config)
            plan = None
            waist = robot.waist_for_urdf(state)
            nearby = load_nearby_seed(seed_path, target, waist, config)
            if model_validation["passed"]:
                plan = plan_side_grasp(
                    target, robot.right_arm_joints(state), robot.waist_for_urdf(state),
                    model, config["grasp"], nearby_seed=nearby,
                )
            status = "MODEL_MISMATCH" if not model_validation["passed"] else plan.status
            report: dict[str, Any] = {
                "status": status,
                "nearby_ik_seed_used": nearby is not None,
                "mode": "execute" if args.execute else "plan_only",
                "target": target_to_dict(target),
                "stability": stability,
                "model_validation": model_validation,
                "ik_joint_names": model.joint_names,
                "plan": None if plan is None else result_to_dict(plan),
                "real_motion_gates": {
                    "cli_execute": args.execute,
                    "cli_enable_real": args.enable_real,
                    "runtime_enable_real_motion": bool(config["runtime"]["enable_real_motion"]),
                    "calibration_enabled": bool(config["calibration"]["enabled"]),
                    "grasp_calibrated": bool(config["grasp"]["calibrated"]),
                },
            }
            preview = render_preview(frame, detection, target, plan, config, model_validation)
            report_path, preview_path = save_outputs(args.save_dir, report, preview)
            report["report_path"] = str(report_path)
            report["preview_path"] = str(preview_path)
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)

            if not args.no_window and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                cv2.imshow("G1 cup grasp V2 plan", preview)
                cv2.waitKey(1500 if args.execute else 0)
                cv2.destroyAllWindows()

            if plan is not None and plan.selected is not None:
                save_ik_seed(seed_path, plan, waist, config)
            if not args.execute:
                if args.repeat:
                    continue
                return 0 if status == "PLAN_READY" else 2
            if status != "PLAN_READY" or plan is None:
                raise RuntimeError(f"Cannot execute plan with status {status}")
            gates = report["real_motion_gates"]
            if not all(gates.values()):
                disabled = [name for name, enabled in gates.items() if not enabled]
                raise RuntimeError("Real motion is disabled by: " + ", ".join(disabled))
            robot.assert_execution_ready(robot.read_ready_state())
            motion_started = True
            report["execution"] = execute_plan(robot, plan, config, args.stop_after)
            report["status"] = "EXECUTION_COMPLETE" if args.stop_after is None else f"STOPPED_AFTER_{args.stop_after.upper()}"
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            print(json.dumps({"status": report["status"], "report_path": str(report_path)},
                             ensure_ascii=False), flush=True)
            motion_started = False
            if not args.repeat:
                return 0
    except BaseException:
        if motion_started:
            robot.stop()
        raise
    finally:
        # shortcut: GDK close can block while holding the GIL; process exit releases these resources.
        pass


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(json.dumps({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"},
                         ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

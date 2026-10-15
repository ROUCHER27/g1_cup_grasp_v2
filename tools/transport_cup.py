#!/usr/bin/env python3
"""Fresh cup detection -> grasp -> carry pose -> B -> taught release. Default: no motion."""
from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"src"),str(ROOT/"tools")]
from g1_cup_grasp.carry import pose_data, pose_from_data, plan_carry, footprint, require_grip
from g1_cup_grasp.config import load_config
from g1_cup_grasp.grasp_planner import plan_side_grasp, result_to_dict
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.transport_arm import FreshTransportArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait
from g1_cup_grasp.kinematics import UrdfRightArm
from grasp_v2 import capture_stable_target, validate_robot_model, execute_plan, target_to_dict
from check_navigation import collect_samples, read_hmi, read_controller, runtime_map_check, summarize
from navigate_station import (HAL_SHA256, navigation_request, validate_preflight,
                              validate_transport_config, load_avoidance_manifest,
                              execute_navigation, pause_and_confirm)


def context_digest(grasp, transport):
    payload=json.dumps({"grasp":grasp,"transport":transport},sort_keys=True).encode()
    return hashlib.sha256(payload+(ROOT/grasp["robot"]["urdf_path"]).read_bytes()).hexdigest()


def check_hold_receipt(receipt, arm, state, config, context):
    if receipt.get("status")!="CARRY_POSE_REACHED" or receipt.get("context")!=context:
        raise RuntimeError("carry receipt missing or configuration changed; prepare a new carry pose")
    age=time.time()-receipt["created_unix_s"]
    if not 0 <= age <= 600 or receipt.get("map_id")!=config["map_id"]:
        raise RuntimeError("carry receipt is stale or belongs to another map")
    require_grip(state,config["carry"])
    actual=arm.current_end_pose()
    wanted=pose_from_data(receipt["pose"])
    error=float(np.linalg.norm(actual.position_m-wanted.position_m))
    rotation=math.degrees((Rotation.from_euler("xyz",actual.rpy_rad).inv()*Rotation.from_euler("xyz",wanted.rpy_rad)).magnitude())
    if error>config["carry"]["position_tolerance_m"] or rotation>config["carry"]["orientation_tolerance_deg"]:
        raise RuntimeError("current held pose differs from the prepared carry pose")


def choose_carry(plan,model,waist,config,cup_height):
    candidates=sorted((i for i,c in enumerate(plan.candidates) if c.feasible),key=lambda i:plan.candidates[i].score)
    failures=[]
    for index in candidates:
        waypoints={p.stage:p for p in plan.candidates[index].waypoints}
        try:
            carry=plan_carry(model,waypoints["lift"].pose,waypoints["lift"].joints_rad,waist,config,
                             waypoints["grasp"].pose,plan.target.point_base_m,cup_height)
            return replace(plan,selected_index=index),carry
        except RuntimeError as error:
            failures.append(f"candidate {index}: {error}")
    raise RuntimeError("no feasible grasp+carry route: "+"; ".join(failures))


def place_pose_at_base(teaching, current_base):
    source=teaching["base_pose"]
    source_rotation=Rotation.from_euler("z",source["agv_angle"])
    current_rotation=Rotation.from_euler("z",current_base["agv_angle"])
    world=source_rotation.apply(teaching["position_m"])+np.array([source["agv_pos_x"],source["agv_pos_y"],0.])
    position=current_rotation.inv().apply(world-np.array([current_base["agv_pos_x"],current_base["agv_pos_y"],0.]))
    orientation=current_rotation.inv()*source_rotation*Rotation.from_euler("xyz",teaching["rpy_rad"])
    return pose_from_data({"position_m":position.tolist(),"rpy_rad":orientation.as_euler("xyz").tolist()})


def execute_place(arm, target, grasp, carry, report):
    report["place_target"]=pose_data(target)
    report["place_execution"]=arm.move_to_pose(target,carry["duration_s"],grasp["robot"]["trajectory_sample_hz"],
        carry["position_tolerance_m"],carry["orientation_tolerance_deg"])
    report["gripper_release_requested"]=True
    arm.move_gripper(float(grasp["robot"]["gripper_open"]))
    time.sleep(1.0)
    report["status"]="ARRIVED_B_CUP_RELEASED"
    report["arm_and_gripper_remain_in_hold"]=False


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute",action="store_true")
    parser.add_argument("--enable-real",action="store_true")
    parser.add_argument("--stop-after",choices=["carry"],default=None)
    parser.add_argument("--resume-navigation",action="store_true",help="keep gripper closed; use fresh carry receipt")
    parser.add_argument("--offline-grasp-report",type=Path,help="saved report for offline kinematics only; cannot execute")
    parser.add_argument("--device",default="cpu")
    args=parser.parse_args(argv)
    report={"started_at_utc":datetime.now(timezone.utc).isoformat(),"read_only":True,
            "request_attempted":False,"request_accepted":False,"gripper_release_requested":False,
            "physical_cup_stability_verified":False,"collision_model_verified":False}
    code=0
    native=arm=slam=None
    arm_motion_started=False
    try:
        if args.execute!=args.enable_real:
            raise RuntimeError("real execution requires both --execute and --enable-real")
        if args.offline_grasp_report and (args.execute or args.resume_navigation or args.stop_after):
            raise RuntimeError("saved/offline grasp data cannot be used for execution")
        if args.resume_navigation and args.stop_after:
            raise RuntimeError("--resume-navigation and --stop-after cannot be combined")
        grasp=load_config(ROOT/"config/grasp_v2.yaml")
        config=yaml.safe_load((ROOT/"config/transport.yaml").read_text(encoding="utf-8-sig"))
        validate_transport_config(config)
        load_avoidance_manifest(config)
        context=context_digest(grasp,config)
        robot=grasp["robot"]
        model=UrdfRightArm(ROOT/robot["urdf_path"],robot["urdf_base_link"],robot["urdf_tip_link"],grasp["ik"])
        report["planned_request"]=navigation_request(config["stations"]["B"])
        if config.get("place") is not None:
            station=config["stations"]["B"]
            report["planned_place_at_B"]=pose_data(place_pose_at_base(config["place"],
                {"agv_pos_x":station["x_m"],"agv_pos_y":station["y_m"],"agv_angle":station["yaw_rad"]}))
        if args.offline_grasp_report:
            saved=json.loads(args.offline_grasp_report.read_text(encoding="utf-8-sig"))
            index=saved["plan"]["selected_index"]
            candidate=saved["plan"]["candidates"][index]
            points={p["stage"]:p for p in candidate["waypoints"]}
            report["carry_plan"]=plan_carry(model,pose_from_data(points["lift"]),points["lift"]["right_arm_joint_rad"],
                saved["model_validation"]["waist_urdf"],config["carry"],pose_from_data(points["grasp"]),
                saved["target"]["point_base_m"],grasp["grasp"]["cup_height_m"])
            report.update(status="OFFLINE_KINEMATICS_READY",offline_source=str(args.offline_grasp_report),runtime_map_verified=False)
        else:
            native=NativeChassisFeedback(config["native_feedback"])
            samples=collect_samples(native)
            report.update(initial_samples=samples,runtime_map=runtime_map_check(samples,config["map_id"]),
                          initial_summary_vs_A=summarize(samples,config["stations"]["A"]))
            if not report["runtime_map"]["verified"]:
                raise RuntimeError(f"updating original position feedback does not confirm map {config['map_id']}")
            if any(s["hal_sha256"]!=HAL_SHA256 for s in samples):
                raise RuntimeError("robot HAL version changed; request units require revalidation")
            hmi,controller=read_hmi(config),read_controller(config)
            report.update(hmi=hmi,controller=controller)
            if args.execute:
                validate_preflight(samples,hmi,controller,config)
                if not all((grasp["runtime"]["enable_real_motion"],grasp["calibration"]["enabled"],grasp["grasp"]["calibrated"])):
                    raise RuntimeError("grasp execution/calibration is disabled")
            arm=FreshTransportArm(robot,model)
            state=arm.read_ready_state()
            report["model_validation"]=validate_robot_model(model,arm,state,grasp)
            if not report["model_validation"]["passed"]:
                raise RuntimeError("URDF and current end-effector feedback do not agree")
            waist=arm.waist_for_urdf(state)
            hold_path=ROOT/"outputs/transport/latest_hold.json"
            if args.resume_navigation:
                receipt=json.loads(hold_path.read_text())
                check_hold_receipt(receipt,arm,state,config,context)
                report["hold_receipt"]=receipt
            else:
                from g1_cup_grasp.io.camera import GdkRgbdSource
                from g1_cup_grasp.perception.yolo_segmenter import YoloCupSegmenter
                camera=GdkRgbdSource(grasp["camera"]["group"],grasp["camera"]["color_name"],grasp["camera"]["depth_name"])
                segmenter=YoloCupSegmenter(ROOT/grasp["model"]["path"],class_id=grasp["model"]["class_id"],
                    class_name=grasp["model"]["class_name"],imgsz=grasp["model"]["imgsz"],device=args.device)
                target,frame,detection,stability=capture_stable_target(camera,segmenter,grasp)
                plan=plan_side_grasp(target,arm.right_arm_joints(state),waist,model,grasp["grasp"],include_return=False)
                report.update(target=target_to_dict(target),stability=stability,grasp_plan=result_to_dict(plan))
                plan,carry=choose_carry(plan,model,waist,config["carry"],grasp["grasp"]["cup_height_m"])
                report.update(target=target_to_dict(target),stability=stability,grasp_plan=result_to_dict(plan),carry_plan=carry)
                left=UrdfRightArm(ROOT/robot["urdf_path"],robot["urdf_base_link"],"arm_l_end_link",grasp["ik"],arm_joint_prefix="idx2")
                report["estimated_footprint"]=footprint(model,left,carry["samples"][-1]["right_arm_joint_rad"],state.arm_joint_rad[:7],waist,carry)
                if args.execute:
                    arm.assert_execution_ready(arm.read_ready_state())
                    validate_preflight(collect_samples(native),read_hmi(config),read_controller(config),config)
                    report["read_only"]=False
                    arm_motion_started=True
                    report["grasp_execution"]=execute_plan(arm,plan,grasp,"lift")
                    require_grip(arm.read_ready_state(),config["carry"])
                    report["carry_execution"]=[]
                    for point in carry["waypoints"]:
                        require_grip(arm.read_ready_state(),config["carry"])
                        report["carry_execution"].append(arm.move_to_pose(pose_from_data(point),config["carry"]["duration_s"],
                            robot["trajectory_sample_hz"],config["carry"]["position_tolerance_m"],config["carry"]["orientation_tolerance_deg"]))
                    receipt={"status":"CARRY_POSE_REACHED","created_unix_s":time.time(),"context":context,
                             "map_id":config["map_id"],"pose":pose_data(arm.current_end_pose())}
                    hold_path.parent.mkdir(parents=True,exist_ok=True)
                    hold_path.write_text(json.dumps(receipt,indent=2))
                    report["hold_receipt"]=receipt
            if not args.execute:
                report["status"]="PLAN_READY_NO_MOTION"
            elif args.stop_after=="carry":
                report["status"]="CARRY_POSE_REACHED_HOLDING"
            else:
                state=arm.read_ready_state()
                arm.assert_execution_ready(state)
                check_hold_receipt(report["hold_receipt"],arm,state,config,context)
                samples=collect_samples(native)
                validate_preflight(samples,read_hmi(config),read_controller(config),config)
                from a2d_sdk.robot import Slam
                slam=Slam()
                execute_navigation(slam,native,arm,config["stations"]["B"],config,samples[-1],report)
                report["status"]="ARRIVED_B_HOLDING_CUP"
                report["arm_and_gripper_remain_in_hold"]=True
                if config.get("place") is not None:
                    target=place_pose_at_base(config["place"],report["navigation_feedback"][-1])
                    arm_motion_started=True
                    execute_place(arm,target,grasp,config["carry"],report)
            arm_motion_started=False
    except (Exception,KeyboardInterrupt) as error:
        code=1
        report.update(status="ERROR",error=f"{type(error).__name__}: {error}")
        if report["request_attempted"] and slam is not None:
            last=report.get("navigation_feedback",[{}])[-1].get("timestamp_ns",0)
            report["stop"]=pause_and_confirm(slam,last,feedback=native)
            if not report["stop"]["stop_confirmed"]:
                report["status"]="STOP_UNCONFIRMED"
        if arm_motion_started and arm is not None:
            try:
                report["arm_stop_response"]=str(arm.stop())
            except Exception as stop_error:
                report["arm_stop_error"]=str(stop_error)
    finally:
        if native is not None:
            native.close()
    directory=ROOT/"outputs/transport"
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+".json")
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({"report_path":str(path),**report},ensure_ascii=False,indent=2),flush=True)
    return code

if __name__=="__main__":
    exit_without_sdk_thread_wait(main())

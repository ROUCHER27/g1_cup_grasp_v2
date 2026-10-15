#!/usr/bin/env python3
"""Record the manually taught carry pose; never command an arm or gripper."""
from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'tools')]
from g1_cup_grasp.carry import pose_data
from g1_cup_grasp.config import load_config
from g1_cup_grasp.io.transport_arm import FreshTransportArm
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait
from g1_cup_grasp.kinematics import UrdfRightArm
from grasp_v2 import validate_robot_model


def capture(arm, model, grasp):
    state = arm.read_ready_state()
    validation = validate_robot_model(model, arm, state, grasp)
    if not validation['passed']:
        raise RuntimeError('taught pose does not agree with the calibrated URDF')
    q = arm.right_arm_joints(state)
    margin = float(np.min(np.minimum(q-model.lower, model.upper-q)))
    if margin < model.limit_margin:
        raise RuntimeError('taught carry pose is too close to a joint limit')
    pose = pose_data(arm.current_end_pose())
    return {'status': 'CARRY_TEACH_CAPTURED', 'read_only': True,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'pose': pose, 'right_arm_joint_rad': q.tolist(),
            'waist_state': state.waist_state, 'head_state': state.head_joint_rad,
            'gripper_feedback_only': state.gripper_state,
            'joint_margin_rad': margin, 'model_validation': validation,
            'carry_config_candidate': pose,
            'note': 'Pose only. Gripper feedback is not replayed; no movement command was sent.'}


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    try:
        grasp = load_config(ROOT / 'config/grasp_v2.yaml')
        robot = grasp['robot']
        model = UrdfRightArm(ROOT / robot['urdf_path'], robot['urdf_base_link'], robot['urdf_tip_link'], grasp['ik'])
        report = capture(FreshTransportArm(robot, model), model, grasp)
        directory = ROOT / 'outputs/carry_teach'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / ('carry_teach_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({'report_path': str(path), **report}, ensure_ascii=False, indent=2), flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps({'status': 'ERROR', 'error': str(error)}, ensure_ascii=False), flush=True)
        return 1


if __name__ == '__main__':
    exit_without_sdk_thread_wait(main())

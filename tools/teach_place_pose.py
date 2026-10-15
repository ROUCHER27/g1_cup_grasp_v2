#!/usr/bin/env python3
"""Record a B-table release pose and chassis pose; no motion commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from g1_cup_grasp.config import load_config
from g1_cup_grasp.io.transport_arm import FreshTransportArm
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait
from g1_cup_grasp.kinematics import UrdfRightArm
from teach_carry_pose import capture


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    native=None
    try:
        grasp=load_config(ROOT/'config/grasp_v2.yaml')
        config=yaml.safe_load((ROOT/'config/transport.yaml').read_text())
        robot=grasp['robot']
        model=UrdfRightArm(ROOT/robot['urdf_path'],robot['urdf_base_link'],robot['urdf_tip_link'],grasp['ik'])
        arm=FreshTransportArm(robot,model)
        native=NativeChassisFeedback(config['native_feedback'])
        base,_=native.get_chassis_position()
        report=capture(arm,model,grasp)
        report.update(status='PLACE_TEACH_CAPTURED',base_pose={key:base[key] for key in
            ('map_id','agv_pos_x','agv_pos_y','agv_angle')})
        report.pop('carry_config_candidate')
        report['place_config_candidate']={**report['pose'],'base_pose':report['base_pose']}
        directory=ROOT/'outputs/place_teach';directory.mkdir(parents=True,exist_ok=True)
        path=directory/('place_teach_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps({'report_path':str(path),**report},ensure_ascii=False,indent=2),flush=True)
        return 0
    except (Exception,KeyboardInterrupt) as error:
        print(json.dumps({'status':'ERROR','error':str(error)},ensure_ascii=False),flush=True)
        return 1
    finally:
        if native is not None:native.close()


if __name__=='__main__':
    exit_without_sdk_thread_wait(main())

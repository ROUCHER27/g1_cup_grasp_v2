from __future__ import annotations

import copy
import importlib.util
import json
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("transport_cup",ROOT/"tools/transport_cup.py")
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)
from g1_cup_grasp.carry import plan_carry, require_grip, pose_from_data
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.transport_arm import FreshTransportArm
from g1_cup_grasp.kinematics import UrdfRightArm


class TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grasp=app.load_config(ROOT/"config/grasp_v2.yaml")
        cls.config=yaml.safe_load((ROOT/"tests/fixtures/transport_config_test.yaml").read_text())
        cls.config.pop("place",None)
        cls.place=json.loads((ROOT/"tests/fixtures/place_teach_20261013.json").read_text())["place_config_candidate"]
        cls.config["carry"]={"lift_height_m":.58,"side_y_m":-.35,"position_m":[.4,-.35,.7],
            "yaw_rad":0.,"duration_s":8.,"position_tolerance_m":.015,"orientation_tolerance_deg":5.,
            "gripper_feedback_range":[5.,90.]}
        cls.fixture=json.loads((ROOT/"tests/fixtures/carry_grasp_20261013.json").read_text())
        r=cls.grasp["robot"]
        cls.model=UrdfRightArm(ROOT/r["urdf_path"],r["urdf_base_link"],r["urdf_tip_link"],cls.grasp["ik"])
        points={p["stage"]:p for p in cls.fixture["plan"]["candidates"][0]["waypoints"]}
        cls.points=points
        cls.waist=cls.fixture["model_validation"]["waist_urdf"]
        cls.carry=plan_carry(cls.model,pose_from_data(points["lift"]),points["lift"]["right_arm_joint_rad"],
            cls.waist,cls.config["carry"],pose_from_data(points["grasp"]),cls.fixture["target"]["point_base_m"])

    def test_carry_route_preserves_upright_cup_and_joint_margin(self):
        self.assertGreater(len(self.carry["samples"]),30)
        self.assertTrue(all(s["joint_margin_rad"]>=self.model.limit_margin for s in self.carry["samples"]))
        self.assertLess(max(s["cup_tilt_deg"] for s in self.carry["samples"]),.01)
        np.testing.assert_allclose(self.carry["final_pose"]["position_m"],[.4,-.35,.7])
        self.assertFalse(self.carry["physical_tracking_verified"])
        self.assertFalse(self.carry["collision_model_verified"])

    def test_taught_carry_uses_full_orientation_without_old_side_detour(self):
        teach=json.loads((ROOT/"tests/fixtures/carry_teach_20261013.json").read_text())
        cfg={**self.config["carry"],**teach["carry_config_candidate"]}
        carry=plan_carry(self.model,pose_from_data(self.points["lift"]),self.points["lift"]["right_arm_joint_rad"],self.waist,cfg)
        self.assertEqual(len(carry["waypoints"]),2)
        np.testing.assert_allclose(carry["final_pose"]["position_m"],teach["pose"]["position_m"])
        np.testing.assert_allclose(carry["final_pose"]["rpy_rad"],teach["pose"]["rpy_rad"])
        self.assertTrue(all(s["joint_margin_rad"]>=self.model.limit_margin for s in carry["samples"]))
        self.assertLess(max(s["cup_tilt_deg"] for s in carry["samples"]),5.)

    def test_teaching_records_empty_gripper_without_commanding_it(self):
        spec=importlib.util.spec_from_file_location("teach_carry_pose",ROOT/"tools/teach_carry_pose.py")
        teach_app=importlib.util.module_from_spec(spec);spec.loader.exec_module(teach_app)
        data=json.loads((ROOT/"tests/fixtures/carry_teach_20261013.json").read_text())
        arm=Mock();arm.current_end_pose.return_value=pose_from_data(data["pose"])
        arm.right_arm_joints.return_value=np.asarray(data["right_arm_joint_rad"])
        arm.read_ready_state.return_value=SimpleNamespace(waist_state=data["waist_state"],
            head_joint_rad=data["head_state"],gripper_state=[120.,0.])
        with patch.object(teach_app,"validate_robot_model",return_value={"passed":True}):
            report=teach_app.capture(arm,self.model,self.grasp)
        self.assertTrue(report["read_only"])
        self.assertEqual(report["gripper_feedback_only"],[120.,0.])
        arm.move_gripper.assert_not_called();arm.move_to_pose.assert_not_called()
        with patch.object(teach_app,"validate_robot_model",return_value={"passed":False}):
            with self.assertRaises(RuntimeError):
                teach_app.capture(arm,self.model,self.grasp)

    def test_infeasible_carry_is_not_accepted_by_relaxing_tolerance(self):
        cfg={**self.config["carry"],"position_m":[.1,0,.7]}
        with self.assertRaises(RuntimeError):
            plan_carry(self.model,pose_from_data(self.points["lift"]),self.points["lift"]["right_arm_joint_rad"],
                self.waist,cfg)

    def test_gripper_open_empty_or_nan_blocks_navigation(self):
        for value in (120.,0.,float("nan")):
            with self.assertRaises(RuntimeError):
                require_grip(SimpleNamespace(gripper_state=[120.,value]),self.config["carry"])
        require_grip(SimpleNamespace(gripper_state=[120.,52.42]),self.config["carry"])

    def test_carry_receipt_rejects_stale_changed_map_and_changed_pose(self):
        receipt={"status":"CARRY_POSE_REACHED","context":"x","created_unix_s":time.time(),
                 "map_id":6,"pose":self.carry["final_pose"]}
        arm=Mock();arm.current_end_pose.return_value=pose_from_data(receipt["pose"])
        state=SimpleNamespace(gripper_state=[120,52.42])
        app.check_hold_receipt(receipt,arm,state,self.config,"x")
        for changes in ({"created_unix_s":time.time()-700},{"map_id":5},{"context":"changed"}):
            with self.assertRaises(RuntimeError):
                app.check_hold_receipt({**receipt,**changes},arm,state,self.config,"x")
        wrong=pose_from_data(receipt["pose"]);wrong.position_m[0]+=.10
        arm.current_end_pose.return_value=wrong
        with self.assertRaises(RuntimeError):
            app.check_hold_receipt(receipt,arm,state,self.config,"x")

    def test_native_reader_keeps_system_ssh_libraries_and_closes_exited_child(self):
        process=Mock();process.stdout=[];process.poll.return_value=0
        with patch.dict("os.environ",{"G1_ROBOT_PASSWORD":"test", "LD_LIBRARY_PATH":"gdk", "LD_PRELOAD":"gdk"}), \
             patch("g1_cup_grasp.io.chassis_feedback.subprocess.Popen",return_value=process) as start, \
             patch("g1_cup_grasp.io.chassis_feedback.os.killpg") as kill:
            reader=NativeChassisFeedback({"ssh_target":"robot", "helper_path":"/helper.py"})
            environment=start.call_args.kwargs["env"]
            self.assertNotIn("LD_LIBRARY_PATH",environment)
            self.assertNotIn("LD_PRELOAD",environment)
            reader.close();reader.close()
            kill.assert_not_called()

    def test_motion_watchdog_reuses_the_existing_subscription(self):
        import threading
        callback=Mock()
        subscription=(object(),callback,None,object(),object(),False)
        node=SimpleNamespace(mutex=threading.Lock(),subs={"/status":subscription})
        core=SimpleNamespace(node_=node,motion_control_status_topic="/status")
        def initialize(arm,*args):
            arm._controller=SimpleNamespace(_motion_controller=core)
        with patch("g1_cup_grasp.io.transport_arm.GdkRightArm.__init__",initialize):
            arm=FreshTransportArm({})
        updated=node.subs["/status"]
        self.assertIs(updated[0],subscription[0])
        self.assertEqual(updated[2:],subscription[2:])
        message=object()
        updated[1](message)
        callback.assert_called_once_with(message)
        self.assertGreater(arm._last_status_received,0.)

    def test_motion_watchdog_updates_sdk_cache_before_marking_feedback_fresh(self):
        arm=FreshTransportArm.__new__(FreshTransportArm)
        core=Mock()
        arm._sdk_status_callback=core.motion_control_status_callback
        arm._last_status_received=0.0
        message=object()
        with patch("g1_cup_grasp.io.transport_arm.time.monotonic",return_value=123.):
            arm._received_status(message)
        core.motion_control_status_callback.assert_called_once_with(message)
        self.assertEqual(arm._last_status_received,123.)

    def test_native_reader_rejects_stale_receipt_time(self):
        reader=NativeChassisFeedback.__new__(NativeChassisFeedback)
        import threading
        reader._lock=threading.Lock();reader._row={"timestamp_ns":123,"map_id":6};reader._received=time.monotonic()-3
        with self.assertRaisesRegex(RuntimeError,"stale"):
            reader.get_chassis_position()

    def test_offline_report_and_incomplete_flags_never_initialize_robot(self):
        for args in (["--execute"],["--enable-real"],["--offline-grasp-report","x.json","--execute","--enable-real"]):
            with TemporaryDirectory() as directory, patch.object(app,"ROOT",Path(directory)), \
                 patch.object(app,"NativeChassisFeedback") as native,patch.object(app,"FreshTransportArm") as arm,patch("builtins.print"):
                self.assertEqual(app.main(args),1)
                native.assert_not_called();arm.assert_not_called()

    def test_left_arm_link_extent_uses_actual_left_joint_chain(self):
        r=self.grasp["robot"]
        left=UrdfRightArm(ROOT/r["urdf_path"],r["urdf_base_link"],"arm_l_end_link",self.grasp["ik"],arm_joint_prefix="idx2")
        q=np.array([0.1]*7)
        self.assertTrue(all(name.startswith("idx2") for name in left.joint_names))
        np.testing.assert_allclose(left.link_positions(q,self.waist)[-1],left.forward(q,self.waist)[:3,3])

    def test_place_target_preserves_map_pose_when_chassis_pose_changes(self):
        source=self.place["base_pose"]
        same=app.place_pose_at_base(self.place,source)
        np.testing.assert_allclose(same.position_m,self.place["position_m"],atol=1e-12)
        current={**source,"agv_pos_x":source["agv_pos_x"]+.03,"agv_pos_y":source["agv_pos_y"]-.02,"agv_angle":source["agv_angle"]-.1}
        target=app.place_pose_at_base(self.place,current)
        R=app.Rotation
        wanted=R.from_euler("z",source["agv_angle"]).apply(self.place["position_m"])+[source["agv_pos_x"],source["agv_pos_y"],0]
        actual=R.from_euler("z",current["agv_angle"]).apply(target.position_m)+[current["agv_pos_x"],current["agv_pos_y"],0]
        np.testing.assert_allclose(actual,wanted,atol=1e-12)
        np.testing.assert_allclose((R.from_euler("z",current["agv_angle"])*R.from_euler("xyz",target.rpy_rad)).as_matrix(),
            (R.from_euler("z",source["agv_angle"])*R.from_euler("xyz",self.place["rpy_rad"])).as_matrix(),atol=1e-12)

    def test_place_releases_only_after_successful_arm_move(self):
        arm=Mock();arm.move_to_pose.return_value={"position_error_mm":.1}
        order=Mock();order.attach_mock(arm.move_to_pose,"move");order.attach_mock(arm.move_gripper,"release")
        report={"gripper_release_requested":False}
        target=pose_from_data(self.place)
        with patch.object(app.time,"sleep"):
            app.execute_place(arm,target,self.grasp,self.config["carry"],report)
        self.assertEqual([c[0] for c in order.mock_calls],["move","release"])
        arm.move_gripper.assert_called_once_with(self.grasp["robot"]["gripper_open"])
        self.assertEqual(report["status"],"ARRIVED_B_CUP_RELEASED")
        failed=Mock();failed.move_to_pose.side_effect=RuntimeError("endpoint error")
        report={"gripper_release_requested":False}
        with self.assertRaises(RuntimeError):
            app.execute_place(failed,target,self.grasp,self.config["carry"],report)
        failed.move_gripper.assert_not_called();self.assertFalse(report["gripper_release_requested"])

    def test_resume_arrives_at_B_then_places_only_when_configured(self):
        for with_place in (False,True):
            config=copy.deepcopy(self.config)
            if with_place:config["place"]=self.place
            with TemporaryDirectory() as directory:
                root=Path(directory);(root/"config").mkdir();(root/"models/robot").mkdir(parents=True)
                (root/"config/grasp_v2.yaml").write_text(yaml.safe_dump(self.grasp))
                (root/"config/transport.yaml").write_text(yaml.safe_dump(config))
                shutil.copyfile(ROOT/self.grasp["robot"]["urdf_path"],root/self.grasp["robot"]["urdf_path"])
                native=Mock();stamp=[1];at_b=[False]
                def feedback():
                    stamp[0]+=1
                    target=config["stations"]["B" if at_b[0] else "A"]
                    return {"map_id":6,"chassis_error":0,"agv_status":8,"position_conf":99.,
                            "agv_pos_x":target["x_m"],"agv_pos_y":target["y_m"],"agv_angle":target["yaw_rad"],
                            "linear_speed":0.,"angular_speed":0.,"hal_sha256":app.HAL_SHA256},stamp[0]
                native.get_chassis_position.side_effect=feedback
                arm=Mock()
                arm.assert_execution_ready=Mock()
                state=SimpleNamespace(gripper_state=[120.,52.42])
                arm.read_ready_state.return_value=state
                arm.move_to_pose.return_value={"position_error_mm":.1,"orientation_error_deg":.1}
                arm.current_end_pose.return_value=pose_from_data(self.carry["final_pose"])
                receipt={"status":"CARRY_POSE_REACHED","context":None,"created_unix_s":time.time(),
                         "map_id":6,"pose":self.carry["final_pose"]}
                controller={"map_selection_matches":True,"serial_number":"424","auto_motion_parameters":{"maxSpeed":100}}
                hmi={"map_assignment_matches":True,"state":{"isNormal":True,"isScram":False,"isEdged":False,"isPause":False,
                        "isHardAuto":True,"isSoftAuto":True,"batteryStateList":[{"isCharging":False}]}}
                def move(x,y,yaw):
                    at_b[0]=True
                    return SimpleNamespace(error_code=0)
                slam=Mock();slam.navigate_to_pose.side_effect=move
                slam.set_agv_col_level.return_value=SimpleNamespace(error_code=0);slam.set_agv_col_ctrl.return_value=SimpleNamespace(success=True)
                module=ModuleType("a2d_sdk.robot");module.Slam=Mock(return_value=slam)
                with patch.object(app,"ROOT",root):
                    receipt["context"]=app.context_digest(self.grasp,config)
                    (root/"outputs/transport").mkdir(parents=True)
                    (root/"outputs/transport/latest_hold.json").write_text(json.dumps(receipt))
                    with patch.object(app,"NativeChassisFeedback",return_value=native),patch.object(app,"FreshTransportArm",return_value=arm), \
                         patch.object(app,"read_hmi",return_value=hmi),patch.object(app,"read_controller",return_value=controller), \
                         patch.object(app,"validate_robot_model",return_value={"passed":True}),patch("builtins.print"), \
                         patch.dict("sys.modules",{"a2d_sdk.robot":module}):
                        code=app.main(["--resume-navigation","--execute","--enable-real"])
                        reports=list((root/"outputs/transport").glob("20*.json"))
                        self.assertEqual(code,0,json.loads(reports[0].read_text()).get("error"))
                reports=list((root/"outputs/transport").glob("20*.json"))
                report=json.loads(reports[0].read_text())
                self.assertEqual(report["status"],"ARRIVED_B_CUP_RELEASED" if with_place else "ARRIVED_B_HOLDING_CUP")
                self.assertEqual(report["gripper_release_requested"],with_place)
                self.assertEqual(report["arm_and_gripper_remain_in_hold"],not with_place)
                expected=app.navigation_request(config["stations"]["B"])
                slam.navigate_to_pose.assert_called_once_with(expected["x_mm"],expected["y_mm"],expected["yaw_deg"])
                slam.set_agv_col_ctrl.assert_called_once_with(True)
                self.assertTrue(report["chassis_collision_avoidance"]["enabled"])
                self.assertEqual(report["avoidance"]["episode_count"],0)
                slam.set_chassis_info.assert_not_called()
                if with_place:
                    arm.move_gripper.assert_called_once_with(self.grasp["robot"]["gripper_open"])
                    arm.move_to_pose.assert_called_once()
                else:
                    arm.move_gripper.assert_not_called();arm.move_to_pose.assert_not_called()
                native.close.assert_called_once()

    def test_map_mismatch_message_names_configured_map(self):
        config=copy.deepcopy(self.config)
        config["map_id"]=7
        with TemporaryDirectory() as directory, patch.object(app,"ROOT",Path(directory)), \
             patch.object(app,"load_config",return_value=self.grasp), \
             patch.object(app,"context_digest",return_value="context"), \
             patch.object(app,"UrdfRightArm",return_value=Mock()), \
             patch.object(app,"NativeChassisFeedback") as native, \
             patch.object(app,"collect_samples",return_value=[
                 {"map_id":6,"timestamp_ns":i+1,"agv_pos_x":1.542,"agv_pos_y":1.344,
                  "agv_angle":0.0,"position_conf":99.0,"linear_speed":0.0,"angular_speed":0.0}
                 for i in range(5)]), \
             patch("builtins.print"):
            (Path(directory)/"config").mkdir()
            (Path(directory)/"config/transport.yaml").write_text(yaml.safe_dump(config))
            code=app.main([])
            report_path=next((Path(directory)/"outputs/transport").glob("20*.json"))
            report=json.loads(report_path.read_text())
        self.assertEqual(code,1)
        self.assertIn("map 7",report["error"])
        native.assert_called_once()

    def test_place_map_mismatch_rejected_before_robot_init(self):
        config=copy.deepcopy(self.config)
        config["place"]=copy.deepcopy(self.place)
        config["place"]["base_pose"]["map_id"]=5
        with TemporaryDirectory() as directory, patch.object(app,"ROOT",Path(directory)), \
             patch.object(app,"load_config",return_value=self.grasp), \
             patch.object(app,"context_digest",return_value="context"), \
             patch.object(app,"UrdfRightArm") as model, \
             patch.object(app,"NativeChassisFeedback") as native, \
             patch("builtins.print"):
            (Path(directory)/"config").mkdir()
            (Path(directory)/"config/transport.yaml").write_text(yaml.safe_dump(config))
            code=app.main([])
        self.assertEqual(code,1)
        native.assert_not_called()
        model.assert_not_called()

if __name__=="__main__":
    unittest.main()

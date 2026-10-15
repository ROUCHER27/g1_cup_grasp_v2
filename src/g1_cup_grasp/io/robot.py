from __future__ import annotations

import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ..domain import EndPose, MotionState
from ..geometry.transforms import matrix_to_rpy, pose_delta, quaternion_xyzw_to_matrix


def _unpack_state(value: Any) -> tuple[list[float], int]:
    if value is None:
        return [], time.time_ns()
    if isinstance(value, tuple) and len(value) >= 2:
        values, timestamp = value[0], value[1]
        return [float(item) for item in (values or [])], int(timestamp or time.time_ns())
    return [float(item) for item in value], time.time_ns()


def _angle_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    delta = Rotation.from_euler("xyz", first).inv() * Rotation.from_euler("xyz", second)
    return float(np.rad2deg(delta.magnitude()))


class GdkRightArm:
    """Small GDK adapter; it never changes the robot motion mode."""

    def __init__(self, config: dict[str, Any], pose_model: Any | None = None) -> None:
        try:
            from a2d_sdk.robot import RobotController, RobotDds
        except ImportError as exc:
            raise RuntimeError("Source ~/a2d_sdk/env.sh and activate the GDK Python environment") from exc
        self.config = config
        self._pose_model = pose_model
        self._dds = RobotDds()
        self._controller = RobotController()
        wait_s = float(config.get("startup_wait_s", 2.0))
        if wait_s > 0:
            time.sleep(wait_s)

    def read_state(self) -> MotionState:
        head, head_ts = _unpack_state(self._dds.head_joint_states())
        waist, waist_ts = _unpack_state(self._dds.waist_joint_states())
        arm, arm_ts = _unpack_state(self._dds.arm_joint_states())
        gripper, gripper_ts = _unpack_state(self._dds.gripper_states())
        status = self._controller.get_motion_status()
        return MotionState(
            max(head_ts, waist_ts, arm_ts, gripper_ts),
            head, waist, arm, gripper, raw=status,
        )

    def read_ready_state(self, timeout_s: float = 5.0) -> MotionState:
        deadline = time.monotonic() + timeout_s
        last = "no feedback"
        while time.monotonic() < deadline:
            state = self.read_state()
            groups = [state.head_joint_rad, state.waist_state,
                      state.arm_joint_rad, state.gripper_state]
            if [len(group) for group in groups] == [2, 2, 14, 2] and all(
                np.all(np.isfinite(group)) for group in groups
            ):
                return state
            last = f"feedback lengths are {[len(group) for group in groups]}"
            time.sleep(0.2)
        raise RuntimeError(f"No complete GDK feedback within {timeout_s:g} s: {last}")

    def current_end_pose(self) -> EndPose:
        status = self._controller.get_motion_status()
        if not isinstance(status, dict):
            raise RuntimeError(f"GDK motion status is unavailable: {status!r}")
        error = status.get("error", {}) or {}
        if error.get("has_error") or int(error.get("code", 0)) != 0:
            raise RuntimeError(f"GDK motion error: {error}")
        frame_name = str(self.config.get("gdk_tip_frame", "arm_right_link7"))
        frame = (status.get("frames") or {}).get(frame_name)
        if not frame:
            raise RuntimeError(f"GDK motion status has no frame {frame_name!r}")
        if "xyzrpy" in frame:
            pose = np.asarray(frame["xyzrpy"], dtype=float).reshape(6)
            return EndPose(pose[:3], pose[3:], str(self.config.get("robot_link", "base_link")))
        position = frame.get("position") or {}
        orientation = frame.get("orientation") or {}
        if "quaternion" in orientation:
            q = orientation["quaternion"]
            rpy = matrix_to_rpy(quaternion_xyzw_to_matrix([q["x"], q["y"], q["z"], q["w"]]))
        elif "euler" in orientation:
            rpy = np.array([orientation["euler"][key] for key in ("roll", "pitch", "yaw")])
        else:
            raise RuntimeError(f"GDK frame {frame_name!r} has no orientation")
        xyz = np.array([position[key] for key in ("x", "y", "z")], dtype=float)
        return EndPose(xyz, rpy, str(self.config.get("robot_link", "base_link")))

    def right_arm_joints(self, state: MotionState) -> np.ndarray:
        start, end = (int(value) for value in self.config.get("right_arm_joint_slice", [7, 14]))
        joints = np.asarray(state.arm_joint_rad[start:end], dtype=float)
        if joints.shape != (7,):
            raise RuntimeError(f"Expected 7 right-arm joints, received {joints.shape}")
        return joints

    def waist_for_urdf(self, state: MotionState) -> np.ndarray:
        order = [int(value) for value in self.config.get("waist_urdf_order", [1, 0])]
        waist = np.asarray(state.waist_state, dtype=float)
        return waist[order]

    def end_pose_from_state(self, state: MotionState) -> EndPose:
        if self._pose_model is None:
            return self.current_end_pose()
        transform = self._pose_model.forward(
            self.right_arm_joints(state), self.waist_for_urdf(state)
        )
        return EndPose(
            transform[:3, 3], matrix_to_rpy(transform[:3, :3]),
            str(self.config.get("robot_link", "base_link")),
        )

    def assert_execution_ready(self, state: MotionState) -> None:
        status = state.raw
        if not isinstance(status, dict):
            raise RuntimeError("Robot motion status is unavailable")
        mode = status.get("mode") or {}
        if mode.get("name") != "SERVO":
            raise RuntimeError(
                f"Robot mode is {mode.get('name', 'MISSING')} (value={mode.get('value', 'MISSING')}); "
                "restore SERVO with the official controls"
            )
        error = status.get("error") or {}
        if error.get("has_error") or int(error.get("code", 0)) != 0 or status.get("collisions"):
            raise RuntimeError(f"Robot fault or collision: {error}, {status.get('collisions')}")
        head_reference = float(self.config["head_pitch_reference_deg"])
        if abs(float(state.head_joint_rad[1]) - head_reference) > float(
            self.config["head_pitch_tolerance_deg"]
        ):
            raise RuntimeError("Head pitch differs from the hand-eye calibration pose")
        waist_reference = np.asarray(self.config["waist_reference"], dtype=float)
        if np.linalg.norm(np.asarray(state.waist_state) - waist_reference) > float(
            self.config["waist_tolerance"]
        ):
            raise RuntimeError("Waist pose differs from the hand-eye calibration pose")

    def move_gripper(self, normalized_value: float) -> Any:
        value = float(normalized_value)
        if not 0.0 <= value <= 1.0:
            raise ValueError("gripper command must be in [0, 1]")
        try:
            from sensor_msgs_pb.msg.JointState_pb2 import JointState
        except ImportError as exc:
            raise RuntimeError("Cannot import the GDK JointState message") from exc
        publisher = getattr(self._dds, "_right_ee_cmd_publisher", None)
        if publisher is None:
            raise RuntimeError("GDK does not expose the right gripper publisher")
        command = JointState()
        command.position.extend([value])
        return publisher.publish(command)

    def move_to_pose(
        self,
        target: EndPose,
        duration_s: float,
        sample_hz: float,
        position_tolerance_m: float,
        orientation_tolerance_deg: float,
    ) -> dict[str, float]:
        state = self.read_ready_state()
        self.assert_execution_ready(state)
        start = self.end_pose_from_state(state)
        steps = max(20, int(np.ceil(duration_s * sample_hz)))
        rotations = Rotation.from_euler("xyz", np.stack([start.rpy_rad, target.rpy_rad]))
        interpolate_rotation = Slerp([0.0, 1.0], rotations)
        previous = start.as_vector6()
        deltas: list[np.ndarray] = []
        for index in range(1, steps + 1):
            u = index / steps
            smooth = 10 * u**3 - 15 * u**4 + 6 * u**5
            xyz = start.position_m + smooth * (target.position_m - start.position_m)
            rpy = interpolate_rotation([smooth])[0].as_euler("xyz")
            current = np.concatenate([xyz, rpy])
            if index == steps:
                current = target.as_vector6()
            deltas.append(pose_delta(previous, current))
            previous = current

        actions = [{"right_arm": {
            "action_data": delta.tolist(), "control_type": "DELTA_POSE",
        }} for delta in deltas]
        started = time.monotonic()
        self._controller.trajectory_tracking_control(
            infer_timestamp=time.time_ns(),
            robot_states={
                "head": state.head_joint_rad,
                "waist": state.waist_state,
                "arm": state.arm_joint_rad,
            },
            robot_actions=actions,
            robot_link=str(self.config.get("robot_link", "base_link")),
            trajectory_reference_time=float(duration_s),
        )
        deadline = started + duration_s + 4.0
        position_error = float("inf")
        orientation_error = float("inf")
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = self.read_ready_state()
            self.assert_execution_ready(state)
            actual = self.end_pose_from_state(state)
            position_error = float(np.linalg.norm(actual.position_m - target.position_m))
            orientation_error = _angle_error_deg(actual.rpy_rad, target.rpy_rad)
            if (time.monotonic() - started >= duration_s + 0.2
                    and position_error <= position_tolerance_m
                    and orientation_error <= orientation_tolerance_deg):
                return {
                    "position_error_mm": position_error * 1000.0,
                    "orientation_error_deg": orientation_error,
                }
        raise RuntimeError(
            f"endpoint error {position_error * 1000:.1f} mm, {orientation_error:.1f} deg"
        )

    def stop(self) -> Any:
        return self._controller.set_motion_stop()

    def close(self) -> None:
        shutdown = getattr(self._dds, "shutdown", None)
        if callable(shutdown):
            shutdown()

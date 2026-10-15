#!/usr/bin/env python3
"""Robot-local subscriber. No publishers, clients, or robot control calls."""
import argparse
import json
import hashlib
from pathlib import Path
import signal
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from genie_msgs.msg import Position


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=600)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("g1_cup_native_feedback_read_only")
    hal_sha = hashlib.sha256(Path("/home/agi/app/lib/hal/hal").read_bytes()).hexdigest()
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    def callback(message):
        row = {key: getattr(message, key) for key in (
            "map_id", "chassis_error", "agv_status", "position_conf", "agv_pos_x", "agv_pos_y",
            "agv_angle", "linear_speed", "angular_speed")}
        row["timestamp_ns"] = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        row["hal_sha256"] = hal_sha
        row["source"] = "robot_native_ros:/hal/position"
        try:
            print(json.dumps(row, allow_nan=False), flush=True)
        except BrokenPipeError:
            stop()
    subscription = node.create_subscription(Position, "/hal/position", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    try:
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
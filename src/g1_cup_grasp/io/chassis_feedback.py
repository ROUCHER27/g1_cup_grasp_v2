"""Read original chassis feedback over SSH; never publish a robot command."""
from __future__ import annotations

import copy
import getpass
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time


class NativeChassisFeedback:
    def __init__(self, config: dict):
        password = os.environ.get("G1_ROBOT_PASSWORD") or getpass.getpass("机器人 SSH/sudo 密码（不保存）: ")
        environment = os.environ.copy()
        environment.pop("G1_ROBOT_PASSWORD", None)
        # GDK libraries must not override the system SSH OpenSSL.
        environment.pop("LD_LIBRARY_PATH", None)
        environment.pop("LD_PRELOAD", None)
        environment["SSHPASS"] = password
        remote = "sudo -S -p '' env " + " ".join(shlex.quote(value) for value in (
            "PYTHONPATH=/home/agi/app/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages",
            "LD_LIBRARY_PATH=/home/agi/app/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib",
            "AMENT_PREFIX_PATH=/home/agi/app:/opt/ros/humble", "ROS_LOCALHOST_ONLY=1"))
        remote += " /usr/bin/python3 -u " + shlex.quote(config["helper_path"])
        command = ["sshpass", "-e", "ssh", "-T", "-o", "ConnectTimeout=5",
                   "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2"]
        known_hosts = config.get("known_hosts_file")
        if known_hosts and Path(known_hosts).is_file():
            command += ["-o", "UserKnownHostsFile=" + known_hosts]
        command += [config["ssh_target"], remote]
        self._lock = threading.Lock()
        self._row = None
        self._received = 0.0
        self._closed = False
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         text=True, bufsize=1, env=environment, start_new_session=True)
        self._process.stdin.write(password + "\n")
        self._process.stdin.flush()
        self._process.stdin.close()
        password = None
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        for line in self._process.stdout:
            try:
                row = json.loads(line)
                keys = ("map_id", "chassis_error", "agv_status", "position_conf", "agv_pos_x",
                        "agv_pos_y", "agv_angle", "linear_speed", "angular_speed", "timestamp_ns")
                if row.get("source") != "robot_native_ros:/hal/position" or any(key not in row for key in keys):
                    continue
                with self._lock:
                    self._row = row
                    self._received = time.monotonic()
            except (ValueError, AttributeError):
                continue

    def get_chassis_position(self):
        deadline = time.monotonic() + 12.0
        while True:
            with self._lock:
                row, received = copy.deepcopy(self._row), self._received
            if row is not None:
                if time.monotonic() - received > 2.0:
                    raise RuntimeError("native chassis feedback is stale")
                return row, row["timestamp_ns"]
            if self._process.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("native chassis feedback unavailable; check SSH/sudo and robot subscriber")
            time.sleep(0.05)

    def close(self):
        if not self._closed:
            self._closed = True
            if self._process.poll() is not None:
                return
            try:
                os.killpg(self._process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self._process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self._process.wait(timeout=1)
from __future__ import annotations

import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from g1_cup_grasp.io.controller_avoidance import read_avoidance_snapshot
from check_navigation import read_hmi


class ControllerAvoidanceSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.config = {"controller_url": "http://robot:6002", "robot_id": 424, "map_id": 7}
        self.device = {"serialNumber": "424", "softwareVersion": "7.2.0.23", "firmwareVersion": "2.0.0.7"}
        self.params = {"id": 424, "mapId": 7, "agvTypeId": 1}

    def response(self, endpoint, content, **changes):
        envelope = {"ErrorCode": 0, "Interface": endpoint, "TimeStamp": "2026-10-14 19:00:00", "Content": content}
        envelope.update(changes)
        return io.BytesIO(json.dumps(envelope).encode())

    def test_reads_only_two_query_endpoints_and_records_local_receipt(self):
        responses = [self.response("/api/AMR/GetDeviceInfo", self.device),
                     self.response("/api/Config/Agv/GetParam", self.params)]
        with patch("g1_cup_grasp.io.controller_avoidance.urllib.request.urlopen", side_effect=responses) as query, \
             patch("g1_cup_grasp.io.controller_avoidance.time.monotonic", side_effect=[10., 10.2]):
            snapshot = read_avoidance_snapshot(self.config)
        self.assertEqual(snapshot["received_monotonic"], 10.2)
        self.assertAlmostEqual(snapshot["query_duration_s"], .2)
        self.assertEqual(snapshot["parameters"], self.params)
        self.assertEqual(snapshot["device"], self.device)
        for call, endpoint in zip(query.call_args_list, ("/api/AMR/GetDeviceInfo", "/api/Config/Agv/GetParam")):
            request = call.args[0]
            self.assertEqual(request.full_url, self.config["controller_url"] + endpoint)
            self.assertEqual(json.loads(request.data), {"id": 424})
            self.assertEqual(request.get_method(), "POST")
            self.assertLessEqual(call.kwargs["timeout"], .5)

    def test_query_error_wrong_endpoint_and_missing_timestamp_are_rejected(self):
        for changes in ({"ErrorCode": 1}, {"ErrorCode": False}, {"Interface": "/api/AMR/Move"}, {"TimeStamp": None}):
            with self.subTest(changes=changes), patch(
                "g1_cup_grasp.io.controller_avoidance.urllib.request.urlopen",
                return_value=self.response("/api/AMR/GetDeviceInfo", self.device, **changes),
            ):
                with self.assertRaises(RuntimeError):
                    read_avoidance_snapshot(self.config)

    def test_robot_and_map_mismatch_are_rejected(self):
        for device, params in (({**self.device, "serialNumber": "425"}, self.params),
                               (self.device, {**self.params, "id": 425}),
                               (self.device, {**self.params, "mapId": 6}),
                               (self.device, {**self.params, "mapId": True})):
            with self.subTest(device=device, params=params), patch(
                "g1_cup_grasp.io.controller_avoidance.urllib.request.urlopen",
                side_effect=[self.response("/api/AMR/GetDeviceInfo", device), self.response("/api/Config/Agv/GetParam", params)],
            ):
                with self.assertRaises(RuntimeError):
                    read_avoidance_snapshot(self.config)

    def test_timeout_is_visible_and_does_not_retry_or_write(self):
        with patch("g1_cup_grasp.io.controller_avoidance.urllib.request.urlopen", side_effect=TimeoutError("query timed out")) as query:
            with self.assertRaisesRegex(RuntimeError, "query timed out"):
                read_avoidance_snapshot(self.config)
        self.assertEqual(query.call_count, 1)

    def test_navigation_report_retains_avoidance_trigger_details(self):
        state = {"id": 424, "isAvoid": True, "avoidStatus": 1,
                 "avoidSensorList": [{"id": 12, "x": 250, "y": 10}]}
        responses = [self.response("/api/AMR/GetState", state),
                     self.response("/api/Config/GetAllParam", {"Agv": [{"id": 424, "mapId": 7}]})]
        with patch("check_navigation.urllib.request.urlopen", side_effect=responses):
            report = read_hmi({**self.config, "hmi_url": "http://robot:8080"})
        for key in ("isAvoid", "avoidStatus", "avoidSensorList"):
            self.assertEqual(report["state"][key], state[key])


if __name__ == "__main__":
    unittest.main()

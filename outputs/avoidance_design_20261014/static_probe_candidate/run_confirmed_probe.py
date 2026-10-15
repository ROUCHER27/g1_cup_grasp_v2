#!/usr/bin/env python3
"""One authorized stationary probe; always compensate the two edited vertices."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import time
import urllib.request
import uuid

HERE = Path(__file__).resolve().parent
CONTROLLER = "http://10.42.0.153:6002"
HMI = "http://10.42.0.153:8080"
UPDATE = "/api/Config/AgvType/UpdateParam"


def save(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def region(value):
    matches = [r for r in value["avoidRangesParam"]["avoidRangeList"] if r["id"] == 1]
    if len(matches) != 1 or matches[0]["sensorId"] != 6:
        raise RuntimeError("front laser region identity changed")
    return matches[0]


def changed(value, before, after):
    result = copy.deepcopy(value)
    points = region(result)["warn1"]["positions"]
    if len(points) != 4 or points[2] != {"x": before, "y": 420} or points[3] != {"x": before, "y": -420}:
        raise RuntimeError("expected two warning vertices are not present")
    for i in (2, 3):
        points[i]["x"] = after
    return result


def same_parameters(actual, expected):
    """Controller regenerates only the root UUID on UpdateParam (observed).

    Keep IDs and every nested UUID/parameter in the comparison. Record both
    complete objects in the journal so the generated identity change is visible.
    """
    try:
        for value in (actual, expected):
            uuid.UUID(value["uuid"].strip("{}"))
    except (KeyError, TypeError, ValueError, AttributeError):
        return False
    return ({k: v for k, v in actual.items() if k != "uuid"}
            == {k: v for k, v in expected.items() if k != "uuid"})


def static_state(state, allow_configuration_sync=False):
    if state.get("id") != 424:
        raise RuntimeError("state robot ID mismatch")
    for key in ("isSoftAuto", "isHardAuto", "isNormal"):
        if state.get(key) is not True:
            raise RuntimeError(key + " is not true")
    for key in ("isPause", "isScram", "isEdged"):
        if state.get(key) is not False:
            raise RuntimeError(key + " is not false")
    for key in ("currentCmdId", "latestCmdId", "taskId", "executeStatus"):
        if type(state.get(key)) is not int or state[key] != 0:
            raise RuntimeError("active/unknown task indicator: " + key)
    if type(state.get("taskState")) is not int or state["taskState"] != 1:
        raise RuntimeError("taskState is not confirmed Idle")
    allowed_work_states = (1, 12) if allow_configuration_sync else (1,)
    if state.get("workStatus") not in allowed_work_states or state.get("protoStatus") not in (5, 8):
        raise RuntimeError("unexpected chassis/work state")
    if not isinstance(state.get("confidence"), (int, float)) or state["confidence"] < 50:
        raise RuntimeError("localization confidence too low or absent")
    if not isinstance(state.get("linearSpeed"), (int, float)) or abs(state["linearSpeed"]) > .1:
        raise RuntimeError("nonzero/unknown linear speed")
    if not isinstance(state.get("angularSpeed"), (int, float)) or abs(state["angularSpeed"]) > .2:
        raise RuntimeError("angular feedback exceeds stationary noise allowance")
    if not isinstance(state.get("exceptions"), list) or any(state["exceptions"]):
        raise RuntimeError("exception feedback is missing or nonzero")


class Probe:
    def __init__(self, directory, transport=None, sleeper=time.sleep):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=False)
        self.path = directory / "report.json"
        self.transport = transport
        self.sleep = sleeper
        self.report = {"status": "PRECHECK", "user_authorized": True,
                       "robot_motion_commands_sent": False, "write_attempted": False,
                       "apply_response_confirmed": False, "restoration_verified": False,
                       "events": [], "samples": {}}

    def request(self, endpoint, body, base=CONTROLLER):
        event = {"time_utc": datetime.now(timezone.utc).isoformat(), "endpoint": base + endpoint,
                 "request_kind": "configuration_update" if endpoint == UPDATE else "read_only"}
        self.report["events"].append(event)
        save(self.path, self.report)
        try:
            if self.transport:
                raw = self.transport(endpoint, body, base)
            else:
                req = urllib.request.Request(base + endpoint, data=json.dumps(body, allow_nan=False).encode(),
                                             headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(req, timeout=2) as response:
                    raw = json.load(response, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            # Never place non-finite response data in the journal: that could
            # prevent subsequent compensation requests from being recorded/sent.
            json.dumps(raw, allow_nan=False)
            event["response"] = raw
            if type(raw.get("ErrorCode")) is not int or raw["ErrorCode"] != 0:
                raise RuntimeError("API error: " + str(raw.get("ErrorMessage")))
            return raw.get("Content")
        except Exception as exc:
            event["error"] = str(exc)
            raise
        finally:
            save(self.path, self.report)

    def group(self):
        return self.request("/api/Config/AgvType/GetParam", {"id": 1})

    def robot(self):
        return self.request("/api/Config/Agv/GetParam", {"id": 424})

    def make_candidate(self, baseline):
        return changed(baseline, 1240, 1100)

    def verify_membership(self):
        all_params = self.request("/api/Config/GetAllParam", {}, HMI)
        members = sorted(a["id"] for a in all_params["Agv"] if a["agvTypeId"] == 1)
        if members != [424]:
            raise RuntimeError("test group membership changed")

    def states(self, phase, count, reference=None):
        rows = self.report["samples"].setdefault(phase, [])
        for _ in range(count):
            state = self.request("/api/AMR/GetState", {"id": 424})
            rows.append(state)
            save(self.path, self.report)
            # HMI staticData workStatus 12 is explicitly "地图同步". Only
            # tolerate this bounded post-update transient, never in precheck.
            static_state(state, allow_configuration_sync=phase in ("temporary", "restored"))
            if reference is None:
                reference = state["globalPose"]
            pose = state["globalPose"]
            delta = abs((pose["theta"] - reference["theta"] + 180) % 360 - 180)
            if abs(pose["x"] - reference["x"]) > 2 or abs(pose["y"] - reference["y"]) > 2 or delta > .3:
                raise RuntimeError("pose changed during stationary test; restoring immediately")
            print(json.dumps({"phase": phase, "isAvoid": state.get("isAvoid"),
                              "protoStatus": state.get("protoStatus"), "avoidStatus": state.get("avoidStatus"),
                              "workStatus": state.get("workStatus"),
                              "sensors": state.get("avoidSensorList"), "pose": pose}), flush=True)
            self.sleep(.7)
        return rows

    def verify_configuration(self, expected_group, expected_robot, stable_count=1):
        consecutive = 0
        for attempt in range(stable_count + 2):
            group, robot = self.group(), self.robot()
            self.report["last_group"] = group
            self.report["last_robot"] = robot
            if same_parameters(group, expected_group) and same_parameters(robot, expected_robot):
                consecutive += 1
                if consecutive >= stable_count:
                    return True
            else:
                consecutive = 0
            if attempt < stable_count + 1:
                self.sleep(.5)
        return False

    def restore(self, baseline_group, baseline_robot):
        errors = []
        for _ in range(3):
            try:
                current = self.group()
                points = region(current)["warn1"]["positions"]
                before_points = region(baseline_group)["warn1"]["positions"]
                if len(points) != 4 or [v["y"] for v in points] != [v["y"] for v in before_points]:
                    raise RuntimeError("concurrent polygon edit; refusing to overwrite it")
                if any(points[i]["x"] not in (1100, 1240) for i in (2, 3)):
                    raise RuntimeError("concurrent far-edge edit; restoration needs review")
                restored = copy.deepcopy(current)
                for i in (2, 3):
                    region(restored)["warn1"]["positions"][i]["x"] = 1240
                # Always send the compensating write, including when an earlier
                # timed-out update might not yet have appeared in readback.
                self.request(UPDATE, restored)
                if self.verify_configuration(baseline_group, baseline_robot, stable_count=3):
                    self.report["restoration_verified"] = True
                    return
                errors.append("group or effective robot parameters do not match saved baseline")
            except Exception as exc:
                errors.append(str(exc))
            self.sleep(.5)
        raise RuntimeError("RESTORATION NOT VERIFIED: " + "; ".join(errors))

    def run(self, expected_baseline):
        baseline_group = baseline_robot = None
        try:
            self.verify_membership()
            baseline_group, baseline_robot = self.group(), self.robot()
            if not same_parameters(baseline_group, expected_baseline) or baseline_group.get("id") != 1:
                raise RuntimeError("group baseline changed since approval candidate")
            if baseline_robot.get("id") != 424 or baseline_robot.get("mapId") != 7 or baseline_robot.get("agvTypeId") != 1:
                raise RuntimeError("robot/map/group assignment changed")
            if baseline_robot["avoidRangesParam"] != baseline_group["avoidRangesParam"]:
                raise RuntimeError("group and effective robot regions differ")
            candidate = self.make_candidate(baseline_group)
            expected_robot = self.make_candidate(baseline_robot)
            save(self.directory / "fresh_group_before.json", baseline_group)
            save(self.directory / "fresh_robot_before.json", baseline_robot)
            save(self.directory / "sent_candidate.json", candidate)
            before = self.states("before", 3)
            if not all(s.get("isAvoid") is True for s in before):
                raise RuntimeError("Avoid baseline no longer reproduced")
            if not any(p.get("id") == 6 and 1100 < p["pos"]["x"] <= 1240 and abs(p["pos"]["y"]) <= 420
                       for s in before for p in s.get("avoidSensorList", [])):
                raise RuntimeError("expected outer laser point not observed before test")
            if self.group() != baseline_group:
                raise RuntimeError("configuration changed immediately before write")
            self.states("immediately_before_write", 1, before[0]["globalPose"])
            self.verify_membership()
            self.report["write_attempted"] = True  # A timeout may still mean the server accepted it.
            self.report["status"] = "TEMPORARY_PROBE"
            save(self.path, self.report)
            self.request(UPDATE, candidate)
            self.report["apply_response_confirmed"] = True
            if not self.verify_configuration(candidate, expected_robot):
                raise RuntimeError("temporary values were not confirmed in group and effective robot readback")
            self.report["temporary_configuration_verified"] = True
            temporary = self.states("temporary", 8, before[0]["globalPose"])
            if not all(s["workStatus"] == 1 for s in temporary[-3:]):
                raise RuntimeError("configuration synchronization did not settle during bounded observation")
            self.report["status"] = "OBSERVATION_COMPLETE"
        except BaseException as exc:
            self.report["status"] = "ERROR"
            self.report["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            if self.report["write_attempted"]:
                self.report["late_apply_risk_unresolved"] = not self.report["apply_response_confirmed"]
                print("Restoring the original configuration now...", flush=True)
                try:
                    self.restore(baseline_group, baseline_robot)
                    postcheck_error = None
                    try:
                        restored_states = self.states("restored", 4)
                        if restored_states[-1]["workStatus"] != 1:
                            postcheck_error = RuntimeError("configuration synchronization still active after restoration")
                    except BaseException as exc:
                        postcheck_error = exc
                    # Even an abnormal post-restore state must not skip the
                    # final parameter check or compensation for a delayed write.
                    self.report["restoration_verified"] = False
                    try:
                        final_match = self.verify_configuration(baseline_group, baseline_robot, stable_count=3)
                    except Exception:
                        self.restore(baseline_group, baseline_robot)
                        raise RuntimeError("post-restore readback failed; compensated again")
                    if not final_match:
                        self.restore(baseline_group, baseline_robot)
                        raise RuntimeError("late configuration drift observed after restoration; compensated again")
                    self.report["restoration_verified"] = True
                    if postcheck_error is not None:
                        raise postcheck_error
                    if not self.report["apply_response_confirmed"]:
                        self.report["late_apply_risk_unresolved"] = True
                        raise RuntimeError("baseline currently restored, but initial write outcome was ambiguous; do not infer transaction completion")
                    if self.report["status"] == "OBSERVATION_COMPLETE":
                        self.report["status"] = "PROBE_COMPLETE_RESTORED"
                except BaseException as exc:
                    self.report["status"] = "ERROR"
                    self.report["restoration_or_postcheck_error"] = f"{type(exc).__name__}: {exc}"
            save(self.path, self.report)
            print(json.dumps({"report_path": str(self.path), "status": self.report["status"],
                              "write_attempted": self.report["write_attempted"],
                              "restoration_verified": self.report["restoration_verified"],
                              "error": self.report.get("error"),
                              "restoration_or_postcheck_error": self.report.get("restoration_or_postcheck_error")}), flush=True)
        return self.report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-user-confirmed-static-probe", action="store_true", required=True)
    args = parser.parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt("probe interrupted; restore temporary values")
    signal.signal(signal.SIGTERM, interrupted)
    destination = HERE / ("run_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
    result = Probe(destination).run(json.loads((HERE / "group_before.json").read_text()))
    raise SystemExit(0 if result["status"] == "PROBE_COMPLETE_RESTORED" else 1)

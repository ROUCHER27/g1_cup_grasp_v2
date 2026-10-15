#!/usr/bin/env python3
"""Explicitly confirmed, no-motion isolation of the front-stop region binding."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import signal

HERE = Path(__file__).resolve().parent
BASE_PATH = HERE.parents[1] / "avoidance_design_20261014/static_probe_candidate/run_confirmed_probe.py"
spec = importlib.util.spec_from_file_location("confirmed_static_probe_base", BASE_PATH)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def front_stop(value):
    rows = [s for s in value["avoidStrategyParam"]["globalStrategy"] if s["id"] == 7]
    if len(rows) != 1 or rows[0]["name"] != "前停车":
        raise RuntimeError("front-stop strategy identity changed")
    return rows[0]


def new_region(value):
    rows = [r for r in value["avoidRangesParam"]["avoidRangeList"] if r["id"] == 8]
    if len(rows) != 1:
        raise RuntimeError("diagnostic region ID is absent or duplicated")
    return rows[0]


class StationaryProbe(base.Probe):
    def __init__(self, directory, clone, **kwargs):
        super().__init__(directory, **kwargs)
        self.clone = copy.deepcopy(clone)
        self.report.update(probe_kind="front_stop_binding_isolation", new_region_id=8,
                           permanent_change_authorized=False, navigation_authorized=False)

    def make_candidate(self, baseline):
        if front_stop(baseline)["rangeList"] != [1]:
            raise RuntimeError("baseline front-stop binding is not [1]")
        if any(r["id"] == 8 for r in baseline["avoidRangesParam"]["avoidRangeList"]):
            raise RuntimeError("diagnostic region ID already exists")
        source = base.region(baseline)
        for tier in ("stop", "warn2", "warn1"):
            if self.clone[tier] != source["stop"]:
                raise RuntimeError("clone must retain exactly the original nonempty stop geometry in every tier")
        expected_clone = copy.deepcopy(source)
        expected_clone.update(id=8, name=self.clone["name"], uuid=self.clone["uuid"],
                              warn1=copy.deepcopy(source["stop"]), warn2=copy.deepcopy(source["stop"]))
        if self.clone != expected_clone:
            raise RuntimeError("unexpected diagnostic clone fields")
        result = copy.deepcopy(baseline)
        result["avoidRangesParam"]["avoidRangeList"].append(copy.deepcopy(self.clone))
        front_stop(result)["rangeList"] = [8]
        return result

    def restore(self, baseline_group, baseline_robot):
        errors = []
        for _ in range(3):
            try:
                current = self.group()
                if base.region(current) != base.region(baseline_group):
                    raise RuntimeError("original region 1 changed concurrently; do not overwrite it")
                if front_stop(current)["rangeList"] not in ([1], [8]):
                    raise RuntimeError("front-stop binding changed concurrently")
                # First restore the original reference while leaving the clone
                # present; this also compensates an ambiguous initial POST.
                rebound = copy.deepcopy(current)
                front_stop(rebound)["rangeList"] = [1]
                self.request(base.UPDATE, rebound)
                rebound = self.group()
                if front_stop(rebound)["rangeList"] != [1]:
                    raise RuntimeError("original front-stop reference not yet restored")
                added = [r for r in rebound["avoidRangesParam"]["avoidRangeList"] if r["id"] == 8]
                if added:
                    if added != [self.clone]:
                        raise RuntimeError("diagnostic region has unexpected fields; original binding restored but deletion needs review")
                    strategies = rebound["avoidStrategyParam"]
                    for family in ("globalStrategy", "localStrtegy"):
                        if any(8 in s["rangeList"] for s in strategies[family]):
                            raise RuntimeError("diagnostic region acquired another reference; do not delete it")
                    cleaned = copy.deepcopy(rebound)
                    cleaned["avoidRangesParam"]["avoidRangeList"] = [r for r in cleaned["avoidRangesParam"]["avoidRangeList"] if r["id"] != 8]
                    self.request(base.UPDATE, cleaned)
                if self.verify_configuration(baseline_group, baseline_robot, stable_count=3):
                    self.report["restoration_verified"] = True
                    return
                errors.append("restored group or effective robot parameters differ from baseline")
            except Exception as exc:
                errors.append(str(exc))
            self.sleep(.5)
        raise RuntimeError("STATIONARY STRATEGY RESTORATION NOT VERIFIED: " + "; ".join(errors))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-user-confirmed-stationary-isolation", action="store_true", required=True)
    args = parser.parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt("probe interrupted; restore original front-stop binding")
    signal.signal(signal.SIGTERM, interrupted)
    baseline = json.loads((HERE / "old_group_candidate.json").read_text())
    candidate = json.loads((HERE / "new_group_candidate.json").read_text())
    destination = HERE / ("run_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
    probe = StationaryProbe(destination, new_region(candidate))
    if not base.same_parameters(probe.make_candidate(baseline), candidate):
        raise RuntimeError("prepared candidate differs from restricted transformation")
    result = probe.run(baseline)
    raise SystemExit(0 if result["status"] == "PROBE_COMPLETE_RESTORED" else 1)

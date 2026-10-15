#!/usr/bin/env python3
"""List maps and compare them with configured and reported map IDs."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from check_navigation import collect_samples, read_controller, read_hmi, runtime_map_check
from navigate_station import validate_transport_config
from g1_cup_grasp.io.chassis_feedback import NativeChassisFeedback
from g1_cup_grasp.io.shutdown import exit_without_sdk_thread_wait


NOTE = "switch_map/relocation never called"


def list_maps(slam, config: dict, native=None) -> dict:
    validate_transport_config(config)
    response = slam.get_map_list()
    if getattr(response, "error_code", None) != 0:
        raise RuntimeError(f"get_map_list failed: {response}")
    if len(response.map_id) != len(response.map_name):
        raise RuntimeError("get_map_list returned map_id and map_name arrays with different lengths")
    maps = [{"map_id": map_id, "map_name": map_name}
            for map_id, map_name in zip(response.map_id, response.map_name)]
    hmi = read_hmi(config)
    controller = read_controller(config)
    runtime_map = None
    if native is not None:
        runtime_map = runtime_map_check(collect_samples(native), config["map_id"])
    runtime_map_verified = runtime_map["verified"] if runtime_map is not None else False
    note = NOTE if native is not None else (
        f"{NOTE}; runtime map verification omitted without --native-feedback")
    config_map_present = any(item["map_id"] == config["map_id"] for item in maps)
    config_name_matches = any(item["map_id"] == config["map_id"] and
                              item["map_name"] == config.get("map_name") for item in maps)
    consistent = (
        config_map_present and
        hmi["robot_config_map_id"] == controller["driver_selected_map_id"] == config["map_id"] and
        (runtime_map_verified if native is not None else True)
    )
    return {
        "status": "MAP_LIST_RECORDED" if consistent else "MAP_INCONSISTENT",
        "read_only": True,
        "commands_sent": [],
        "maps": maps,
        "config_map_id": config["map_id"],
        "config_map_name": config.get("map_name"),
        "config_map_present": config_map_present,
        "config_name_matches": config_name_matches,
        "hmi_robot_config_map_id": hmi["robot_config_map_id"],
        "controller_selected_map_id": controller["driver_selected_map_id"],
        "runtime_map": runtime_map,
        "runtime_map_verified": runtime_map_verified,
        "consistent": consistent,
        "note": note,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", required=True,
                        help="record the current map list and consistency checks")
    parser.add_argument("--native-feedback", action="store_true",
                        help="include runtime map ID from native chassis feedback")
    args = parser.parse_args(argv)
    report = {
        "status": "ERROR", "read_only": True, "commands_sent": [], "maps": [],
        "config_map_id": None, "config_map_name": None,
        "config_map_present": False, "config_name_matches": False,
        "hmi_robot_config_map_id": None, "controller_selected_map_id": None,
        "runtime_map": None, "runtime_map_verified": False,
        "consistent": False, "note": NOTE,
    }
    native = None
    code = 1
    try:
        config = yaml.safe_load((PROJECT_ROOT / "config" / "transport.yaml").read_text(encoding="utf-8"))
        report.update(config_map_id=config.get("map_id"), config_map_name=config.get("map_name"))
        validate_transport_config(config)
        from a2d_sdk.robot import Slam

        slam = Slam()
        if args.native_feedback:
            native = NativeChassisFeedback(config["native_feedback"])
        report.update(list_maps(slam, config, native=native))
        code = 0 if report["consistent"] else 2
    except Exception as exc:
        report.update(status="ERROR", error=f"{type(exc).__name__}: {exc}")
        code = 1
    finally:
        if native is not None:
            try:
                native.close()
            except BaseException as exc:
                report["close_error"] = f"{type(exc).__name__}: {exc}"

    directory = PROJECT_ROOT / "outputs" / "map_manager"
    directory.mkdir(parents=True, exist_ok=True)
    created = datetime.now(timezone.utc)
    path = directory / f"{created:%Y%m%d_%H%M%S_%f}_list.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"report_path": str(path), **report}, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    exit_without_sdk_thread_wait(main())

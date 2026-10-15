#!/usr/bin/env python3
"""Compare a saved controller snapshot with a proposal; never contact a robot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.avoidance_comparison import (
    compare_snapshot_to_profile,
    load_snapshot_report,
)
from g1_cup_grasp.avoidance_profile import load_profile


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, type=Path,
                        help="saved report JSON containing report.snapshot")
    parser.add_argument("--profile", required=True, type=Path,
                        help="proposal YAML profile")
    parser.add_argument("--output", required=True, type=Path,
                        help="path for the offline comparison JSON")
    args = parser.parse_args(argv)

    try:
        report = load_snapshot_report(args.snapshot)
        profile = load_profile(args.profile)
        comparison = compare_snapshot_to_profile(report, profile)
        comparison["source_snapshot"]["path"] = str(args.snapshot.resolve())
        comparison["profile_path"] = str(args.profile.resolve())
        rendered = json.dumps(comparison, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))

    region_blocker_count = sum(len(region["blocker_codes"]) for region in comparison["regions"])
    candidate_blocker_count = sum(
        len(candidate["blocker_codes"])
        for region in comparison["regions"]
        for tier in region["tiers"].values()
        for candidate in [tier["suggested_candidate"]]
        if candidate is not None
    )
    global_blocker_count = len(comparison["global_blockers"])
    print(json.dumps({
        "comparison_path": str(args.output.resolve()),
        "status": comparison["status"],
        "controller_contacted": False,
        "global_blocker_count": global_blocker_count,
        "region_blocker_count": region_blocker_count,
        "candidate_blocker_count": candidate_blocker_count,
        "blocker_count": global_blocker_count + region_blocker_count + candidate_blocker_count,
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Print an offline, proposal-only three-level avoidance geometry report."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from g1_cup_grasp.avoidance_profile import build_proposal, load_profile


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "avoidance_profile.example.yaml",
        help="local YAML proposal profile (defaults to the included example)",
    )
    parser.add_argument(
        "--output", type=Path, help="optional path for a JSON copy of the proposal"
    )
    args = parser.parse_args(argv)

    try:
        proposal = build_proposal(load_profile(args.config))
        rendered = json.dumps(proposal, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

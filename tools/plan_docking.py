#!/usr/bin/env python3
"""Write a read-only JSON proposal for the A-to-B docking phases."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.config import ConfigError, load_yaml
from g1_cup_grasp.docking_plan import DockingPlanError, create_docking_plan


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create an offline-only docking proposal. The output is never an "
            "executable robot command."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config/docking_plan.example.yaml",
        help="docking proposal assumptions (default: config/docking_plan.example.yaml)",
    )
    parser.add_argument(
        "--transport-config",
        type=Path,
        default=ROOT / "config/transport.yaml",
        help="station/map source (default: config/transport.yaml)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="write JSON to this path; without it, print JSON to stdout",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        docking_config = load_yaml(args.config)
        transport_config = load_yaml(args.transport_config)
        plan = create_docking_plan(docking_config, transport_config)
        rendered = json.dumps(plan, indent=2, allow_nan=False) + "\n"
        if args.output is None:
            sys.stdout.write(rendered)
        else:
            args.output.write_text(rendered, encoding="utf-8")
    except (ConfigError, DockingPlanError, OSError, TypeError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

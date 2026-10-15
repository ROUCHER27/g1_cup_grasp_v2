#!/usr/bin/env python3
"""Write an offline near-table clearance and braking-distance report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from g1_cup_grasp.config import ConfigError, load_yaml
from g1_cup_grasp.docking_clearance import (
    DockingClearanceError,
    evaluate_docking_clearance,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate proposal-only near-table geometry checks. This tool "
            "never authorizes robot motion."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config/docking_clearance.example.yaml",
        help="measured clearance inputs (default: config/docking_clearance.example.yaml)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "outputs/avoidance_design_20261014/docking_clearance_check.json",
        help="report path (default: outputs/avoidance_design_20261014/docking_clearance_check.json)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        config = load_yaml(args.config)
        report = evaluate_docking_clearance(config)
        rendered = json.dumps(report, indent=2, allow_nan=False) + "\n"
        args.output.write_text(rendered, encoding="utf-8")
    except (ConfigError, DockingClearanceError, OSError, TypeError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

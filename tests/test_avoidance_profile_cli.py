from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools")]
import check_avoidance_profile as app
from test_avoidance_guard import snapshot
from g1_cup_grasp.avoidance_guard import build_manifest_candidate, canonical_digest


class AvoidanceProfileCliTests(unittest.TestCase):
    def run_cli(self, saved, *, approved=False, overflow_number=False):
        with TemporaryDirectory() as folder:
            directory = Path(folder)
            config = directory / "config.yaml"
            config.write_text(yaml.safe_dump({"robot_id": 424, "map_id": 7, "map_name": "cup_new"}))
            source = directory / "snapshot.json"
            source_text = json.dumps(saved)
            if overflow_number:
                source_text = source_text.replace('"query_duration_s": ' + json.dumps(saved['query_duration_s']),
                                                  '"query_duration_s": 1e9999')
            source.write_text(source_text)
            output = directory / "report.json"
            args = ["--config", str(config), "--snapshot", str(source), "--output", str(output)]
            if approved:
                manifest = build_manifest_candidate(saved, map_name="cup_new", hal_sha256=app.HAL_SHA256)
                manifest["status"] = "approved_controller_snapshot"
                manifest["canonical_sha256"] = canonical_digest(manifest)
                path = directory / "manifest.json"; path.write_text(json.dumps(manifest))
                args += ["--manifest", str(path)]
            with patch.object(app, "read_avoidance_snapshot", side_effect=AssertionError("offline CLI accessed network")), patch("builtins.print"):
                code = app.main(args)
            return code, json.loads(output.read_text())

    def test_offline_capture_is_always_review_required_and_not_live(self):
        code, report = self.run_cli(snapshot())
        self.assertEqual(code, 0, report.get("error"))
        self.assertEqual(report["status"], "MANIFEST_REVIEW_REQUIRED")
        self.assertEqual(report["manifest_candidate"]["status"], "review_required")
        self.assertFalse(report["live_read"])
        self.assertFalse(report["live_freshness_verified"])
        self.assertFalse(report["robot_commands_sent"])

    def test_offline_comparison_never_claims_live_or_physical_validation(self):
        code, report = self.run_cli(snapshot(), approved=True)
        self.assertEqual(code, 0, report.get("error"))
        self.assertEqual(report["status"], "OFFLINE_CONFIGURATION_MATCH")
        self.assertFalse(report["verification"]["live_freshness_verified"])
        self.assertFalse(report["physical_3d_collision_verified"])

    def test_historical_other_map_is_not_accepted_as_map_seven(self):
        saved = snapshot(); saved["parameters"]["mapId"] = 6
        code, report = self.run_cli(saved)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertIn("map", report["error"])

    def test_nan_input_is_reported_without_invalid_json_or_network(self):
        saved = snapshot(); saved["query_duration_s"] = float("nan")
        code, report = self.run_cli(saved)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertIn("non-finite", report["error"])

    def test_overflow_json_number_is_a_structured_error(self):
        code, report = self.run_cli(snapshot(), overflow_number=True)
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertIn("finite", report["error"])


if __name__ == "__main__":
    unittest.main()

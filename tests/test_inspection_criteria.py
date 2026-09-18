from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ego_calibration.inspection import InspectionSettings, StereoInspector, write_report
from ego_calibration.inspection_criteria import calibration_comparison, inspection_criteria
from ego_calibration.models import CalibrationError
from test_inspection import inspection_payload, inspection_settings, observations


class InspectionCriteriaTest(unittest.TestCase):
    def test_error_limits_include_boundary_and_do_not_pass_missing_observations(self):
        settings = inspection_settings()
        for increment, status in ((0, "pass"), (0.0001, "fail")):
            stats = {key: {field: limit + increment} for key, field, limit in (
                ("left", "rms", 1), ("right", "rms", 1),
                ("left_to_right", "rms", 1.5), ("right_to_left", "rms", 1.5),
                ("epipolar", "p95", 1),
            )}
            criteria = inspection_criteria(settings, stats)["criteria"]
            self.assertEqual([c["status"] for c in criteria], [status] * 5)
        empty = inspection_criteria(settings, {})["criteria"]
        self.assertTrue(all(c["status"] == "incomplete" and c["value"] is None for c in empty))

    def test_larger_error_than_historical_fit_can_still_pass_current_limits(self):
        settings, payload = inspection_settings(), inspection_payload()
        inspector = StereoInspector("ego-std", payload, settings)
        image = np.full((800, 1280), 128, np.uint8)
        for i, (rotation, depth) in enumerate((((0, 0, 0), 1), ((0.6, 0.1, 0.1), 1.3), ((-0.3, -0.4, 0.2), 0.9))):
            inspector.inspect(image, image, i, observations(settings, rotation, (-0.2, -0.2, depth)))
        inspector.errors["left"] = [0.4] * len(inspector.errors["left"])
        report = inspector.report(finished=True, source={})
        self.assertEqual(report["status"], "pass")
        row = report["calibration_comparison"]["rows"][0]
        self.assertAlmostEqual(row["delta_px"], 0.3)
        self.assertEqual(row["status"], "pass")
        self.assertEqual(report["calibration"], payload)
        self.assertTrue(report["calibration_comparison"]["parameters_fixed"])

    def test_joint_fit_is_not_compared_to_cross_prediction_or_epipolar_error(self):
        acceptance = inspection_criteria(inspection_settings(), {
            "left_to_right": {"rms": 0.5}, "right_to_left": {"rms": 0.6}, "epipolar": {"p95": 0.7},
        })
        rows = {r["key"]: r for r in calibration_comparison(inspection_payload(), acceptance, resolution_confirmed=True)["rows"]}
        for key in ("left_to_right", "right_to_left", "epipolar"):
            self.assertIsNone(rows[key]["historical_value"])
            self.assertIsNone(rows[key]["delta_px"])
        self.assertEqual(rows["historical_stereo"]["historical_value"], 0.1)
        self.assertIsNone(rows["historical_stereo"]["value"])
        self.assertEqual(rows["historical_stereo"]["status"], "reference")

    def test_missing_or_invalid_history_and_bundled_reference_do_not_become_zero(self):
        acceptance = inspection_criteria(inspection_settings(), {"left": {"rms": 0.4}})
        for value in (None, "0.1", True, -1, float("nan"), float("inf")):
            payload = inspection_payload()
            payload["metrics"]["left_calibrate_rms"] = value
            payload["common_calibration"] = {"metrics": {"left_calibrate_rms": 0.05}}
            row = calibration_comparison(payload, acceptance, resolution_confirmed=True)["rows"][0]
            self.assertIsNone(row["historical_value"])
            self.assertIsNone(row["delta_px"])
        # Generic / Kalibr / OAK formats do not promise the same history fields.
        payload["format"] = "generic_stereo"
        payload["metrics"]["left_calibrate_rms"] = 0.1
        row = calibration_comparison(payload, acceptance, resolution_confirmed=True)["rows"][0]
        self.assertIsNone(row["historical_value"])

    def test_zero_history_is_valid_but_unknown_resolution_suppresses_delta(self):
        payload = inspection_payload()
        payload["metrics"]["left_calibrate_rms"] = 0
        acceptance = inspection_criteria(inspection_settings(), {"left": {"rms": 0.4}})
        for confirmed in (False, True):
            row = calibration_comparison(payload, acceptance, resolution_confirmed=confirmed)["rows"][0]
            self.assertEqual(row["historical_value"], 0)
            self.assertEqual(row["delta_px"], 0.4 if confirmed else None)

    def test_reference_note_does_not_claim_standard_certification(self):
        for bad in (None, 123, "a" * 201):
            with self.assertRaises(CalibrationError):
                InspectionSettings.from_dict({"criteria_reference": bad})
        settings = inspection_settings(mono_rms_limit=0.5, criteria_reference=" 项目测试文件 v1 §2 ")
        acceptance = inspection_criteria(settings, {})
        self.assertEqual(acceptance["basis"], "user_configured")
        self.assertEqual(acceptance["reference"], "项目测试文件 v1 §2")
        self.assertEqual(acceptance["standards_compliance"], "not_assessed")

    def test_json_and_html_persist_limits_and_history_and_escape_notes(self):
        settings = inspection_settings(criteria_reference='<script>alert("x")</script>')
        report = StereoInspector("ego-std", inspection_payload(), settings).report(finished=True, source={})
        with tempfile.TemporaryDirectory() as directory:
            write_report(Path(directory), report)
            saved = json.loads((Path(directory) / "report.json").read_text())
            self.assertEqual(saved["acceptance"], report["acceptance"])
            self.assertEqual(saved["calibration_comparison"], report["calibration_comparison"])
            html = (Path(directory) / "report.html").read_text()
            self.assertIn("判定要求与已有标定对比", html)
            self.assertIn("≤ 1.5", html)
            self.assertIn("0.100", html)
            self.assertIn("未提供同口径极线 P95", html)
            self.assertIn("&lt;script&gt;", html)
            self.assertNotIn("<script>", html)
            self.assertEqual(saved["status"], "incomplete")


if __name__ == "__main__":
    unittest.main()

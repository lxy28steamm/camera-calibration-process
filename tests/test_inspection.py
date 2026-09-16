from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from ego_calibration.inspection import InspectionSettings, StereoInspector, StereoModel, write_report
from ego_calibration.models import CalibrationError
from test_validation import _lite_payload, _std_payload


def inspection_payload():
    payload = _std_payload()
    K = [[800.0, 0, 640], [0, 800.0, 400], [0, 0, 1]]
    payload["calibration"].update(K1=K, K2=copy.deepcopy(K), D1=[0.0]*8, D2=[0.0]*8, T=[[-60.0], [0.0], [0.0]])
    payload["metrics"]["baseline_mm"] = 60.0
    return payload


def inspection_settings(**values):
    return InspectionSettings.from_dict({"width": 1280, "height": 800, "resolution_confirmed": True, "min_sharpness": 0, "min_views": 3, "min_cells": 1, **values})


def observations(settings, rotation=(0.2, -0.3, 0.1), translation=(-0.2, -0.2, 1.0), corner_order=(0, 1, 2, 3)):
    model = StereoModel.from_payload("ego-std", inspection_payload(), settings)
    ids = list(range(36))
    points = settings.points(ids, corner_order)
    R = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0]
    T = np.asarray(translation, dtype=float).reshape(3, 1)
    left = model.left.project(points, R, T).reshape(-1, 4, 2)
    right = model.right.project(points, model.R@R, model.R@T+model.T).reshape(-1, 4, 2)
    return ({i: p for i, p in zip(ids, left)}, {i: p for i, p in zip(ids, right)})


class StereoInspectionTest(unittest.TestCase):
    def setUp(self):
        self.settings = inspection_settings()
        self.image = np.full((800, 1280), 128, np.uint8)

    def inspect(self, inspector, **kwargs):
        return inspector.inspect(self.image, self.image, len(inspector.samples), observations(self.settings, **kwargs))[0]

    def test_exact_calibration_has_negligible_mono_and_cross_camera_errors(self):
        inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
        sample = self.inspect(inspector)
        self.assertTrue(sample["accepted"])
        for stats in sample["metrics"].values():
            self.assertLess(stats["rms"], 0.001)
        self.assertAlmostEqual(sample["positive_depth_fraction"], 1.0)

    def test_bad_extrinsic_cannot_hide_behind_independent_monocular_pose_fits(self):
        payload = inspection_payload()
        payload["calibration"]["R"] = cv2.Rodrigues(np.array([0.0, 0.04, 0.0]))[0].tolist()
        inspector = StereoInspector("ego-std", payload, self.settings)
        sample = self.inspect(inspector)
        self.assertLess(sample["metrics"]["left"]["rms"], 0.001)
        self.assertLess(sample["metrics"]["right"]["rms"], 0.001)
        self.assertGreater(sample["metrics"]["left_to_right"]["rms"], 20)

    def test_incorrect_baseline_is_detected_by_metric_scale_and_cross_projection(self):
        payload = inspection_payload()
        payload["calibration"]["T"][0][0] = -120
        payload["metrics"]["baseline_mm"] = 120
        sample = self.inspect(StereoInspector("ego-std", payload, self.settings))
        self.assertGreater(sample["metrics"]["left_to_right"]["rms"], 20)
        self.assertAlmostEqual(sample["metrics"]["tag_scale_percent"]["mean"], 100, places=4)

    def test_bad_intrinsics_are_visible_on_tilted_board(self):
        payload = inspection_payload()
        payload["calibration"]["K1"][1][1] *= 1.3
        sample = self.inspect(StereoInspector("ego-std", payload, self.settings), rotation=(0.5, -0.4, 0.1))
        self.assertGreater(sample["metrics"]["left"]["rms"], 1)

    def test_duplicate_pose_is_not_counted_as_additional_coverage(self):
        inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
        self.inspect(inspector)
        sample = self.inspect(inspector)
        self.assertFalse(sample["accepted"])
        self.assertIn("位置与已有样本接近", sample["reason"])
        self.assertEqual(inspector.report(finished=True, source={})["counts"]["accepted"], 1)

    def test_unknown_resolution_and_no_board_never_pass(self):
        inspector = StereoInspector("ego-std", inspection_payload(), inspection_settings(resolution_confirmed=False))
        sample, _ = inspector.inspect(self.image, self.image, 0)
        self.assertFalse(sample["accepted"])
        report = inspector.report(finished=True, source={})
        self.assertEqual(report["status"], "incomplete")
        self.assertIsNone(report["metrics"]["left"])
        self.assertEqual(report["resolution_source"], "unconfirmed")

    def test_sufficient_good_views_pass_and_report_can_be_exported(self):
        inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
        for rotation, depth in (((0, 0, 0), 1), ((0.6, 0.1, 0.1), 1.3), ((-0.3, -0.4, 0.2), 0.9)):
            self.inspect(inspector, rotation=rotation, translation=(-0.2, -0.2, depth))
        report = inspector.report(finished=True, source={})
        self.assertEqual(report["status"], "pass", report["checks"])
        with tempfile.TemporaryDirectory() as directory:
            write_report(Path(directory), report)
            saved = json.loads((Path(directory)/"report.json").read_text())
            self.assertEqual(saved["counts"]["accepted"], 3)
            self.assertIn("通过本次设置阈值", (Path(directory)/"report.html").read_text())
            self.assertTrue((Path(directory)/"samples.csv").exists())

    def test_mismatched_image_resolution_is_not_silently_resized(self):
        inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
        with self.assertRaisesRegex(CalibrationError, "未自动缩放内参"):
            inspector.inspect(self.image[:400], self.image[:400], 0)

    def test_board_corner_conventions_are_resolved_without_changing_calibration(self):
        for order in ((0, 1, 2, 3), (3, 2, 1, 0), (2, 3, 0, 1)):
            inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
            sample = self.inspect(inspector, corner_order=order)
            self.assertTrue(sample["accepted"], sample["reason"])
            self.assertLess(sample["metrics"]["left_to_right"]["rms"], 0.001)

    def test_detector_accepts_standard_and_kalibr_black_borders(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        inspector = StereoInspector("ego-std", inspection_payload(), self.settings)
        for border in (1, 2):
            frame = np.full((300, 300), 255, np.uint8)
            frame[40:240, 40:240] = cv2.aruco.generateImageMarker(dictionary, 7, 200, borderBits=border)
            self.assertIn(7, inspector.detect(frame))

    def test_invalid_settings_and_baseline_units_are_rejected(self):
        for value in ({"width": 1}, {"tag_size_m": float("nan")}, {"rows": True}, {"resolution_confirmed": "true"}):
            with self.assertRaises(CalibrationError):
                InspectionSettings.from_dict(value)
        payload = inspection_payload()
        payload["calibration"]["T"][0][0] = -0.06
        with self.assertRaisesRegex(CalibrationError, "单位"):
            StereoModel.from_payload("ego-std", payload, self.settings)

    def test_lite_uses_full_distortion_and_meter_transform(self):
        payload = _lite_payload()
        for name in ("left_mono", "right_mono"):
            payload["cameras"][name]["K"] = inspection_payload()["calibration"]["K1"]
        payload["camera_extrinsics"]["T_right_mono_from_left_mono"]["matrix_m"][0][3] = -0.06
        model = StereoModel.from_payload("ego-lite", payload, self.settings)
        self.assertEqual(model.resolution_source, "device")
        self.assertEqual(model.left.D.size, 14)
        self.assertAlmostEqual(model.T[0, 0], -0.06)

    def test_kalibr_lens_models_preserve_device_resolution_and_projection(self):
        for lens, distortion in (("radtan", "[0, 0, 0, 0]"), ("equidistant", "[0.01, 0, 0, 0]"), ("none", "[]")):
            text = "".join(f"{name}:\n  camera_model: pinhole\n  intrinsics: [800, 800, 640, 400]\n  distortion_model: {lens}\n  distortion_coeffs: {distortion}\n  resolution: [1280, 800]\n" for name in ("cam0", "cam1"))
            transform = np.eye(4)
            transform[0, 3] = -0.06
            payload = {"format": "kalibr_camchain_imucam", "kalibr_yaml": text, "kalibr_calibration": {"cam1": {"T_cn_cnm1": transform.tolist()}}}
            model = StereoModel.from_payload("ego-std", payload, InspectionSettings())
            self.assertEqual(model.left.resolution, (1280, 800))
            self.assertEqual(model.left.model, lens)
            points = self.settings.points(list(range(36)))
            rotation = cv2.Rodrigues(np.array([0.2, 0.1, 0.3]))[0]
            translation = np.array([[-0.2], [-0.2], [1.0]])
            pixels = model.left.project(points, rotation, translation)
            solved_r, solved_t = model.left.pose(points, pixels)
            np.testing.assert_allclose(solved_r, rotation, atol=1e-6)
            np.testing.assert_allclose(solved_t, translation, atol=1e-6)


if __name__ == "__main__":
    unittest.main()

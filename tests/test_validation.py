from __future__ import annotations

import copy
import unittest

from ego_calibration.validation import validate_calibration


def _identity(size: int) -> list[list[float]]:
    return [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]


def _transform() -> dict[str, object]:
    return {
        "available": True,
        "matrix_cm": _identity(4),
        "matrix_m": _identity(4),
    }


def _lite_payload() -> dict[str, object]:
    camera_names = ("rgb", "left_mono", "right_mono")
    camera_transforms = (
        "T_left_mono_from_rgb",
        "T_left_mono_from_right_mono",
        "T_rgb_from_left_mono",
        "T_rgb_from_right_mono",
        "T_right_mono_from_left_mono",
        "T_right_mono_from_rgb",
    )
    imu_transforms = (
        name
        for camera in camera_names
        for name in (f"T_imu_from_{camera}", f"T_{camera}_from_imu")
    )
    return {
        "device": {"mxid": "MXID-001", "imu_type": "BNO086"},
        "raw_eeprom": {"version": 7},
        "schema": {"matrix_cm": "4x4"},
        "cameras": {
            name: {
                "K": _identity(3),
                "calibration_resolution": [1280, 800],
                "distortion_coefficients": [0.0] * 14,
                "distortion_model": "Perspective",
                "socket": f"CAM_{index}",
            }
            for index, name in enumerate(camera_names)
        },
        "camera_extrinsics": {name: _transform() for name in camera_transforms},
        "imu_extrinsics": {name: _transform() for name in imu_transforms},
    }


def _std_payload() -> dict[str, object]:
    return {
        "format": "stereo_calibration",
        "header": {
            "magic": "ZXCZ",
            "schema_version": 1,
            "payload_length": 896,
            "serial_number": "YCTC-001",
        },
        "calibration": {
            "K1": _identity(3),
            "D1": [0.0] * 8,
            "K2": _identity(3),
            "D2": [0.0] * 8,
            "R": _identity(3),
            "T": [[0.06], [0.0], [0.0]],
            "R1": _identity(3),
            "R2": _identity(3),
            "P1": [[1.0, 0.0, 0.0, 0.0]] * 3,
            "P2": [[1.0, 0.0, 0.0, 0.0]] * 3,
            "Q": _identity(4),
        },
        "metrics": {
            "stereo_rms": 0.1,
            "left_calibrate_rms": 0.1,
            "right_calibrate_rms": 0.1,
            "sample_count": 20.0,
            "baseline_mm": 60.0,
            "yaw_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
        },
        "common_calibration": {
            "cam0": {"T_cam_imu": _identity(4)},
            "cam1": {
                "T_cam_imu": _identity(4),
                "T_cn_cnm1": _identity(4),
            },
            "imu0": {"T_i_b": _identity(4)},
        },
    }


def _std_kalibr_payload() -> dict[str, object]:
    required_fields = [
        "cam0:",
        "cam1:",
        "T_cam_imu:",
        "T_cn_cnm1:",
        "timeshift_cam_imu:",
    ]
    return {
        "format": "kalibr_camchain_imucam",
        "header": {
            "magic": "ZXCZ",
            "schema_version": 2,
            "payload_length": 1024,
            "serial_number": "ZXCZ-001",
        },
        "payload": {
            "length": 1024,
            "parser": "kalibr_yaml_v2",
            "encoding": "utf-8",
            "required_fields": required_fields,
        },
        "kalibr_yaml": "\n".join(required_fields),
        "kalibr_calibration": {
            "cam0": {"T_cam_imu": _identity(4)},
            "cam1": {
                "T_cam_imu": _identity(4),
                "T_cn_cnm1": _identity(4),
            },
        },
        "common_calibration": {
            "cam0": {"T_cam_imu": _identity(4)},
            "cam1": {
                "T_cam_imu": _identity(4),
                "T_cn_cnm1": _identity(4),
            },
            "imu0": {"T_i_b": _identity(4)},
        },
    }


class CalibrationValidationTest(unittest.TestCase):
    def test_accepts_complete_ego_lite_calibration(self) -> None:
        report = validate_calibration("ego-lite", _lite_payload())

        self.assertFalse(report.passed)
        self.assertEqual(report.failure_count, 0)
        self.assertEqual(report.rotation_count, 12)
        self.assertEqual(report.skipped_count, 4)

    def test_rejects_zero_imu_rotation(self) -> None:
        payload = _lite_payload()
        matrix = payload["imu_extrinsics"]["T_imu_from_rgb"]["matrix_cm"]
        for row in range(3):
            for column in range(3):
                matrix[row][column] = 0.0

        report = validate_calibration("ego-lite", payload)
        failures = [check for check in report.checks if not check.passed]

        self.assertFalse(report.passed)
        self.assertTrue(any("IMU 外参" in check.name for check in failures))
        self.assertTrue(any("全为 0" in check.detail for check in failures))

    def test_rejects_non_orthogonal_rotation(self) -> None:
        payload = _lite_payload()
        payload["camera_extrinsics"]["T_left_mono_from_rgb"]["matrix_cm"][2][2] = 2.0

        report = validate_calibration("ego-lite", payload)

        self.assertTrue(
            any(
                not check.passed and "正交误差" in check.detail
                for check in report.checks
            )
        )

    def test_reports_missing_calibration_field(self) -> None:
        payload = _lite_payload()
        del payload["cameras"]["rgb"]["K"]

        report = validate_calibration("ego-lite", payload)

        self.assertTrue(
            any(not check.passed and check.name == "相机 rgb" for check in report.checks)
        )

    def test_accepts_complete_ego_std_calibration(self) -> None:
        report = validate_calibration("ego-std", _std_payload())

        self.assertTrue(report.passed)
        self.assertEqual(report.rotation_count, 3)
        self.assertEqual(report.info_count, 3)
        self.assertEqual(report.skipped_count, 0)

    def test_does_not_fail_for_unmapped_usb_and_calibration_serials(self) -> None:
        payload = _std_payload()
        payload["device_identity"] = {
            "usb_serial_number": "064014231235",
            "calibration_serial_number": "YC202607090121",
            "serials_match": False,
            "serial_comparison": "unmapped",
        }

        report = validate_calibration("ego-std", payload)

        self.assertTrue(report.passed)
        identity_check = next(
            check for check in report.checks if check.name == "USB 与标定序列号"
        )
        self.assertIn("未提供两套编号的映射", identity_check.detail)

    def test_std_variants_ignore_missing_or_invalid_common_calibration(self) -> None:
        for kind in ("ego-std", "ego-std-235"):
            for make_payload in (_std_payload, _std_kalibr_payload):
                original = make_payload()
                expected = validate_calibration(kind, original)
                for common in (None, {}, "invalid", {"imu0": {"T_i_b": [[0.0] * 4] * 4}}):
                    with self.subTest(kind=kind, format=original["format"], common=common):
                        payload = copy.deepcopy(original)
                        if common is None:
                            del payload["common_calibration"]
                        else:
                            payload["common_calibration"] = common
                        self.assertEqual(validate_calibration(kind, payload), expected)

    def test_displays_each_stored_rms_without_applying_a_quality_threshold(self) -> None:
        payload = _std_payload()
        payload["metrics"].update(
            left_calibrate_rms=0.0, right_calibrate_rms=1.5, stereo_rms=20.0
        )

        report = validate_calibration("ego-std", payload)
        checks = [check for check in report.checks if check.informational]

        self.assertTrue(report.passed)
        self.assertEqual(report.failure_count, 0)
        self.assertEqual(len(checks), 3)
        for check, name, value, source in zip(
            checks,
            ("左目内参 RMS", "右目内参 RMS", "双目内外参联合 RMS"),
            ("0", "1.5", "20"),
            ("left_calibrate_rms", "right_calibrate_rms", "stereo_rms"),
        ):
            self.assertEqual(check.name, name)
            self.assertIn(f"RMS={value} px", check.detail)
            self.assertIn(source, check.detail)
            self.assertIn("未设置合格阈值", check.detail)
        self.assertEqual(report.passed_count + report.info_count, len(report.checks))

    def test_rejects_invalid_rms_data(self) -> None:
        for field in ("left_calibrate_rms", "right_calibrate_rms", "stereo_rms"):
            for value in (-0.1, float("nan"), float("inf"), True, "0.1", None):
                with self.subTest(field=field, value=value):
                    payload = _std_payload()
                    payload["metrics"][field] = value
                    report = validate_calibration("ego-std", payload)
                    failures = [
                        check for check in report.checks
                        if check.category == "二维重投影误差" and check.passed is False
                    ]
                    self.assertEqual(len(failures), 1)
                    self.assertIn(field, failures[0].detail)
                    self.assertFalse(report.passed)

    def test_missing_rms_is_unavailable_and_never_uses_common_calibration(self) -> None:
        payload = _std_payload()
        del payload["metrics"]["stereo_rms"]
        payload["common_calibration"]["metrics"] = {"stereo_rms": 0.01}

        report = validate_calibration("ego-std", payload)

        self.assertFalse(report.passed)
        self.assertEqual(report.skipped_count, 1)
        self.assertEqual(report.info_count, 2)
        unavailable = next(check for check in report.checks if check.passed is None)
        self.assertEqual(unavailable.name, "双目内外参联合 RMS")
        self.assertIn("metrics.stereo_rms", unavailable.detail)

    def test_accepts_schema_v2_kalibr_calibration(self) -> None:
        report = validate_calibration("ego-std", _std_kalibr_payload())

        self.assertFalse(report.passed)
        self.assertEqual(report.failure_count, 0)
        self.assertEqual(report.rotation_count, 3)
        self.assertEqual(report.skipped_count, 3)
        self.assertEqual(report.info_count, 0)

    def test_rejects_zero_schema_v2_imu_rotation(self) -> None:
        payload = _std_kalibr_payload()
        matrix = payload["kalibr_calibration"]["cam0"]["T_cam_imu"]
        for row in range(3):
            for column in range(3):
                matrix[row][column] = 0.0

        report = validate_calibration("ego-std", payload)

        self.assertTrue(
            any(
                not check.passed
                and check.name == "Kalibr cam0.T_cam_imu"
                and "全为 0" in check.detail
                for check in report.checks
            )
        )


if __name__ == "__main__":
    unittest.main()

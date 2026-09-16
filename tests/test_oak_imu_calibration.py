from __future__ import annotations

import json
import tempfile
import unittest
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from ego_calibration.models import CalibrationError
from ego_calibration.oak_imu_calibration import (
    build_camchain_yaml,
    build_undistorted_camchain_yaml,
    capture_imu_noise_dataset,
    create_imu_noise_session_directory,
    default_aprilgrid_path,
    default_imu_path,
    detect_kalibr_environment,
    flash_backend_capabilities,
    flash_calibration_handler,
    kalibr_commands,
    kalibr_process_spec,
    load_kalibr_result,
    missing_kalibr_commands,
    write_ego_lite_report,
    _create_imu_capture_pipeline,
    _create_undistortion_map,
    _imu_csv_row,
    _match_stereo_timestamps,
    _pop_stereo_pairs,
    _queue_try_get_all,
    _undistort_frame,
    _verify_stereo_chain,
    validate_kalibr_dataset,
    write_default_aprilgrid_yaml,
)


class _Calibration:
    def getDefaultIntrinsics(self, socket: str):
        focal = 500.0 if socket == "CAM_B" else 501.0
        return [[focal, 0, 640], [0, focal, 400], [0, 0, 1]], 1280, 800

    def getDistortionCoefficients(self, _socket: str):
        return [0.1, -0.2, 0.003, -0.004, 0.05]

    def getDistortionModel(self, _socket: str):
        return "CameraModel.Perspective"

    def getCameraExtrinsics(self, source: str, target: str, use_spec: bool):
        assert (source, target, use_spec) == ("CAM_B", "CAM_C", False)
        return [
            [1, 0, 0, 7.5],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]


DAI = SimpleNamespace(
    CameraBoardSocket=SimpleNamespace(CAM_B="CAM_B", CAM_C="CAM_C")
)


def _result_yaml(rotation: float = 1.0) -> str:
    return f"""cam0:
  T_cam_imu:
    - [{rotation}, 0.0, 0.0, 0.012]
    - [0.0, {rotation}, 0.0, -0.003]
    - [0.0, 0.0, {rotation}, 0.004]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -0.00125
cam1:
  T_cn_cnm1:
    - [1.0, 0.0, 0.0, 0.075]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
"""


class OakImuCalibrationTest(unittest.TestCase):
    def test_creates_separate_imu_noise_session_directory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = create_imu_noise_session_directory(
                Path(directory), "MX-ID_123456789012"
            )

            self.assertTrue(result.is_dir())
            self.assertTrue(result.name.startswith("oak-imu-noise-123456789012-"))

    def test_rejects_invalid_imu_noise_duration_before_opening_device(self) -> None:
        with self.assertRaisesRegex(CalibrationError, "必须大于 0 秒"):
            capture_imu_noise_dataset("mxid", Path("unused"), 0)

    def test_stationary_capture_pipeline_only_enables_raw_imu(self) -> None:
        class _Output:
            def link(self, target) -> None:
                self.target = target

        class _Imu:
            out = _Output()

            def enableIMUSensor(self, sensors, rate) -> None:
                self.sensors = sensors
                self.rate = rate

            def setBatchReportThreshold(self, value) -> None:
                self.threshold = value

            def setMaxBatchReports(self, value) -> None:
                self.maximum = value

        class _XLinkOut:
            input = object()

            def setStreamName(self, value) -> None:
                self.name = value

        class _Pipeline:
            def __init__(self) -> None:
                self.imu = _Imu()
                self.output = _XLinkOut()

            def createIMU(self):
                return self.imu

            def createXLinkOut(self):
                return self.output

        pipeline = _create_imu_capture_pipeline(
            SimpleNamespace(
                Pipeline=_Pipeline,
                IMUSensor=SimpleNamespace(
                    ACCELEROMETER_RAW="accel_raw",
                    GYROSCOPE_RAW="gyro_raw",
                ),
            )
        )

        self.assertEqual(pipeline.imu.sensors, ["accel_raw", "gyro_raw"])
        self.assertEqual(pipeline.imu.rate, 200)
        self.assertEqual(pipeline.output.name, "calib_imu")

    def test_bundles_default_ego_lite_aprilgrid(self) -> None:
        text = default_aprilgrid_path().read_text(encoding="utf-8")

        self.assertIn("tagCols: 6", text)
        self.assertIn("tagRows: 6", text)
        self.assertIn("tagSize: 0.055", text)
        self.assertIn("tagSpacing: 0.3", text)

    def test_writes_default_aprilgrid_into_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = write_default_aprilgrid_yaml(Path(directory))

            self.assertEqual(target.name, "target.yaml")
            self.assertEqual(
                target.read_text(encoding="utf-8"),
                default_aprilgrid_path().read_text(encoding="utf-8"),
            )

    def test_default_imu_config_has_required_kalibr_fields(self) -> None:
        path = default_imu_path()
        text = path.read_text(encoding="utf-8")

        self.assertIn("accelerometer_noise_density:", text)
        self.assertIn("accelerometer_random_walk:", text)
        self.assertIn("gyroscope_noise_density:", text)
        self.assertIn("gyroscope_random_walk:", text)
        self.assertIn("rostopic: /imu0", text)
        self.assertIn("update_rate: 200.0", text)

    def test_prefers_measured_bno086_imu_config_when_available(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            measured = Path(directory) / "measured.yaml"
            measured.write_text("update_rate: 200.0\n")
            with patch("ego_calibration.oak_imu_calibration.MEASURED_BNO086_IMU_YAML", measured):
                self.assertEqual(default_imu_path(), measured)

    def test_formats_raw_imu_packet_for_kalibr_csv(self) -> None:
        gyro = SimpleNamespace(
            x=0.1,
            y=-0.2,
            z=0.3,
            getTimestampDevice=lambda: timedelta(seconds=1.25),
        )
        accel = SimpleNamespace(x=1.0, y=2.0, z=9.8)
        packet = SimpleNamespace(gyroscope=gyro, acceleroMeter=accel)

        row = _imu_csv_row(packet, 2_000_000_000)

        self.assertEqual(row, [3_250_000_000, 0.1, -0.2, 0.3, 1.0, 2.0, 9.8])

    def test_drains_all_available_output_packets(self) -> None:
        queue = SimpleNamespace(tryGetAll=lambda: ["first", "second"])

        self.assertEqual(_queue_try_get_all(queue), ["first", "second"])

    def test_pairs_stereo_packets_by_device_timestamp(self) -> None:
        left_one = object()
        left_two = object()
        right_two = object()
        right_three = object()
        left = {1_000_000_000: left_one, 1_050_000_000: left_two}
        right = {1_050_012_000: right_two, 1_100_012_000: right_three}

        pairs, discarded_left, discarded_right = _pop_stereo_pairs(left, right)

        self.assertEqual(pairs, [(left_two, right_two)])
        self.assertEqual(discarded_left, 1)
        self.assertEqual(discarded_right, 0)
        self.assertEqual(left, {})
        self.assertEqual(right, {1_100_012_000: right_three})

    def test_matches_stereo_image_timestamps_with_small_sensor_offset(self) -> None:
        left = [1_000_000_000, 1_050_000_000, 1_100_000_000]
        right = [1_000_012_000, 1_100_011_000, 1_150_012_000]

        result = _match_stereo_timestamps(left, right)

        self.assertEqual(result, (2, 1, 1))

    def test_builds_kalibr_camchain_from_eeprom(self) -> None:
        text = build_camchain_yaml(DAI, _Calibration())

        self.assertIn("cam0:", text)
        self.assertIn("cam1:", text)
        self.assertIn("intrinsics: [500", text)
        self.assertEqual(text.count("distortion_model: none"), 2)
        self.assertEqual(text.count("distortion_coeffs: []"), 2)
        self.assertIn("- [1, 0, 0, 0.075]", text)
        self.assertIn("rostopic: /cam1/image_raw", text)

    def test_builds_zero_distortion_camchain_for_undistorted_capture(self) -> None:
        text = build_undistorted_camchain_yaml(DAI, _Calibration())

        self.assertEqual(text.count("distortion_model: none"), 2)
        self.assertEqual(text.count("distortion_coeffs: []"), 2)
        self.assertIn("resolution: [1280, 800]", text)

    def test_undistortion_map_accepts_full_depthai_perspective_coefficients(self) -> None:
        class _Perspective14(_Calibration):
            def getDistortionCoefficients(self, _socket: str):
                return [0.0] * 14

        import cv2
        import numpy as np

        mapping = _create_undistortion_map(cv2, _Perspective14(), "CAM_B")
        frame = np.zeros((800, 1280), dtype=np.uint8)
        corrected = _undistort_frame(cv2, frame, mapping)

        self.assertEqual(mapping.resolution, (1280, 800))
        self.assertEqual(corrected.shape, frame.shape)

    def test_loads_and_validates_kalibr_imu_to_left_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result-camchain-imucam.yaml"
            path.write_text(_result_yaml(), encoding="utf-8")

            result = load_kalibr_result(path)

        self.assertEqual(result.translation_m, (0.012, -0.003, 0.004))
        self.assertEqual(result.determinant, 1.0)
        self.assertEqual(result.timeshift_seconds, -0.00125)

    def test_rejects_zero_rotation_before_eeprom_write(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.yaml"
            path.write_text(_result_yaml(0.0), encoding="utf-8")

            with self.assertRaisesRegex(CalibrationError, "旋转矩阵无效"):
                load_kalibr_result(path)

    def test_rejects_result_from_a_different_stereo_chain(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.yaml"
            path.write_text(_result_yaml(), encoding="utf-8")
            result = load_kalibr_result(path)

        result.stereo_matrix_m[0][3] = 0.2
        with self.assertRaisesRegex(CalibrationError, "不一致"):
            _verify_stereo_chain(DAI, _Calibration(), result)

    def test_builds_shell_free_kalibr_commands(self) -> None:
        dataset = Path("/tmp/dataset with space")
        commands = kalibr_commands(dataset, Path("/tmp/target.yaml"), Path("/tmp/imu.yaml"))

        self.assertEqual(commands[0][0], "kalibr_bagcreater")
        self.assertEqual(commands[1][0], "kalibr_calibrate_imu_camera")
        self.assertIn("--cams", commands[1])
        self.assertIn(str(dataset.resolve() / "camchain.yaml"), commands[1])

    def test_reports_missing_kalibr_executables(self) -> None:
        with (
            patch("ego_calibration.oak_imu_calibration.shutil.which", return_value=None),
            patch("ego_calibration.oak_imu_calibration.kalibr_setup_candidates", return_value=()),
        ):
            missing = missing_kalibr_commands()

        self.assertEqual(
            missing,
            ("kalibr_bagcreater", "kalibr_calibrate_imu_camera"),
        )

    def test_validates_capture_dataset_before_kalibr(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "cam0").mkdir()
            (root / "cam1").mkdir()
            (root / "cam0" / "1.png").write_bytes(b"")
            (root / "cam1" / "1.png").write_bytes(b"")
            (root / "imu0.csv").write_text(
                "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n"
                + "".join(
                    f"{1000000000 + index * 5000000},0,0,0,0,0,9.8\n"
                    for index in range(10)
                ),
                encoding="utf-8",
            )
            (root / "camchain.yaml").write_text(
                "cam0:\n  rostopic: /cam0/image_raw\n"
                "cam1:\n  rostopic: /cam1/image_raw\n",
                encoding="utf-8",
            )
            target = root / "target.yaml"
            target.write_text(
                "target_type: aprilgrid\ntagCols: 6\ntagRows: 6\n"
                "tagSize: 0.055\ntagSpacing: 0.3\n",
                encoding="utf-8",
            )
            imu = root / "imu.yaml"
            imu.write_text(
                "accelerometer_noise_density: 1\n"
                "accelerometer_random_walk: 1\n"
                "gyroscope_noise_density: 1\n"
                "gyroscope_random_walk: 1\n",
                encoding="utf-8",
            )
            summary = validate_kalibr_dataset(root, target, imu)
            self.assertIn("左右各 1 帧", summary)

    def test_wraps_kalibr_command_with_setup_script(self) -> None:
        from ego_calibration.oak_imu_calibration import KalibrEnvironment

        env = KalibrEnvironment(Path("/tmp/setup.bash"), {}, ())
        program, args = kalibr_process_spec(("kalibr_bagcreater", "--help"), env)
        self.assertEqual(program, "bash")
        self.assertEqual(args[0], "-lc")
        self.assertIn("source /tmp/setup.bash", args[1])

    def test_v2_flash_backend_uses_flash_calibration2_by_default(self) -> None:
        calls: list[str] = []
        device = SimpleNamespace(
            flashCalibration2=lambda _calibration: calls.append("v2"),
            flashCalibration=lambda _calibration: calls.append("v3"),
        )

        flash_calibration_handler(device, object(), "v2")

        self.assertEqual(calls, ["v2"])

    def test_v3_flash_backend_uses_flash_calibration(self) -> None:
        calls: list[str] = []
        device = SimpleNamespace(
            flashCalibration=lambda _calibration: calls.append("v3") or True,
        )

        flash_calibration_handler(device, object(), "v3")

        self.assertEqual(calls, ["v3"])

    def test_detects_v3_runtime_without_flash_calibration2(self) -> None:
        class _V3Device:
            def flashCalibration(self, _calibration):
                return None

        capabilities = flash_backend_capabilities(SimpleNamespace(Device=_V3Device))

        self.assertEqual(capabilities, {"v2": False, "v3": True})

    def test_writes_full_ego_lite_report_as_utf8_json(self) -> None:
        payload = {
            "camera_extrinsics": {},
            "cameras": {},
            "depthai_version": "2.32.0.0",
            "device": {"mxid": "测试设备"},
            "imu_extrinsics": {},
            "raw_eeprom": {"version": 7},
            "schema": {},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ego-lite-calibration.json"
            write_ego_lite_report(payload, path)
            text = path.read_text(encoding="utf-8")

        self.assertEqual(json.loads(text), payload)
        self.assertIn("测试设备", text)
        self.assertTrue(text.endswith("\n"))


if __name__ == "__main__":
    unittest.main()

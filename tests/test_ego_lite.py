from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from ego_calibration.ego_lite import _read_calibration_json, scan_devices


class _Calibration:
    _index = {"CAM_A": 1, "CAM_B": 2, "CAM_C": 3}

    def getDefaultIntrinsics(self, socket: str):
        value = self._index[socket] * 100.0
        return [[value, 0, 640], [0, value, 400], [0, 0, 1]], 1280, 800

    def getDistortionCoefficients(self, _socket: str):
        return [0.0] * 14

    def getDistortionModel(self, _socket: str):
        return "CameraModel.Perspective"

    def getLensPosition(self, socket: str):
        return 116 if socket == "CAM_A" else 0

    def getCameraExtrinsics(self, source: str, target: str, use_spec: bool):
        self._require_calibrated(use_spec)
        translation = float(self._index[target] - self._index[source])
        return [
            [1, 0, 0, translation],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]

    def getCameraToImuExtrinsics(self, _socket: str, use_spec: bool):
        self._require_calibrated(use_spec)
        return self._identity()

    def getImuToCameraExtrinsics(self, _socket: str, use_spec: bool):
        self._require_calibrated(use_spec)
        return self._identity()

    def eepromToJson(self):
        return {"version": 7, "cameraData": []}

    @staticmethod
    def _require_calibrated(use_spec: bool) -> None:
        if use_spec:
            raise AssertionError("must use calibrated translation")

    @staticmethod
    def _identity():
        return [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]


class _Device:
    def readCalibration2(self):
        return _Calibration()

    def getConnectedCameraFeatures(self):
        return [
            SimpleNamespace(socket="CAM_A", sensorName="IMX378"),
            SimpleNamespace(socket="CAM_B", sensorName="OV9282"),
            SimpleNamespace(socket="CAM_C", sensorName="OV9282"),
        ]

    def getIMUFirmwareVersion(self):
        return "3.9.9"

    def getConnectedIMU(self):
        return "BNO086"

    def getUsbSpeed(self):
        return "UsbSpeed.SUPER"


class _MissingImuCalibration(_Calibration):
    def getCameraToImuExtrinsics(self, _socket: str, _use_spec: bool):
        raise RuntimeError("IMU calibration data is not available on device yet.")

    def getImuToCameraExtrinsics(self, _socket: str, _use_spec: bool):
        raise RuntimeError("IMU calibration data is not available on device yet.")


class _MissingImuDevice(_Device):
    def readCalibration2(self):
        return _MissingImuCalibration()


DAI = SimpleNamespace(
    __version__="2.32.0.0",
    CameraBoardSocket=SimpleNamespace(CAM_A="CAM_A", CAM_B="CAM_B", CAM_C="CAM_C"),
)


class EgoLiteCalibrationTest(unittest.TestCase):
    def test_scan_identifies_depthai_device(self) -> None:
        info = SimpleNamespace(
            getMxId=lambda: "MXID-001",
            getProtocol=lambda: "X_LINK_USB_VSC",
            getName=lambda: "1.3.2",
        )
        dai = SimpleNamespace(
            Device=SimpleNamespace(getAllAvailableDevices=lambda: [info])
        )

        with patch("ego_calibration.ego_lite._load_depthai", return_value=dai):
            devices = scan_devices()

        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].kind, "ego-lite")
        self.assertEqual(devices[0].serial, "MXID-001")
        self.assertIn("DepthAI", devices[0].transport)
        self.assertEqual(devices[0].path, "1.3.2")

    def test_matches_runtime_shape_and_units(self) -> None:
        payload = _read_calibration_json(DAI, _Device(), "MXID-001")

        self.assertEqual(
            list(payload),
            [
                "camera_extrinsics",
                "cameras",
                "depthai_version",
                "device",
                "imu_extrinsics",
                "raw_eeprom",
                "schema",
            ],
        )
        self.assertEqual(set(payload["cameras"]), {"rgb", "left_mono", "right_mono"})
        self.assertEqual(payload["cameras"]["rgb"]["sensor"], "IMX378")
        transform = payload["camera_extrinsics"]["T_left_mono_from_rgb"]
        self.assertEqual(transform["matrix_cm"][0][3], 1.0)
        self.assertEqual(transform["matrix_m"][0][3], 0.01)
        self.assertEqual(payload["device"]["mxid"], "MXID-001")
        self.assertEqual(payload["raw_eeprom"]["version"], 7)

    def test_missing_imu_extrinsics_do_not_hide_camera_calibration(self) -> None:
        payload = _read_calibration_json(DAI, _MissingImuDevice(), "MXID-001")

        self.assertEqual(payload["cameras"]["left_mono"]["sensor"], "OV9282")
        transform = payload["imu_extrinsics"]["T_left_mono_from_imu"]
        self.assertFalse(transform["available"])
        self.assertIn("not available", transform["error"])


if __name__ == "__main__":
    unittest.main()

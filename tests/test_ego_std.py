from __future__ import annotations

import struct
import tempfile
import unittest
import zlib
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from ego_calibration.ego_std import (
    SELECTOR_CALIB_COMMAND,
    SELECTOR_CALIB_INFO,
    SELECTOR_CALIB_TRANSFER,
    UVC_GET_CUR,
    UVC_SET_CUR,
    _UsbXuQuery,
    _crc16_modbus,
    _extract_yaml_resolution,
    _scan_linux_video_devices,
    _usb_serial_from_identifier,
    read_calibration,
)
from ego_calibration.windows_ks import _identifier, _serial_from_path


def _calibration_blob() -> bytes:
    header = bytearray(64)
    struct.pack_into("<4sHHI", header, 0, b"ZXCZ", 1, 0, 112 * 8)
    header[12:24] = b"YCTC-TEST-01"
    values = struct.pack("<112d", *range(1, 113))
    body = bytes(header) + values
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def _kalibr_blob(*, zero_imu_rotation: bool = False) -> bytes:
    rotation = "0.0" if zero_imu_rotation else "1.0"
    yaml_text = f"""cam0:
  resolution: [1600, 1200]
  T_cam_imu:
    - [{rotation}, 0.0, 0.0, 0.0]
    - [0.0, {rotation}, 0.0, 0.0]
    - [0.0, 0.0, {rotation}, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: 0.0
cam1:
  resolution: [1600, 1200]
  T_cam_imu: [[1.0, 0.0, 0.0, 0.06],
              [0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0]]
  T_cn_cnm1:
    - [1.0, 0.0, 0.0, 0.06]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: 0.0
"""
    yaml_payload = yaml_text.encode("utf-8")
    header = bytearray(64)
    struct.pack_into("<4sHHI", header, 0, b"ZXCZ", 2, 0, len(yaml_payload))
    header[12:25] = b"ZXCZ-TEST-001"
    body = bytes(header) + yaml_payload
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


class _XuDevice:
    def __init__(self, blob: bytes) -> None:
        self.blob = blob
        self.pending: tuple[int, int, int] | None = None

    def query(self, selector: int, request: int, size: int, payload: bytes | None):
        if selector == SELECTOR_CALIB_INFO and request == UVC_GET_CUR:
            schema_version = struct.unpack_from("<H", self.blob, 4)[0]
            return struct.pack(
                "<HHIIB3x",
                schema_version,
                7,
                zlib.crc32(self.blob) & 0xFFFFFFFF,
                len(self.blob),
                0,
            )
        if selector == SELECTOR_CALIB_COMMAND and request == UVC_SET_CUR:
            assert payload is not None and size == 16
            command, _reserved, _schema, session, offset, length = struct.unpack(
                "<BBHIII", payload
            )
            assert command == 0x05
            self.pending = session, offset, length
            return b""
        if selector == SELECTOR_CALIB_TRANSFER and request == UVC_GET_CUR:
            assert self.pending is not None and size == 60
            session, offset, length = self.pending
            chunk = self.blob[offset : offset + length]
            self.pending = None
            return (
                struct.pack("<IIHH", session, offset, len(chunk), _crc16_modbus(chunk))
                + chunk
                + bytes(48 - len(chunk))
            )
        raise AssertionError((selector, request, size))


class EgoStdCalibrationTest(unittest.TestCase):
    def test_linux_scan_recognizes_sc235hgs_mic_as_ego_std(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "usb-YCTC_YCTC_SC235HGS_MIC_0152312181647-video-index0"
            path.touch()
            devices = _scan_linux_video_devices(Path(directory))
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].kind, "ego-std")
        self.assertEqual(devices[0].identifier, str(path))

    def test_linux_scan_identifies_yctc_video_device(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "usb-YCTC_YCTC_SC233HGS_0152312181647-video-index0"
            path.touch()

            devices = _scan_linux_video_devices(Path(directory))

        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].kind, "ego-std")
        self.assertEqual(devices[0].serial, "0152312181647")
        self.assertEqual(devices[0].model, "YCTC Stereo UVC")
        self.assertTrue(devices[0].accessible)

    def test_linux_scan_identifies_new_zxcz_video_device(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "usb-ZXCZ_ZXCZ_SC233HGS_Dual_064014231235-video-index0"
            path.touch()

            devices = _scan_linux_video_devices(Path(directory))

        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].serial, "064014231235")
        self.assertEqual(devices[0].model, "ZXCZ SC233HGS Dual")
        self.assertEqual(devices[0].transport, "UVC XU V5 · USB")

    def test_reads_blob_without_default_calibration(self) -> None:
        identifier = "/dev/v4l/by-id/usb-ZXCZ_ZXCZ_SC233HGS_Dual_064014231235-video-index0"
        payload = read_calibration(identifier, query=_XuDevice(_calibration_blob()).query)

        self.assertEqual(payload["format"], "stereo_calibration")
        self.assertEqual(payload["header"]["serial_number"], "YCTC-TEST-01")
        self.assertEqual(payload["calibration"]["K1"][0], [1.0, 2.0, 3.0])
        self.assertEqual(payload["calibration"]["T"], [[44.0], [45.0], [46.0]])
        self.assertEqual(payload["metrics"]["baseline_mm"], 109.0)
        self.assertEqual(payload["device_identity"]["usb_serial_number"], "064014231235")
        self.assertFalse(payload["device_identity"]["serials_match"])
        self.assertEqual(payload["device_identity"]["serial_comparison"], "unmapped")
        self.assertNotIn("common_calibration", payload)

    def test_extracts_usb_serial_from_identifiers(self) -> None:
        self.assertEqual(
            _usb_serial_from_identifier(
                "/dev/v4l/by-id/usb-ZXCZ_ZXCZ_SC233HGS_Dual_064014231235-video-index0"
            ),
            "064014231235",
        )
        self.assertEqual(
            _usb_serial_from_identifier("usb://1d6b:0004?serial=064014231235&interface=0"),
            "064014231235",
        )

    def test_reads_schema_v2_kalibr_yaml_and_matrices(self) -> None:
        blob = _kalibr_blob()

        payload = read_calibration("test", query=_XuDevice(blob).query)

        self.assertEqual(payload["format"], "kalibr_camchain_imucam")
        self.assertNotIn("common_calibration", payload)
        self.assertEqual(payload["header"]["schema_version"], 2)
        self.assertEqual(payload["kalibr_calibration"]["cam0"]["resolution"], [1600, 1200])
        self.assertEqual(payload["kalibr_calibration"]["cam1"]["resolution"], [1600, 1200])
        self.assertEqual(payload["header"]["serial_number"], "ZXCZ-TEST-001")
        self.assertEqual(payload["payload"]["parser"], "kalibr_yaml_v2")
        self.assertEqual(len(payload["payload"]["sha256"]), 64)
        self.assertEqual(
            payload["kalibr_calibration"]["cam0"]["T_cam_imu"][0],
            [1.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            payload["kalibr_calibration"]["cam1"]["T_cn_cnm1"][0][3],
            0.06,
        )
        self.assertEqual(payload["kalibr_yaml"].encode("utf-8"), blob[64:-4])

    def test_yaml_resolution_is_scoped_to_camera_and_requires_positive_integers(self) -> None:
        text = "cam0:\n  timeshift_cam_imu: 0\ncam1:\n  resolution: [1920, 1080] # pixels\n"
        self.assertIsNone(_extract_yaml_resolution(text, "cam0"))
        self.assertEqual(_extract_yaml_resolution(text, "cam1"), [1920, 1080])
        for value in ("[-1, 1200]", "[1600.5, 1200]", "[1600]", "[1e999, 1200]"):
            self.assertIsNone(_extract_yaml_resolution(f"cam0:\n  resolution: {value}", "cam0"))

    def test_rejects_tail_crc_mismatch(self) -> None:
        blob = bytearray(_calibration_blob())
        blob[100] ^= 1

        with self.assertRaisesRegex(RuntimeError, "尾部 CRC32"):
            read_calibration("test", query=_XuDevice(bytes(blob)).query)

    def test_usb_transport_uses_uvc_extension_unit_request(self) -> None:
        class Device:
            def __init__(self) -> None:
                self.calls = []

            def ctrl_transfer(self, *args, **kwargs):
                self.calls.append((args, kwargs))
                return bytes(range(args[4])) if isinstance(args[4], int) else len(args[4])

        transport = _UsbXuQuery("usb://1234:5678?interface=2")
        device = Device()
        transport._device = device

        self.assertEqual(
            transport.query(SELECTOR_CALIB_INFO, UVC_GET_CUR, 16, None),
            bytes(range(16)),
        )
        transport.query(SELECTOR_CALIB_COMMAND, UVC_SET_CUR, 3, b"abc")

        get_args, _get_kwargs = device.calls[0]
        set_args, _set_kwargs = device.calls[1]
        self.assertEqual(get_args[:4], (0xA1, UVC_GET_CUR, 0x0200, 0x0A02))
        self.assertEqual(set_args[:4], (0x21, UVC_SET_CUR, 0x0400, 0x0A02))

    def test_windows_ks_identifier_preserves_device_path(self) -> None:
        device_path = (
            r"@device:pnp:\\?\usb#vid_1234&pid_5678#064014231235#"
            r"{860bb310-5d01-11d0-bd3b-00a0c911ce86}"
        )

        identifier = _identifier(
            {"filter_index": 3, "node_id": 7, "device_path": device_path}
        )
        parsed = urlparse(identifier)
        values = parse_qs(parsed.query)

        self.assertEqual(parsed.scheme, "ks")
        self.assertEqual(parsed.netloc, "3")
        self.assertEqual(values["node"], ["7"])
        self.assertEqual(values["path"], [device_path])
        self.assertEqual(_serial_from_path(device_path), "064014231235")


if __name__ == "__main__":
    unittest.main()

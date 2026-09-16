from __future__ import annotations

import unittest
from unittest.mock import Mock, call, patch

import cv2
import numpy

from ego_calibration.preview import (
    AprilGridPreview,
    DEPTHAI_PREVIEW_SOURCES,
    EGO_STD_SINGLE_RESOLUTIONS,
    EGO_STD_STEREO_RESOLUTIONS,
    _combine_stereo_grayscale,
    _frame_to_image,
    _open_video_capture,
    _read_first_frame,
    read_uvc_resolution,
    uvc_sources,
)


class CameraPreviewTest(unittest.TestCase):
    def test_aprilgrid_preview_combines_pair_and_reports_detection(self) -> None:
        preview = AprilGridPreview()
        left = numpy.zeros((8, 12), dtype=numpy.uint8)
        right = numpy.zeros((8, 12), dtype=numpy.uint8)

        image, status = preview.render(left, right)

        self.assertEqual((image.width(), image.height()), (28, 8))
        self.assertIn("AprilTag", status)

    def test_aprilgrid_preview_detects_tag36h11_marker(self) -> None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 160)
        frame = numpy.full((220, 220), 255, dtype=numpy.uint8)
        frame[30:190, 30:190] = marker

        _image, status = AprilGridPreview().render(frame, frame)

        self.assertIn("左 1/36", status)
        self.assertIn("右 1/36", status)

    def test_offers_stereo_grayscale_for_oak_camera(self) -> None:
        self.assertIn(("双目灰度（左 | 右）", "stereo"), DEPTHAI_PREVIEW_SOURCES)

    def test_ego_std_supports_new_and_legacy_stereo_resolutions(self) -> None:
        self.assertEqual(
            EGO_STD_SINGLE_RESOLUTIONS,
            ((1920, 1080), (1600, 1200)),
        )
        self.assertEqual(
            EGO_STD_STEREO_RESOLUTIONS,
            ((3840, 1080), (3200, 1200)),
        )

    def test_combines_left_and_right_grayscale_frames(self) -> None:
        left = numpy.full((2, 3), 10, dtype=numpy.uint8)
        right = numpy.full((2, 3), 20, dtype=numpy.uint8)

        combined = _combine_stereo_grayscale(left, right)

        self.assertEqual(combined.shape, (2, 10))
        self.assertTrue(numpy.all(combined[:, :3] == 10))
        self.assertTrue(numpy.all(combined[:, 3:7] == 0))
        self.assertTrue(numpy.all(combined[:, 7:] == 20))

    def test_prefers_selected_linux_video_device(self) -> None:
        sources = uvc_sources("/dev/video2")

        self.assertEqual(sources[0][1], "/dev/video2")
        self.assertEqual(sources[1][1], 0)

    def test_offers_camera_indexes_for_usb_identifier(self) -> None:
        sources = uvc_sources("usb://1234:5678?interface=0")

        self.assertEqual(sources[0], ("视频设备 0", 0))
        self.assertEqual(sources[-1], ("视频设备 9", 9))

    def test_prefers_windows_ks_directshow_filter_index(self) -> None:
        sources = uvc_sources("ks://3?node=7&path=device")

        self.assertEqual(sources[0], ("已选 Ego-Std · 视频设备 3", 3))
        self.assertNotIn(("视频设备 3", 3), sources[1:])
        self.assertIn(("视频设备 0", 0), sources[1:])

    def test_windows_ego_std_negotiates_mjpeg_before_opening(self) -> None:
        class Capture:
            def __init__(self) -> None:
                self.open_args = None

            def set(self, *_args):
                return True

            def open(self, *args):
                self.open_args = args
                return True

            def isOpened(self):
                return True

            def read(self):
                return True, numpy.zeros((1080, 3840, 3), dtype=numpy.uint8)

            def release(self):
                pass

        class Cv2:
            CAP_DSHOW = 700
            CAP_MSMF = 1400
            CAP_ANY = 0
            CAP_PROP_FRAME_WIDTH = 3
            CAP_PROP_FRAME_HEIGHT = 4
            CAP_PROP_FPS = 5
            CAP_PROP_FOURCC = 6
            CAP_PROP_BUFFERSIZE = 38
            CAP_PROP_OPEN_TIMEOUT_MSEC = 53
            CAP_PROP_READ_TIMEOUT_MSEC = 54

            def __init__(self) -> None:
                self.capture = Capture()

            def VideoCapture(self):
                return self.capture

            @staticmethod
            def VideoWriter_fourcc(*codec):
                return int.from_bytes("".join(codec).encode("ascii"), "little")

        cv2_stub = Cv2()
        with patch("ego_calibration.preview.platform.system", return_value="Windows"):
            capture = _open_video_capture(cv2_stub, 1, EGO_STD_STEREO_RESOLUTIONS)

        self.assertIs(capture, cv2_stub.capture)
        source, backend, params = capture.open_args
        self.assertEqual((source, backend), (1, cv2_stub.CAP_DSHOW))
        self.assertIn(cv2_stub.VideoWriter_fourcc(*"MJPG"), params)
        self.assertIn(3840, params)
        self.assertIn(1080, params)

    def test_linux_prefers_mjpeg_and_accepts_actual_frame_resolution(self) -> None:
        capture = Mock()
        capture.read.return_value = (
            True, numpy.zeros((1200, 3840, 3), dtype=numpy.uint8)
        )
        with (
            patch("ego_calibration.preview.platform.system", return_value="Linux"),
            patch.object(cv2, "VideoCapture", return_value=capture),
        ):
            result = _open_video_capture(cv2, "/dev/video2", EGO_STD_STEREO_RESOLUTIONS)

        self.assertIs(result, capture)
        capture.set.assert_any_call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.assertNotIn(
            call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"H264")),
            capture.set.call_args_list,
        )
        capture.release.assert_not_called()

    def test_read_error_releases_capture_and_tries_next_format(self) -> None:
        for system, source in (("Linux", "/dev/video2"), ("Windows", 1)):
            with self.subTest(system=system):
                broken, working = Mock(), Mock()
                broken.read.side_effect = cv2.error("matrix.cpp: reshape")
                working.read.return_value = (
                    True, numpy.zeros((8, 12, 3), dtype=numpy.uint8)
                )
                with (
                    patch("ego_calibration.preview.platform.system", return_value=system),
                    patch.object(cv2, "VideoCapture", side_effect=[broken, working]),
                ):
                    result = _open_video_capture(cv2, source, EGO_STD_STEREO_RESOLUTIONS)

                self.assertIs(result, working)
                broken.release.assert_called_once()
                working.release.assert_not_called()

    def test_all_failed_formats_release_device_and_report_errors(self) -> None:
        captures = [Mock(), Mock()]
        for capture in captures:
            capture.read.side_effect = cv2.error("reshape")
        with (
            patch("ego_calibration.preview.platform.system", return_value="Linux"),
            patch.object(cv2, "VideoCapture", side_effect=captures),
            self.assertRaisesRegex(RuntimeError, "无法打开 UVC.*reshape"),
        ):
            _open_video_capture(cv2, "/dev/video2", ((3200, 1200),))
        for capture in captures:
            capture.release.assert_called_once()

    def test_compressed_byte_buffer_is_not_treated_as_an_image(self) -> None:
        capture = Mock()
        capture.read.return_value = (True, numpy.zeros((1, 4096), dtype=numpy.uint8))
        with patch("ego_calibration.preview.time.sleep"):
            self.assertIsNone(_read_first_frame(capture))

    def test_resolution_probe_uses_decoded_frame_and_releases_device(self) -> None:
        capture = Mock()
        capture.read.return_value = (
            True, numpy.zeros((1080, 3840, 3), dtype=numpy.uint8)
        )
        with patch("ego_calibration.capture._open_video_capture", return_value=capture):
            self.assertEqual(read_uvc_resolution("/dev/video2"), [3840, 1080])
        capture.release.assert_called_once()

    def test_resolution_probe_does_not_guess_video_index_from_usb_identifier(self) -> None:
        with (
            patch("ego_calibration.capture._open_video_capture") as open_capture,
            self.assertRaisesRegex(RuntimeError, "未提供可确认"),
        ):
            read_uvc_resolution("usb://1234:5678?interface=0")
        open_capture.assert_not_called()

    def test_numeric_identifier_is_opened_as_camera_index(self) -> None:
        sources = uvc_sources("2")

        self.assertEqual(sources[0][1], 2)
        self.assertNotIn(("视频设备 2", 2), sources[1:])

    def test_converts_bgr_frame_to_detached_qimage(self) -> None:
        frame = numpy.zeros((2, 3, 3), dtype=numpy.uint8)
        frame[0, 0] = [10, 20, 30]

        image = _frame_to_image(frame)
        frame[0, 0] = [0, 0, 0]
        color = image.pixelColor(0, 0)

        self.assertEqual((image.width(), image.height()), (3, 2))
        self.assertEqual((color.red(), color.green(), color.blue()), (30, 20, 10))


if __name__ == "__main__":
    unittest.main()

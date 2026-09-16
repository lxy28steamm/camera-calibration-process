from __future__ import annotations

import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication

from ego_calibration.app import MainWindow, _read_device_calibration
from ego_calibration.models import CameraDevice
from test_validation import _lite_payload, _std_kalibr_payload, _std_payload


class CalibrationValidationUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication(
            ["test", "-platform", "offscreen"]
        )

    def setUp(self) -> None:
        # 不启动设备扫描，只验证读取结果到界面呈现的流程。
        with patch("ego_calibration.app.QTimer.singleShot"):
            self.window = MainWindow()

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        self.application.processEvents()

    def _reprojection_results(self) -> list[str]:
        view = self.window.validation_view
        return [
            view.topLevelItem(index).text(2)
            for index in range(view.topLevelItemCount())
            if view.topLevelItem(index).text(0) == "二维重投影误差"
        ]

    def _summary_rows(self) -> dict[str, str]:
        view = self.window.summary
        return {
            view.topLevelItem(index).text(0): view.topLevelItem(index).text(1)
            for index in range(view.topLevelItemCount())
        }

    def test_resolution_summary_uses_measured_image_not_common_calibration(self) -> None:
        payload = _std_payload()
        payload["common_calibration"]["cam0"]["resolution"] = [1920, 1080]
        payload["uvc_preview"] = {"resolution": [3200, 1200]}

        self.window._calibration_loaded(("ego-std", payload))

        rows = self._summary_rows()
        self.assertEqual(rows["双目 UVC 输出"], "3200×1200")
        self.assertEqual(rows["单目画面分辨率"], "1600×1200（实测双目画面左右平分）")
        self.assertIn("未提供", rows["单目标定分辨率"])

    def test_kalibr_resolution_is_independent_of_video_resolution(self) -> None:
        payload = _std_kalibr_payload()
        for camera in ("cam0", "cam1"):
            payload["kalibr_calibration"][camera]["resolution"] = [1920, 1080]
        payload["uvc_preview"] = {"resolution": [3200, 1200]}

        self.window._calibration_loaded(("ego-std", payload))

        rows = self._summary_rows()
        self.assertEqual(rows["单目标定分辨率"], "1920×1080")
        self.assertEqual(rows["双目 UVC 输出"], "3200×1200")

    def test_preview_updates_only_matching_device_and_export(self) -> None:
        self.window._calibration_loaded(("ego-std", _std_payload()))
        self.window._calibration_identifier = "/dev/video2"
        self.window._preview_resolution_ready("/dev/video0", 640, 480)
        self.assertIn("未测得", self._summary_rows()["双目 UVC 输出"])

        self.window._preview_resolution_ready("/dev/video2", 3840, 1200)

        self.assertEqual(self._summary_rows()["双目 UVC 输出"], "3840×1200")
        self.assertEqual(self.window._calibration["uvc_preview"]["resolution"], [3840, 1200])
        self.assertIn('"uvc_preview"', self.window.json_view.toPlainText())

    def test_read_calibration_measures_video_and_preserves_calibration_on_video_error(self) -> None:
        device = CameraDevice("ego-std", "/dev/video2", "Std")
        for probe_result in ([3200, 1200], RuntimeError("相机忙")):
            with self.subTest(probe_result=probe_result):
                with (
                    patch("ego_calibration.app.ego_std.read_calibration", return_value=_std_payload()),
                    patch("ego_calibration.app.read_uvc_resolution") as probe,
                ):
                    if isinstance(probe_result, Exception):
                        probe.side_effect = probe_result
                    else:
                        probe.return_value = probe_result
                    kind, payload = _read_device_calibration(device)
                self.assertEqual(kind, "ego-std")
                self.assertIn("calibration", payload)
                if isinstance(probe_result, Exception):
                    self.assertEqual(payload["uvc_preview"]["error"], "相机忙")
                else:
                    self.assertEqual(payload["uvc_preview"]["resolution"], probe_result)

    def test_large_rms_is_display_only_in_summary_list_and_dialog(self) -> None:
        payload = _std_payload()
        payload["metrics"]["stereo_rms"] = 20.0
        payload["common_calibration"] = None

        self.window._calibration_loaded(("ego-std", payload))

        self.assertEqual(self._reprojection_results(), ["仅展示"] * 3)
        self.assertIn("基础检验通过", self.window.result_badge.text())
        self.assertIn("未判定误差是否合格", self.window.status.text())
        rows = {
            self.window.summary.topLevelItem(index).text(0):
            self.window.summary.topLevelItem(index).text(1)
            for index in range(self.window.summary.topLevelItemCount())
        }
        self.assertEqual(rows["双目内外参联合 RMS"], "20 px（仅展示，未判定合格）")
        with patch("ego_calibration.app.QMessageBox.information") as dialog:
            self.window._validate_current(True)
        self.assertIn("未判定误差是否合格", dialog.call_args.args[2])

    def test_missing_rms_does_not_display_success(self) -> None:
        for kind, payload, missing_count in (
            ("ego-lite", _lite_payload(), 4),
            ("ego-std", _std_kalibr_payload(), 3),
            ("ego-std-235", _std_kalibr_payload(), 3),
        ):
            with self.subTest(kind=kind):
                self.window._calibration_loaded((kind, payload))
                self.assertEqual(self._reprojection_results(), ["无法评估"] * missing_count)
                self.assertEqual(self.window.result_badge.objectName(), "warningBadge")
                self.assertIn("检验未完成", self.window.result_badge.text())
                self.assertIn("无法评估", self.window.status.text())

    def test_failure_and_missing_rms_are_rendered_before_display_only_values(self) -> None:
        payload = _std_payload()
        payload["metrics"]["left_calibrate_rms"] = -1.0
        del payload["metrics"]["right_calibrate_rms"]

        self.window._calibration_loaded(("ego-std", payload))

        self.assertEqual(self._reprojection_results(), ["失败", "无法评估", "仅展示"])
        self.assertEqual(self.window.result_badge.objectName(), "errorBadge")
        self.assertIn("检验失败", self.window.result_badge.text())


if __name__ == "__main__":
    unittest.main()

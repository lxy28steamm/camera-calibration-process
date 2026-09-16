from __future__ import annotations

import json
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from ego_calibration.dex_service import DexService, DexSettings, DexStream, child_environment
from ego_calibration.inspection_service import InspectionService
from ego_calibration.models import CalibrationError, CameraDevice


class DexIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.project = self.root / "camera_calibration"
        self.project.mkdir()
        self.service = DexService(self.root / "output", self.project)
        self.device = CameraDevice("dex-mono", "/dev/fake-dex", "Test Dex")

    def tearDown(self):
        self.service.close_preview()
        self.temp.cleanup()

    def test_board_physical_dimensions_and_models_are_preserved(self):
        settings = DexSettings.from_dict({})
        self.assertIn("tagSize: 0.055\n", settings.target_yaml())
        self.assertIn("tagSpacing: 0.3\n", settings.target_yaml())
        self.assertEqual(settings.model, "pinhole-equi")
        checker = DexSettings.from_dict({"board": "checkerboard", "columns": 9, "size_mm": 25})
        self.assertIn("targetCols: 9", checker.target_yaml())
        self.assertIn("rowSpacingMeters: 0.025", checker.target_yaml())
        for bad in ({"width": True}, {"sample_hz": float("nan")}, {"model": "fake"}, {"board": "fake"}):
            with self.assertRaises(CalibrationError):
                DexSettings.from_dict(bad)

    def test_ego_camera_cannot_be_opened_as_dex(self):
        with self.assertRaisesRegex(CalibrationError, "不会把 Ego"):
            self.service.capture(CameraDevice("ego-std", "/dev/video2", "Ego"), DexSettings())

    def test_file_access_rejects_escape_symlink_and_backend_code(self):
        path = self.project / "results/session/calibration/camera-camchain.yaml"
        path.parent.mkdir(parents=True)
        path.write_text("cam0: {}")
        self.assertEqual(self.service.file(self.service.file_id(path)), path)
        outside = self.root / "private.yaml"
        outside.write_text("private")
        (path.parent / "escape.yaml").symlink_to(outside)
        for identifier in ("project/results/../../private.yaml", "project/results/session/calibration/escape.yaml", "project/run.sh", "local/../private.yaml"):
            with self.assertRaises(CalibrationError):
                self.service.file(identifier)

    def test_existing_results_without_processing_manifest_are_listed(self):
        path = self.project / "results/recheck/calibration/camera-camchain.yaml"
        path.parent.mkdir(parents=True)
        path.write_text("cam0: {}")
        results = self.service.catalog()["results"]
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["status"], "待复核")
        self.assertEqual(results[0]["yaml_id"], self.service.file_id(path))

    def test_import_stays_in_managed_directory_without_overwrite(self):
        uploads = self.root / "uploads"
        uploads.mkdir()
        for _ in range(2):
            source = uploads / "video.mkv"
            source.write_bytes(b"synthetic video")
            output = self.service.import_video("video.mkv", uploads)
            self.assertEqual(output.read_bytes(), b"synthetic video")
            self.assertFalse(source.exists())
        self.assertEqual(len(list(self.service.directory.glob("imports/*/video.mkv"))), 2)
        with self.assertRaises(CalibrationError):
            self.service.import_video("../private.mkv", uploads)

    def prepare_review(self):
        self.sdk_patch = patch.object(self.service, "environment", return_value={"flash_sdk":True})
        self.sdk_patch.start()
        self.addCleanup(self.sdk_patch.stop)
        self.service.review = {"ready_to_write": True, "ready_to_read": True, "yaml_sha256": "yaml", "profile_sha256": "profile"}
        self.service.review_device = self.device.identifier
        self.service.review_token = "current-review"

    def test_flash_write_requires_explicit_confirmation_of_current_review_and_device(self):
        self.prepare_review()
        with patch.object(self.service, "run") as run:
            for data in ({}, {"confirmed": True}, {"confirmed": True, "review_token": "stale"}, {"confirmed": "true", "review_token": "current-review"}):
                with self.assertRaises(CalibrationError):
                    self.service.flash("write", data, self.device, threading.Event())
            with self.assertRaises(CalibrationError):
                self.service.flash("write", {"confirmed": True, "review_token": "current-review"}, CameraDevice("dex-mono", "/dev/other", "Other"), threading.Event())
            run.assert_not_called()

    def test_stop_does_not_interrupt_flash_transaction(self):
        service = InspectionService(self.root / "service", self.project)
        service.operation = "dex_flash_write"
        with self.assertRaisesRegex(CalibrationError, "等待完成"):
            service.dispatch("stop", {})
        self.assertFalse(service.stop_event.is_set())

    def test_failed_flash_operation_cannot_reuse_previous_success(self):
        self.prepare_review()
        for name in ("run.sh", "video_pipeline.py", "kalibr_runner.py", "calibration_flash.py"):
            (self.project / name).touch()
        self.service.profile_path = self.project / "profile.json"
        self.service.flash_result = {"status": "written_and_verified"}
        with patch.object(self.service, "run", side_effect=CalibrationError("simulated crash")):
            with self.assertRaises(CalibrationError):
                self.service.flash("read", {}, self.device, threading.Event())
        self.assertNotEqual(self.service.flash_result["status"], "written_and_verified")
        self.assertTrue(self.service.flash_result["files"])

    def test_process_cancellation_preserves_log_and_ends_worker(self):
        stop = threading.Event()
        timer = threading.Timer(.25, stop.set)
        timer.start()
        log = self.root / "worker.log"
        try:
            with self.assertRaisesRegex(CalibrationError, "已停止"):
                self.service.run([sys.executable, "-u", "-c", "import time; print('worker started'); time.sleep(30)"], log, stop)
        finally:
            timer.cancel()
        self.assertIn("worker started", log.read_text())

    def test_frozen_library_paths_do_not_leak_to_ffmpeg_or_kalibr(self):
        with patch.object(sys, "frozen", True, create=True), patch.dict("os.environ", {"LD_LIBRARY_PATH": "/tmp/_MEI/libs", "LD_LIBRARY_PATH_ORIG": "/system/libs", "PYTHONPATH": "/tmp/_MEI/python"}):
            env = child_environment()
        self.assertEqual(env["LD_LIBRARY_PATH"], "/system/libs")
        self.assertNotIn("PYTHONPATH", env)

    @unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "FFmpeg integration requires system tools")
    def test_ffmpeg_record_preview_and_actual_dimensions(self):
        class SyntheticPipeline:
            @staticmethod
            def ffmpeg_command(_device, width, height, fps, preview_height, video, seconds):
                return ["-hide_banner", "-loglevel", "error", "-n", "-re", "-f", "lavfi", "-i", f"testsrc2=size={width}x{height}:rate={fps}", "-map", "0:v:0", "-c:v", "mjpeg", "-threads:v", "1", "-t", str(seconds), str(video), "-map", "0:v:0", "-vf", f"fps=8,scale=960:{preview_height}", "-pix_fmt", "rgb24", "-c:v", "rawvideo", "-threads:v", "1", "-t", str(seconds), "-f", "rawvideo", "pipe:1"]
        directory = self.service.new_directory("data")
        stream = DexStream(SyntheticPipeline, self.device, DexSettings(width=320, height=240, fps=10, duration_s=1), directory, recording=True)
        stream.thread.join(20)
        self.assertFalse(stream.thread.is_alive())
        self.assertFalse(stream.state()["error"], stream.state())
        self.assertGreater(stream.state()["preview_frames"], 0)
        self.assertEqual(stream.state()["resolution"], [320, 240])
        self.assertGreater(len(stream.preview), 1000)
        metadata = json.loads((directory / "capture.json").read_text())
        self.assertEqual(metadata["status"], "complete")
        self.assertEqual(metadata["video_info"]["streams"][0]["codec_name"], "mjpeg")
        manual = DexStream(SyntheticPipeline, self.device, DexSettings(width=320, height=240, fps=10, duration_s=10), self.service.new_directory("data"), recording=True)
        deadline = time.monotonic() + 5
        while manual.state()["preview_frames"] < 2 and time.monotonic() < deadline:
            time.sleep(.05)
        manual.close()
        self.assertFalse(manual.state()["running"])
        self.assertFalse(manual.state()["error"], manual.state())
        self.assertEqual(manual.metadata["status"], "complete")
        self.assertLess(float(manual.metadata["video_info"]["format"]["duration"]), 10)


if __name__ == "__main__":
    unittest.main()

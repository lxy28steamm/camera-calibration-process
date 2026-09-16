from __future__ import annotations

import http.client
import json
import tempfile
import threading
import unittest
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from ego_calibration.inspection_service import InspectionService, jpeg
from ego_calibration.models import CameraDevice
from ego_calibration.webapp import CameraWebServer
from test_inspection import inspection_payload, inspection_settings, observations


def rendered_pair(settings, rotation, depth):
    detections = observations(settings, rotation=rotation, translation=(-0.2, -0.2, depth))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    images = []
    for markers in detections:
        image = np.full((800, 1280), 255, np.uint8)
        for tag, corners in markers.items():
            marker = cv2.aruco.generateImageMarker(dictionary, tag, 160)
            transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [159, 0], [159, 159], [0, 159]]), corners.astype(np.float32))
            warped = cv2.warpPerspective(marker, transform, (1280, 800), borderValue=255)
            image = np.minimum(image, warped)
        images.append(image)
    return cv2.cvtColor(np.concatenate(images, axis=1), cv2.COLOR_GRAY2BGR)


class WebInspectionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.service = InspectionService(self.root)
        self.server = CameraWebServer(("127.0.0.1", 0), self.service)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.service.close()
        self.server.server_close()
        self.thread.join()
        self.temporary.cleanup()

    def request(self, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=15)
        connection.request("POST" if body is not None else "GET", path, body, headers or {})
        response = connection.getresponse()
        status, content = response.status, response.read()
        connection.close()
        return status, content

    def action(self, action, data):
        status, content = self.request("/api/action", json.dumps({"action": action, "data": data}), {"X-Camera-Token": self.server.token})
        self.assertEqual(status, 202, content)
        self.service.job.join(20)
        self.assertFalse(self.service.job.is_alive())
        self.assertFalse(self.service.error, self.service.error)

    def test_full_video_upload_detection_export_and_lossless_replay(self):
        settings = inspection_settings(duration_s=5)
        video = self.root / "synthetic.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10, (2560, 800))
        self.assertTrue(writer.isOpened())
        for rotation, depth in (((0, 0, 0), 1), ((0.6, 0.1, 0.1), 1.3), ((-0.3, -0.4, 0.2), 0.9)):
            frame = rendered_pair(settings, rotation, depth)
            for _ in range(10):
                writer.write(frame)
        writer.release()
        self.action("load_calibration", {"kind": "ego-std", "payload": inspection_payload()})
        status, body = self.request("/api/upload?name=synthetic.avi", video.read_bytes(), {"X-Camera-Token": self.server.token})
        self.assertEqual(status, 200, body)
        self.action("video", {"upload": json.loads(body)["upload"], "settings": asdict(settings)})
        report = self.service.report
        self.assertEqual(report["status"], "pass", report["checks"])
        self.assertEqual(report["counts"]["accepted"], 3)
        self.assertLess(report["metrics"]["left_to_right"]["rms"], 1.0)
        original = self.service.session_id
        for filename in ("report.html", "report.json", "samples.csv", "cam0/00000.png"):
            self.assertEqual(self.request(f"/reports/{original}/{filename}")[0], 200)
        self.assertEqual(self.request("/api/frame.jpg")[0], 200)
        self.assertEqual(self.request("/api/sample.jpg")[0], 200)
        self.action("replay", {"session": original})
        self.assertNotEqual(self.service.session_id, original)
        self.assertEqual(self.service.report["metrics"], report["metrics"])
        self.assertEqual(len(self.service.records()), 2)
        saved = json.loads((self.root / original / "report.json").read_text())
        self.assertEqual(saved["source"]["mode"], "video")
        self.assertEqual(len(saved["samples"][0]["observed_corners"]), 2)

    def test_cross_site_mutations_and_report_path_escape_are_blocked(self):
        body = json.dumps({"action": "stop"})
        self.assertEqual(self.request("/api/action", body)[0], 403)
        headers = {"X-Camera-Token": self.server.token, "Origin": "https://example.com"}
        self.assertEqual(self.request("/api/action", body, headers)[0], 403)
        self.assertEqual(self.request("/api/state", headers={"Host": "example.com"})[0], 403)
        self.assertEqual(self.request("/reports/../../outside.json")[0], 404)
        self.assertEqual(self.request("/api/action", '{"action":[]}', {"X-Camera-Token": self.server.token})[0], 400)

    def test_partial_video_failure_still_generates_incomplete_report(self):
        self.service.payload = inspection_payload()
        self.service.selected = CameraDevice("ego-std", "offline", "Synthetic")
        (self.root / "uploads").mkdir()
        (self.root / "uploads" / "invalid.avi").write_bytes(b"invalid video")
        self.service.dispatch("video", {"upload": "invalid.avi", "settings": asdict(inspection_settings())})
        self.service.job.join(10)
        self.assertTrue(self.service.error)
        self.assertEqual(self.service.report["status"], "incomplete")
        self.assertEqual(self.service.report["counts"]["accepted"], 0)
        self.assertTrue((self.root / self.service.session_id / "report.json").is_file())

    def test_packaged_assets_have_injected_request_token(self):
        status, page = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(self.server.token.encode(), page)
        self.assertNotIn(b"__CAMERA_TOKEN__", page)
        for path in ("/app.js", "/preview.js", "/dex.js", "/workflows.js", "/style.css", "/dex.css", "/favicon.svg", "/api/state", "/api/records", "/api/dex/catalog", "/api/lite/catalog", "/api/std/catalog"):
            self.assertEqual(self.request(path)[0], 200)

    def test_camera_imu_downloads_and_h264_upload_remain_separate(self):
        for name in ('lite', 'std'):
            service = getattr(self.service, name)
            directory = service.directory/'job'
            directory.mkdir(parents=True)
            (directory/'result.yaml').write_text(name)
            self.assertEqual(self.request(f'/{name}-files/job/result.yaml'), (200, name.encode()))
            self.assertEqual(self.request(f'/{name}-files/../secret.yaml')[0], 400)
        self.assertEqual(self.request('/api/lite/frame.jpg')[0], 204)
        self.service.lite.preview = b'jpeg-preview'
        self.assertEqual(self.request('/api/lite/frame.jpg'), (200, b'jpeg-preview'))
        headers = {'X-Camera-Token': self.server.token}
        self.assertEqual(self.request('/api/upload?purpose=std&name=test.mp4', b'invalid', headers)[0], 400)
        status, body = self.request('/api/upload?purpose=std&name=test.h264', b'raw-h264', headers)
        self.assertEqual(status, 200, body)
        uploaded = self.root/'uploads'/json.loads(body)['upload']
        self.assertEqual(uploaded.read_bytes(), b'raw-h264')

    def test_live_stream_keeps_updating_while_inspection_sample_stays_unchanged(self):
        first = jpeg(np.zeros((20, 40, 3), np.uint8))
        second = jpeg(np.full((20, 40, 3), 127, np.uint8))
        overlay = jpeg(np.full((20, 40, 3), 255, np.uint8))
        self.service.live = SimpleNamespace(lock=threading.Lock(), preview=first, close=lambda: None,
                                            state=lambda: {"running": True, "fps": 30})
        self.service.operation = "inspect"
        self.service.report = {"samples": [{"index": 0}]}
        self.service.annotated = self.service.sample_preview = overlay
        self.assertEqual(self.request("/api/sample.jpg"), (200, overlay))
        self.assertTrue(self.service.state()["sample_available"])
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        try:
            connection.request("GET", "/api/stream.mjpg")
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn("multipart/x-mixed-replace", response.getheader("Content-Type"))

            def read_frame():
                self.assertEqual(response.readline(), b"--frame\r\n")
                headers = {}
                while (line := response.readline()) != b"\r\n":
                    key, value = line.decode().split(":", 1)
                    headers[key] = value.strip()
                frame = response.read(int(headers["Content-Length"]))
                self.assertEqual(response.read(2), b"\r\n")
                return frame

            self.assertEqual(read_frame(), first)
            self.service.live.preview = second
            self.assertEqual(read_frame(), second)
            self.assertEqual(self.request("/api/sample.jpg"), (200, overlay))
            response.close()
        finally:
            connection.close()
        self.assertEqual(self.request("/api/stream.mjpg", headers={"Origin": "https://example.com"})[0], 403)
        # Starting an ordinary preview or health check must not replace the
        # saved inspection image with an unannotated camera frame.
        self.service.annotated = second
        self.assertEqual(self.request("/api/sample.jpg"), (200, overlay))
        self.service.report = None
        self.assertEqual(self.request("/api/sample.jpg")[0], 204)
        self.assertFalse(self.service.state()["sample_available"])

    def test_dex_result_downloads_are_confined_to_data_files(self):
        result = self.service.dex.new_directory("results") / "camera-camchain.yaml"
        result.write_text("cam0: {}\n")
        status, body = self.request("/dex-files/" + self.service.dex.file_id(result))
        self.assertEqual(status, 200)
        self.assertEqual(body, b"cam0: {}\n")
        self.assertEqual(self.request("/dex-files/local/../../outside.yaml")[0], 400)
        self.assertEqual(self.request("/dex-files/project/calibration_flash.py")[0], 400)


if __name__ == "__main__":
    unittest.main()

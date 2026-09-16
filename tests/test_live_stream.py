import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from ego_calibration.inspection_service import LiveCamera, jpeg
from ego_calibration.models import CameraDevice


class LiveStreamTest(unittest.TestCase):
    def test_slow_preview_encoding_does_not_block_original_frame_capture(self):
        encoding, release_encoder, captured = threading.Event(), threading.Event(), threading.Event()

        class Capture:
            count = 0
            released = False

            def read(self):
                time.sleep(.002)
                self.count += 1
                if self.count >= 7:
                    captured.set()
                return True, np.full((40, 160, 3), self.count % 255, np.uint8)

            def release(self):
                self.released = True

        capture = Capture()

        def slow_jpeg(frame):
            encoding.set()
            release_encoder.wait(3)
            return jpeg(frame)

        with patch('ego_calibration.inspection_service._open_video_capture', return_value=capture), patch('ego_calibration.inspection_service.jpeg', side_effect=slow_jpeg):
            camera = LiveCamera(CameraDevice('ego-std', '/dev/test', 'Test camera'))
            try:
                self.assertTrue(encoding.wait(2))
                self.assertTrue(captured.wait(2), 'Capture stalled behind the preview encoder')
                sequence, _, left, right = camera.snapshot()
                self.assertGreaterEqual(sequence, 6)
                self.assertEqual(left.shape, (40, 80, 3))
                np.testing.assert_array_equal(left, right)
            finally:
                release_encoder.set()
                camera.close()
            self.assertTrue(capture.released)
            self.assertFalse(camera.thread.is_alive())
            self.assertFalse(camera.preview_thread.is_alive())


if __name__ == '__main__':
    unittest.main()

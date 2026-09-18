from __future__ import annotations

import errno
import io
import signal
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import Mock, patch

from ego_calibration.models import CalibrationError
from ego_calibration.inspection_service import InspectionService
from ego_calibration.webapp import main


class WebLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.service = Mock(directory=Path('/test-data'))
        self.server = Mock(server_port=8765)
        self.server.serve_forever.side_effect = KeyboardInterrupt
        self.output = io.StringIO()
        self.addCleanup(patch.stopall)
        self.services = patch('ego_calibration.webapp.InspectionService', return_value=self.service).start()
        self.servers = patch('ego_calibration.webapp.CameraWebServer', return_value=self.server).start()
        self.browser = patch('ego_calibration.webapp.webbrowser.open').start()
        self.existing = patch('ego_calibration.webapp.existing_workbench', return_value=True).start()

    def run_main(self, *arguments):
        with patch('sys.argv', ['camera-web', '--host', '127.0.0.1', *arguments]), redirect_stdout(self.output):
            return main()

    def test_ctrl_c_closes_listener_and_camera_and_restores_signal_handler(self):
        previous = signal.getsignal(signal.SIGINT)
        self.assertEqual(self.run_main('--foreground', '--no-browser'), 0)
        self.server.server_close.assert_called_once()
        self.service.close.assert_called_once()
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.browser.assert_not_called()
        self.assertIn('网页服务已停止', self.output.getvalue())

    def test_foreground_refuses_to_attach_to_old_background_process(self):
        self.servers.side_effect = OSError(errno.EADDRINUSE, 'occupied')
        self.assertEqual(self.run_main('--foreground', '--no-browser'), 2)
        self.existing.assert_not_called()
        self.browser.assert_not_called()
        self.service.close.assert_called_once()
        self.assertIn('此终端没有启动新服务', self.output.getvalue())

    def test_bind_failure_still_cleans_up_service(self):
        self.servers.side_effect = PermissionError(errno.EPERM, 'denied')
        with self.assertRaises(CalibrationError):
            self.run_main('--foreground', '--no-browser')
        self.service.close.assert_called_once()

    def test_shutdown_cancels_browser_timer(self):
        timer = Mock()
        with patch('ego_calibration.webapp.threading.Timer', return_value=timer):
            self.assertEqual(self.run_main('--foreground'), 0)
        timer.start.assert_called_once()
        timer.cancel.assert_called_once()

    def test_listener_is_closed_even_if_camera_cleanup_fails(self):
        previous = signal.getsignal(signal.SIGINT)
        self.service.close.side_effect = RuntimeError('camera failure')
        with self.assertRaisesRegex(RuntimeError, 'camera failure'):
            self.run_main('--foreground', '--no-browser')
        self.server.server_close.assert_called_once()
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)

    def test_legacy_duplicate_launch_still_opens_existing_page(self):
        self.servers.side_effect = OSError(errno.EADDRINUSE, 'occupied')
        self.assertEqual(self.run_main(), 0)
        self.browser.assert_called_once_with('http://127.0.0.1:8765')
        self.service.close.assert_called_once()

    def test_closing_service_rejects_requests_already_accepted_by_http_threads(self):
        with tempfile.TemporaryDirectory() as temporary:
            service = InspectionService(Path(temporary))
            service.close()
            with self.assertRaisesRegex(CalibrationError, '正在停止'):
                service.dispatch('scan', {})
            self.assertIsNone(service.job)


if __name__ == '__main__':
    unittest.main()

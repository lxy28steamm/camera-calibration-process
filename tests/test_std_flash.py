"""Full vendor transaction on a simulated XU; never opens physical hardware."""
import copy
import http.client
import json
import os
import struct
import tempfile
import threading
import unittest
import zlib
import stat
from types import SimpleNamespace
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import yaml

from ego_calibration import std_flash as flash
from ego_calibration.inspection_service import InspectionService
from ego_calibration.models import CalibrationError, CameraDevice
from ego_calibration.std_service import StdService
from ego_calibration.webapp import CameraWebServer
from test_ego_std import _calibration_blob


def result_yaml():
    identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    right = copy.deepcopy(identity)
    right[0][3] = -.06
    camera = dict(camera_model='pinhole', intrinsics=[900, 901, 960, 540],
                  distortion_model='radtan', distortion_coeffs=[0, 0, 0, 0],
                  resolution=[1920, 1080], timeshift_cam_imu=.0004)
    return yaml.safe_dump({'cam0': {**copy.deepcopy(camera), 'T_cam_imu': identity, 'rostopic': '/cam0/image_raw'},
                           'cam1': {**copy.deepcopy(camera), 'T_cam_imu': right, 'T_cn_cnm1': copy.deepcopy(right), 'rostopic': '/cam1/image_raw'}}, sort_keys=False)


class FakeXu:
    name = 'simulated-yctc'

    def __init__(self):
        self.identity = {'usb_serial': 'USB-TEST', 'vid': '1234', 'pid': '5678'}
        self.blob = _calibration_blob()
        self.state = self.session = self.flags = 0
        self.received = b''
        self.commands = []
        self.fail_stage = None
        self.on_unlock = lambda: None

    def query(self, unit, selector, request, size, payload=None):
        assert unit == 10
        if selector == 2 and request == 0x81:
            return struct.pack('<HHIIB3x', struct.unpack_from('<H', self.blob, 4)[0], 7,
                               zlib.crc32(self.blob), len(self.blob), 0)
        if selector == 5 and request == 0x81:
            return struct.pack('<BBHI', self.state, 0, 0, len(self.received))
        if selector == 4 and request == 0x81:
            return struct.pack('<BBHIII', self.flags, self.state, 1, 42, 30000, self.session)
        if selector == 4 and request == 1:
            command, reserved, schema, session, arg0, arg1 = struct.unpack('<BBHIII', payload)
            if command == 5:
                self.pending = session, arg0, arg1
                return b''
            self.commands.append(command)
            if self.fail_stage == command:
                raise OSError('simulated transport rejection')
            if command == 6:
                self.on_unlock()
                info = struct.unpack_from('<H', self.blob, 4)[0]
                expected = zlib.crc32(b'test-only-secret' + struct.pack('<IIIIH', 42, session, zlib.crc32(self.blob), len(self.blob), info))
                assert arg1 == expected and arg0 == 42
                self.flags, self.session = 3, session
            elif command == 1:
                assert session == self.session and schema == 2
                self.state, self.received, self.expected = 1, b'', (arg0, arg1)
            elif command == 2:
                assert self.expected == (len(self.received), zlib.crc32(self.received))
                self.state = 2
            elif command == 3:
                self.blob, self.received, self.state, self.flags = self.received, b'', 0, 0
                if self.fail_stage == 'after-activate':
                    raise OSError('activation acknowledgement lost')
            elif command == 4:
                self.state, self.received, self.flags = 0, b'', 0
            else:
                raise AssertionError(command)
            return b''
        if selector == 3 and request == 1:
            session, offset, length, crc = struct.unpack('<IIHH', payload[:12])
            data = payload[12:12+length]
            assert session == self.session and offset == len(self.received)
            assert 0 < length <= 48 and flash.writer.base.crc16_modbus(data) == crc
            self.received += data
            return b''
        if selector == 3 and request == 0x81:
            session, offset, length = self.pending
            chunk = self.blob[offset:offset+length]
            crc = flash.writer.base.crc16_modbus(chunk)
            if self.fail_stage == 'readback' and 3 in self.commands:
                crc ^= 1
            return struct.pack('<IIHH', session, offset, length, crc) + chunk + bytes(48-length)
        raise AssertionError((selector, request, size))


class StdFlashTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.service = StdService(self.root)
        self.stop = threading.Event()
        self.device = CameraDevice('ego-std', '/dev/video-test', '模拟 Ego-Std', serial='USB-TEST')
        self.backend = FakeXu()
        @contextmanager
        def connection(device):
            flash.require_device(device)
            yield self.backend
        self.patch = patch.object(flash, 'connection', connection)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        environment = patch.dict(os.environ, {'YCTC_XU_UNLOCK_SECRET': 'test-only-secret', 'YCTC_XU_UNLOCK_SECRET_FILE': ''})
        environment.start()
        self.addCleanup(environment.stop)

    def review(self, **values):
        self.service.inspect_result({'yaml_text': result_yaml(), **values}, self.device, self.stop)
        return {'confirmed': True, 'review_token': self.service.review_token}

    def test_review_reads_and_durably_backs_up_without_write_commands(self):
        self.review()
        self.assertEqual(self.backend.commands, [])
        self.assertEqual(self.service.review['serial_number'], 'YCTC-TEST-01')
        self.assertEqual(self.service.review['new']['resolution'], [1920, 1080])
        backup = self.service.review_path.with_name('before.bin')
        self.assertEqual(backup.read_bytes(), self.backend.blob)
        self.assertTrue(any(f['name'] == 'before.bin' for f in self.service.review['files']))

    def test_full_write_preserves_yaml_and_serial_and_verifies_exact_readback(self):
        values = self.review()
        original = self.backend.blob
        def before_unlock():
            directory = self.service.file(self.service.result['id'], directory=True)
            self.assertEqual((directory/'before.bin').read_bytes(), original)
            self.assertTrue((directory/'before.json').is_file())
        self.backend.on_unlock = before_unlock
        payload = self.service.flash(values, self.device, self.stop)
        self.assertEqual(self.backend.commands, [6, 1, 2, 3])
        self.assertEqual(payload['kalibr_yaml'], result_yaml())
        self.assertEqual(payload['header']['serial_number'], 'YCTC-TEST-01')
        self.assertEqual(self.service.result['status'], 'complete')
        self.assertTrue(self.service.result['verified'])
        self.assertEqual(self.service.review_token, '')
        for path in self.service.directory.rglob('*'):
            if path.is_file():
                self.assertNotIn(b'test-only-secret', path.read_bytes())
        self.assertNotIn('test-only-secret', json.dumps(self.service.state()))
        with self.assertRaises(CalibrationError):
            self.service.flash(values, self.device, self.stop)

    def test_before_write_guards(self):
        for mode in ('confirmation', 'token', 'device', 'yaml', 'current', 'identity', 'credential'):
            with self.subTest(mode=mode):
                self.backend = FakeXu()
                values = self.review()
                device = self.device
                if mode == 'confirmation': values['confirmed'] = False
                if mode == 'token': values['review_token'] = 'wrong'
                if mode == 'device': device = replace(device, serial='OTHER')
                if mode == 'yaml': self.service.review_path.write_text(result_yaml()+'\n# changed')
                if mode == 'current': self.backend.blob = flash.writer.build_kalibr_yaml_blob(result_yaml().encode(), 'OTHER')
                if mode == 'identity': self.backend.identity['pid'] = 'OTHER'
                with patch.dict(os.environ, {'YCTC_XU_UNLOCK_SECRET': '' if mode == 'credential' else 'test-only-secret'}):
                    with self.assertRaises(CalibrationError):
                        self.service.flash(values, device, self.stop)
                self.assertEqual(self.backend.commands, [])

    def test_invalid_geometry_and_ambiguous_yaml_never_access_device(self):
        for mode in ('missing_imu', 'missing_shift', 'nan', 'rotation', 'chain', 'dimensions', 'zero_baseline', 'duplicate', 'size', 'only_camera'):
            with self.subTest(mode=mode):
                chain = yaml.safe_load(result_yaml())
                if mode == 'missing_imu': del chain['cam0']['T_cam_imu']
                if mode == 'missing_shift': del chain['cam1']['timeshift_cam_imu']
                if mode == 'nan': chain['cam0']['intrinsics'][0] = float('nan')
                if mode == 'rotation': chain['cam1']['T_cam_imu'][0][0] = 0
                if mode == 'chain': chain['cam1']['T_cam_imu'][0][3] = -.07
                if mode == 'dimensions': chain['cam1']['resolution'] = [960, 540]
                if mode == 'zero_baseline':
                    for key in ('T_cam_imu', 'T_cn_cnm1'): chain['cam1'][key][0][3] = 0
                if mode == 'only_camera': del chain['cam1']
                text = yaml.safe_dump(chain)
                if mode == 'duplicate': text += '\ncam0: {}\n'
                if mode == 'size': text += '\n#' + 'a' * 8192
                with patch.object(flash, 'connection') as connect:
                    with self.assertRaises(CalibrationError): self.review(yaml_text=text)
                    connect.assert_not_called()
                self.assertFalse(self.service.review_token)

    def test_failed_backup_never_unlocks(self):
        values = self.review()
        with patch.object(flash, 'backup', side_effect=OSError('disk full')):
            with self.assertRaises(OSError): self.service.flash(values, self.device, self.stop)
        self.assertEqual(self.backend.commands, [])
        self.assertEqual(self.service.result['status'], 'failed')

    def test_transfer_commit_and_readback_failures_keep_backup_and_do_not_claim_success(self):
        for failure in (1, 2, 'after-activate', 'readback'):
            with self.subTest(failure=failure):
                self.backend = FakeXu()
                values = self.review()
                self.backend.fail_stage = failure
                original = self.backend.blob
                with self.assertRaisesRegex(CalibrationError, '未确认成功'):
                    self.service.flash(values, self.device, self.stop)
                self.assertEqual(self.service.result['status'], 'failed')
                self.assertFalse(self.service.result.get('verified'))
                directory = self.service.file(self.service.result['id'], directory=True)
                self.assertEqual((directory/'before.bin').read_bytes(), original)
                if failure in (2, 'after-activate'): self.assertIn(4, self.backend.commands)
                if failure == 'readback': self.assertNotIn(4, self.backend.commands)

    def test_wrong_camera_or_serial_rejected(self):
        for kind in ('dex-mono', 'ego-lite', 'uvc'):
            with self.assertRaises(CalibrationError):
                self.service.inspect_result({'yaml_text': result_yaml()}, replace(self.device, kind=kind), self.stop)
        for serial in ('with space', '中', 'a'*33):
            with self.assertRaises(Exception): self.review(serial_number=serial)
        self.assertEqual(self.backend.commands, [])

    def test_read_schema_v2_backup_preserves_yaml(self):
        self.backend.blob = flash.writer.build_kalibr_yaml_blob(result_yaml().encode(), 'OLD-V2')
        self.service.read_device({}, self.device, self.stop)
        directory = self.service.file(self.service.result['id'], directory=True)
        self.assertEqual((directory/'before.yaml').read_text(), result_yaml())
        self.assertEqual(self.backend.commands, [])

    def test_linux_transport_pins_and_closes_one_descriptor(self):
        # Exercise the real connection adapter without opening a device node.
        self.patch.stop()
        metadata = SimpleNamespace(st_mode=stat.S_IFCHR, st_dev=1, st_ino=2, st_rdev=3)
        with patch.object(Path, 'resolve', return_value=Path('/dev/video-test')), \
             patch.object(Path, 'stat', return_value=metadata), \
             patch.object(flash, 'linux_identity', return_value=self.backend.identity), \
             patch('os.open', return_value=77) as opened, patch('os.fstat', return_value=metadata), \
             patch('os.close') as closed, patch('fcntl.ioctl') as ioctl:
            with flash.connection(self.device) as backend:
                backend.query(10, 2, 0x81, 16)
                backend.query(10, 4, 1, 16, bytes(16))
            self.assertEqual(opened.call_count, 1)
            self.assertEqual([call.args[0] for call in ioctl.call_args_list], [77, 77])
            closed.assert_called_once_with(77)

    def test_linux_transport_rejects_replaced_device_before_any_query(self):
        self.patch.stop()
        metadata = SimpleNamespace(st_mode=stat.S_IFCHR, st_dev=1, st_ino=2, st_rdev=3)
        changed = {**self.backend.identity, 'usb_serial': 'OTHER'}
        with patch.object(Path, 'resolve', return_value=Path('/dev/video-test')), \
             patch.object(Path, 'stat', return_value=metadata), \
             patch.object(flash, 'linux_identity', side_effect=[self.backend.identity, changed]), \
             patch('os.open', return_value=77), patch('os.fstat', return_value=metadata), \
             patch('os.close') as closed, patch('fcntl.ioctl') as ioctl:
            with self.assertRaises(CalibrationError):
                with flash.connection(self.device): pass
            ioctl.assert_not_called()
            closed.assert_called_once_with(77)

    def test_http_routes_download_backup_and_scheduler_guards(self):
        service = InspectionService(self.root)
        service.std = self.service
        service.devices, service.selected = [self.device], self.device
        server = CameraWebServer(('127.0.0.1', 0), service)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            def request(path, data=None):
                client = http.client.HTTPConnection('127.0.0.1', server.server_port)
                client.request('POST' if data else 'GET', path, json.dumps(data) if data else None, {'X-Camera-Token': server.token})
                response = client.getresponse()
                result = response.status, response.read()
                client.close()
                return result
            for action, data in [('std_inspect_result', {'yaml_text': result_yaml()}),
                                 ('std_flash_write', None)]:
                if data is None: data = {'confirmed': True, 'review_token': self.service.review_token}
                status, body = request('/api/action', {'action': action, 'data': data})
                self.assertEqual(status, 202, body)
                service.job.join(10)
                self.assertFalse(service.error, service.error)
            self.assertEqual(service.payload['kalibr_yaml'], result_yaml())
            backup = next(f for f in self.service.result['files'] if f['name'] == 'before.bin')
            status, content = request('/std-files/'+backup['id'])
            self.assertEqual(status, 200)
            self.assertEqual(content, _calibration_blob())
            self.assertNotIn(b'test-only-secret', request('/api/state')[1])
            service.operation = 'std_flash_write'
            with self.assertRaises(CalibrationError): service.dispatch('stop', {})
            self.assertFalse(service.stop_event.is_set())
            service.operation = ''
            self.review()
            service._select({'identifier': self.device.identifier})
            self.assertFalse(self.service.review_token)
        finally:
            service.operation = ''
            server.shutdown()
            service.close()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()

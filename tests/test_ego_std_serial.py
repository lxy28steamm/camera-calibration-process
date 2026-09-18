from __future__ import annotations

import struct
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from ego_calibration.ego_std import _crc16_modbus
from ego_calibration.ego_std_serial import _reply, _request, camera_port, read_camera_serial
from ego_calibration.inspection_service import InspectionService
from ego_calibration.models import CalibrationError, CameraDevice


def frame(payload, *, kind=0x0D, seq=7, flags=1, reserved=0):
    data = struct.pack('<HBBBHHB', 0x5953, 3, kind, flags, seq, len(payload), reserved) + payload
    return data + struct.pack('<H', _crc16_modbus(data))


class SerialProtocolTest(unittest.TestCase):
    def test_only_sn_request_is_sent_with_nonzero_sequence_and_crc(self):
        data = _request(7)
        # Generated independently by the supplied SDK's C frame encoder.
        self.assertEqual(data, bytes.fromhex('53 59 03 0d 00 07 00 00 00 00 3c 0f'))
        self.assertEqual(_crc16_modbus(data), 0)

    def test_decodes_reply_generated_by_sdk_c_encoder(self):
        reply = bytes.fromhex('5359030d0107000800000600534e30373337d94f')
        self.assertEqual(_reply(bytearray(reply), 7), 'SN0737')

    def test_fragmented_reply_ignores_other_sequences_streams_and_bad_frames(self):
        payload = b'\x06\x00SN0737'
        corrupt = bytearray(frame(payload))
        corrupt[-1] ^= 1
        buffer = bytearray(b'noise' + frame(payload, seq=6) + frame(b'imu', kind=4, seq=0, flags=0)
                           + corrupt + frame(payload, reserved=1) + frame(payload)[:9])
        self.assertIsNone(_reply(buffer, 7))
        buffer.extend(frame(payload)[9:])
        self.assertEqual(_reply(buffer, 7), 'SN0737')
        self.assertEqual(buffer, b'')

    def test_rejects_invalid_sn_lengths_and_non_ascii(self):
        for payload in (b'\0\0', b'\x21\0'+b'A'*33, b'\x06\0short', b'\x01\0\xff', b'\x01\0\0'):
            with self.subTest(payload=payload):
                self.assertIsNone(_reply(bytearray(frame(payload)), 7))

    def test_device_error_reports_failure_without_usb_fallback(self):
        error = frame(struct.pack('<iHH', -1, 0x0D, 6), kind=0x0A)
        with self.assertRaisesRegex(CalibrationError, 'detail=0x0006'):
            _reply(bytearray(error), 7)

    def test_cdc_port_matches_usb_parent_not_shared_usb_serial(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sys, dev = root/'sys', root/'dev'
            dev.mkdir()
            (dev/'video2').touch()
            video = sys/'class/video4linux/video2'
            video.mkdir(parents=True)
            for index in range(2):
                usb = sys/'devices'/f'usb{index}'
                usb.mkdir(parents=True)
                (usb/'idVendor').write_text('1d6b')
                (usb/'idProduct').write_text('0004')
                (usb/'serial').write_text('same-usb-serial')
                tty = sys/'class/tty'/f'ttyACM{index}'
                tty.mkdir(parents=True)
                (tty/'device').symlink_to(usb)
                if index == 1:
                    (video/'device').symlink_to(usb)
            self.assertEqual(camera_port(str(dev/'video2'), sys_root=sys, dev_root=dev), dev/'ttyACM1')
            (sys/'class/tty/ttyACM1/device').unlink()
            with self.assertRaises(CalibrationError):
                camera_port(str(dev/'video2'), sys_root=sys, dev_root=dev)

    def test_transport_restores_settings_and_closes_on_timeout(self):
        with patch('ego_calibration.ego_std_serial.camera_port', return_value=Path('/dev/ttyACM1')), \
             patch('ego_calibration.ego_std_serial.platform.system', return_value='Linux'), \
             patch('os.open', return_value=9), patch('os.close') as close, \
             patch('fcntl.flock'), patch('tty.setraw'), \
             patch('termios.tcgetattr', side_effect=lambda _: [0, 0, 0, 0, 0, 0, []]), \
             patch('termios.tcsetattr') as restore, \
             patch('ego_calibration.ego_std_serial.time.monotonic', side_effect=[0, 2]):
            with self.assertRaisesRegex(CalibrationError, '超时'):
                read_camera_serial('/dev/video2', timeout=1)
            close.assert_called_once_with(9)
            self.assertEqual(restore.call_args.args[2], [0, 0, 0, 0, 0, 0, []])

    def test_transport_handles_partial_writes_and_fragmented_reads(self):
        reply = bytes.fromhex('5359030d0107000800000600534e30373337d94f')
        with patch('ego_calibration.ego_std_serial.camera_port', return_value=Path('/dev/ttyACM1')), \
             patch('ego_calibration.ego_std_serial.platform.system', return_value='Linux'), \
             patch('os.open', return_value=9), patch('os.close') as close, \
             patch('fcntl.flock'), patch('tty.setraw'), \
             patch('termios.tcgetattr', side_effect=lambda _: [0, 0, 0, 0, 0, 0, []]), \
             patch('termios.tcsetattr') as restore, \
             patch('ego_calibration.ego_std_serial.secrets.randbelow', return_value=6), \
             patch('ego_calibration.ego_std_serial.select.select', side_effect=[([], [9], []), ([9], [9], []), ([9], [], [])]), \
             patch('os.write', side_effect=[3, 9]) as write, \
             patch('os.read', side_effect=[reply[:7], reply[7:]]):
            self.assertEqual(read_camera_serial('/dev/video2'), 'SN0737')
            self.assertEqual([call.args[1] for call in write.call_args_list], [_request(7), _request(7)[3:]])
            close.assert_called_once_with(9)
            self.assertEqual(restore.call_args.args[2], [0, 0, 0, 0, 0, 0, []])


class DeviceSerialStateTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.service = InspectionService(Path(temporary.name))
        self.addCleanup(self.service.close)
        self.device = CameraDevice('ego-std', '/dev/video2', 'Ego-Std · USB-SHARED', serial='USB-SHARED')

    def test_rescan_same_usb_identity_refreshes_both_serials_and_clears_old_calibration(self):
        self.service.selected = replace(self.device, calibration_serial='OLD', camera_serial='OLD')
        self.service.payload = {'header': {'serial_number': 'OLD'}}
        self.service.std.review_token = 'old-review'
        with patch('ego_calibration.inspection_service.ego_std.scan_devices', return_value=[self.device]), \
             patch('ego_calibration.inspection_service.ego_lite.scan_devices', return_value=[]), \
             patch('ego_calibration.inspection_service.scan_uvc_devices', return_value=[]), \
             patch('ego_calibration.inspection_service.ego_std.read_calibration', return_value={'header': {'serial_number': 'CAL-NEW'}}), \
             patch('ego_calibration.inspection_service.read_camera_serial', return_value='CAM-NEW'):
            self.service._scan({})
        self.assertEqual(self.service.selected.calibration_serial, 'CAL-NEW')
        self.assertEqual(self.service.selected.camera_serial, 'CAM-NEW')
        self.assertIsNone(self.service.payload)
        self.assertEqual(self.service.std.review_token, '')
        self.assertEqual(self.service.devices, [self.service.selected])

    def test_read_updates_dropdown_data_and_records_uart_source(self):
        self.service.devices = [self.device]
        self.service.selected = self.device
        with patch('ego_calibration.inspection_service.ego_std.read_calibration', return_value={'header': {'serial_number': 'CAL-2'}}), \
             patch('ego_calibration.inspection_service.read_camera_serial', return_value='CAM-2'):
            self.service._read({})
        self.assertEqual(self.service.devices[0].camera_serial, 'CAM-2')
        self.assertEqual(self.service.devices[0].calibration_serial, 'CAL-2')
        self.assertEqual(self.service.payload['device_identity']['camera_serial_source'], 'uart_sn_0x0d')

    def test_reconnected_camera_selection_does_not_stick_to_laptop_or_override_unchanged_choice(self):
        laptop = CameraDevice('uvc', '/dev/video0', 'Integrated Camera')
        other = replace(self.device, identifier='/dev/video4')
        cases = (
            ('new camera replaces laptop fallback', [laptop], laptop, [self.device], self.device),
            ('unchanged scan preserves explicit laptop choice', [self.device, laptop], laptop, [self.device], laptop),
            ('keep another selected calibration camera', [other, laptop], other, [self.device, other], other),
            ('multiple new cameras need a user selection', [laptop], laptop, [self.device, other], laptop),
        )
        for description, before, selected, cameras, expected in cases:
            with self.subTest(description=description), \
                 patch('ego_calibration.inspection_service.ego_std.scan_devices', return_value=cameras), \
                 patch('ego_calibration.inspection_service.ego_lite.scan_devices', return_value=[]), \
                 patch('ego_calibration.inspection_service.scan_uvc_devices', return_value=[laptop]), \
                 patch('ego_calibration.inspection_service.ego_std.read_calibration', return_value={'header': {'serial_number': 'CAL-NEW'}}), \
                 patch('ego_calibration.inspection_service.read_camera_serial', return_value='CAM-NEW'):
                self.service.devices, self.service.selected = before, selected
                self.service._scan({})
                self.assertEqual(self.service.selected.identifier, expected.identifier)
                if expected.identifier == self.device.identifier:
                    self.assertIn('已选择新接入', self.service.notice)

    def test_unavailable_camera_sn_never_reuses_previous_or_usb_serial(self):
        device = replace(self.device, camera_serial='OLD')
        for response in (CalibrationError('没有 CDC 串口'), 'USB-SHARED'):
            with self.subTest(response=response), \
                 patch('ego_calibration.inspection_service.read_camera_serial',
                       side_effect=response if isinstance(response, Exception) else None,
                       return_value=response):
                current = self.service._std_serials(device, {'header': {'serial_number': 'CAL-2'}})
            self.assertEqual(current.camera_serial, '')
            self.assertEqual(current.calibration_serial, 'CAL-2')
            self.assertTrue(current.serial_error)

    def test_calibration_failure_does_not_prevent_uart_sn_read(self):
        with patch('ego_calibration.inspection_service.ego_std.read_calibration', side_effect=CalibrationError('CRC 错误')), \
             patch('ego_calibration.inspection_service.read_camera_serial', return_value='CAM-2'):
            current = self.service._std_serials(self.device)
        self.assertEqual(current.calibration_serial, '')
        self.assertEqual(current.camera_serial, 'CAM-2')
        self.assertIn('CRC 错误', current.serial_error)

    def test_disconnected_camera_does_not_keep_previous_device_or_serials(self):
        self.service.selected = self.device
        self.service.payload = {'header': {'serial_number': 'OLD'}}
        with patch('ego_calibration.inspection_service.ego_std.scan_devices', return_value=[]), \
             patch('ego_calibration.inspection_service.ego_lite.scan_devices', return_value=[]), \
             patch('ego_calibration.inspection_service.scan_uvc_devices', return_value=[]):
            self.service._scan({})
        self.assertIsNone(self.service.selected)
        self.assertIsNone(self.service.payload)
        self.assertEqual(self.service.devices, [])


if __name__ == '__main__':
    unittest.main()

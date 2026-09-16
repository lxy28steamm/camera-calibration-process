from __future__ import annotations

import json
import tempfile
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ego_calibration import oak_imu_calibration as oak
from ego_calibration.backends.std_runner import load_pipeline, native_command, prepare_source
from ego_calibration.inspection_service import InspectionService
from ego_calibration.lite_service import LiteService
from ego_calibration.models import CalibrationError, CameraDevice
from ego_calibration.std_service import StdService
from test_oak_imu_calibration import _result_yaml


class WorkflowServicesTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.stop = threading.Event()
        self.lite = LiteService(self.root)
        self.std = StdService(self.root)
        self.device = CameraDevice('ego-lite', 'MX-123', 'Test Ego-Lite')

    def test_lite_capture_keeps_confirmed_board_and_device_identity(self):
        def capture(identifier, directory, duration, **kwargs):
            self.assertEqual(identifier, self.device.identifier)
            self.assertIn('tagSize: 0.055', (directory/'target.yaml').read_text())
            kwargs['progress'](1, 20, 20, 200)
            kwargs['preview'](np.zeros((100, 200), np.uint8), np.zeros((100, 200), np.uint8), 20, 20)
            return oak.CaptureSummary(directory, 20, 20, 200, 1, False)
        with patch.object(oak, 'capture_kalibr_dataset', side_effect=capture):
            self.lite.capture({'duration_s': 10}, self.device, self.stop)
        self.assertEqual(self.lite.result['status'], 'complete')
        self.assertEqual(self.lite.result['device']['identifier'], 'MX-123')
        self.assertTrue(self.lite.preview.startswith(b'\xff\xd8'))
        self.assertEqual(self.lite.state()['progress']['imu_samples'], 200)
        self.assertEqual(self.lite.catalog()[0]['mode'], 'capture')

    def test_lite_does_not_accept_dex_or_offline_devices(self):
        with patch.object(oak, 'capture_kalibr_dataset') as capture:
            for device in (None, CameraDevice('dex-mono', '/dev/video9', 'Dex'), replace(self.device, identifier='offline')):
                with self.assertRaises(CalibrationError):
                    self.lite.capture({}, device, self.stop)
            capture.assert_not_called()

    def test_noise_capture_is_separate_and_keeps_cancelled_data(self):
        def capture(identifier, directory, duration, **kwargs):
            (directory/'imu0.csv').write_text('timestamp,omega_x\n1,0\n')
            self.stop.set()
            return oak.ImuNoiseCaptureSummary(directory, directory/'imu0.csv', 200, 1, 200, True)
        with patch.object(oak, 'capture_imu_noise_dataset', side_effect=capture):
            self.lite.capture({'duration_s': 10}, self.device, self.stop, noise=True)
        self.assertEqual(self.lite.result['status'], 'cancelled')
        self.assertEqual(self.lite.result['mode'], 'imu_noise')
        self.assertTrue(any(f['name']=='imu0.csv' for f in self.lite.result['files']))

    def test_flash_requires_review_same_device_unchanged_snapshot_and_confirmation(self):
        with patch.object(oak, 'flash_kalibr_result') as flash:
            with self.assertRaises(CalibrationError):
                self.lite.flash({'confirmed': True}, self.device, self.stop)
            self.lite.inspect_result({'yaml_text': _result_yaml()}, self.device)
            valid = {'confirmed': True, 'review_token': self.lite.review_token}
            for values, device in (({**valid, 'confirmed': False}, self.device), ({**valid, 'review_token': 'wrong'}, self.device), (valid, replace(self.device, identifier='OTHER'))):
                with self.assertRaises(CalibrationError):
                    self.lite.flash(values, device, self.stop)
            self.lite.review_path.write_text(_result_yaml()+'\n# changed')
            with self.assertRaises(CalibrationError):
                self.lite.flash(valid, self.device, self.stop)
            flash.assert_not_called()

    def test_explicit_lite_write_uses_backup_backend_and_consumes_review(self):
        self.lite.inspect_result({'yaml_text': _result_yaml()}, self.device)
        values = {'confirmed': True, 'review_token': self.lite.review_token}
        with patch.object(oak, 'flash_kalibr_result', return_value={'maximum_error': 0}) as flash:
            self.lite.flash(values, self.device, self.stop)
        self.assertEqual(flash.call_args.args[0], 'MX-123')
        self.assertTrue(flash.call_args.args[2].is_relative_to(self.lite.directory))
        self.assertEqual(self.lite.review_token, '')
        self.assertEqual(self.lite.catalog()[0]['mode'], 'flash')

    def test_bad_result_invalidates_previous_lite_review(self):
        self.lite.inspect_result({'yaml_text': _result_yaml()}, self.device)
        with self.assertRaises(CalibrationError):
            self.lite.inspect_result({'yaml_text': 'cam0: {}'}, self.device)
        self.assertIsNone(self.lite.review)
        self.assertEqual(self.lite.review_token, '')

    def test_lite_solve_requires_noise_confirmation_before_running(self):
        dataset = self.lite.directory/'dataset'
        dataset.mkdir(parents=True)
        with patch.object(self.lite, 'run') as run:
            with self.assertRaisesRegex(CalibrationError, '噪声'):
                self.lite.solve({'dataset_id': 'dataset'}, self.stop)
            run.assert_not_called()

    def test_lite_solve_exports_verified_result_and_preserves_input(self):
        dataset = self.lite.directory/'dataset'
        for name in ('cam0', 'cam1'):
            (dataset/name).mkdir(parents=True)
            (dataset/name/'100.png').write_bytes(b'original')
        (dataset/'imu0.csv').write_text('original imu')
        (dataset/'camchain.yaml').write_text('original camera')
        imu = 'rostopic: /imu0\nupdate_rate: 200\n' + ''.join(f'{key}: 0.001\n' for key in ('accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density','gyroscope_random_walk'))
        def run(command, stop, *, cwd):
            (cwd/'oak-cam-imu-camchain-imucam.yaml').write_text(_result_yaml())
        with patch.object(self.lite, 'check_environment', return_value=oak.KalibrEnvironment(None, {}, ())), patch.object(oak, 'validate_kalibr_dataset', return_value='valid'), patch.object(self.lite, 'run', side_effect=run) as execute:
            self.lite.solve({'dataset_id': 'dataset', 'noise_confirmed': True, 'imu_yaml': imu}, self.stop)
        self.assertEqual(execute.call_count, 2)
        self.assertEqual((dataset/'camchain.yaml').read_text(), 'original camera')
        self.assertIn('T_cam_imu', self.lite.file(self.lite.result['yaml_id']).read_text())

    def test_std_settings_are_finite_bounded_and_do_not_use_sample_board(self):
        defaults = self.std.settings({})
        self.assertEqual((defaults['tag_size'], defaults['tag_spacing'], defaults['scale']), (.055, .0165, 1))
        for settings in ({'scale': 0}, {'max_iter': True}, {'tag_size': float('nan')}, {'gyro_noise_density': -1}, {'shell': 'bad'}):
            with self.assertRaises(CalibrationError):
                self.std.settings({'settings': settings})

    def test_std_requires_parameters_confirmation_except_dump(self):
        for stage in ('bag', 'camera', 'imu'):
            with self.assertRaises(CalibrationError):
                self.std.command(self.root/'input.h264', self.root/'output', self.std.settings({}), {'stage': stage})
        command = self.std.command(self.root/'input.h264', self.root/'output', self.std.settings({}), {'stage': 'dump'})
        self.assertIn('--stop-after', command)
        self.assertNotIn('docker', command)

    def test_std_rejects_generic_h264_before_copy_or_calibration(self):
        source = self.root/'generic.h264'
        source.write_bytes(b'\x00\x00\x01\x67sps\x00\x00\x01\x68pps\x00\x00\x01\x65frame')
        with self.assertRaisesRegex(CalibrationError, 'YCTC SEI'):
            self.std.import_video({'path': str(source)}, self.root/'uploads', self.stop)
        self.assertFalse(self.std.directory.exists())

    def test_native_adapter_preserves_paths_with_spaces_and_delivery_patches(self):
        pipeline = load_pipeline()
        source = self.root/'source with spaces'
        output = self.root/'output with spaces'
        command = native_command(pipeline, source, output, ['python3', '/work/h264_sei_to_kalibr_bag.py', '--video', '/input/source.h264', '--output-dir', '/data'], self.root/'input video.h264')
        self.assertIn(str(self.root/'input video.h264'), command)
        self.assertEqual(command[-1], str(output))
        command = native_command(pipeline, source, output, ['rosrun', 'kalibr', 'kalibr_calibrate_imu_camera', '--bag', '/data/file.bag'])
        self.assertTrue(any(str(source) in arg and arg.endswith('/kalibr_calibrate_imu_camera') for arg in command))
        self.assertEqual(command[-1], str(output/'file.bag'))

    def test_delivery_source_archive_extracts_without_cyclic_symlinks(self):
        source = prepare_source(self.root/'runtime')
        self.assertIn('--time-offset-init', (source/'aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera').read_text())
        self.assertTrue((source/'LICENSE').is_file())
        self.assertFalse(any(p.is_symlink() for p in source.rglob('*')))

    def test_std_resume_keeps_recorded_settings_and_does_not_rewrite_fixed_chain(self):
        directory = self.std.directory/'job'
        directory.mkdir(parents=True)
        video = directory/'recording.h264'
        video.write_bytes(b'input')
        fixed = directory/'fixed-camchain.yaml'
        fixed.write_text('preserved')
        request = {'video_id': 'job/recording.h264', 'stage': 'bag', 'parameters_confirmed': False,
                   'settings': {'tag_size': .0352, 'tag_spacing': .01056}, 'camchain_yaml': 'preserved'}
        (directory/'workflow.json').write_text(json.dumps({'mode': 'solve', 'request': request}))
        timestamp = fixed.stat().st_mtime_ns
        with patch.object(self.std, 'run') as run, patch('ego_calibration.std_service.load_pipeline'):
            self.std.solve({'resume_id': 'job', 'stage': 'imu', 'parameters_confirmed': True, 'settings': {'tag_size': .1}}, self.stop)
        command = run.call_args.args[0]
        self.assertIn('--resume', command)
        self.assertEqual(command[command.index('--tag-size')+1], '0.0352')
        self.assertEqual(command[command.index('--stop-after')+1], 'imu')
        self.assertEqual(fixed.stat().st_mtime_ns, timestamp)
        self.assertEqual(self.std.result['id'], 'job')

    def test_native_camera_report_uses_headless_adapter(self):
        pipeline = load_pipeline()
        source = self.root/'source'
        command = native_command(pipeline, source, self.root/'output',
                                 ['python3', pipeline.PATCHED_KALIBR_CAMERA_CALIBRATOR, '--dont-show-report'])
        self.assertTrue(any(arg.endswith('/std_kalibr_runner.py') for arg in command))

    def test_workflow_downloads_and_dataset_ids_cannot_escape_their_root(self):
        (self.root/'secret.yaml').write_text('private')
        for service in (self.lite, self.std):
            service.directory.mkdir()
            (service.directory/'escape.yaml').symlink_to(self.root/'secret.yaml')
            for identifier in ('../secret.yaml', str(self.root/'secret.yaml'), 'escape.yaml', ''):
                with self.assertRaises(CalibrationError):
                    service.file(identifier)

    def test_scheduler_keeps_flash_non_cancellable_and_workflows_exclusive(self):
        service = InspectionService(self.root)
        service.operation = 'lite_flash_write'
        with self.assertRaises(CalibrationError):
            service.dispatch('stop', {})
        with self.assertRaises(CalibrationError):
            service.dispatch('std_solve', {})
        self.assertFalse(service.stop_event.is_set())
        service.operation = 'lite_solve'
        service.dispatch('stop', {})
        self.assertTrue(service.stop_event.is_set())


if __name__ == '__main__':
    unittest.main()

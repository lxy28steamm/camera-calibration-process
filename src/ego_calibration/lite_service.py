"""Headless Ego-Lite camera/IMU workflow, separate from monocular calibration."""
from __future__ import annotations

import hashlib
import json
import os
import secrets
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

from ego_calibration import oak_imu_calibration as oak
from ego_calibration.models import CalibrationError
from ego_calibration.mono_service import MonoSettings
from ego_calibration.workflow_service import WorkflowService, bounded, serializable


class LiteService(WorkflowService):
    def __init__(self, root):
        super().__init__(root, 'ego_lite')
        self.review = None
        self.review_token = ''
        self.review_path = None
        self.flash_result = None

    def state(self):
        return {**super().state(), 'review': self.review, 'review_token': self.review_token, 'flash_result': self.flash_result}

    @staticmethod
    def require_device(device):
        if not device or device.kind != 'ego-lite' or device.identifier == 'offline':
            raise CalibrationError('此入口仅支持已连接的 Ego-Lite / DepthAI 设备')

    def invalidate_review(self):
        self.review, self.review_token, self.review_path = None, '', None

    def check_environment(self, _data):
        configured = os.environ.get('CAMERA_KALIBR_SETUP', '').strip()
        environment = oak.detect_kalibr_environment(Path(configured)) if configured else oak.detect_kalibr_environment()
        self.environment = {'checked': True, 'ready': environment.ready, 'label': environment.label,
                            'missing': list(environment.missing + environment.runtime_missing)}
        return environment

    def capture(self, data, device, stop, *, noise=False):
        self.require_device(device)
        duration = bounded(data, 'duration_s', 7200 if noise else 120, 10, 21600 if noise else 600, integer=True)
        board = MonoSettings.from_dict(data.get('board', {}))
        if board.board != 'aprilgrid':
            raise CalibrationError('Ego-Lite 联合标定使用 AprilGrid')
        self.invalidate_review()
        with self.task('imu_noise' if noise else 'capture', data, stop, device=device) as directory:
            if noise:
                def progress(elapsed, count):
                    with self.lock:
                        self.progress = {'elapsed_s': elapsed, 'imu_samples': count, 'duration_s': duration}
                result = oak.capture_imu_noise_dataset(device.identifier, directory, duration, should_stop=stop.is_set, progress=progress)
            else:
                (directory / 'target.yaml').write_text(board.target_yaml())
                def progress(elapsed, left, right, imu):
                    with self.lock:
                        self.progress = {'elapsed_s': elapsed, 'left_frames': left, 'right_frames': right, 'imu_samples': imu, 'duration_s': duration}
                def preview(left, right, *_):
                    image = np.concatenate((left, right), axis=1)
                    if image.shape[1] > 1600:
                        image = cv2.resize(image, (1600, round(image.shape[0] * 1600 / image.shape[1])))
                    ok, encoded = cv2.imencode('.jpg', image)
                    if ok:
                        self.preview = encoded.tobytes()
                result = oak.capture_kalibr_dataset(device.identifier, directory, duration, should_stop=stop.is_set, progress=progress, preview=preview)
            self.result['capture'] = serializable(result)
            self.result['dataset_id'] = self.file_id(directory)

    def import_dataset(self, data, stop):
        source = Path(data.get('path', '')).expanduser().resolve()
        if not all((source / name).exists() for name in ('cam0', 'cam1', 'imu0.csv', 'camchain.yaml')):
            raise CalibrationError('数据集需包含 cam0、cam1、imu0.csv、camchain.yaml')
        with self.task('import', {'source': str(source)}, stop) as directory:
            for name in ('cam0', 'cam1'):
                (directory / name).mkdir()
                for image in (source / name).glob('*.png'):
                    if stop.is_set():
                        return
                    if image.is_symlink():
                        raise CalibrationError('导入数据集不接受图像符号链接')
                    shutil.copy2(image, directory / name / image.name)
            for name in ('imu0.csv', 'camchain.yaml', 'target.yaml', 'capture.json'):
                if (source / name).is_file():
                    shutil.copy2(source / name, directory / name)
            self.result['dataset_id'] = self.file_id(directory)

    def solve(self, data, stop):
        source = self.file(data.get('dataset_id'), directory=True)
        if data.get('noise_confirmed') is not True:
            raise CalibrationError('请核实并确认当前设备的 IMU 噪声参数')
        imu = data.get('imu_yaml', '')
        try:
            config = yaml.safe_load(imu)
            for name in ('accelerometer_noise_density', 'accelerometer_random_walk', 'gyroscope_noise_density', 'gyroscope_random_walk', 'update_rate'):
                bounded(config, name, None, 1e-15, 100000)
            if config.get('rostopic') != '/imu0':
                raise CalibrationError('IMU rostopic 必须为 /imu0')
        except (yaml.YAMLError, TypeError, AttributeError) as exc:
            raise CalibrationError('IMU YAML 格式无效') from exc
        target = MonoSettings.from_dict(data.get('board', {}))
        if target.board != 'aprilgrid':
            raise CalibrationError('请选择 AprilGrid')
        environment = self.check_environment({})
        if not environment.ready:
            raise CalibrationError('Kalibr 环境不完整：' + ', '.join(environment.missing + environment.runtime_missing))
        self.invalidate_review()
        with self.task('solve', data, stop) as directory:
            # Each run has its own folder; original acquisitions remain unchanged.
            for name in ('cam0', 'cam1'):
                if not (source / name).is_dir():
                    raise CalibrationError(f'数据集缺少 {name}')
                shutil.copytree(source / name, directory / name)
            for name in ('imu0.csv', 'camchain.yaml', 'capture.json'):
                if (source / name).is_file():
                    shutil.copy2(source / name, directory / name)
            (directory / 'target.yaml').write_text(target.target_yaml())
            (directory / 'imu.yaml').write_text(imu)
            checked = oak.validate_kalibr_dataset(directory, directory/'target.yaml', directory/'imu.yaml')
            self.log_path.write_text(checked + '\n')
            for command in oak.kalibr_commands(directory, directory/'target.yaml', directory/'imu.yaml'):
                program, arguments = oak.kalibr_process_spec(command, environment)
                self.run([program, *arguments], stop, cwd=directory)
            result = directory / 'oak-cam-imu-camchain-imucam.yaml'
            self.result.update(calibration=serializable(oak.load_kalibr_result(result)), yaml_id=self.file_id(result))

    def inspect_result(self, data, device):
        self.require_device(device)
        self.invalidate_review()
        text = data.get('yaml_text')
        if not text:
            text = self.file(data.get('yaml_id')).read_text()
        if not isinstance(text, str) or not 0 < len(text.encode()) <= 1024**2:
            raise CalibrationError('结果 YAML 大小无效')
        directory = self.directory / 'review'
        directory.mkdir(parents=True, exist_ok=True)
        snapshot = directory / (secrets.token_hex(12) + '.yaml')
        snapshot.write_text(text)
        result = oak.load_kalibr_result(snapshot)
        self.review_path = snapshot
        self.review_token = secrets.token_urlsafe(32)
        self.review = {'device': serializable(device), 'sha256': hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                       'result': serializable(result), 'yaml_text': text,
                       'note': '仅写入相机—IMU 外参。时间偏移保留在 YAML，不写入 EEPROM；写入时核对设备原有双目链。'}

    def flash(self, data, device, stop):
        self.require_device(device)
        if data.get('confirmed') is not True or not self.review or not secrets.compare_digest(str(data.get('review_token', '')), self.review_token):
            raise CalibrationError('请先检查结果并明确确认本次 Ego-Lite EEPROM 写入')
        if self.review['device']['identifier'] != device.identifier or hashlib.sha256(self.review_path.read_bytes()).hexdigest() != self.review['sha256']:
            raise CalibrationError('设备或待写结果已变化，请重新检查')
        snapshot = self.review_path
        self.invalidate_review()
        with self.task('flash', {'yaml_sha256': hashlib.sha256(snapshot.read_bytes()).hexdigest()}, stop, device=device) as directory:
            result = oak.flash_kalibr_result(device.identifier, snapshot, directory, backend='v2')
            self.flash_result = serializable(result)
            self.result['flash'] = self.flash_result

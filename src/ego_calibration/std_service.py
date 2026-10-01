"""Ego-Std/YCTC H.264 + SEI calibration, using the supplied customer backend."""
from __future__ import annotations

import json
import copy
from concurrent.futures import ThreadPoolExecutor
import shutil
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import yaml

from ego_calibration.backends.std_runner import DELIVERY, load_pipeline
from ego_calibration.models import CalibrationError
from ego_calibration import std_flash
from ego_calibration.mono_service import child_environment
from ego_calibration.workflow_service import WorkflowService, bounded


class StdService(WorkflowService):
    def __init__(self, root):
        super().__init__(root, 'ego_std')
        self.backends = Path(__file__).with_name('backends')
        self.video_id = ''
        self.review = None
        self.review_token = ''
        self.review_path = None

    def state(self):
        with self.lock:
            return {**super().state(), 'video_id': self.video_id,
                    'review': copy.deepcopy(self.review), 'review_token': self.review_token,
                    'write_credential_configured': std_flash.credential_configured()}

    def invalidate_review(self):
        self.review, self.review_path, self.review_token = None, None, ''

    def read_device(self, data, device, stop):
        self.invalidate_review()
        std_flash.require_device(device)
        with self.task('backup', {}, stop, device=device) as directory:
            with std_flash.connection(device) as backend:
                blob, current = std_flash.read_current(backend)
                std_flash.backup(directory, blob, current)
            self.result['current'] = current
        return current['calibration']

    def inspect_result(self, data, device, stop):
        self.invalidate_review()
        std_flash.require_device(device)
        text = data.get('yaml_text')
        if not text:
            text = self.file(data.get('yaml_id')).read_text(encoding='utf-8')
        if not isinstance(text, str) or len(text.encode('utf-8')) > 8124:
            raise CalibrationError('Ego-Std YAML 最大为 8124 字节')
        with self.task('review', {}, stop, device=device) as directory:
            path = directory / 'candidate.yaml'
            std_flash.durable_write(path, text.encode('utf-8'))
            summary = std_flash.validate_result(path)
            with std_flash.connection(device) as backend:
                blob, current = std_flash.read_current(backend)
                std_flash.backup(directory, blob, current)
            self.result['current'] = current
            if current['protocol_version'] != 1:
                raise CalibrationError('设备不支持所提供的写入协议版本，已保留现有标定备份')
            serial = data.get('serial_number', '').strip() or current['calibration']['header'].get('serial_number', '')
            std_flash.writer.encode_calibration_serial_number(serial)
            std_flash.writer.build_kalibr_yaml_blob(path.read_bytes(), serial)
            if stop.is_set():
                raise CalibrationError('检查已停止，请重新检查')
            self.review = {'device': {'identifier': device.identifier, 'serial': device.serial, 'label': device.label},
                           'serial_number': serial, 'new': summary, 'current': current,
                           'files': self.files(directory)}
            self.review_path = path
            self.review_token = uuid.uuid4().hex
            self.result['review'] = self.review

    def flash(self, data, device, stop):
        if data.get('confirmed') is not True:
            raise CalibrationError('请核对设备、标定序列号和 YAML 尺寸，并确认写入')
        if not self.review_token or data.get('review_token') != self.review_token:
            raise CalibrationError('请先重新检查待写入结果与目标设备')
        review, path = copy.deepcopy(self.review), self.review_path
        self.invalidate_review()  # One attempt per review, including failed attempts.
        std_flash.require_device(device)
        if device.identifier != review['device']['identifier'] or device.serial != review['device']['serial']:
            raise CalibrationError('目标设备已改变，请重新检查')
        summary = std_flash.validate_result(path)
        if summary['sha256'] != review['new']['sha256']:
            raise CalibrationError('待写入 YAML 已改变，请重新检查')
        secret = std_flash.load_secret()
        request = {'sha256': summary['sha256'], 'serial_number': review['serial_number']}
        with self.task('flash', request, stop, device=device) as directory:
            candidate = directory / 'candidate.yaml'
            std_flash.durable_write(candidate, path.read_bytes())
            if std_flash.digest(candidate.read_bytes()) != summary['sha256']:
                raise CalibrationError('YAML 在准备期间发生变化，请重新检查')
            with std_flash.connection(device) as backend:
                blob, current = std_flash.read_current(backend)
                if current['identity'] != review['current']['identity'] or current['sha256'] != review['current']['sha256']:
                    raise CalibrationError('设备身份或当前标定已改变，请重新检查')
                std_flash.backup(directory, blob, current)
                if stop.is_set():
                    raise CalibrationError('写入尚未开始，任务已停止')
                self.progress = {'stage': '备份完成，正在写入并回读；请保持连接'}
                result, payload = std_flash.write_verified(backend, candidate, review['serial_number'], directory, secret)
                self.result.update(write=result, current=current, verified=True, calibration=payload)
                self.progress = {'stage': '写入、激活及逐字节回读验证完成'}
        return payload

    def check_environment(self, _data):
        command = ['bash', str(self.backends/'run.sh'), 'python', '-c', 'import rosbag, cv_bridge, kalibr_camera_calibration, kalibr_imu_camera_calibration; print("ready")']
        try:
            native = subprocess.run(command, env=child_environment(), capture_output=True, text=True, timeout=30)
            native_ready = native.returncode == 0
            detail = (native.stdout + native.stderr)[-2000:]
        except (OSError, subprocess.TimeoutExpired) as exc:
            native_ready, detail = False, str(exc)
        docker_ready = False
        if shutil.which('docker'):
            try:
                docker_ready = subprocess.run(['docker', 'image', 'inspect', 'kalibr-h264-imu-demo:20260805'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5).returncode == 0
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.environment = {'checked': True, 'native_ready': native_ready, 'docker_ready': docker_ready,
                            'dump_ready': (DELIVERY/'bin/video_user_data_dump').is_file(), 'detail': detail}

    def import_video(self, data, uploads, stop):
        if data.get('upload'):
            source = (uploads / data['upload']).resolve()
            if not source.is_relative_to(uploads.resolve()):
                raise CalibrationError('上传路径无效')
        else:
            source = Path(data.get('path', '')).expanduser().resolve()
        if not source.is_file() or source.suffix.lower() not in ('.h264', '.264'):
            raise CalibrationError('请选择包含 YCTC SEI 元数据的原始 .h264 文件')
        inspection = load_pipeline().inspect_h264(source)
        if not inspection.prefix_has_yctc_sei:
            raise CalibrationError('首个关键帧前未检测到 YCTC SEI，普通 H.264 不能用于相机—IMU 标定')
        with self.task('import', {'filename': source.name, 'source_path': str(source)}, stop) as directory:
            destination = directory/'recording.h264'
            # Uploaded videos already occupy the persistent uploads directory.
            # Move them into their import record instead of keeping a second
            # full-size copy. Files selected by a local path still need copying
            # so the workflow owns a stable input file.
            if source.is_relative_to(uploads.resolve()):
                shutil.move(str(source), str(destination))
            else:
                shutil.copy2(source, destination)
            self.video_id = self.file_id(destination)
            self.result['video_id'] = self.video_id

    @staticmethod
    def settings(data):
        values = data.get('settings', {})
        specs = (
            ('tag_rows', 6, 2, 20, True), ('tag_cols', 6, 2, 20, True),
            ('tag_size', .055, .005, 1, False), ('tag_spacing', .0165, 0, 1, False),
            ('image_stride', 3, 1, 60, True), ('scale', 1, .1, 1, False),
            ('bag_freq', 4, .1, 30, False), ('min_laplacian_variance', 150, 0, 10000, False),
            ('approx_sync', .02, .0001, .2, False), ('max_corner_reproj_px', .85, .05, 10, False),
            ('max_corner_filter_rounds', 10, 1, 30, True), ('timeoffset_padding', .03, .001, .5, False),
            ('time_offset_init', .001, -.5, .5, False), ('max_iter', 30, 1, 200, True),
            ('acc_noise_density', .02, 1e-12, 100, False), ('acc_random_walk', 1.92084189233e-5, 1e-15, 10, False),
            ('gyro_noise_density', .002, 1e-12, 100, False), ('gyro_random_walk', 2.26588533832e-6, 1e-15, 10, False),
        )
        if not isinstance(values, dict) or set(values) - {s[0] for s in specs}:
            raise CalibrationError('Ego-Std 设置包含未知字段')
        return {key: bounded(values, key, default, low, high, integer=integer) for key, default, low, high, integer in specs}

    def command(self, video, output, settings, data):
        stage = data.get('stage', 'imu')
        runtime = data.get('runtime', 'native')
        if stage not in ('dump', 'bag', 'camera', 'imu') or runtime not in ('native', 'docker'):
            raise CalibrationError('未知求解阶段或运行环境')
        if stage != 'dump' and data.get('parameters_confirmed') is not True:
            raise CalibrationError('请确认实物板尺寸、缩放比例和当前设备 IMU 噪声参数')
        script = self.backends/'std_runner.py'
        if stage == 'dump' or runtime == 'docker':
            command = [sys.executable, '--std-backend'] if getattr(sys, 'frozen', False) else [sys.executable, str(script)]
        else:
            command = ['bash', str(self.backends/'run.sh'), 'python', str(script)]
        command += ['--runtime', runtime, '--runtime-dir', str(self.directory/'runtime'), str(video), '--output-dir', str(output), '--stop-after', stage]
        for key, value in settings.items():
            command += ['--'+key.replace('_', '-'), str(value)]
        model = data.get('model', 'pinhole-radtan')
        if model not in ('pinhole-radtan', 'pinhole-equi', 'omni-radtan', 'ds-none'):
            raise CalibrationError('相机模型不支持')
        command += ['--camera-model', model]
        return command

    def solve(self, data, stop):
        resume_directory = None
        if data.get('resume_id'):
            resume_directory = self.file(data['resume_id'], directory=True)
            try:
                previous = json.loads((resume_directory/'workflow.json').read_text())
                if previous.get('mode') != 'solve':
                    raise ValueError('不是求解任务')
                data = {**previous['request'], 'stage': data.get('stage', previous['request'].get('stage', 'imu')),
                        'parameters_confirmed': previous['request'].get('parameters_confirmed') is True or data.get('parameters_confirmed') is True}
            except (OSError, ValueError, KeyError) as exc:
                raise CalibrationError('请选择有效的 Ego-Std 求解记录继续') from exc
        settings = self.settings(data)
        video = self.file(data.get('video_id') or self.video_id)
        if video.suffix != '.h264':
            raise CalibrationError('请选择原始 H.264 视频')
        self.video_id = self.file_id(video)
        # Validate requests before creating output directories or starting tools.
        self.command(video, self.directory/'pending', settings, data)
        with self.task('solve', {**data, 'video_id': self.video_id, 'settings': settings}, stop, directory=resume_directory) as directory:
            output = directory/'calibration'
            command = self.command(video, output, settings, data)
            if resume_directory:
                command.append('--resume')
            if data.get('camchain_yaml'):
                path = directory/'fixed-camchain.yaml'
                if not resume_directory:
                    path.write_text(data['camchain_yaml'])
                load_pipeline().validate_precalibrated_camchain(path)
                command += ['--camchain', str(path)]
            self.run(command, stop)
            self.result['stage'] = data.get('stage', 'imu')
            for name in ('calibration_summary.json', 'conversion_summary.json', 'calibration_run.json'):
                path = output/name
                if path.is_file():
                    self.result[name.removesuffix('.json')] = json.loads(path.read_text())
            chains = sorted(output.glob('*-camchain-imucam.yaml')) or sorted(output.glob('*-camchain.yaml'))
            if chains:
                self.result['yaml_id'] = self.file_id(chains[0])
                self.result['calibration'] = yaml.safe_load(chains[0].read_text())

    def calibration_text(self, identifier):
        path = self.file(identifier)
        load_pipeline().validate_precalibrated_camchain(path)
        return path.read_text()


class StdBatchService:
    """Run isolated Ego-Std H.264 calibration jobs, with at most six workers."""

    max_workers = 6

    def __init__(self, root):
        self.directory = (Path(root) / 'ego_std_batches').resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers, thread_name_prefix='std-calibration')
        self.batch = None
        self.services = {}
        self.stops = {}
        self.futures = {}
        self._restore_latest()

    def _save_locked(self):
        if not self.batch:
            return
        directory = self.directory / self.batch['id']
        directory.mkdir(parents=True, exist_ok=True)
        temporary = directory / 'batch.json.partial'
        temporary.write_text(json.dumps(self.batch, ensure_ascii=False, indent=2))
        temporary.replace(directory / 'batch.json')

    def _restore_latest(self):
        manifests = sorted(self.directory.glob('*/batch.json'), reverse=True)
        if not manifests:
            return
        try:
            batch = json.loads(manifests[0].read_text(encoding='utf-8'))
            if not isinstance(batch, dict) or not isinstance(batch.get('jobs'), list):
                return
            self.batch = batch
            for job in batch['jobs']:
                key = (batch['id'], job['id'])
                service = StdService(self.directory / batch['id'] / job['id'])
                self.services[key] = service
                self.stops[key] = threading.Event()
                if job.get('status') in ('queued', 'running'):
                    job['status'], job['stage'] = 'interrupted', '服务重启前任务中断；日志和已完成文件已保留'
            if any(job.get('status') == 'interrupted' for job in batch['jobs']):
                batch['status'] = 'interrupted'
            self._save_locked()
        except (OSError, ValueError, KeyError, TypeError):
            self.batch = None

    @property
    def active(self):
        with self.lock:
            return bool(self.batch and any(job['status'] in ('queued', 'running') for job in self.batch['jobs']))

    def start(self, data, uploads):
        filenames = data.get('uploads')
        if not isinstance(filenames, list) or not filenames or len(filenames) > 100:
            raise CalibrationError('请选择 1–100 个 H.264 文件')
        if data.get('parameters_confirmed') is not True:
            raise CalibrationError('请先核实标定板尺寸、图像比例和 IMU 噪声参数')
        settings = self.settings_for_batch(data)
        runtime = data.get('runtime', 'native')
        model = data.get('model', 'pinhole-radtan')
        if runtime not in ('native', 'docker') or model not in ('pinhole-radtan', 'pinhole-equi', 'omni-radtan', 'ds-none'):
            raise CalibrationError('运行环境或相机模型不支持')
        if runtime == 'docker':
            if not shutil.which('docker'):
                raise CalibrationError('Docker 不可用；请先检查运行环境')
            try:
                image_ready = subprocess.run(['docker', 'image', 'inspect', 'kalibr-h264-imu-demo:20260805'],
                                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8).returncode == 0
            except (OSError, subprocess.SubprocessError):
                image_ready = False
            if not image_ready:
                raise CalibrationError('批量并行前请先构建交付 Docker 镜像，避免多个任务同时构建')
        sources = []
        for entry in filenames:
            name = entry.get('upload') if isinstance(entry, dict) else entry
            display_name = entry.get('filename') if isinstance(entry, dict) else name
            if not isinstance(name, str) or Path(name).name != name:
                raise CalibrationError('上传文件名无效')
            if not isinstance(display_name, str) or Path(display_name).name != display_name or Path(display_name).suffix.lower() not in ('.h264', '.264'):
                raise CalibrationError('原始视频文件名无效')
            source = (Path(uploads) / name).resolve()
            if not source.is_relative_to(Path(uploads).resolve()) or not source.is_file() or source.suffix.lower() not in ('.h264', '.264'):
                raise CalibrationError(f'找不到有效的 H.264 上传文件：{name}')
            sources.append((source, display_name))
        with self.lock:
            if self.active:
                raise CalibrationError('当前批次仍在处理，请等待完成后再开始新批次')
            batch_id = time.strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8]
            jobs = []
            for index, (source, display_name) in enumerate(sources, 1):
                job_id = f'{index:03d}-{uuid.uuid4().hex[:8]}'
                jobs.append({'id': job_id, 'filename': display_name, 'status': 'queued', 'stage': '排队等待', 'files': [], 'error': ''})
                self.stops[(batch_id, job_id)] = threading.Event()
            self.batch = {'id': batch_id, 'status': 'running', 'max_workers': self.max_workers, 'jobs': jobs}
            self._save_locked()
            request = {'settings': settings, 'runtime': runtime, 'model': model,
                       'camchain_yaml': data.get('camchain_yaml', ''), 'parameters_confirmed': True}
            for job, (source, _display_name) in zip(jobs, sources):
                job_dir = self.directory / batch_id / job['id']
                service = StdService(job_dir)
                self.services[(batch_id, job['id'])] = service
                future = self.executor.submit(self._run_job, batch_id, job, source, request, uploads, service)
                self.futures[(batch_id, job['id'])] = future

    @staticmethod
    def settings_for_batch(data):
        return StdService.settings(data)

    def _run_job(self, batch_id, job, source, request, uploads, service):
        key = (batch_id, job['id'])
        stop = self.stops[key]
        with self.lock:
            if stop.is_set():
                job['status'], job['stage'] = 'cancelled', '已停止'
                self._save_locked()
                return
            job['status'], job['stage'] = 'running', '检查 SEI / IMU 元数据'
            self._save_locked()
        try:
            service.import_video({'path': str(source)}, uploads, stop)
            if stop.is_set():
                raise CalibrationError('任务已停止')
            with self.lock:
                job['stage'] = '生成 ROS bag 并运行双目 / IMU 标定'
                self._save_locked()
            service.solve({**request, 'video_id': service.video_id, 'stage': 'imu'}, stop)
            result = service.state().get('result') or {}
            with self.lock:
                job['status'] = 'complete'
                job['stage'] = '标定完成'
                job['yaml_id'] = result.get('yaml_id', '')
                job['summary'] = result.get('calibration_summary', {})
                job['files'] = [{'name': item['name'], 'id': f"{batch_id}/{job['id']}/{item['id']}"}
                                for item in service.files(service.directory)]
                self._save_locked()
        except Exception as exc:
            with self.lock:
                job['status'] = 'cancelled' if stop.is_set() else 'failed'
                job['stage'] = '已停止' if stop.is_set() else '处理失败'
                job['error'] = str(exc) or type(exc).__name__
                job['files'] = [{'name': item['name'], 'id': f"{batch_id}/{job['id']}/{item['id']}"}
                                for item in service.files(service.directory)]
                self._save_locked()
        finally:
            with self.lock:
                if self.batch and self.batch['id'] == batch_id:
                    statuses = [item['status'] for item in self.batch['jobs']]
                    if not any(status in ('queued', 'running') for status in statuses):
                        self.batch['status'] = ('complete' if all(status == 'complete' for status in statuses)
                                                else 'cancelled' if all(status == 'cancelled' for status in statuses)
                                                else 'finished_with_errors')
                    self._save_locked()

    def state(self):
        with self.lock:
            return copy.deepcopy(self.batch or {'id': '', 'status': 'idle', 'max_workers': self.max_workers, 'jobs': []})

    def cancel(self):
        with self.lock:
            if not self.batch:
                return
            for job in self.batch['jobs']:
                key = (self.batch['id'], job['id'])
                self.stops[key].set()
                future = self.futures.get(key)
                if future and future.cancel():
                    job['status'], job['stage'] = 'cancelled', '已停止'
            statuses = [job['status'] for job in self.batch['jobs']]
            if not any(status in ('queued', 'running') for status in statuses):
                self.batch['status'] = ('complete' if all(status == 'complete' for status in statuses)
                                        else 'cancelled' if all(status == 'cancelled' for status in statuses)
                                        else 'finished_with_errors')
            self._save_locked()

    def file(self, identifier):
        if not isinstance(identifier, str):
            raise CalibrationError('批次文件路径无效')
        parts = identifier.split('/', 2)
        if len(parts) != 3:
            raise CalibrationError('批次文件路径无效')
        service = self.services.get((parts[0], parts[1]))
        if service is None:
            raise CalibrationError('找不到该批次任务')
        return service.file(parts[2])

    def close(self):
        self.cancel()
        self.executor.shutdown(wait=True, cancel_futures=True)

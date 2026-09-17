"""Small shared job/file helpers for the two camera-IMU workflows."""
from __future__ import annotations

import copy
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid
from contextlib import contextmanager
from dataclasses import asdict, is_dataclass
from pathlib import Path

from ego_calibration.models import CalibrationError
from ego_calibration.mono_service import child_environment


def serializable(value):
    if is_dataclass(value):
        return serializable(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    return value


def bounded(values, key, default, low, high, *, integer=False):
    value = values.get(key, default)
    if type(value) not in ((int,) if integer else (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise CalibrationError(f"{key} 必须为 {low}–{high} 范围内的{'整数' if integer else '数值'}")
    return value


class WorkflowService:
    def __init__(self, root, name):
        self.directory = (root / name).resolve()
        self.lock = threading.RLock()
        self.result = None
        self.progress = {}
        self.log_path = None
        self.preview = b""
        self.environment = {"checked": False}

    def file_id(self, path):
        return path.resolve().relative_to(self.directory).as_posix()

    def file(self, identifier, *, directory=False):
        if not isinstance(identifier, str) or not identifier:
            raise CalibrationError("请选择本流程的数据或结果")
        path = (self.directory / identifier).resolve()
        if not path.is_relative_to(self.directory) or path == self.directory or not (path.is_dir() if directory else path.is_file()):
            raise CalibrationError("文件不在当前标定流程的数据目录")
        return path

    def files(self, directory):
        return [{"name": p.relative_to(directory).as_posix(), "id": self.file_id(p)}
                for p in sorted(directory.rglob('*')) if p.is_file() and p.suffix.lower() in
                ('.yaml', '.yml', '.json', '.txt', '.log', '.pdf', '.csv', '.bag', '.zip', '.bin')]

    def state(self):
        with self.lock:
            log = ""
            if self.log_path and self.log_path.exists():
                with self.log_path.open('rb') as stream:
                    stream.seek(max(0, self.log_path.stat().st_size - 16000))
                    log = stream.read().decode(errors='replace')
            return copy.deepcopy({"result": self.result, "progress": self.progress, "log": log, "environment": self.environment})

    def catalog(self):
        entries = []
        for path in sorted(self.directory.glob('*/workflow.json'), reverse=True)[:100]:
            try:
                item = json.loads(path.read_text())
                item['id'] = self.file_id(path.parent)
                item['files'] = self.files(path.parent)
                entries.append(item)
            except (OSError, ValueError):
                continue
        return entries

    @contextmanager
    def task(self, mode, data, stop, *, device=None, directory=None):
        if directory is None:
            directory = self.directory / (time.strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8])
            directory.mkdir(parents=True)
        with self.lock:
            self.progress, self.preview = {}, b''
            self.log_path = directory / 'workflow.log'
            self.result = {'id': self.file_id(directory), 'mode': mode, 'status': 'running', 'request': serializable(data), 'device': serializable(device), 'files': []}
            (directory / 'workflow.json').write_text(json.dumps(self.result, ensure_ascii=False, indent=2))
        try:
            yield directory
            self.result['status'] = 'cancelled' if stop.is_set() else 'complete'
        except Exception as exc:
            self.result.update(status='cancelled' if stop.is_set() else 'failed', error=str(exc))
            raise
        finally:
            with self.lock:
                self.result['files'] = self.files(directory)
                (directory / 'workflow.json').write_text(json.dumps(self.result, ensure_ascii=False, indent=2, allow_nan=False))

    def run(self, command, stop, *, cwd=None):
        with self.log_path.open('ab') as log:
            log.write(('\n运行：' + repr(list(map(str, command))) + '\n').encode()); log.flush()
            process = subprocess.Popen(list(map(str, command)), cwd=cwd, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, env=child_environment(), start_new_session=True)
            cancelled = None
            try:
                while process.poll() is None:
                    if stop.is_set() and cancelled is None:
                        cancelled = time.monotonic()
                        os.killpg(process.pid, signal.SIGINT)
                    if cancelled is not None and time.monotonic() - cancelled > 12:
                        os.killpg(process.pid, signal.SIGKILL)
                    time.sleep(.1)
                code = process.wait()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        if cancelled is not None:
            raise CalibrationError('任务已停止，已有数据和日志已保留')
        if code:
            raise CalibrationError(f'标定后端退出码 {code}，请查看运行日志')

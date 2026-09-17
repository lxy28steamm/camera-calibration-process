"""Guarded Linux adapter for the supplied YCTC schema v2 writer."""
from __future__ import annotations

import ctypes
import hashlib
import json
import math
import os
import stat
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import yaml

from ego_calibration import ego_std
from ego_calibration.backends.ego_std_write import yctc_kalibr_calibration_write as writer
from ego_calibration.backends.std_runner import load_pipeline
from ego_calibration.models import CalibrationError


def digest(data):
    return hashlib.sha256(data).hexdigest()


def credential_configured():
    return bool(os.environ.get('YCTC_XU_UNLOCK_SECRET') or
                os.environ.get('YCTC_XU_UNLOCK_SECRET_FILE') and
                Path(os.environ['YCTC_XU_UNLOCK_SECRET_FILE']).is_file())


def load_secret():
    secret = os.environ.get('YCTC_XU_UNLOCK_SECRET', '')
    if not secret and os.environ.get('YCTC_XU_UNLOCK_SECRET_FILE'):
        try:
            secret = Path(os.environ['YCTC_XU_UNLOCK_SECRET_FILE']).read_text().strip()
        except OSError as exc:
            raise CalibrationError('无法读取本机 Ego-Std 写入凭据文件') from exc
    if not secret or '\x00' in secret:
        raise CalibrationError('请在运行主机配置 Ego-Std 写入凭据文件，再重新检查结果')
    return secret


class UniqueLoader(yaml.SafeLoader):
    """Ambiguous duplicate keys must never reach persistent device storage."""
    def construct_mapping(self, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in result:
                raise CalibrationError('YAML 包含重复字段')
            result[key] = self.construct_object(value_node, deep=deep)
        return result


def validate_result(path):
    raw = path.read_bytes()
    if not raw or len(raw) + 68 > 8192 or b'\x00' in raw:
        raise CalibrationError('YAML 必须非空、不含 NUL，且 UTF-8 编码不得超过 8124 字节')
    try:
        yaml.load(raw.decode('utf-8'), Loader=UniqueLoader)
        pipeline = load_pipeline()
        chain = pipeline.validate_precalibrated_camchain(path)
        json.dumps(chain, allow_nan=False)  # Reject unsupported/non-finite extra values too.
        transforms = []
        for key, value in [('cam0.T_cam_imu', chain['cam0'].get('T_cam_imu')),
                           ('cam1.T_cam_imu', chain['cam1'].get('T_cam_imu')),
                           ('cam1.T_cn_cnm1', chain['cam1']['T_cn_cnm1'])]:
            pipeline.validate_transform(value, name=key)
            matrix = np.asarray(value, dtype=float)
            rotation = matrix[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0) or abs(np.linalg.det(rotation) - 1) > 1e-5:
                raise CalibrationError(f'{key} 不是有效的刚体旋转')
            transforms.append(matrix)
        if not np.allclose(transforms[1], transforms[2] @ transforms[0], atol=1e-5, rtol=0):
            raise CalibrationError('相机—IMU 外参与双目外参链不一致')
        if np.linalg.norm(transforms[2][:3, 3]) <= 1e-9:
            raise CalibrationError('双目基线不能为零')
        if chain['cam0']['resolution'] != chain['cam1']['resolution']:
            raise CalibrationError('左右目标定分辨率不一致')
        for camera in chain.values():
            shift = camera.get('timeshift_cam_imu')
            if type(shift) not in (int, float) or not math.isfinite(shift):
                raise CalibrationError('两目均须包含有限数值 timeshift_cam_imu；仅双目 camchain 不可写入')
        # Run the original payload checks too, without altering the original YAML.
        writer.base.validate_kalibr_yaml_payload(raw)
    except CalibrationError:
        raise
    except Exception as exc:
        raise CalibrationError(f'Ego-Std 联合标定 YAML 无效：{exc}') from exc
    return {'sha256': digest(raw), 'bytes': len(raw), 'calibration': chain,
            'resolution': chain['cam0']['resolution'],
            'baseline_mm': float(np.linalg.norm(transforms[2][:3, 3]) * 1000)}


def require_device(device):
    if not device or device.kind not in ('ego-std', 'ego-std-235') or not device.accessible:
        raise CalibrationError('请先选择可访问的 Ego-Std 相机，此写入流程不适用于 Dex 或 Ego-Lite')
    if not device.identifier.startswith('/dev/') or not device.serial:
        raise CalibrationError('Ego-Std 网页写入需要带 USB 序列号的 Linux V4L2 设备，请重新扫描')


def linux_identity(path):
    node = Path('/sys/class/video4linux') / path.name / 'device'
    for parent in node.resolve().parents:
        if (parent / 'idVendor').is_file() and (parent / 'idProduct').is_file():
            return {key: (parent / name).read_text().strip() for key, name in
                    [('usb_serial', 'serial'), ('vid', 'idVendor'), ('pid', 'idProduct')]}
    raise CalibrationError('无法核对所选设备的物理 USB 身份')


@contextmanager
def connection(device):
    """Pin one file descriptor: reconnecting /dev/videoN cannot redirect chunks."""
    import fcntl
    require_device(device)
    path = Path(device.identifier).resolve(strict=True)
    identity = linux_identity(path)
    if identity['usb_serial'] != device.serial:
        raise CalibrationError('USB 序列号与所选设备不符，请重新扫描')
    fd = os.open(path, os.O_RDWR | os.O_CLOEXEC)
    try:
        opened, current = os.fstat(fd), path.stat()
        if not stat.S_ISCHR(opened.st_mode) or (opened.st_dev, opened.st_ino, opened.st_rdev) != (current.st_dev, current.st_ino, current.st_rdev) or linux_identity(path) != identity:
            raise CalibrationError('设备在打开期间发生变化，请重新扫描')
        class Backend:
            name = 'linux-ioctl-pinned'

            def query(self, unit, selector, request, size, payload=None):
                buffer = (ctypes.c_uint8 * size)()
                if payload is not None:
                    if len(payload) != size:
                        raise CalibrationError('UVC 写入长度不匹配')
                    buffer[:] = payload
                control = ego_std._UvcXuControlQuery(unit, selector, request, size, buffer)
                fcntl.ioctl(fd, ego_std._linux_ioc(3, 'u', 0x21, ctypes.sizeof(control)), control)
                return bytes(buffer)
        backend = Backend()
        backend.identity = identity
        yield backend
    finally:
        os.close(fd)


def arguments():
    return writer.build_arg_parser().parse_args([])


def read_current(backend):
    args = arguments()
    status = writer.base.read_calib_status(backend, args)
    command = writer.read_calib_command(backend, args)
    if status['state'] != 0 or status['status_code'] != 0 or command['write_unlocked']:
        raise CalibrationError('相机有未结束的写入事务或错误状态，请先核实设备状态')
    blob, info = writer.base.read_calibration_blob(backend, args)
    parsed = ego_std._parse_blob(blob)
    try:
        json.dumps(parsed, allow_nan=False)
    except ValueError as exc:
        raise CalibrationError('设备现有标定包含非有限数值，拒绝覆盖，请先核实设备') from exc
    return blob, {'identity': dict(backend.identity), 'sha256': digest(blob), 'info': info,
                  'protocol_version': command['protocol_version'], 'calibration': parsed}


def durable_write(path, data):
    with path.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def backup(directory, blob, current):
    durable_write(directory / 'before.bin', blob)
    durable_write(directory / 'before.json', json.dumps(current, ensure_ascii=False, indent=2).encode())
    text = current['calibration'].get('kalibr_yaml')
    if text:
        durable_write(directory / 'before.yaml', text.encode('utf-8'))


def write_verified(backend, path, serial, directory, secret):
    args = arguments()
    args.write_kalibr_yaml = str(path)
    args.serial_number = serial
    args.unlock_secret = secret
    args.yaml_out = str(directory / 'readback.yaml')
    try:
        result = writer.run_calib_write_kalibr(backend, args)
    except Exception as exc:
        # Keep unlock credentials out of workflow errors even if a backend echoes them.
        message = str(exc).replace(secret, '[已隐藏]')
        raise CalibrationError(f'Ego-Std 写入未确认成功：{message}。备份已保留，请重新读取设备核实；未自动回滚。') from None
    blob = writer.build_kalibr_yaml_blob(path.read_bytes(), serial)
    durable_write(directory / 'verified.bin', blob)
    durable_write(directory / 'write-result.json', json.dumps(result, indent=2).encode())
    return result, ego_std._parse_blob(blob)

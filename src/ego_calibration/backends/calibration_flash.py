#!/usr/bin/env python3
"""Store Kalibr YAML in a vendor region or an explicitly selected trial sector."""
import argparse
import ctypes as ct
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time
import zlib

import yaml

ROOT = Path(__file__).resolve().parent
SECTOR = 4096
HEADER = struct.Struct('<8sHHII')
MAGIC = b'KALIBR01'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def calibration(data):
    value = yaml.safe_load(data.decode('utf-8'))
    if not isinstance(value, dict) or set(value) != {'cam0'}:
        raise ValueError('请选择只包含 cam0 的单目 Kalibr camchain YAML')
    camera = value['cam0']
    if not isinstance(camera, dict):
        raise ValueError('cam0 必须是参数字典')
    models = {'pinhole': (4, 0), 'omni': (5, 1), 'eucm': (6, 2), 'ds': (6, 2)}
    distortion_lengths = {'radtan': 4, 'equidistant': 4, 'fov': 1, 'none': 0}
    model, distortion = camera.get('camera_model'), camera.get('distortion_model')
    if model not in models or distortion not in distortion_lengths:
        raise ValueError('不支持的相机模型或畸变模型')
    expected, focal_index = models[model]
    intrinsics, coefficients = camera.get('intrinsics'), camera.get('distortion_coeffs')
    for name, values, length in [('intrinsics', intrinsics, expected),
                                 ('distortion_coeffs', coefficients, distortion_lengths[distortion])]:
        if not isinstance(values, list) or len(values) != length:
            raise ValueError('%s 的参数个数应为 %d' % (name, length))
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError(name + ' 包含无效数值')
    if min(intrinsics[focal_index:focal_index + 2]) <= 0:
        raise ValueError('焦距必须为正数')
    resolution = camera.get('resolution')
    if not isinstance(resolution, list) or len(resolution) != 2 or any(type(v) is not int or v <= 0 for v in resolution):
        raise ValueError('resolution 必须是两个正整数')
    return camera


def pack(data):
    calibration(data)
    header = HEADER.pack(MAGIC, 1, HEADER.size, len(data), zlib.crc32(data))
    blob = header + data
    return blob + b'\xff' * (-len(blob) % SECTOR)


def unpack(blob):
    if len(blob) < HEADER.size:
        raise ValueError('Flash 数据长度不足')
    magic, version, header_size, length, crc = HEADER.unpack_from(blob)
    if magic != MAGIC or version != 1 or header_size != HEADER.size:
        raise ValueError('指定区域没有本工具格式的标定记录（KALIBR01）')
    if not 0 < length <= len(blob) - HEADER.size:
        raise ValueError('标定记录长度越界')
    data = blob[HEADER.size:HEADER.size + length]
    if zlib.crc32(data) != crc:
        raise ValueError('标定记录 CRC32 校验失败')
    calibration(data)
    return data


def reservation(path):
    raw = Path(path).read_bytes()
    profile = json.loads(raw)
    if not isinstance(profile, dict) or profile.get('format_version') != 1:
        raise ValueError('保留区配置版本应为 1')
    if not isinstance(profile.get('reservation_source'), str) or not profile['reservation_source'].strip():
        raise ValueError('缺少 Flash 区域选择依据或确认记录')
    mode = profile.get('selection_mode', 'vendor_reserved')
    if mode == 'vendor_reserved':
        if profile.get('dedicated_for_calibration') is not True:
            raise ValueError('厂家尚未确认该范围由标定数据独占')
    elif mode == 'user_selected_blank':
        if profile.get('dedicated_for_calibration') is not False:
            raise ValueError('用户指定试写区不能标记为厂家已确认的独占保留区')
    else:
        raise ValueError('不支持的 Flash 区域选择方式')
    match = profile.get('device_match', {})
    if any(type(match.get(k)) is not int for k in ('vid', 'pid', 'bcd_device', 'asic', 'flash_bytes')):
        raise ValueError('保留区配置缺少完整设备匹配参数')
    if (match['vid'], match['pid'], match['asic'], match['flash_bytes']) != (0x1bcf, 0x28c4, 110, 524288):
        raise ValueError('当前写入后端仅支持已检查的 1bcf:28c4 / ASIC 110 / 512 KiB 设备')
    first, count = profile.get('first_sector'), profile.get('sector_count')
    if type(first) is not int or type(count) is not int or first <= 0 or count <= 0 or first + count > 128:
        raise ValueError('起始扇区必须 > 0，范围不得超出 128 个扇区')
    if mode == 'user_selected_blank' and (count != 1 or first == 127):
        raise ValueError('用户指定试写区仅允许一个扇区，并排除扇区 0 和 127')
    if profile.get('write_mode') != 'rom_same_handle':
        raise ValueError('当前后端仅支持 ASIC 110 示例中的 rom_same_handle 写入流程')
    return profile, digest(raw)


def inspect_inputs(yaml_path, profile_path=None):
    result = {'camera': None, 'ready_to_write': False, 'ready_to_read': False}
    blob = None
    if yaml_path:
        data = Path(yaml_path).read_bytes()
        camera, blob = calibration(data), pack(data)
        result.update(camera=camera, yaml_text=data.decode('utf-8'), yaml_sha256=digest(data),
                      payload_bytes=len(data), storage_bytes=len(blob), required_sectors=len(blob) // SECTOR)
    try:
        if not profile_path:
            raise ValueError('未加载 Flash 区域配置；写入尚未启用')
        profile, profile_hash = reservation(profile_path)
        result.update(reservation=profile, profile_sha256=profile_hash, ready_to_read=True)
        if blob is not None and len(blob) > profile['sector_count'] * SECTOR:
            raise ValueError('YAML 超出所选 Flash 区域容量')
        result['ready_to_write'] = blob is not None
    except (OSError, ValueError, TypeError) as exc:
        result['reservation_error'] = str(exc)
    return result


def durable_save(path, data):
    with Path(path).open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    directory = os.open(str(Path(path).parent), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def record_stage(directory, stage, **details):
    # Flush before native SDK calls: a segmentation fault bypasses Python finally.
    entry = dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), stage=stage, **details)
    with (directory / 'stages.jsonl').open('a') as stream:
        stream.write(json.dumps(entry, ensure_ascii=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    print('Flash 阶段：' + stage, flush=True)


class DeviceInfo(ct.Structure):
    _fields_ = [(name, ct.c_uint32) for name in ('vid', 'pid', 'bcd_device', 'asic', 'flash_type', 'flash_bytes')]


class SunplusDevice:
    def __init__(self, path):
        self.handle = ct.c_void_p()
        self.path = str(Path(path).resolve(strict=True))
        self.usb_port = str((Path('/sys/class/video4linux') / Path(self.path).name / 'device').resolve().parent)
        self.lib = ct.CDLL(str(ROOT / 'flash_tools/libcamera_flash.so'))
        self.lib.sp_open.argtypes = [ct.c_char_p, ct.POINTER(ct.c_void_p)]
        self.lib.sp_info.argtypes = [ct.c_void_p, ct.POINTER(DeviceInfo)]
        for operation in ('sp_close', 'sp_rom', 'sp_reset'):
            getattr(self.lib, operation).argtypes = [ct.c_void_p]
        for operation in ('sp_read', 'sp_write'):
            getattr(self.lib, operation).argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(ct.c_uint8)]
        self.check(self.lib.sp_open(os.fsencode(self.path), ct.byref(self.handle)), '连接相机')
        info = DeviceInfo()
        try:
            self.check(self.lib.sp_info(self.handle, ct.byref(info)), '获取相机信息')
            self.info = {name: getattr(info, name) for name, _ in DeviceInfo._fields_}
            self.info['usb_port'] = self.usb_port
        except Exception:
            self.close()
            raise

    @staticmethod
    def check(code, operation):
        if code != 0:
            raise RuntimeError('%s失败，SDK 返回 %d' % (operation, code))

    def close(self):
        if self.handle:
            self.lib.sp_close(self.handle)
            self.handle = ct.c_void_p()

    def read(self, sector):
        buffers = []
        for fill in (0xa5, 0x5a):
            buffer = (ct.c_uint8 * SECTOR)(*([fill] * SECTOR))
            self.check(self.lib.sp_read(self.handle, sector, buffer), '读取扇区 %d' % sector)
            buffers.append(bytes(buffer))
        if buffers[0] != buffers[1]:
            raise RuntimeError('扇区两次读取不一致或读取不完整')
        return buffers[0]

    def write(self, sector, data):
        if len(data) != SECTOR:
            raise ValueError('写入缓冲区必须为 4096 字节')
        buffer = (ct.c_uint8 * SECTOR).from_buffer_copy(data)
        self.check(self.lib.sp_write(self.handle, sector, buffer), '写入扇区 %d' % sector)

    def begin_write(self):
        # Matches the vendor's ASIC 110 example. No additional reset/erase commands.
        self.check(self.lib.sp_rom(self.handle), '进入 ROM 模式')

    def finish_write(self):
        self.check(self.lib.sp_reset(self.handle), '恢复正常模式')


def check_device(info, profile):
    for key, value in profile['device_match'].items():
        if info.get(key) != value:
            raise ValueError('相机 %s 与 Flash 区域配置不匹配' % key)


def reconnect(device_path, expected, factory):
    deadline = time.monotonic() + 20
    while True:
        try:
            device = factory(device_path)
        except (OSError, RuntimeError):
            if time.monotonic() >= deadline:
                raise RuntimeError('相机复位后未重新连接；备份和待写记录已保留')
            time.sleep(0.5)
            continue
        if device.info != expected:
            device.close()
            raise RuntimeError('复位后设备信息或 USB 端口变化，停止回读以避免选错相机')
        return device


def write_calibration(device_path, data, profile, directory, factory=SunplusDevice):
    blob = pack(data)
    count = len(blob) // SECTOR
    if count > profile['sector_count']:
        raise ValueError('标定记录超出保留区')
    device = factory(device_path)
    try:
        check_device(device.info, profile)
        identity = dict(device.info)
        first = profile['first_sector']
        print('读取并备份将写入的 %d 个扇区…' % count, flush=True)
        original = b''.join(device.read(first + i) for i in range(count))
        durable_save(directory / 'original-sectors.bin', original)
        durable_save(directory / 'pending-sectors.bin', blob)
        durable_save(directory / 'source.yaml', data)
        durable_save(directory / 'reservation.json', json.dumps(profile, ensure_ascii=False, indent=2).encode())
        durable_save(directory / 'backup.json', json.dumps({
            'device': identity, 'first_sector': first, 'sector_count': count,
            'original_sha256': digest(original), 'pending_sha256': digest(blob),
        }, ensure_ascii=False, indent=2).encode())
        if profile.get('selection_mode') == 'user_selected_blank' and original != b'\xff' * len(original):
            # Only replace our own intact record; never treat unknown bytes as disposable.
            try:
                if pack(unpack(original)) != original:
                    raise ValueError('记录以外存在其他数据')
            except ValueError as exc:
                raise ValueError('试写扇区含未知或损坏的数据，已备份并停止，未进入 ROM 或写入') from exc
        print('备份已落盘，开始 Flash 写入；请保持连接…', flush=True)
        error = None
        try:
            record_stage(directory, '进入 ROM 模式')
            device.begin_write()
            record_stage(directory, 'ROM 模式已连接')
            for index in range(count):
                record_stage(directory, '写入扇区', sector=first + index)
                device.write(first + index, blob[index * SECTOR:(index + 1) * SECTOR])
                record_stage(directory, '扇区写入已返回', sector=first + index)
                print('已写入 %d / %d 个扇区' % (index + 1, count), flush=True)
        except Exception as exc:
            error = str(exc)
        finally:
            try:
                try:
                    record_stage(directory, '恢复正常模式')
                finally:
                    device.finish_write()
            except Exception as exc:
                error = ((error + '；') if error else '') + str(exc)
        device.close()
        if error:
            raise RuntimeError('写入/复位未成功：%s。未自动回滚，原扇区备份已保留。' % error)
        record_stage(directory, '等待正常相机重连')
        device = reconnect(device_path, identity, factory)
        record_stage(directory, '回读校验')
        readback = b''.join(device.read(first + i) for i in range(count))
        durable_save(directory / 'readback-sectors.bin', readback)
        if readback != blob or unpack(readback) != data:
            raise RuntimeError('写入后回读不一致，不能视为成功；原扇区备份已保留')
        durable_save(directory / 'readback.yaml', unpack(readback))
        record_stage(directory, '写入和回读校验成功')
        return {'status': 'written_and_verified', 'device': identity, 'first_sector': first,
                'written_sectors': count, 'yaml_sha256': digest(data), 'directory': str(directory)}
    finally:
        device.close()


def read_calibration(device_path, profile, directory):
    device = SunplusDevice(device_path)
    try:
        check_device(device.info, profile)
        blob = b''.join(device.read(profile['first_sector'] + i) for i in range(profile['sector_count']))
        durable_save(directory / 'read-sectors.bin', blob)
        data = unpack(blob)
        durable_save(directory / 'camera-camchain.yaml', data)
        return {'status': 'read_and_verified', 'camera': calibration(data),
                'yaml_text': data.decode(), 'yaml_sha256': digest(data), 'directory': str(directory)}
    finally:
        device.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspect', 'probe', 'read', 'write'))
    parser.add_argument('--device')
    parser.add_argument('--yaml', type=Path)
    parser.add_argument('--reservation', type=Path)
    parser.add_argument('--yaml-sha256')
    parser.add_argument('--profile-sha256')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = None
    try:
        if args.action == 'inspect':
            result = inspect_inputs(args.yaml, args.reservation)
        else:
            if not args.device:
                raise ValueError('请指定相机设备')
            lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / ('camera-workbench-flash-%s.lock' % os.getuid())
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            with lock_path.open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                if args.action == 'probe':
                    device = SunplusDevice(args.device)
                    try:
                        result = {'device': device.info}
                    finally:
                        device.close()
                else:
                    profile, profile_hash = reservation(args.reservation)
                    if args.profile_sha256 != profile_hash:
                        raise ValueError('保留区配置未校验或校验后已改变，请重新校验')
                    if args.action == 'read':
                        result = read_calibration(args.device, profile, args.output)
                    else:
                        data = args.yaml.read_bytes()
                        if args.yaml_sha256 != digest(data):
                            raise ValueError('YAML 未校验或校验后已改变，请重新校验')
                        users = subprocess.run(['fuser', str(Path(args.device).resolve(strict=True))],
                                               capture_output=True, text=True)
                        if users.returncode not in (0, 1) or users.stdout.strip():
                            raise ValueError('相机仍被其他进程使用，请先停止预览/录制')
                        result = write_calibration(args.device, data, profile, args.output)
        (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('操作完成：' + str(args.output), flush=True)
    except Exception as exc:
        (args.output / 'result.json').write_text(json.dumps({'error': str(exc)}, ensure_ascii=False) + '\n')
        raise


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit('操作失败：' + str(exc))

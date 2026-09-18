"""Read YCTC Sensor Bridge v3 SN (0x0D) over the matching USB CDC port."""
from __future__ import annotations

import os
import platform
import secrets
import select
import struct
import time
from pathlib import Path

from ego_calibration.ego_std import _crc16_modbus
from ego_calibration.models import CalibrationError


def _usb_parent(path):
    resolved = path.resolve()
    return next((p for p in (resolved, *resolved.parents)
                 if (p / 'idVendor').is_file() and (p / 'idProduct').is_file()), None)


def camera_port(identifier, *, sys_root=Path('/sys'), dev_root=Path('/dev')):
    """Match USB topology, since multiple cameras may share a USB serial."""
    video = Path(identifier).resolve(strict=True)
    parent = _usb_parent(sys_root / 'class/video4linux' / video.name / 'device')
    if parent is None:
        raise CalibrationError('无法定位所选相机的 USB 设备')
    ports = [dev_root / node.name for node in sorted((sys_root / 'class/tty').glob('ttyACM*'))
             if _usb_parent(node / 'device') == parent]
    if len(ports) != 1:
        raise CalibrationError('所选相机没有唯一匹配的 CDC 串口，无法读取相机 SN')
    return ports[0]


def _request(sequence):
    header = struct.pack('<HBBBHHB', 0x5953, 3, 0x0D, 0, sequence, 0, 0)
    return header + struct.pack('<H', _crc16_modbus(header))


def _reply(buffer, sequence):
    while len(buffer) >= 10:
        if buffer[:3] != b'SY\x03':
            del buffer[0]
            continue
        _, _, kind, flags, seq, size, reserved = struct.unpack_from('<HBBBHHB', buffer)
        if reserved or size > 4632 or flags not in (0, 1):
            del buffer[0]
            continue
        total = 12 + size
        if len(buffer) < total:
            return None
        if _crc16_modbus(buffer[:total-2]) != struct.unpack_from('<H', buffer, total-2)[0]:
            del buffer[0]
            continue
        payload = bytes(buffer[10:total-2])
        del buffer[:total]
        if flags != 1 or seq != sequence:
            continue
        if kind == 0x0A and size == 8:
            code, response_to, detail = struct.unpack('<iHH', payload)
            if response_to == 0x0D:
                raise CalibrationError(f'相机 SN 读取失败：code={code}, detail=0x{detail:04X}')
        if kind != 0x0D or size < 3:
            continue
        length = struct.unpack_from('<H', payload)[0]
        if not 1 <= length <= 32 or size != 2 + length:
            continue
        if any(byte < 0x20 or byte > 0x7E for byte in payload[2:]):
            continue
        return payload[2:].decode('ascii')
    return None


def read_camera_serial(identifier, *, timeout=1.5):
    if platform.system() != 'Linux' or not identifier.startswith('/dev/'):
        raise CalibrationError('相机 SN 读取需要 Linux CDC 串口')
    import fcntl
    import termios
    import tty

    port = camera_port(identifier)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    previous = None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        previous = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)
        settings = termios.tcgetattr(fd)
        settings[2] = (settings[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
        settings[4] = settings[5] = termios.B921600
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        sequence = secrets.randbelow(65535) + 1
        pending = _request(sequence)
        buffer = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            readable, writable, _ = select.select([fd], [fd] if pending else [], [],
                                                  max(0, deadline - time.monotonic()))
            try:
                if writable:
                    pending = pending[os.write(fd, pending):]
                if readable:
                    block = os.read(fd, 8192)
                    if not block:
                        raise CalibrationError('相机串口已断开')
                    buffer.extend(block)
                    serial = _reply(buffer, sequence)
                    if serial is not None and not pending:
                        return serial
            except BlockingIOError:
                continue
        raise CalibrationError('读取相机 SN 超时')
    finally:
        try:
            if previous is not None:
                termios.tcsetattr(fd, termios.TCSANOW, previous)
        finally:
            os.close(fd)

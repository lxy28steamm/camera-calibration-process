"""Linux capture-device discovery and explicit per-adapter capabilities."""
from __future__ import annotations
import os
import platform
import struct
from pathlib import Path
from ego_calibration.models import CameraDevice


def query_capture(path):
    import fcntl
    buffer = bytearray(104)  # struct v4l2_capability
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(descriptor, 0x80685600, buffer)
    finally:
        os.close(descriptor)
    capabilities, device_caps = struct.unpack_from('II', buffer, 84)
    caps = device_caps if capabilities & 0x80000000 else capabilities
    return bool(caps & (0x1 | 0x1000))  # VIDEO_CAPTURE / VIDEO_CAPTURE_MPLANE


def scan_uvc_devices(known=()):
    if platform.system() != 'Linux':
        return []
    excluded = {str(Path(d.identifier).resolve()) for d in known if d.identifier.startswith('/dev/')}
    aliases = {str(p.resolve()): p for p in sorted(Path('/dev/v4l/by-id').glob('*'))}
    found = []
    for path in sorted(Path('/dev').glob('video[0-9]*')):
        if str(path.resolve()) in excluded:
            continue
        try:
            if not query_capture(path):
                continue
            sys = Path('/sys/class/video4linux') / path.name
            name = (sys / 'name').read_text().strip()
            node = (sys / 'device').resolve()
            usb = next((p for p in (node, *node.parents) if (p / 'idVendor').is_file()), None)
            def attr(key):
                return (usb / key).read_text().strip() if usb and (usb / key).exists() else ''
            vendor, product, serial = attr('idVendor'), attr('idProduct'), attr('serial')
            kind = 'dex-mono' if (vendor, product) == ('1bcf', '28c4') else 'uvc'
            identifier = str(aliases.get(str(path.resolve()), path))
            found.append(CameraDevice(kind, identifier, f'{name} · {path.name}', name, serial, 'V4L2 / UVC', identifier))
        except PermissionError:
            found.append(CameraDevice('uvc', str(path), f'{path.name} · 无访问权限', transport='V4L2', path=str(path), accessible=False))
        except OSError:
            continue
    return found


def capabilities(device):
    kind = device.kind if device else ''
    return {'mono': kind in ('uvc', 'dex-mono'), 'stereo': kind in ('ego-std', 'ego-std-235', 'ego-lite', 'uvc-stereo'),
            'read_calibration': kind in ('ego-std', 'ego-std-235', 'ego-lite', 'dex-mono'),
            'flash': kind == 'dex-mono', 'health': kind in ('uvc', 'uvc-stereo', 'dex-mono', 'ego-std', 'ego-std-235'),
            'select_layout': kind in ('uvc', 'uvc-stereo')}

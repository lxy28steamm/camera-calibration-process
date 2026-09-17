#!/usr/bin/env python3
"""YCTC_SC233HGS and ZXCZ_SC233HGS_DUAL UVC XU protocol tool.

Implemented according to docs/UVC_XU_PROTOCOL_en.md and
docs/UVC_XU_PROTOCOL_zh.md. The customer-facing
operations are:
  - output synchronization mode get/set
  - stereo exposure range, system gain, and CVBR average bitrate get/set
  - active calibration blob readback and offline parsing
  - recovery mode trigger

Linux uses native UVCIOC_CTRL_QUERY on /dev/videoX. Windows uses the system
UVC driver through DirectShow/KS IKsControl. Other platforms can use the
optional pyusb/libusb backend when the VideoControl interface is accessible.
"""

import argparse
import ctypes
import glob
import hashlib
import json
import os
import platform
import re
import struct
import sys
import textwrap
import uuid
import zlib


DEFAULT_UNIT_ID = 0x0A
DEFAULT_XU_GUID = "a29e7641-de04-47e3-8b2b-f4341aff003b"

SELECTOR_RECOVERY = 0x01
SELECTOR_CALIB_INFO = 0x02
SELECTOR_CALIB_TRANSFER = 0x03
SELECTOR_CALIB_COMMAND = 0x04
SELECTOR_CALIB_STATUS = 0x05
SELECTOR_OUTPUT_MODE = 0x06
SELECTOR_EXPOSURE_TIME = 0x07
SELECTOR_SYSTEM_GAIN = 0x08
SELECTOR_BITRATE = 0x09

UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_MIN = 0x82
UVC_GET_MAX = 0x83
UVC_GET_RES = 0x84
UVC_GET_LEN = 0x85
UVC_GET_INFO = 0x86
UVC_GET_DEF = 0x87

YCTC_XU_CMD_READ_CHUNK = 0x05
YCTC_XU_INFO_LEN = 16
YCTC_XU_STATUS_LEN = 8
YCTC_XU_COMMAND_LEN = 16
YCTC_XU_TRANSFER_LEN = 60
YCTC_XU_TRANSFER_DATA_MAX = 48
U32_CONTROL_LEN = 4
EXPOSURE_RANGE_CONTROL_LEN = 8

OUTPUT_MODE_INTERNAL = 1
OUTPUT_MODE_EXTERNAL = 2

CALIBRATION_BLOB_MAGIC = b"ZXCZ"
CALIBRATION_SCHEMA_V1 = 1
CALIBRATION_SCHEMA_V2 = 2
CALIBRATION_HEADER_SIZE = 64
CALIBRATION_SN_OFFSET = 12
CALIBRATION_SN_SIZE = 32
CALIBRATION_SN_END = CALIBRATION_SN_OFFSET + CALIBRATION_SN_SIZE
CALIBRATION_PAYLOAD_DOUBLE_COUNT = 112
CALIBRATION_PAYLOAD_DOUBLE_BYTES = CALIBRATION_PAYLOAD_DOUBLE_COUNT * 8
KALIBR_SCHEMA_V2_REQUIRED_FIELDS = (
    "cam0:",
    "cam1:",
    "T_cam_imu:",
    "T_cn_cnm1:",
    "timeshift_cam_imu:",
)
SELECTOR_NAMES = {
    SELECTOR_RECOVERY: "RECOVERY",
    SELECTOR_CALIB_INFO: "CALIB_INFO",
    SELECTOR_CALIB_TRANSFER: "CALIB_TRANSFER",
    SELECTOR_CALIB_COMMAND: "CALIB_COMMAND",
    SELECTOR_CALIB_STATUS: "CALIB_STATUS",
    SELECTOR_OUTPUT_MODE: "OUTPUT_MODE",
    SELECTOR_EXPOSURE_TIME: "EXPOSURE_TIME",
    SELECTOR_SYSTEM_GAIN: "SYSTEM_GAIN",
    SELECTOR_BITRATE: "BITRATE",
}

EXPECTED_SELECTOR_LENGTHS = {
    SELECTOR_RECOVERY: 4,
    SELECTOR_CALIB_INFO: 16,
    SELECTOR_CALIB_TRANSFER: 60,
    SELECTOR_CALIB_COMMAND: 16,
    SELECTOR_CALIB_STATUS: 8,
    SELECTOR_OUTPUT_MODE: 1,
    SELECTOR_EXPOSURE_TIME: EXPOSURE_RANGE_CONTROL_LEN,
    SELECTOR_SYSTEM_GAIN: U32_CONTROL_LEN,
    SELECTOR_BITRATE: U32_CONTROL_LEN,
}

U32_CONTROL_SPECS = {
    SELECTOR_EXPOSURE_TIME: {
        "unit": "us",
        "minimum": 250,
        "maximum": 10000,
        "resolution": 1,
        "default": 10000,
    },
    SELECTOR_SYSTEM_GAIN: {
        "unit": "22.10 fixed-point",
        "minimum": 0x400,
        "maximum": 0xFFFFFFFF,
        "resolution": 1,
        "default": 0x400,
    },
    SELECTOR_BITRATE: {
        "unit": "Kbps",
        "minimum": 4096,
        "maximum": 65536,
        "resolution": 1,
        "default": 12288,
    },
}

U32_CAPABILITY_QUERIES = (
    ("minimum", UVC_GET_MIN),
    ("maximum", UVC_GET_MAX),
    ("resolution", UVC_GET_RES),
    ("default", UVC_GET_DEF),
)

U32_CAPABILITY_QUERY_FIELDS = dict((query, field) for field, query in U32_CAPABILITY_QUERIES)


class UvcXuError(RuntimeError):
    pass


class UvcXuControlQuery(ctypes.Structure):
    _fields_ = [
        ("unit", ctypes.c_uint8),
        ("selector", ctypes.c_uint8),
        ("query", ctypes.c_uint8),
        ("size", ctypes.c_uint16),
        ("data", ctypes.POINTER(ctypes.c_uint8)),
    ]


def parse_int(text):
    try:
        return int(text, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("invalid integer: {}".format(text)) from exc


def parse_u8(text):
    value = parse_int(text)
    if value < 0 or value > 0xFF:
        raise argparse.ArgumentTypeError("value out of uint8 range: {}".format(text))
    return value


def parse_u16(text):
    value = parse_int(text)
    if value < 0 or value > 0xFFFF:
        raise argparse.ArgumentTypeError("value out of uint16 range: {}".format(text))
    return value


def parse_u32(text):
    value = parse_int(text)
    if value < 0 or value > 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("value out of uint32 range: {}".format(text))
    return value


def parse_guid(text):
    try:
        return str(uuid.UUID(str(text).strip().strip("{}")))
    except (ValueError, AttributeError) as exc:
        raise argparse.ArgumentTypeError("invalid GUID: {}".format(text)) from exc


def parse_output_mode(text):
    normalized = str(text).strip().lower()
    if normalized in ("1", "internal", "in"):
        return OUTPUT_MODE_INTERNAL
    if normalized in ("2", "external", "ex"):
        return OUTPUT_MODE_EXTERNAL
    raise argparse.ArgumentTypeError("output mode must be internal/1 or external/2")


def output_mode_name(mode):
    if int(mode) == OUTPUT_MODE_INTERNAL:
        return "INTERNAL"
    if int(mode) == OUTPUT_MODE_EXTERNAL:
        return "EXTERNAL"
    return "UNKNOWN"


def md5_hex(data):
    return hashlib.md5(data).hexdigest()


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def normalize_linux_video_device(device):
    if device is None:
        return None
    if device.startswith("/dev/"):
        return device
    if re.match(r"^video[0-9]+$", device):
        return "/dev/{}".format(device)
    return device


def read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fp:
            return fp.read().strip()
    except OSError:
        return None


def decode_fixed_text(data):
    return bytes(data).split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def is_valid_serial_field(data):
    has_text = False
    for byte in bytes(data):
        if byte == 0:
            return has_text
        if byte < 0x21 or byte > 0x7E:
            return False
        has_text = True
    return has_text


def reshape(values, rows, cols):
    return [
        [float(values[row * cols + col]) for col in range(cols)]
        for row in range(rows)
    ]


def validate_kalibr_yaml_payload(payload):
    data = bytes(payload)
    if not data:
        raise ValueError("Kalibr YAML payload is empty")
    if b"\x00" in data:
        raise ValueError("Kalibr YAML payload contains a NUL byte")
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("Kalibr YAML payload is not valid UTF-8") from exc

    missing = [field for field in KALIBR_SCHEMA_V2_REQUIRED_FIELDS if field.encode("ascii") not in data]
    if missing:
        raise ValueError("Kalibr YAML payload is missing required fields: {}".format(", ".join(missing)))
    return text


def parse_calibration_blob(blob, strict_stereo_payload=False):
    data = bytes(blob)
    if len(data) < CALIBRATION_HEADER_SIZE + 4:
        raise ValueError("blob is too short")

    magic, schema_version = struct.unpack_from("<4sH", data, 0)
    if magic != CALIBRATION_BLOB_MAGIC:
        raise ValueError("invalid blob magic: {!r}".format(magic))
    if int(schema_version) not in (CALIBRATION_SCHEMA_V1, CALIBRATION_SCHEMA_V2):
        raise ValueError("unsupported schema version: {}".format(schema_version))

    payload_length = struct.unpack_from("<I", data, 8)[0]
    expected_total = CALIBRATION_HEADER_SIZE + int(payload_length) + 4
    if len(data) != expected_total:
        raise ValueError("blob length mismatch: {} != {}".format(len(data), expected_total))

    stored_crc32 = struct.unpack_from("<I", data, len(data) - 4)[0]
    actual_crc32 = zlib.crc32(data[:-4]) & 0xFFFFFFFF
    if int(stored_crc32) != int(actual_crc32):
        raise ValueError("blob tail crc32 mismatch: {} != {}".format(stored_crc32, actual_crc32))

    serial_field = data[CALIBRATION_SN_OFFSET:CALIBRATION_SN_END]
    reserved = data[CALIBRATION_SN_END:CALIBRATION_HEADER_SIZE]
    firmware_sn_header = is_valid_serial_field(serial_field) and reserved == (b"\x00" * len(reserved))
    if firmware_sn_header:
        serial_number = decode_fixed_text(serial_field)
        header_format = "firmware_sn"
        header_sample_count = 0
        device_model = ""
        lens_type = ""
        tool_version = ""
    else:
        serial_number = ""
        header_format = "legacy_metadata"
        header_sample_count = int(struct.unpack_from("<I", data, 12)[0])
        device_model = decode_fixed_text(data[16:32])
        lens_type = decode_fixed_text(data[32:48])
        tool_version = decode_fixed_text(data[48:64])

    payload = data[CALIBRATION_HEADER_SIZE:-4]
    if int(schema_version) == CALIBRATION_SCHEMA_V2:
        if strict_stereo_payload:
            raise ValueError("--strict-parse only supports the schema v1 stereo-double payload")
        yaml_text = validate_kalibr_yaml_payload(payload)
        return {
            "format": "kalibr_camchain_imucam",
            "header": {
                "magic": magic.decode("ascii"),
                "schema_version": int(schema_version),
                "payload_length": int(payload_length),
                "serial_number": serial_number,
                "header_format": header_format,
                "header_sample_count": header_sample_count,
                "device_model": device_model,
                "lens_type": lens_type,
                "tool_version": tool_version,
                "blob_crc32": int(stored_crc32),
            },
            "payload": {
                "length": int(payload_length),
                "parser": "kalibr_yaml_v2",
                "encoding": "utf-8",
                "sha256": hashlib.sha256(payload).hexdigest(),
                "required_fields": list(KALIBR_SCHEMA_V2_REQUIRED_FIELDS),
            },
            "kalibr_yaml": yaml_text,
        }

    parsed = {
        "format": "stereo_calibration",
        "header": {
            "magic": magic.decode("ascii"),
            "schema_version": int(schema_version),
            "payload_length": int(payload_length),
            "serial_number": serial_number,
            "header_format": header_format,
            "header_sample_count": header_sample_count,
            "device_model": device_model,
            "lens_type": lens_type,
            "tool_version": tool_version,
            "blob_crc32": int(stored_crc32),
        },
        "payload": {
            "length": int(payload_length),
            "parser": "stereo_double_v1"
            if int(payload_length) == CALIBRATION_PAYLOAD_DOUBLE_BYTES else "unparsed",
        },
        "calibration": {},
        "metrics": {},
    }

    if len(payload) != CALIBRATION_PAYLOAD_DOUBLE_BYTES:
        if strict_stereo_payload:
            raise ValueError(
                "payload length mismatch: {} != {}".format(len(payload), CALIBRATION_PAYLOAD_DOUBLE_BYTES)
            )
        parsed["payload"]["warning"] = "unsupported payload length; binary readback is still valid"
        return parsed

    values = list(struct.unpack("<" + ("d" * CALIBRATION_PAYLOAD_DOUBLE_COUNT), payload))
    cursor = 0

    def take(count):
        nonlocal cursor
        chunk = values[cursor:cursor + count]
        if len(chunk) != count:
            raise ValueError("payload truncated at {}, expected {} values".format(cursor, count))
        cursor += count
        return chunk

    parsed["calibration"]["K1"] = reshape(take(9), 3, 3)
    parsed["calibration"]["D1"] = [float(value) for value in take(8)]
    parsed["calibration"]["K2"] = reshape(take(9), 3, 3)
    parsed["calibration"]["D2"] = [float(value) for value in take(8)]
    parsed["calibration"]["R"] = reshape(take(9), 3, 3)
    parsed["calibration"]["T"] = reshape(take(3), 3, 1)
    parsed["calibration"]["R1"] = reshape(take(9), 3, 3)
    parsed["calibration"]["R2"] = reshape(take(9), 3, 3)
    parsed["calibration"]["P1"] = reshape(take(12), 3, 4)
    parsed["calibration"]["P2"] = reshape(take(12), 3, 4)
    parsed["calibration"]["Q"] = reshape(take(16), 4, 4)
    parsed["metrics"]["stereo_rms"] = float(take(1)[0])
    parsed["metrics"]["left_calibrate_rms"] = float(take(1)[0])
    parsed["metrics"]["right_calibrate_rms"] = float(take(1)[0])
    parsed["metrics"]["sample_count"] = float(take(1)[0])
    parsed["metrics"]["baseline_mm"] = float(take(1)[0])
    parsed["metrics"]["yaw_deg"] = float(take(1)[0])
    parsed["metrics"]["pitch_deg"] = float(take(1)[0])
    parsed["metrics"]["roll_deg"] = float(take(1)[0])

    if cursor != len(values):
        raise ValueError("payload parse did not consume all values: {} != {}".format(cursor, len(values)))
    return parsed


def extract_kalibr_yaml_payload(blob):
    data = bytes(blob)
    parsed = parse_calibration_blob(data)
    if int(parsed["header"]["schema_version"]) != CALIBRATION_SCHEMA_V2:
        raise ValueError("blob schema is {}, not Kalibr YAML schema v2".format(parsed["header"]["schema_version"]))
    payload = data[CALIBRATION_HEADER_SIZE:-4]
    validate_kalibr_yaml_payload(payload)
    return payload


def default_json_output_path(binary_path):
    root, _ext = os.path.splitext(os.path.abspath(binary_path))
    return root + ".json"


def write_parsed_json(blob, json_path, strict_stereo_payload=False):
    parsed = parse_calibration_blob(blob, strict_stereo_payload=strict_stereo_payload)
    out_dir = os.path.dirname(os.path.abspath(json_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as fp:
        json.dump(parsed, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    return parsed


def linux_ioc(direction, type_char, nr, size):
    ioc_nrbits = 8
    ioc_typebits = 8
    ioc_sizebits = 14
    ioc_nrshift = 0
    ioc_typeshift = ioc_nrshift + ioc_nrbits
    ioc_sizeshift = ioc_typeshift + ioc_typebits
    ioc_dirshift = ioc_sizeshift + ioc_sizebits
    return (
        (direction << ioc_dirshift)
        | (ord(type_char) << ioc_typeshift)
        | (nr << ioc_nrshift)
        | (size << ioc_sizeshift)
    )


def linux_uvcioc_ctrl_query():
    ioc_write = 1
    ioc_read = 2
    return linux_ioc(ioc_read | ioc_write, "u", 0x21, ctypes.sizeof(UvcXuControlQuery))


def linux_xu_query(video_device, unit, selector, query, size, payload=None):
    import fcntl

    buf_len = max(size, 1)
    buf = (ctypes.c_uint8 * buf_len)()
    if payload is not None:
        if len(payload) != size:
            raise UvcXuError("payload length {} does not match query size {}".format(len(payload), size))
        for idx, value in enumerate(payload):
            buf[idx] = value

    q = UvcXuControlQuery(unit, selector, query, size, buf)
    fd = os.open(video_device, os.O_RDWR)
    try:
        fcntl.ioctl(fd, linux_uvcioc_ctrl_query(), q, True)
    finally:
        os.close(fd)
    return bytes(buf[:size])


def linux_video_devices():
    def key(path):
        suffix = re.sub(r"^\D+", "", os.path.basename(path))
        return int(suffix) if suffix.isdigit() else 0

    return sorted(glob.glob("/dev/video*"), key=key)


def linux_video_sysfs_dir(video_device):
    base = os.path.basename(os.path.realpath(video_device))
    path = "/sys/class/video4linux/{}/device".format(base)
    if not os.path.exists(path):
        return None
    return os.path.realpath(path)


def linux_video_metadata(video_device):
    base = os.path.basename(os.path.realpath(video_device))
    meta = {
        "path": video_device,
        "name": read_text("/sys/class/video4linux/{}/name".format(base)),
        "vid": None,
        "pid": None,
    }

    sys_dir = linux_video_sysfs_dir(video_device)
    while sys_dir and sys_dir != "/":
        id_vendor = read_text(os.path.join(sys_dir, "idVendor"))
        id_product = read_text(os.path.join(sys_dir, "idProduct"))
        if id_vendor is not None and id_product is not None:
            try:
                meta["vid"] = int(id_vendor, 16)
                meta["pid"] = int(id_product, 16)
            except ValueError:
                pass
            break
        parent = os.path.dirname(sys_dir)
        if parent == sys_dir:
            break
        sys_dir = parent
    return meta


def expected_selector_for_args(args):
    override = getattr(args, "xu_expected_selector_override", None)
    if override is not None:
        return int(override)
    if args.recovery:
        return SELECTOR_RECOVERY
    if args.get_output_mode or args.set_output_mode is not None:
        return SELECTOR_OUTPUT_MODE
    if (args.get_exposure_time or args.set_exposure_time_us is not None or
            args.set_exposure_range_us is not None):
        return SELECTOR_EXPOSURE_TIME
    if args.get_system_gain or args.set_system_gain is not None:
        return SELECTOR_SYSTEM_GAIN
    if args.get_bitrate or args.set_bitrate_kbps is not None:
        return SELECTOR_BITRATE
    return SELECTOR_CALIB_INFO


def expected_selector_requires_set(args):
    if getattr(args, "xu_expected_selector_requires_set", False):
        return True
    return bool(
        args.recovery
        or args.set_output_mode is not None
        or args.set_exposure_time_us is not None
        or args.set_exposure_range_us is not None
        or args.set_system_gain is not None
        or args.set_bitrate_kbps is not None
    )


def probe_selector(query_func, unit, selector):
    length_raw = query_func(unit, selector, UVC_GET_LEN, 2)
    info_raw = query_func(unit, selector, UVC_GET_INFO, 1)
    length = struct.unpack("<H", length_raw)[0]
    info = info_raw[0]
    expected_len = EXPECTED_SELECTOR_LENGTHS.get(selector)
    return {
        "selector": selector,
        "name": SELECTOR_NAMES.get(selector, "UNKNOWN"),
        "length": length,
        "expected_length": expected_len,
        "info": info,
        "get_supported": bool(info & 0x01),
        "set_supported": bool(info & 0x02),
        "length_ok": expected_len is None or expected_len == length,
    }


def selector_probe_summary(query_func, unit):
    result = {}
    for selector in sorted(SELECTOR_NAMES):
        name = SELECTOR_NAMES[selector].lower()
        try:
            result[name] = probe_selector(query_func, unit, selector)
        except Exception as exc:
            result[name] = {
                "selector": selector,
                "name": SELECTOR_NAMES[selector],
                "error": str(exc),
            }
    return result


class LinuxIoctlBackend:
    name = "linux-ioctl"

    def __init__(self, args):
        self.args = args
        self.target = None

    def query(self, unit, selector, query, size, payload=None):
        if self.target is None:
            self.target = self.select_target()
        return linux_xu_query(self.target["path"], unit, selector, query, size, payload)

    def close(self):
        return

    def _query_on_device(self, video_device):
        return lambda unit, selector, query, size: linux_xu_query(video_device, unit, selector, query, size)

    def probe(self, video_device):
        meta = linux_video_metadata(video_device)
        meta.update({"selector_probe": {}, "xu_supported": False, "xu_error": ""})
        if self.args.vid is not None and meta["vid"] != self.args.vid:
            return None
        if self.args.pid is not None and meta["pid"] != self.args.pid:
            return None
        if self.args.skip_probe:
            meta["xu_supported"] = True
            return meta

        expected_selector = expected_selector_for_args(self.args)
        query_func = self._query_on_device(video_device)
        try:
            probe = probe_selector(query_func, self.args.unit_id, expected_selector)
            meta["selector_probe"][SELECTOR_NAMES[expected_selector].lower()] = probe
            meta["xu_supported"] = bool(probe["length_ok"] and (probe["info"] & 0x01))
            if expected_selector_requires_set(self.args):
                meta["xu_supported"] = meta["xu_supported"] and bool(probe["info"] & 0x02)
        except Exception as exc:
            meta["xu_error"] = "{}: {}".format(exc.__class__.__name__, exc)
        return meta

    def list_targets(self):
        if self.args.device:
            device = normalize_linux_video_device(self.args.device)
            entry = self.probe(device)
            return [] if entry is None else [entry]

        entries = []
        for device in linux_video_devices():
            entry = self.probe(device)
            if entry is not None:
                entries.append(entry)
        return entries

    def select_target(self):
        entries = self.list_targets()
        if self.args.list:
            return None

        if self.args.index is not None:
            if self.args.index < 0 or self.args.index >= len(entries):
                print_target_list(entries, stream=sys.stderr)
                raise UvcXuError("--index is out of range")
            entry = entries[self.args.index]
            if not self.args.skip_probe and not entry.get("xu_supported"):
                print_target_list([entry], stream=sys.stderr)
                raise UvcXuError("selected Linux video node does not report the requested YCTC XU selector")
            return entry

        if self.args.device:
            if not entries:
                raise UvcXuError("selected Linux video node does not match filters")
            entry = entries[0]
            if not self.args.skip_probe and not entry.get("xu_supported"):
                print_target_list([entry], stream=sys.stderr)
                raise UvcXuError("selected Linux video node does not report the requested YCTC XU selector")
            return entry

        supported = [entry for entry in entries if entry.get("xu_supported")]
        if len(supported) == 1:
            return supported[0]
        print_target_list(entries, stream=sys.stderr)
        if not supported:
            raise UvcXuError("no Linux video node reported the requested YCTC XU selector")
        raise UvcXuError("multiple matching Linux video nodes; pass --device or --index")


class PyUsbBackend:
    name = "pyusb"

    def __init__(self, args):
        self.args = args
        self.target = None
        self.claimed = False
        try:
            import usb.core
            import usb.util
            import usb.backend.libusb1
        except ImportError as exc:
            raise UvcXuError("pyusb backend requires: python -m pip install pyusb") from exc
        self.usb_core = usb.core
        self.usb_util = usb.util
        self.usb_backend = usb.backend.libusb1.get_backend()
        if self.usb_backend is None:
            try:
                import libusb_package
                self.usb_backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
            except ImportError:
                self.usb_backend = None
        if self.usb_backend is None:
            raise UvcXuError("pyusb backend cannot find libusb; install libusb or python -m pip install libusb-package")

    def _iter_candidates(self):
        candidates = []
        for dev in self.usb_core.find(find_all=True, backend=self.usb_backend):
            vid = int(dev.idVendor)
            pid = int(dev.idProduct)
            if self.args.vid is not None and vid != self.args.vid:
                continue
            if self.args.pid is not None and pid != self.args.pid:
                continue
            if self.args.bus is not None and getattr(dev, "bus", None) != self.args.bus:
                continue
            if self.args.address is not None and getattr(dev, "address", None) != self.args.address:
                continue
            try:
                for cfg in dev:
                    for intf in cfg:
                        if int(intf.bInterfaceClass) != 0x0E or int(intf.bInterfaceSubClass) != 0x01:
                            continue
                        interface_number = int(intf.bInterfaceNumber)
                        if self.args.interface is not None and interface_number != self.args.interface:
                            continue
                        candidates.append({
                            "backend": self.name,
                            "dev": dev,
                            "vid": vid,
                            "pid": pid,
                            "bus": getattr(dev, "bus", None),
                            "address": getattr(dev, "address", None),
                            "configuration": int(cfg.bConfigurationValue),
                            "interface": interface_number,
                            "name": "",
                            "xu_supported": True,
                        })
            except Exception:
                continue
        return candidates

    def list_targets(self):
        return self._iter_candidates()

    def select_target(self):
        entries = self.list_targets()
        if self.args.list:
            return None
        if self.args.index is not None:
            if self.args.index < 0 or self.args.index >= len(entries):
                print_target_list(entries, stream=sys.stderr)
                raise UvcXuError("--index is out of range")
            return entries[self.args.index]
        if len(entries) == 1:
            return entries[0]
        print_target_list(entries, stream=sys.stderr)
        if not entries:
            raise UvcXuError("no pyusb-accessible UVC VideoControl interface found")
        raise UvcXuError("multiple pyusb candidates; pass --vid/--pid, --interface, or --index")

    def ensure_interface_ready(self, dev, interface_number):
        if self.claimed or self.args.no_claim:
            return
        try:
            if platform.system() != "Windows" and dev.is_kernel_driver_active(interface_number):
                dev.detach_kernel_driver(interface_number)
        except (NotImplementedError, AttributeError):
            pass
        try:
            self.usb_util.claim_interface(dev, interface_number)
            self.claimed = True
        except Exception:
            self.claimed = False

    def query(self, unit, selector, query, size, payload=None):
        if self.target is None:
            self.target = self.select_target()
        dev = self.target["dev"]
        interface_number = self.target["interface"]
        request_type = 0xA1 if query & 0x80 else 0x21
        w_value = int(selector) << 8
        w_index = (int(unit) << 8) | int(interface_number)
        try:
            self.ensure_interface_ready(dev, interface_number)
            if query & 0x80:
                data = dev.ctrl_transfer(request_type, int(query), w_value, w_index, int(size),
                                         timeout=self.args.timeout_ms)
                return bytes(data)
            if payload is None or len(payload) != size:
                raise UvcXuError("SET_CUR payload length must be {}".format(size))
            written = dev.ctrl_transfer(request_type, int(query), w_value, w_index, payload,
                                        timeout=self.args.timeout_ms)
            if int(written) != int(size):
                raise UvcXuError("short SET_CUR write: {} != {}".format(written, size))
            return b""
        except Exception as exc:
            hint = ""
            if platform.system() == "Windows":
                hint = "; on Windows pyusb needs a libusb/WinUSB-compatible driver for the device/interface"
            raise UvcXuError("pyusb control transfer failed: {}{}".format(exc, hint)) from exc

    def close(self):
        if self.target is None or not self.claimed:
            return
        try:
            self.usb_util.release_interface(self.target["dev"], self.target["interface"])
        except Exception:
            pass
        self.claimed = False


class WindowsKsBackend:
    name = "windows-ks"

    KSPROPERTY_TYPE_GET = 0x00000001
    KSPROPERTY_TYPE_SET = 0x00000002
    KSPROPERTY_TYPE_BASICSUPPORT = 0x00000200
    KSPROPERTY_TYPE_TOPOLOGY = 0x10000000

    def __init__(self, args):
        if platform.system() != "Windows":
            raise UvcXuError("windows-ks backend is only available on Windows")
        self.args = args
        self.target = None
        self.com = self._load_com_types()
        self.xu_guid = self.com["GUID"]("{" + parse_guid(args.xu_guid or DEFAULT_XU_GUID) + "}")

    def _load_com_types(self):
        try:
            import comtypes
            from comtypes import GUID, IUnknown, COMMETHOD, HRESULT
            from comtypes.client import CreateObject
        except ImportError as exc:
            raise UvcXuError("windows-ks backend requires: python -m pip install comtypes") from exc

        POINTER = ctypes.POINTER
        c_ulong = ctypes.c_ulong
        c_longlong = ctypes.c_longlong
        c_void_p = ctypes.c_void_p

        class IBindCtx(IUnknown):
            _iid_ = GUID("{0000000e-0000-0000-C000-000000000046}")
            _methods_ = []

        class IStream(IUnknown):
            _iid_ = GUID("{0000000c-0000-0000-C000-000000000046}")
            _methods_ = []

        class IMoniker(IUnknown):
            _iid_ = GUID("{0000000f-0000-0000-C000-000000000046}")
            _methods_ = []

        class IEnumMoniker(IUnknown):
            _iid_ = GUID("{00000102-0000-0000-C000-000000000046}")
            _methods_ = []

        IEnumMoniker._methods_ = [
            COMMETHOD([], HRESULT, "Next",
                      (["in"], c_ulong, "celt"),
                      (["out"], POINTER(POINTER(IMoniker)), "rgelt"),
                      (["out"], POINTER(c_ulong), "pceltFetched")),
            COMMETHOD([], HRESULT, "Skip", (["in"], c_ulong, "celt")),
            COMMETHOD([], HRESULT, "Reset"),
            COMMETHOD([], HRESULT, "Clone", (["out"], POINTER(POINTER(IEnumMoniker)), "ppenum")),
        ]

        IMoniker._methods_ = [
            COMMETHOD([], HRESULT, "GetClassID", (["out"], POINTER(GUID), "pClassID")),
            COMMETHOD([], HRESULT, "IsDirty"),
            COMMETHOD([], HRESULT, "Load", (["in"], POINTER(IStream), "pStm")),
            COMMETHOD([], HRESULT, "Save",
                      (["in"], POINTER(IStream), "pStm"),
                      (["in"], ctypes.c_int, "fClearDirty")),
            COMMETHOD([], HRESULT, "GetSizeMax", (["out"], POINTER(c_longlong), "pcbSize")),
            COMMETHOD([], HRESULT, "BindToObject",
                      (["in"], POINTER(IBindCtx), "pbc"),
                      (["in"], POINTER(IMoniker), "pmkToLeft"),
                      (["in"], POINTER(GUID), "riidResult"),
                      (["out"], POINTER(POINTER(IUnknown)), "ppvResult")),
            COMMETHOD([], HRESULT, "BindToStorage",
                      (["in"], POINTER(IBindCtx), "pbc"),
                      (["in"], POINTER(IMoniker), "pmkToLeft"),
                      (["in"], POINTER(GUID), "riid"),
                      (["out"], POINTER(POINTER(IUnknown)), "ppvObj")),
            COMMETHOD([], HRESULT, "Reduce"),
            COMMETHOD([], HRESULT, "ComposeWith"),
            COMMETHOD([], HRESULT, "Enum"),
            COMMETHOD([], HRESULT, "IsEqual"),
            COMMETHOD([], HRESULT, "Hash"),
            COMMETHOD([], HRESULT, "IsRunning"),
            COMMETHOD([], HRESULT, "GetTimeOfLastChange"),
            COMMETHOD([], HRESULT, "Inverse"),
            COMMETHOD([], HRESULT, "CommonPrefixWith"),
            COMMETHOD([], HRESULT, "RelativePathTo"),
            COMMETHOD([], HRESULT, "GetDisplayName",
                      (["in"], POINTER(IBindCtx), "pbc"),
                      (["in"], POINTER(IMoniker), "pmkToLeft"),
                      (["out"], POINTER(ctypes.c_wchar_p), "ppszDisplayName")),
        ]

        class ICreateDevEnum(IUnknown):
            _iid_ = GUID("{29840822-5B84-11D0-BD3B-00A0C911CE86}")
            _methods_ = [
                COMMETHOD([], HRESULT, "CreateClassEnumerator",
                          (["in"], POINTER(GUID), "clsidDeviceClass"),
                          (["out"], POINTER(POINTER(IEnumMoniker)), "ppEnumMoniker"),
                          (["in"], c_ulong, "dwFlags")),
            ]

        class KsTopologyConnection(ctypes.Structure):
            _fields_ = [
                ("from_node", c_ulong),
                ("from_node_pin", c_ulong),
                ("to_node", c_ulong),
                ("to_node_pin", c_ulong),
            ]

        class IKsTopologyInfo(IUnknown):
            _iid_ = GUID("{720D4AC0-7533-11D0-A5D6-28DB04C10000}")
            _methods_ = [
                COMMETHOD([], HRESULT, "get_NumCategories", (["out"], POINTER(c_ulong), "pdwNumCategories")),
                COMMETHOD([], HRESULT, "get_Category",
                          (["in"], c_ulong, "dwIndex"),
                          (["out"], POINTER(GUID), "pCategory")),
                COMMETHOD([], HRESULT, "get_NumConnections", (["out"], POINTER(c_ulong), "pdwNumConnections")),
                COMMETHOD([], HRESULT, "get_ConnectionInfo",
                          (["in"], c_ulong, "dwIndex"),
                          (["out"], POINTER(KsTopologyConnection), "pConnectionInfo")),
                COMMETHOD([], HRESULT, "get_NodeName",
                          (["in"], c_ulong, "dwNodeId"),
                          (["out"], ctypes.c_wchar_p, "pwchNodeName"),
                          (["in"], c_ulong, "dwBufSize"),
                          (["out"], POINTER(c_ulong), "pdwNameLen")),
                COMMETHOD([], HRESULT, "get_NumNodes", (["out"], POINTER(c_ulong), "pdwNumNodes")),
                COMMETHOD([], HRESULT, "get_NodeType",
                          (["in"], c_ulong, "dwNodeId"),
                          (["out"], POINTER(GUID), "pNodeType")),
                COMMETHOD([], HRESULT, "CreateNodeInstance",
                          (["in"], c_ulong, "dwNodeId"),
                          (["in"], POINTER(GUID), "iid"),
                          (["out"], POINTER(POINTER(IUnknown)), "ppvObject")),
            ]

        class KsProperty(ctypes.Structure):
            _fields_ = [
                ("set", GUID),
                ("id", c_ulong),
                ("flags", c_ulong),
            ]

        class KsNodeProperty(ctypes.Structure):
            _fields_ = [
                ("property", KsProperty),
                ("node_id", c_ulong),
                ("reserved", c_ulong),
            ]

        class IKsControl(IUnknown):
            _iid_ = GUID("{28F54685-06FD-11D2-B27A-00A0C9223196}")
            _methods_ = [
                COMMETHOD([], HRESULT, "KsProperty",
                          (["in"], POINTER(KsProperty), "Property"),
                          (["in"], c_ulong, "PropertyLength"),
                          (["in", "out"], c_void_p, "PropertyData"),
                          (["in"], c_ulong, "DataLength"),
                          (["out"], POINTER(c_ulong), "BytesReturned")),
                COMMETHOD([], HRESULT, "KsMethod"),
                COMMETHOD([], HRESULT, "KsEvent"),
            ]

        return {
            "GUID": GUID,
            "IBindCtx": IBindCtx,
            "IUnknown": IUnknown,
            "CreateObject": CreateObject,
            "ICreateDevEnum": ICreateDevEnum,
            "IEnumMoniker": IEnumMoniker,
            "IKsTopologyInfo": IKsTopologyInfo,
            "IKsControl": IKsControl,
            "KsProperty": KsProperty,
            "KsNodeProperty": KsNodeProperty,
            "CLSID_SystemDeviceEnum": GUID("{62BE5D10-60EB-11d0-BD3B-00A0C911CE86}"),
            "CLSID_VideoInputDeviceCategory": GUID("{860BB310-5D01-11d0-BD3B-00A0C911CE86}"),
        }

    def _format_com_error(self, exc):
        hresult = getattr(exc, "hresult", None)
        if hresult is None and getattr(exc, "args", None):
            hresult = exc.args[0]
        if isinstance(hresult, int):
            return "HRESULT 0x{:08x}: {}".format(hresult & 0xFFFFFFFF, exc)
        return str(exc)

    def _new_node_property(self, node_id, selector, flags):
        node = self.com["KsNodeProperty"](
            self.com["KsProperty"](self.xu_guid, int(selector), int(flags) | self.KSPROPERTY_TYPE_TOPOLOGY),
            int(node_id),
            0,
        )
        return node

    def _ks_property(self, ks, node_id, selector, flags, size, payload=None):
        buf_len = max(int(size), 1)
        buf = (ctypes.c_ubyte * buf_len)()
        if payload is not None:
            if len(payload) != size:
                raise UvcXuError("payload length {} does not match query size {}".format(len(payload), size))
            for idx, value in enumerate(payload):
                buf[idx] = value

        node = self._new_node_property(node_id, selector, flags)
        try:
            ks.KsProperty(
                ctypes.cast(ctypes.byref(node), ctypes.POINTER(self.com["KsProperty"])),
                ctypes.sizeof(node),
                ctypes.cast(buf, ctypes.c_void_p),
                int(size),
            )
        except Exception as exc:
            raise UvcXuError("Windows KS XU query failed: {}".format(self._format_com_error(exc))) from exc
        return bytes(buf[:size])

    def _basic_support(self, ks, node_id, selector):
        raw = self._ks_property(
            ks,
            node_id,
            selector,
            self.KSPROPERTY_TYPE_BASICSUPPORT,
            4,
        )
        return struct.unpack("<I", raw)[0]

    def _uvc_info_from_basic_support(self, flags):
        info = 0
        if flags & self.KSPROPERTY_TYPE_GET:
            info |= 0x01
        if flags & self.KSPROPERTY_TYPE_SET:
            info |= 0x02
        return info

    def _create_bind_ctx(self):
        bind_ctx = ctypes.POINTER(self.com["IBindCtx"])()
        hr = ctypes.oledll.ole32.CreateBindCtx(0, ctypes.byref(bind_ctx))
        if hr != 0:
            raise UvcXuError("CreateBindCtx failed: HRESULT 0x{:08x}".format(hr & 0xFFFFFFFF))
        return bind_ctx

    def _moniker_display_name(self, moniker):
        try:
            return moniker.GetDisplayName(self._create_bind_ctx(), None)
        except Exception:
            return ""

    def _vid_pid_from_display_name(self, display_name):
        match = re.search(r"vid_([0-9a-fA-F]{4})&pid_([0-9a-fA-F]{4})", display_name or "")
        if not match:
            return None, None
        return int(match.group(1), 16), int(match.group(2), 16)

    def _iter_filter_monikers(self):
        dev_enum = self.com["CreateObject"](
            self.com["CLSID_SystemDeviceEnum"],
            interface=self.com["ICreateDevEnum"],
        )
        enum = dev_enum.CreateClassEnumerator(
            ctypes.byref(self.com["CLSID_VideoInputDeviceCategory"]),
            0,
        )
        idx = 0
        while True:
            moniker, fetched = enum.Next(1)
            if fetched != 1 or not moniker:
                break
            display_name = self._moniker_display_name(moniker)
            vid, pid = self._vid_pid_from_display_name(display_name)
            if self.args.vid is not None and vid != self.args.vid:
                idx += 1
                continue
            if self.args.pid is not None and pid != self.args.pid:
                idx += 1
                continue
            yield idx, moniker, display_name, vid, pid
            idx += 1

    def _entry_supports_expected_operation(self, info):
        if not (info & 0x01):
            return False
        if expected_selector_requires_set(self.args) and not (info & 0x02):
            return False
        return True

    def _probe_filter(self, filter_index, moniker, display_name, vid, pid):
        try:
            unknown = moniker.BindToObject(
                None,
                None,
                ctypes.byref(self.com["IKsTopologyInfo"]._iid_),
            )
            topo = unknown.QueryInterface(self.com["IKsTopologyInfo"])
            ks = unknown.QueryInterface(self.com["IKsControl"])
        except Exception as exc:
            return [{
                "backend": self.name,
                "filter_index": filter_index,
                "vid": vid,
                "pid": pid,
                "device_path": display_name,
                "xu_supported": False,
                "xu_error": self._format_com_error(exc),
            }]

        expected_selector = expected_selector_for_args(self.args)
        node_ids = [self.args.node_id] if self.args.node_id is not None else range(int(topo.get_NumNodes()))
        entries = []
        for node_id in node_ids:
            try:
                basic_flags = self._basic_support(ks, node_id, expected_selector)
                info = self._uvc_info_from_basic_support(basic_flags)
                node_type = str(topo.get_NodeType(node_id))
                entries.append({
                    "backend": self.name,
                    "filter_index": filter_index,
                    "vid": vid,
                    "pid": pid,
                    "device_path": display_name,
                    "node_id": int(node_id),
                    "node_type": node_type,
                    "xu_guid": str(self.xu_guid),
                    "xu_info": info,
                    "xu_basic_support": basic_flags,
                    "xu_supported": self._entry_supports_expected_operation(info),
                    "unknown": unknown,
                    "topo": topo,
                    "ks": ks,
                })
            except Exception:
                continue
        if not entries and self.args.node_id is not None:
            return [{
                "backend": self.name,
                "filter_index": filter_index,
                "vid": vid,
                "pid": pid,
                "device_path": display_name,
                "node_id": int(self.args.node_id),
                "xu_guid": str(self.xu_guid),
                "xu_supported": False,
                "xu_error": "node does not expose the requested XU selector",
            }]
        return entries

    def list_targets(self):
        entries = []
        for filter_index, moniker, display_name, vid, pid in self._iter_filter_monikers():
            entries.extend(self._probe_filter(filter_index, moniker, display_name, vid, pid))
        return entries

    def select_target(self):
        entries = self.list_targets()
        if self.args.list:
            return None
        supported = [entry for entry in entries if entry.get("xu_supported")]
        if self.args.index is not None:
            if self.args.index < 0 or self.args.index >= len(entries):
                print_target_list(entries, stream=sys.stderr)
                raise UvcXuError("--index is out of range")
            entry = entries[self.args.index]
            if not entry.get("xu_supported"):
                print_target_list([entry], stream=sys.stderr)
                raise UvcXuError("selected Windows KS node does not support the requested YCTC XU selector")
            return entry
        if len(supported) == 1:
            return supported[0]
        print_target_list(entries, stream=sys.stderr)
        if not supported:
            raise UvcXuError("no Windows KS node reported the requested YCTC XU selector")
        raise UvcXuError("multiple matching Windows KS nodes; pass --index or --node-id")

    def query(self, unit, selector, query, size, payload=None):
        if int(unit) != int(self.args.unit_id):
            raise UvcXuError("windows-ks uses KS topology node ids; --unit-id must remain the device XU unit id")
        if self.target is None:
            self.target = self.select_target()
        ks = self.target["ks"]
        node_id = self.target["node_id"]

        if query == UVC_GET_LEN:
            expected_len = EXPECTED_SELECTOR_LENGTHS.get(selector)
            if expected_len is None:
                raise UvcXuError("Windows KS backend does not know selector 0x{:02x} length".format(selector))
            return struct.pack("<H", expected_len)
        if query == UVC_GET_INFO:
            basic_flags = self._basic_support(ks, node_id, selector)
            return bytes([self._uvc_info_from_basic_support(basic_flags)])
        if query in U32_CAPABILITY_QUERY_FIELDS:
            spec = U32_CONTROL_SPECS.get(selector)
            if spec is None:
                raise UvcXuError("Windows KS backend does not know selector 0x{:02x} capabilities".format(selector))
            if selector == SELECTOR_EXPOSURE_TIME:
                if int(size) != EXPOSURE_RANGE_CONTROL_LEN:
                    raise UvcXuError("selector 0x{:02x} capability length must be {}".format(
                        selector, EXPOSURE_RANGE_CONTROL_LEN))
                exposure_capabilities = {
                    UVC_GET_MIN: (250, 250),
                    UVC_GET_MAX: (10000, 10000),
                    UVC_GET_RES: (1, 1),
                    UVC_GET_DEF: (10000, 250),
                }
                return struct.pack("<II", *exposure_capabilities[query])
            if int(size) != U32_CONTROL_LEN:
                raise UvcXuError("selector 0x{:02x} capability length must be {}".format(
                    selector, U32_CONTROL_LEN))
            # IKsControl exposes GET_CUR/SET_CUR, not the raw UVC range requests.
            return struct.pack("<I", int(spec[U32_CAPABILITY_QUERY_FIELDS[query]]))
        if query == UVC_GET_CUR:
            return self._ks_property(ks, node_id, selector, self.KSPROPERTY_TYPE_GET, size)
        if query == UVC_SET_CUR:
            self._ks_property(ks, node_id, selector, self.KSPROPERTY_TYPE_SET, size, payload=payload)
            return b""
        raise UvcXuError("Windows KS backend supports GET_LEN/GET_INFO/GET_CUR/SET_CUR only")

    def close(self):
        self.target = None


def build_backend(args):
    backend = args.backend
    if backend == "auto":
        backend = "linux-ioctl" if platform.system() == "Linux" else "windows-ks"
    if backend == "linux-ioctl":
        if platform.system() != "Linux":
            raise UvcXuError("linux-ioctl backend is only available on Linux")
        return LinuxIoctlBackend(args)
    if backend == "windows-ks":
        return WindowsKsBackend(args)
    if backend == "pyusb":
        return PyUsbBackend(args)
    raise UvcXuError("unsupported backend: {}".format(backend))


def xu_get_cur(backend, unit, selector, size):
    data = backend.query(unit, selector, UVC_GET_CUR, size)
    if len(data) != size:
        raise UvcXuError("GET_CUR selector 0x{:02x} length mismatch: {} != {}".format(selector, len(data), size))
    return data


def xu_set_cur(backend, unit, selector, payload):
    backend.query(unit, selector, UVC_SET_CUR, len(payload), payload=payload)


def xu_get_u32(backend, unit, selector, query):
    payload = backend.query(unit, selector, query, U32_CONTROL_LEN)
    if len(payload) != U32_CONTROL_LEN:
        raise UvcXuError(
            "selector 0x{:02x} query 0x{:02x} length mismatch: {} != {}".format(
                selector, query, len(payload), U32_CONTROL_LEN))
    return struct.unpack("<I", payload)[0]


def xu_get_exposure_range(backend, unit, selector, query):
    payload = backend.query(unit, selector, query, EXPOSURE_RANGE_CONTROL_LEN)
    if len(payload) != EXPOSURE_RANGE_CONTROL_LEN:
        raise UvcXuError(
            "selector 0x{:02x} query 0x{:02x} length mismatch: {} != {}".format(
                selector, query, len(payload), EXPOSURE_RANGE_CONTROL_LEN))
    max_us, min_us = struct.unpack("<II", payload)
    return max_us, min_us


def read_u32_control_capabilities(backend, args, selector):
    expected_name = SELECTOR_NAMES.get(selector, "UNKNOWN")
    length_raw = backend.query(args.unit_id, selector, UVC_GET_LEN, 2)
    if len(length_raw) != 2:
        raise UvcXuError("{} GET_LEN response length mismatch: {} != 2".format(expected_name, len(length_raw)))
    length = struct.unpack("<H", length_raw)[0]
    if length != U32_CONTROL_LEN:
        raise UvcXuError("{} GET_LEN returned {} instead of {}".format(
            expected_name, length, U32_CONTROL_LEN))

    info_raw = backend.query(args.unit_id, selector, UVC_GET_INFO, 1)
    if len(info_raw) != 1:
        raise UvcXuError("{} GET_INFO response length mismatch: {} != 1".format(expected_name, len(info_raw)))
    info = info_raw[0]
    if not (info & 0x01):
        raise UvcXuError("{} does not support GET_CUR".format(expected_name))

    capabilities = {
        "selector": selector,
        "selector_name": expected_name,
        "unit": U32_CONTROL_SPECS[selector]["unit"],
        "length": length,
        "info": info,
        "get_supported": True,
        "set_supported": bool(info & 0x02),
        "capability_source": "protocol" if getattr(backend, "name", "") == "windows-ks" else "device",
    }
    for field, query in U32_CAPABILITY_QUERIES:
        capabilities[field] = xu_get_u32(backend, args.unit_id, selector, query)

    if capabilities["minimum"] > capabilities["maximum"]:
        raise UvcXuError("{} reports minimum greater than maximum".format(expected_name))
    if capabilities["resolution"] == 0:
        raise UvcXuError("{} reports zero resolution".format(expected_name))
    return capabilities


def read_exposure_range_capabilities(backend, args):
    selector = SELECTOR_EXPOSURE_TIME
    expected_name = SELECTOR_NAMES[selector]
    length_raw = backend.query(args.unit_id, selector, UVC_GET_LEN, 2)
    if len(length_raw) != 2:
        raise UvcXuError("{} GET_LEN response length mismatch: {} != 2".format(expected_name, len(length_raw)))
    length = struct.unpack("<H", length_raw)[0]
    if length != EXPOSURE_RANGE_CONTROL_LEN:
        raise UvcXuError("{} GET_LEN returned {} instead of {}".format(
            expected_name, length, EXPOSURE_RANGE_CONTROL_LEN))

    info_raw = backend.query(args.unit_id, selector, UVC_GET_INFO, 1)
    if len(info_raw) != 1:
        raise UvcXuError("{} GET_INFO response length mismatch: {} != 1".format(expected_name, len(info_raw)))
    info = info_raw[0]
    if not (info & 0x01):
        raise UvcXuError("{} does not support GET_CUR".format(expected_name))

    pairs = {}
    for field, query in U32_CAPABILITY_QUERIES:
        max_us, min_us = xu_get_exposure_range(backend, args.unit_id, selector, query)
        pairs[field] = {"max_us": max_us, "min_us": min_us}

    minimum = pairs["minimum"]["min_us"]
    maximum = pairs["maximum"]["max_us"]
    resolution = pairs["resolution"]["max_us"]
    default = pairs["default"]["max_us"]
    if minimum > maximum:
        raise UvcXuError("{} reports minimum greater than maximum".format(expected_name))
    if resolution == 0 or pairs["resolution"]["min_us"] == 0:
        raise UvcXuError("{} reports zero resolution".format(expected_name))

    return {
        "selector": selector,
        "selector_name": expected_name,
        "unit": "us",
        "length": length,
        "info": info,
        "get_supported": True,
        "set_supported": bool(info & 0x02),
        "minimum": minimum,
        "maximum": maximum,
        "resolution": resolution,
        "default": default,
        "minimum_pair": pairs["minimum"],
        "maximum_pair": pairs["maximum"],
        "resolution_pair": pairs["resolution"],
        "default_pair": pairs["default"],
        "capability_source": "protocol" if getattr(backend, "name", "") == "windows-ks" else "device",
    }


def u32_control_result(capabilities, value):
    result = dict(capabilities)
    result["value"] = int(value)
    result["raw_hex"] = struct.pack("<I", int(value)).hex()
    if result["selector"] == SELECTOR_SYSTEM_GAIN:
        result["gain_multiplier"] = float(value) / 1024.0
    if result["selector"] == SELECTOR_EXPOSURE_TIME:
        result["value_ms"] = float(value) / 1000.0
    if result["selector"] == SELECTOR_BITRATE:
        result["value_mbps"] = float(value) / 1024.0
    return result


def exposure_range_result(capabilities, max_us, min_us):
    result = dict(capabilities)
    result["max_us"] = int(max_us)
    result["min_us"] = int(min_us)
    result["value"] = int(max_us)
    result["raw_hex"] = struct.pack("<II", int(max_us), int(min_us)).hex()
    result["value_ms"] = float(max_us) / 1000.0
    result["min_ms"] = float(min_us) / 1000.0
    return result


def read_u32_control(backend, args, selector):
    capabilities = read_u32_control_capabilities(backend, args, selector)
    value = xu_get_u32(backend, args.unit_id, selector, UVC_GET_CUR)
    return u32_control_result(capabilities, value)


def read_exposure_range_control(backend, args):
    capabilities = read_exposure_range_capabilities(backend, args)
    max_us, min_us = xu_get_exposure_range(backend, args.unit_id, SELECTOR_EXPOSURE_TIME, UVC_GET_CUR)
    if min_us > max_us:
        raise UvcXuError("EXPOSURE_TIME GET_CUR reports min_us greater than max_us")
    return exposure_range_result(capabilities, max_us, min_us)


def validate_u32_control_value(capabilities, value):
    selector_name = capabilities["selector_name"]
    minimum = capabilities["minimum"]
    maximum = capabilities["maximum"]
    resolution = capabilities["resolution"]
    if value < minimum or value > maximum:
        raise UvcXuError("{} value {} is outside device range {}..{}".format(
            selector_name, value, minimum, maximum))
    if (value - minimum) % resolution != 0:
        raise UvcXuError("{} value {} does not match device resolution {}".format(
            selector_name, value, resolution))


def set_u32_control(backend, args, selector, value):
    capabilities = read_u32_control_capabilities(backend, args, selector)
    if not capabilities["set_supported"]:
        raise UvcXuError("{} does not support SET_CUR".format(capabilities["selector_name"]))
    validate_u32_control_value(capabilities, value)

    xu_set_cur(backend, args.unit_id, selector, struct.pack("<I", int(value)))
    readback = xu_get_u32(backend, args.unit_id, selector, UVC_GET_CUR)
    if readback != int(value):
        raise UvcXuError("{} SET_CUR readback mismatch: {} != {}".format(
            capabilities["selector_name"], readback, value))

    result = u32_control_result(capabilities, readback)
    result["requested_value"] = int(value)
    return result


def validate_exposure_range_values(capabilities, min_us, max_us):
    if min_us < capabilities["minimum"] or max_us > capabilities["maximum"]:
        raise UvcXuError("EXPOSURE_TIME range {}..{} is outside device range {}..{}".format(
            min_us, max_us, capabilities["minimum"], capabilities["maximum"]))
    if min_us > max_us:
        raise UvcXuError("EXPOSURE_TIME min_us {} is greater than max_us {}".format(min_us, max_us))
    resolution = capabilities["resolution"]
    if ((min_us - capabilities["minimum"]) % resolution != 0 or
            (max_us - capabilities["minimum"]) % resolution != 0):
        raise UvcXuError("EXPOSURE_TIME range does not match device resolution {}".format(resolution))


def set_exposure_range(backend, args, min_us, max_us):
    capabilities = read_exposure_range_capabilities(backend, args)
    if not capabilities["set_supported"]:
        raise UvcXuError("EXPOSURE_TIME does not support SET_CUR")
    validate_exposure_range_values(capabilities, int(min_us), int(max_us))

    xu_set_cur(backend, args.unit_id, SELECTOR_EXPOSURE_TIME,
        struct.pack("<II", int(max_us), int(min_us)))
    readback_max, readback_min = xu_get_exposure_range(
        backend, args.unit_id, SELECTOR_EXPOSURE_TIME, UVC_GET_CUR)
    if (readback_max, readback_min) != (int(max_us), int(min_us)):
        raise UvcXuError("EXPOSURE_TIME SET_CUR readback mismatch: max/min {}..{} != {}..{}".format(
            readback_min, readback_max, min_us, max_us))

    result = exposure_range_result(capabilities, readback_max, readback_min)
    result["requested_max_us"] = int(max_us)
    result["requested_min_us"] = int(min_us)
    result["payload_length"] = EXPOSURE_RANGE_CONTROL_LEN
    return result


def set_exposure_max_compat(backend, args, max_us):
    capabilities = read_exposure_range_capabilities(backend, args)
    if not capabilities["set_supported"]:
        raise UvcXuError("EXPOSURE_TIME does not support SET_CUR")
    _current_max, current_min = xu_get_exposure_range(
        backend, args.unit_id, SELECTOR_EXPOSURE_TIME, UVC_GET_CUR)
    validate_exposure_range_values(capabilities, current_min, int(max_us))

    # Keep the historical 4-byte max-only payload for old host applications.
    xu_set_cur(backend, args.unit_id, SELECTOR_EXPOSURE_TIME, struct.pack("<I", int(max_us)))
    readback_max, readback_min = xu_get_exposure_range(
        backend, args.unit_id, SELECTOR_EXPOSURE_TIME, UVC_GET_CUR)
    if readback_max != int(max_us) or readback_min != current_min:
        raise UvcXuError("EXPOSURE_TIME max-only SET_CUR readback mismatch: max/min {}..{}".format(
            readback_min, readback_max))

    result = exposure_range_result(capabilities, readback_max, readback_min)
    result["requested_max_us"] = int(max_us)
    result["requested_min_us"] = current_min
    result["payload_length"] = U32_CONTROL_LEN
    result["compatibility_mode"] = "max-only"
    return result


def read_calib_info(backend, args):
    payload = xu_get_cur(backend, args.unit_id, SELECTOR_CALIB_INFO, YCTC_XU_INFO_LEN)
    schema_version, calibration_version, payload_crc32, payload_length = struct.unpack("<HHII", payload[:12])
    return {
        "schema_version": schema_version,
        "calibration_version": calibration_version,
        "payload_crc32": payload_crc32,
        "payload_length": payload_length,
        "status_code": payload[12],
        "raw_hex": payload.hex(),
    }


def read_calib_status(backend, args):
    payload = xu_get_cur(backend, args.unit_id, SELECTOR_CALIB_STATUS, YCTC_XU_STATUS_LEN)
    state, status_code, detail_code, received_length = struct.unpack("<BBHI", payload)
    return {
        "state": state,
        "state_name": {
            0: "IDLE",
            1: "RECEIVING",
            2: "READY",
            3: "ERROR",
        }.get(state, "UNKNOWN"),
        "status_code": status_code,
        "detail_code": detail_code,
        "received_length": received_length,
        "raw_hex": payload.hex(),
    }


def pack_read_chunk_command(session_id, offset, read_size):
    return struct.pack(
        "<BBHIII",
        YCTC_XU_CMD_READ_CHUNK,
        0,
        0,
        int(session_id),
        int(offset),
        int(read_size),
    )


def unpack_transfer(payload, expected_session_id, expected_offset):
    if len(payload) != YCTC_XU_TRANSFER_LEN:
        raise UvcXuError("TRANSFER length mismatch: {} != {}".format(len(payload), YCTC_XU_TRANSFER_LEN))
    session_id, offset, chunk_len, chunk_crc16 = struct.unpack("<IIHH", payload[:12])
    if chunk_len == 0 or chunk_len > YCTC_XU_TRANSFER_DATA_MAX:
        raise UvcXuError("TRANSFER chunk_len out of range: {}".format(chunk_len))
    data = payload[12:12 + chunk_len]
    actual_crc16 = crc16_modbus(data)
    if actual_crc16 != chunk_crc16:
        raise UvcXuError("TRANSFER chunk crc16 mismatch: {:#06x} != {:#06x}".format(actual_crc16, chunk_crc16))
    if session_id != int(expected_session_id):
        raise UvcXuError("TRANSFER session_id mismatch: {} != {}".format(session_id, expected_session_id))
    if offset != int(expected_offset):
        raise UvcXuError("TRANSFER offset mismatch: {} != {}".format(offset, expected_offset))
    return {
        "session_id": session_id,
        "offset": offset,
        "chunk_len": chunk_len,
        "chunk_crc16": chunk_crc16,
        "data": data,
        "raw_hex": payload.hex(),
    }


def read_calibration_blob(backend, args, session_id=None):
    info = read_calib_info(backend, args)
    total = int(info["payload_length"])
    if total <= 0:
        raise UvcXuError("device reports no active calibration blob")
    if total > int(args.max_blob_size):
        raise UvcXuError("reported blob length {} exceeds --max-blob-size {}".format(total, args.max_blob_size))
    if int(info["status_code"]) != 0:
        raise UvcXuError("CALIB_INFO status_code is not OK: {}".format(info["status_code"]))

    read_session_id = int(args.session_id) if session_id is None else int(session_id)
    blob = bytearray()
    for offset in range(0, total, int(args.chunk_size)):
        read_size = min(int(args.chunk_size), total - offset)
        command = pack_read_chunk_command(read_session_id, offset, read_size)
        xu_set_cur(backend, args.unit_id, SELECTOR_CALIB_COMMAND, command)
        transfer = unpack_transfer(
            xu_get_cur(backend, args.unit_id, SELECTOR_CALIB_TRANSFER, YCTC_XU_TRANSFER_LEN),
            read_session_id,
            offset,
        )
        if int(transfer["chunk_len"]) != int(read_size):
            raise UvcXuError("TRANSFER chunk_len mismatch: {} != {}".format(transfer["chunk_len"], read_size))
        blob.extend(transfer["data"])

    if len(blob) != total:
        raise UvcXuError("readback length mismatch: {} != {}".format(len(blob), total))
    actual_crc32 = zlib.crc32(blob) & 0xFFFFFFFF
    if actual_crc32 != int(info["payload_crc32"]):
        raise UvcXuError("blob CRC32 mismatch: {} != {}".format(actual_crc32, info["payload_crc32"]))
    return bytes(blob), info


def read_output_mode(backend, args):
    payload = xu_get_cur(backend, args.unit_id, SELECTOR_OUTPUT_MODE, 1)
    mode = payload[0]
    return {
        "mode": mode,
        "mode_name": output_mode_name(mode),
        "raw_hex": payload.hex(),
    }


def set_output_mode(backend, args):
    payload = bytes([int(args.set_output_mode)])
    xu_set_cur(backend, args.unit_id, SELECTOR_OUTPUT_MODE, payload)
    return read_output_mode(backend, args)


def recovery_payload_from_args(args):
    if args.recovery_payload and args.recovery_value is not None:
        raise UvcXuError("--recovery-payload and --recovery-value are mutually exclusive")
    if args.recovery_payload:
        hex_text = re.sub(r"[^0-9a-fA-F]", "", args.recovery_payload)
        if len(hex_text) == 0 or len(hex_text) % 2 != 0:
            raise UvcXuError("--recovery-payload must contain an even number of hex digits")
        payload = bytes.fromhex(hex_text)
    else:
        value = 1 if args.recovery_value is None else int(args.recovery_value)
        if value < 0 or value > 0xFFFFFFFF:
            raise UvcXuError("--recovery-value must fit uint32")
        payload = struct.pack("<I", value)
    if len(payload) != 4:
        raise UvcXuError("RECOVERY selector expects a 4-byte payload, got {}".format(len(payload)))
    return payload


def trigger_recovery(backend, args):
    payload = recovery_payload_from_args(args)
    if args.dry_run:
        return {
            "sent": False,
            "unit_id": args.unit_id,
            "selector": SELECTOR_RECOVERY,
            "payload_hex": payload.hex(),
        }
    xu_set_cur(backend, args.unit_id, SELECTOR_RECOVERY, payload)
    return {
        "sent": True,
        "unit_id": args.unit_id,
        "selector": SELECTOR_RECOVERY,
        "payload_hex": payload.hex(),
    }


def run_probe(backend, args):
    def query_func(unit, selector, query, size):
        return backend.query(unit, selector, query, size)

    return selector_probe_summary(query_func, args.unit_id)


def print_target_list(entries, stream=sys.stdout):
    for idx, entry in enumerate(entries):
        parts = ["[{}]".format(idx), entry.get("backend", "linux-ioctl")]
        if entry.get("path"):
            parts.append(entry["path"])
        if entry.get("name"):
            parts.append("name={!r}".format(entry["name"]))
        if entry.get("vid") is not None and entry.get("pid") is not None:
            parts.append("vid:pid=0x{:04x}:0x{:04x}".format(entry["vid"], entry["pid"]))
        if entry.get("bus") is not None:
            parts.append("bus={}".format(entry["bus"]))
        if entry.get("address") is not None:
            parts.append("address={}".format(entry["address"]))
        if entry.get("interface") is not None:
            parts.append("interface={}".format(entry["interface"]))
        if entry.get("filter_index") is not None:
            parts.append("filter={}".format(entry["filter_index"]))
        if entry.get("node_id") is not None:
            parts.append("node={}".format(entry["node_id"]))
        if entry.get("xu_guid"):
            parts.append("xu_guid={}".format(entry["xu_guid"]))
        if entry.get("xu_info") is not None:
            parts.append("xu_info=0x{:02x}".format(entry["xu_info"]))
        if entry.get("xu_supported") is not None:
            parts.append("xu_supported={}".format(entry["xu_supported"]))
        if entry.get("xu_error"):
            parts.append("error={}".format(entry["xu_error"]))
        print(" ".join(parts), file=stream)


def write_binary_output(path, data):
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, "wb") as fp:
        fp.write(data)


def write_kalibr_yaml_output(path, blob):
    payload = extract_kalibr_yaml_payload(blob)
    write_binary_output(path, payload)
    return os.path.abspath(path)


def run_calib_parse(args):
    with open(args.parse_file, "rb") as fp:
        blob = fp.read()
    parsed = None
    json_out = "" if args.no_json_out else (args.json_out or default_json_output_path(args.parse_file))
    if json_out:
        parsed = write_parsed_json(blob, json_out, strict_stereo_payload=args.strict_parse)
    elif args.yaml_out or args.strict_parse:
        parsed = parse_calibration_blob(blob, strict_stereo_payload=args.strict_parse)
    yaml_out = ""
    if args.yaml_out:
        yaml_out = write_kalibr_yaml_output(args.yaml_out, blob)
    return {
        "ok": True,
        "operation": "calib-parse",
        "input_file": os.path.abspath(args.parse_file),
        "blob_length": len(blob),
        "md5": md5_hex(blob),
        "json_out": os.path.abspath(json_out) if json_out else "",
        "yaml_out": yaml_out,
        "header": parsed.get("header", {}) if parsed else {},
    }, 0


def run_calib_read(backend, args):
    protocol_probe = run_probe(backend, args) if args.validate_protocol else None
    blob, info = read_calibration_blob(backend, args)
    write_binary_output(args.out, blob)

    json_out = "" if args.no_json_out else (args.json_out or default_json_output_path(args.out))
    parsed = None
    if json_out:
        parsed = write_parsed_json(blob, json_out, strict_stereo_payload=args.strict_parse)
    elif args.yaml_out or args.strict_parse:
        parsed = parse_calibration_blob(blob, strict_stereo_payload=args.strict_parse)
    yaml_out = ""
    if args.yaml_out:
        yaml_out = write_kalibr_yaml_output(args.yaml_out, blob)

    output_md5 = md5_hex(blob)
    expected_file_match = None
    expected_file_md5 = None
    if args.expected_file:
        with open(args.expected_file, "rb") as fp:
            expected_blob = fp.read()
        expected_file_md5 = md5_hex(expected_blob)
        expected_file_match = expected_blob == blob

    expected_md5_match = None
    if args.expected_md5:
        expected_md5_match = output_md5.lower() == args.expected_md5.lower()

    result = {
        "ok": True,
        "operation": "calib-read",
        "backend": backend.name,
        "output_file": os.path.abspath(args.out),
        "json_out": os.path.abspath(json_out) if json_out else "",
        "yaml_out": yaml_out,
        "schema_version": info["schema_version"],
        "calibration_version": info["calibration_version"],
        "blob_crc32": info["payload_crc32"],
        "blob_length": info["payload_length"],
        "output_md5": output_md5,
        "expected_file": os.path.abspath(args.expected_file) if args.expected_file else "",
        "expected_file_md5": expected_file_md5,
        "expected_file_match": expected_file_match,
        "expected_md5": args.expected_md5,
        "expected_md5_match": expected_md5_match,
        "header": parsed.get("header", {}) if parsed else {},
    }
    if protocol_probe is not None:
        result["protocol_probe"] = protocol_probe

    if expected_file_match is False:
        return result, 2
    if expected_md5_match is False:
        return result, 3
    return result, 0


def determine_operation(args):
    operations = [
        bool(args.probe),
        bool(args.get_output_mode),
        args.set_output_mode is not None,
        bool(args.get_exposure_time),
        args.set_exposure_time_us is not None,
        args.set_exposure_range_us is not None,
        bool(args.get_system_gain),
        args.set_system_gain is not None,
        bool(args.get_bitrate),
        args.set_bitrate_kbps is not None,
        bool(args.recovery),
        bool(args.calib_info),
        bool(args.calib_status),
        bool(args.parse_file),
    ]
    if sum(1 for item in operations if item) > 1:
        raise UvcXuError("select only one operation")
    if args.probe:
        return "probe"
    if args.get_output_mode:
        return "output-get"
    if args.set_output_mode is not None:
        return "output-set"
    if args.get_exposure_time:
        return "exposure-get"
    if args.set_exposure_time_us is not None:
        return "exposure-set"
    if args.set_exposure_range_us is not None:
        return "exposure-range-set"
    if args.get_system_gain:
        return "system-gain-get"
    if args.set_system_gain is not None:
        return "system-gain-set"
    if args.get_bitrate:
        return "bitrate-get"
    if args.set_bitrate_kbps is not None:
        return "bitrate-set"
    if args.recovery:
        return "recovery"
    if args.calib_info:
        return "calib-info"
    if args.calib_status:
        return "calib-status"
    if args.parse_file:
        return "calib-parse"
    return "calib-read"


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="YCTC_SC233HGS and ZXCZ_SC233HGS_DUAL UVC XU protocol tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            Examples:
              List Linux video nodes:
                %(prog)s --list

              Output synchronization mode:
                %(prog)s --device /dev/video1 --get-output-mode
                %(prog)s --device /dev/video1 --set-output-mode external

              ZXCZ_SC233HGS_DUAL controls:
                %(prog)s --device /dev/video1 --get-exposure-time
                %(prog)s --device /dev/video1 --set-exposure-time-us 3000
                %(prog)s --device /dev/video1 --set-exposure-range-us 250 3000
                %(prog)s --device /dev/video1 --get-system-gain
                %(prog)s --device /dev/video1 --set-system-gain 0x800
                %(prog)s --device /dev/video1 --get-bitrate
                %(prog)s --device /dev/video1 --set-bitrate-kbps 12288

              Calibration blob readback:
                %(prog)s --device /dev/video1 --out stereo_calibration.bin
                %(prog)s --parse-file stereo_calibration.bin

              Recovery:
                %(prog)s --device /dev/video1 --recovery
                %(prog)s --device /dev/video1 --recovery --dry-run
            """
        ),
    )
    parser.add_argument("--backend", choices=("auto", "linux-ioctl", "windows-ks", "pyusb"), default="auto")
    parser.add_argument("--list", action="store_true", help="list candidate devices/interfaces and exit")
    parser.add_argument("--device", help="Linux video node, for example /dev/video1 or video1")
    parser.add_argument("--vid", type=parse_u16, help="USB vendor id filter, for example 0x1234")
    parser.add_argument("--pid", type=parse_u16, help="USB product id filter, for example 0x5678")
    parser.add_argument("--index", type=int, help="select an entry printed by --list")
    parser.add_argument("--interface", type=parse_u8, help="USB VideoControl interface number for pyusb")
    parser.add_argument("--bus", type=parse_int, help="pyusb bus filter")
    parser.add_argument("--address", type=parse_int, help="pyusb address filter")
    parser.add_argument("--xu-guid", type=parse_guid, default=DEFAULT_XU_GUID,
                        help="Windows KS Extension Unit GUID, default {}".format(DEFAULT_XU_GUID))
    parser.add_argument("--node-id", type=parse_int,
                        help="Windows KS topology node id override; auto-scanned by default")
    parser.add_argument("--timeout-ms", type=int, default=3000, help="USB control transfer timeout, default 3000")
    parser.add_argument("--skip-probe", action="store_true", help="Linux: skip GET_LEN/GET_INFO support check")
    parser.add_argument("--no-claim", action="store_true", help="pyusb: do not claim the VideoControl interface")
    parser.add_argument("--unit-id", type=parse_u8, default=DEFAULT_UNIT_ID, help="XU unit id, default 0x0a")

    parser.add_argument("--probe", action="store_true", help="probe all documented selectors")
    parser.add_argument("--get-output-mode", action="store_true", help="read OUTPUT_MODE selector 0x06")
    parser.add_argument("--set-output-mode", type=parse_output_mode, help="set output mode: internal/1 or external/2")
    parser.add_argument("--get-exposure-time", action="store_true",
                        help="read EXPOSURE_TIME selector 0x07 and its device limits")
    parser.add_argument("--set-exposure-time-us", "--set-exposure-time", dest="set_exposure_time_us",
                        type=parse_u32, help="set EXPOSURE_TIME selector 0x07 max only (legacy 4-byte format) in us")
    parser.add_argument("--set-exposure-range-us", nargs=2, type=parse_u32,
                        metavar=("MIN_US", "MAX_US"),
                        help="set EXPOSURE_TIME selector 0x07 range using 8-byte max/min format")
    parser.add_argument("--get-system-gain", action="store_true",
                        help="read SYSTEM_GAIN selector 0x08 and its device limits")
    parser.add_argument("--set-system-gain", type=parse_u32,
                        help="set SYSTEM_GAIN selector 0x08 as a 22.10 fixed-point uint32")
    parser.add_argument("--get-bitrate", "--get-bitrate-kbps", dest="get_bitrate", action="store_true",
                        help="read BITRATE selector 0x09 and its device limits")
    parser.add_argument("--set-bitrate-kbps", "--set-bitrate", dest="set_bitrate_kbps", type=parse_u32,
                        help="set BITRATE selector 0x09 in Kbps")
    parser.add_argument("--recovery", action="store_true", help="trigger RECOVERY selector 0x01")
    parser.add_argument("--dry-run", action="store_true", help="show recovery payload without sending it")
    parser.add_argument("--recovery-value", type=parse_int, help="uint32 little-endian recovery value, default 1")
    parser.add_argument("--recovery-payload", help="raw 4-byte recovery payload hex, default 01000000")
    parser.add_argument("--calib-info", action="store_true", help="read CALIB_INFO selector 0x02")
    parser.add_argument("--calib-status", action="store_true", help="read CALIB_STATUS selector 0x05")

    parser.add_argument("--chunk-size", type=int, default=YCTC_XU_TRANSFER_DATA_MAX,
                        help="calibration read chunk size, range 1..48, default 48")
    parser.add_argument("--session-id", type=parse_int, default=0,
                        help="READ_CHUNK session_id echoed by the device")
    parser.add_argument("--max-blob-size", type=int, default=8192, help="maximum accepted blob size, default 8192")
    parser.add_argument("--out", default="stereo_calibration.bin", help="calibration binary output path")
    parser.add_argument("--json-out", default="", help="calibration JSON output path")
    parser.add_argument("--yaml-out", default="", help="extract schema v2 Kalibr YAML to this path after parse/read")
    parser.add_argument("--no-json-out", action="store_true", help="do not write parsed calibration JSON")
    parser.add_argument("--strict-parse", action="store_true",
                        help="require the known schema v1 stereo-double payload; reject schema v2")
    parser.add_argument("--parse-file", default="", help="parse an existing calibration blob without device access")
    parser.add_argument("--validate-protocol", action="store_true",
                        help="with calibration read, also probe documented selector metadata")
    parser.add_argument("--expected-file", default="", help="optional expected calibration blob for byte comparison")
    parser.add_argument("--expected-md5", default="", help="optional expected MD5 for output comparison")
    return parser


def validate_args(args):
    if args.timeout_ms <= 0:
        raise UvcXuError("--timeout-ms must be positive")
    if args.chunk_size <= 0 or args.chunk_size > YCTC_XU_TRANSFER_DATA_MAX:
        raise UvcXuError("--chunk-size must be in range 1..{}".format(YCTC_XU_TRANSFER_DATA_MAX))
    if args.session_id < 0 or args.session_id > 0xFFFFFFFF:
        raise UvcXuError("--session-id must fit uint32")
    if args.node_id is not None and (args.node_id < 0 or args.node_id > 0xFFFFFFFF):
        raise UvcXuError("--node-id must fit uint32")
    if args.max_blob_size <= 0:
        raise UvcXuError("--max-blob-size must be positive")
    if args.dry_run and not args.recovery:
        raise UvcXuError("--dry-run is only valid with --recovery")
    if args.yaml_out and (
            args.probe or args.get_output_mode or args.set_output_mode is not None or
            args.get_exposure_time or args.set_exposure_time_us is not None or
            args.set_exposure_range_us is not None or
            args.get_system_gain or args.set_system_gain is not None or
            args.get_bitrate or args.set_bitrate_kbps is not None or
            args.recovery or args.calib_info or args.calib_status):
        raise UvcXuError("--yaml-out is only valid with calibration read or parse")


def run_device_operation(args, operation):
    backend = build_backend(args)
    if args.list:
        print_target_list(backend.list_targets())
        return None, 0

    try:
        if operation == "probe":
            return {"ok": True, "operation": operation, "backend": backend.name, "selectors": run_probe(backend, args)}, 0
        if operation == "output-get":
            result = read_output_mode(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "output-set":
            result = set_output_mode(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "exposure-get":
            result = read_exposure_range_control(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "exposure-set":
            result = set_exposure_max_compat(backend, args, args.set_exposure_time_us)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "exposure-range-set":
            min_us, max_us = args.set_exposure_range_us
            result = set_exposure_range(backend, args, min_us, max_us)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "system-gain-get":
            result = read_u32_control(backend, args, SELECTOR_SYSTEM_GAIN)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "system-gain-set":
            result = set_u32_control(backend, args, SELECTOR_SYSTEM_GAIN, args.set_system_gain)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "bitrate-get":
            result = read_u32_control(backend, args, SELECTOR_BITRATE)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "bitrate-set":
            result = set_u32_control(backend, args, SELECTOR_BITRATE, args.set_bitrate_kbps)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "recovery":
            result = trigger_recovery(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "calib-info":
            result = read_calib_info(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "calib-status":
            result = read_calib_status(backend, args)
            result.update({"ok": True, "operation": operation, "backend": backend.name})
            return result, 0
        if operation == "calib-read":
            return run_calib_read(backend, args)
        raise UvcXuError("unsupported operation: {}".format(operation))
    finally:
        backend.close()


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    validate_args(args)
    operation = determine_operation(args)

    if operation == "calib-parse":
        result, rc = run_calib_parse(args)
    else:
        result, rc = run_device_operation(args, operation)

    if result is not None:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    return rc


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2), file=sys.stderr)
        sys.exit(1)

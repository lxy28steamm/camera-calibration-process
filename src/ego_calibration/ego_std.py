from __future__ import annotations

import ast
import ctypes
import hashlib
import math
import os
import platform
import struct
import zlib
from collections.abc import Callable
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, quote, urlparse

from ego_calibration.models import CalibrationError, CameraDevice


UNIT_ID = 0x0A
SELECTOR_CALIB_INFO = 0x02
SELECTOR_CALIB_TRANSFER = 0x03
SELECTOR_CALIB_COMMAND = 0x04
UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
COMMAND_READ_CHUNK = 0x05
INFO_SIZE = 16
TRANSFER_SIZE = 60
CHUNK_SIZE = 48
MAX_BLOB_SIZE = 8192
HEADER_SIZE = 64
PAYLOAD_DOUBLE_COUNT = 112
PAYLOAD_SIZE = PAYLOAD_DOUBLE_COUNT * 8
SCHEMA_STEREO_DOUBLE = 1
SCHEMA_KALIBR_YAML = 2
KALIBR_REQUIRED_FIELDS = (
    "cam0:",
    "cam1:",
    "T_cam_imu:",
    "T_cn_cnm1:",
    "timeshift_cam_imu:",
)
EGO_STD_IDENTITY_MARKERS = ("yctc", "zxcz", "sc233hgs")

XuQuery = Callable[[int, int, int, bytes | None], bytes]


class _UvcXuControlQuery(ctypes.Structure):
    _fields_ = [
        ("unit", ctypes.c_uint8),
        ("selector", ctypes.c_uint8),
        ("query", ctypes.c_uint8),
        ("size", ctypes.c_uint16),
        ("data", ctypes.POINTER(ctypes.c_uint8)),
    ]


def scan_devices() -> tuple[CameraDevice, ...]:
    system = platform.system()
    if system == "Windows":
        from ego_calibration.windows_ks import scan_devices as scan_windows_devices

        return scan_windows_devices()
    if system == "Linux":
        devices = _scan_linux_video_devices()
        if devices:
            return devices
    return _scan_usb_video_devices()


def read_calibration(
    identifier: str,
    *,
    query: XuQuery | None = None,
) -> dict[str, Any]:
    if not identifier.strip() and query is None:
        raise CalibrationError("请先选择 Ego-Std 相机")
    selected_query = query or _query_for(identifier)
    blob = _read_blob(selected_query)
    payload = _parse_blob(blob)
    calibration_serial = str(payload.get("header", {}).get("serial_number", "")).strip()
    usb_serial = _usb_serial_from_identifier(identifier)
    serials_match = bool(
        usb_serial and calibration_serial and usb_serial == calibration_serial
    )
    payload["device_identity"] = {
        "usb_serial_number": usb_serial,
        "calibration_serial_number": calibration_serial,
        "serials_match": serials_match,
        # 固件标定 SN 与 USB 描述符 SN 的编号规则由厂家分别维护，协议没有
        # 提供两者之间的映射；不相等只能说明字符串不同，不能据此判定串标。
        "serial_comparison": (
            "match" if serials_match else "unmapped"
            if usb_serial and calibration_serial
            else "unavailable"
        ),
    }
    return payload


def _scan_linux_video_devices(
    root: Path = Path("/dev/v4l/by-id"),
) -> tuple[CameraDevice, ...]:
    devices = []
    for path in sorted(root.glob("*-video-index0")):
        if not _is_ego_std_identity(path.name):
            continue
        serial = path.name.rsplit("-video-index", 1)[0].split("_")[-1]
        model = (
            "ZXCZ SC233HGS Dual"
            if "zxcz" in path.name.lower()
            else "YCTC Stereo UVC"
        )
        devices.append(
            CameraDevice(
                kind="ego-std",
                identifier=str(path),
                label=f"Ego-Std · {serial}",
                model=model,
                serial=serial,
                transport="UVC XU V5 · USB",
                path=str(path),
                accessible=os.access(path, os.R_OK | os.W_OK),
            )
        )
    return tuple(devices)


def _scan_usb_video_devices() -> tuple[CameraDevice, ...]:
    usb_core, usb_util, backend = _load_usb()
    devices: list[CameraDevice] = []
    try:
        candidates = usb_core.find(find_all=True, backend=backend) or ()
        for device in candidates:
            interface = _video_control_interface(device)
            if interface is None:
                continue
            manufacturer = _usb_string(usb_util, device, device.iManufacturer)
            product = _usb_string(usb_util, device, device.iProduct)
            serial = _usb_string(usb_util, device, device.iSerialNumber)
            identity = " ".join((manufacturer, product, serial)).lower()
            if not _is_ego_std_identity(identity):
                continue
            identifier = _usb_identifier(device, interface, serial)
            label = f"Ego-Std · {serial or product or identifier}"
            devices.append(
                CameraDevice(
                    kind="ego-std",
                    identifier=identifier,
                    label=label,
                    model=product or "YCTC Stereo UVC",
                    serial=serial,
                    transport=(
                        f"USB {int(device.idVendor):04x}:{int(device.idProduct):04x}"
                        " · UVC XU"
                    ),
                    path=identifier,
                )
            )
    except Exception as exc:
        raise CalibrationError(f"USB 相机检测失败：{exc}") from exc
    return tuple(sorted(devices, key=lambda item: item.label))


def _usb_serial_from_identifier(identifier: str) -> str:
    """Extract the USB descriptor serial from a selected device identifier."""
    if identifier.startswith("usb://"):
        values = parse_qs(urlparse(identifier).query)
        return values.get("serial", [""])[0].strip()
    name = Path(identifier).name
    if "-video-index" not in name:
        return ""
    return name.rsplit("-video-index", 1)[0].rsplit("_", 1)[-1].strip()


def _is_ego_std_identity(value: str) -> bool:
    identity = value.lower()
    return any(marker in identity for marker in EGO_STD_IDENTITY_MARKERS)


def _video_control_interface(device: Any) -> int | None:
    try:
        for configuration in device:
            for interface in configuration:
                if interface.bInterfaceClass == 0x0E and interface.bInterfaceSubClass == 1:
                    return int(interface.bInterfaceNumber)
    except Exception:
        return None
    return None


def _usb_identifier(device: Any, interface: int, serial: str) -> str:
    query = [f"interface={interface}"]
    if serial:
        query.append(f"serial={quote(serial, safe='')}")
    bus = getattr(device, "bus", None)
    address = getattr(device, "address", None)
    if bus is not None:
        query.append(f"bus={int(bus)}")
    if address is not None:
        query.append(f"address={int(address)}")
    return (
        f"usb://{int(device.idVendor):04x}:{int(device.idProduct):04x}"
        f"?{'&'.join(query)}"
    )


def _usb_string(usb_util: Any, device: Any, index: int) -> str:
    if not index:
        return ""
    try:
        return str(usb_util.get_string(device, index) or "").strip()
    except Exception:
        return ""


def _query_for(identifier: str) -> XuQuery:
    if identifier.startswith("ks://"):
        from ego_calibration.windows_ks import query_for as windows_query_for

        return windows_query_for(identifier)
    if identifier.startswith("usb://"):
        return _UsbXuQuery(identifier).query
    if platform.system() != "Linux":
        raise CalibrationError(
            "Windows/macOS 上请从设备列表选择 usb:// 开头的 Ego-Std 相机"
        )
    return _linux_query(Path(identifier))


class _UsbXuQuery:
    def __init__(self, identifier: str) -> None:
        parsed = urlparse(identifier)
        try:
            vendor_text, product_text = parsed.netloc.split(":", 1)
            self.vendor_id = int(vendor_text, 16)
            self.product_id = int(product_text, 16)
        except (ValueError, TypeError) as exc:
            raise CalibrationError(f"USB 设备标识无效：{identifier}") from exc
        values = parse_qs(parsed.query)
        self.interface = int(values.get("interface", ["0"])[0])
        self.serial = values.get("serial", [""])[0]
        self.bus = _optional_int(values.get("bus", [""])[0])
        self.address = _optional_int(values.get("address", [""])[0])
        self._device: Any | None = None

    def query(
        self,
        selector: int,
        request: int,
        size: int,
        payload: bytes | None,
    ) -> bytes:
        device = self._device or self._find_device()
        self._device = device
        request_type = 0xA1 if request == UVC_GET_CUR else 0x21
        data: int | bytes = size if payload is None else payload
        try:
            result = device.ctrl_transfer(
                request_type,
                request,
                selector << 8,
                UNIT_ID << 8 | self.interface,
                data,
                timeout=3000,
            )
        except Exception as exc:
            raise CalibrationError(
                "YCTC UVC XU 控制传输失败；请检查相机权限和 libusb/WinUSB 驱动："
                f"{exc}"
            ) from exc
        if payload is not None:
            if int(result) != size:
                raise CalibrationError("YCTC UVC XU SET_CUR 写入长度不完整")
            return b""
        value = bytes(result)
        if len(value) != size:
            raise CalibrationError("YCTC UVC XU GET_CUR 返回长度不完整")
        return value

    def _find_device(self) -> Any:
        usb_core, usb_util, backend = _load_usb()
        candidates = usb_core.find(
            find_all=True,
            idVendor=self.vendor_id,
            idProduct=self.product_id,
            backend=backend,
        ) or ()
        for device in candidates:
            if self.bus is not None and getattr(device, "bus", None) != self.bus:
                continue
            if self.address is not None and getattr(device, "address", None) != self.address:
                continue
            serial = _usb_string(usb_util, device, device.iSerialNumber)
            if self.serial and serial != self.serial:
                continue
            return device
        raise CalibrationError("所选 Ego-Std USB 相机已断开")


def _load_usb() -> tuple[Any, Any, Any]:
    try:
        import libusb_package
        import usb.backend.libusb1
        import usb.core
        import usb.util
    except ImportError as exc:
        raise CalibrationError("未安装 PyUSB/libusb 运行库") from exc
    backend = libusb_package.get_libusb1_backend()
    if backend is None:
        raise CalibrationError("未找到 libusb 后端")
    return usb.core, usb.util, backend


def _optional_int(value: str) -> int | None:
    try:
        return int(value)
    except ValueError:
        return None


def _linux_query(video_device: Path) -> XuQuery:
    import fcntl

    ioctl_number = _linux_ioc(3, "u", 0x21, ctypes.sizeof(_UvcXuControlQuery))

    def query(selector: int, request: int, size: int, payload: bytes | None) -> bytes:
        buffer = (ctypes.c_uint8 * max(size, 1))()
        if payload is not None:
            if len(payload) != size:
                raise CalibrationError("UVC XU 请求 payload 长度不匹配")
            for index, value in enumerate(payload):
                buffer[index] = value
        control = _UvcXuControlQuery(UNIT_ID, selector, request, size, buffer)
        try:
            fd = os.open(video_device, os.O_RDWR)
        except OSError as exc:
            raise CalibrationError(f"无法打开 Ego-Std 设备 {video_device}：{exc}") from exc
        try:
            fcntl.ioctl(fd, ioctl_number, control, True)
        except OSError as exc:
            raise CalibrationError(f"YCTC UVC XU ioctl 失败：{exc}") from exc
        finally:
            os.close(fd)
        return bytes(buffer[:size])

    return query


def _linux_ioc(direction: int, type_char: str, number: int, size: int) -> int:
    return direction << 30 | ord(type_char) << 8 | number | size << 16


def _read_blob(query: XuQuery) -> bytes:
    raw_info = query(SELECTOR_CALIB_INFO, UVC_GET_CUR, INFO_SIZE, None)
    if len(raw_info) != INFO_SIZE:
        raise CalibrationError("CALIB_INFO 长度无效")
    _schema_version, _calibration_version, expected_crc32, total = struct.unpack(
        "<HHII", raw_info[:12]
    )
    if raw_info[12] != 0:
        raise CalibrationError(f"CALIB_INFO 状态异常：{raw_info[12]}")
    if total <= 0 or total > MAX_BLOB_SIZE:
        raise CalibrationError(f"标定 blob 长度无效：{total}")

    blob = bytearray()
    session_id = 1
    for offset in range(0, total, CHUNK_SIZE):
        read_size = min(CHUNK_SIZE, total - offset)
        command = struct.pack(
            "<BBHIII",
            COMMAND_READ_CHUNK,
            0,
            0,
            session_id,
            offset,
            read_size,
        )
        query(SELECTOR_CALIB_COMMAND, UVC_SET_CUR, len(command), command)
        transfer = query(SELECTOR_CALIB_TRANSFER, UVC_GET_CUR, TRANSFER_SIZE, None)
        if len(transfer) != TRANSFER_SIZE:
            raise CalibrationError("CALIB_TRANSFER 长度无效")
        reply_session, reply_offset, chunk_len, chunk_crc16 = struct.unpack(
            "<IIHH", transfer[:12]
        )
        chunk = transfer[12 : 12 + chunk_len]
        if (
            reply_session != session_id
            or reply_offset != offset
            or chunk_len != read_size
        ):
            raise CalibrationError("CALIB_TRANSFER 分块元数据不匹配")
        if _crc16_modbus(chunk) != chunk_crc16:
            raise CalibrationError("CALIB_TRANSFER 分块 CRC16 不匹配")
        blob.extend(chunk)
    if zlib.crc32(blob) & 0xFFFFFFFF != expected_crc32:
        raise CalibrationError("标定 blob CRC32 不匹配")
    return bytes(blob)


def _parse_blob(blob: bytes) -> dict[str, Any]:
    if len(blob) < HEADER_SIZE + 4 or blob[:4] != b"ZXCZ":
        raise CalibrationError("标定 blob 头无效")
    schema_version = struct.unpack_from("<H", blob, 4)[0]
    payload_length = struct.unpack_from("<I", blob, 8)[0]
    if schema_version not in (SCHEMA_STEREO_DOUBLE, SCHEMA_KALIBR_YAML):
        raise CalibrationError(f"不支持的标定 blob schema：{schema_version}")
    if len(blob) != HEADER_SIZE + payload_length + 4:
        raise CalibrationError("标定 blob 总长度不匹配")
    blob_crc32 = struct.unpack_from("<I", blob, len(blob) - 4)[0]
    if zlib.crc32(blob[:-4]) & 0xFFFFFFFF != blob_crc32:
        raise CalibrationError("标定 blob 尾部 CRC32 不匹配")

    header = _parse_blob_header(blob, schema_version, payload_length, blob_crc32)
    payload = blob[HEADER_SIZE:-4]
    if schema_version == SCHEMA_KALIBR_YAML:
        return _parse_kalibr_blob(payload, header)
    return _parse_stereo_blob(payload, header)


def _parse_blob_header(
    blob: bytes,
    schema_version: int,
    payload_length: int,
    blob_crc32: int,
) -> dict[str, Any]:
    serial_field = blob[12:44]
    reserved = blob[44:64]
    if _is_valid_serial_field(serial_field) and not any(reserved):
        serial_number = _header_text(serial_field)
        header_format = "firmware_sn"
        header_sample_count = 0
        device_model = lens_type = tool_version = ""
    else:
        serial_number = ""
        header_format = "legacy_metadata"
        header_sample_count = struct.unpack_from("<I", blob, 12)[0]
        device_model = _header_text(blob[16:32])
        lens_type = _header_text(blob[32:48])
        tool_version = _header_text(blob[48:64])
    return {
        "magic": "ZXCZ",
        "schema_version": schema_version,
        "payload_length": payload_length,
        "serial_number": serial_number,
        "header_format": header_format,
        "header_sample_count": header_sample_count,
        "device_model": device_model,
        "lens_type": lens_type,
        "tool_version": tool_version,
        "blob_crc32": blob_crc32,
    }


def _parse_stereo_blob(
    payload: bytes,
    header: dict[str, Any],
) -> dict[str, Any]:
    if len(payload) != PAYLOAD_SIZE:
        raise CalibrationError(
            f"schema v1 payload 长度无效：{len(payload)}，应为 {PAYLOAD_SIZE}"
        )

    values = struct.unpack(f"<{PAYLOAD_DOUBLE_COUNT}d", payload)
    if not all(math.isfinite(value) for value in values):
        raise CalibrationError("标定 blob 包含非有限数值")
    cursor = 0

    def take(count: int) -> tuple[float, ...]:
        nonlocal cursor
        result = values[cursor : cursor + count]
        cursor += count
        return result

    calibration = {
        "K1": _matrix(take(9), 3, 3),
        "D1": list(take(8)),
        "K2": _matrix(take(9), 3, 3),
        "D2": list(take(8)),
        "R": _matrix(take(9), 3, 3),
        "T": _matrix(take(3), 3, 1),
        "R1": _matrix(take(9), 3, 3),
        "R2": _matrix(take(9), 3, 3),
        "P1": _matrix(take(12), 3, 4),
        "P2": _matrix(take(12), 3, 4),
        "Q": _matrix(take(16), 4, 4),
    }
    metric_names = (
        "stereo_rms",
        "left_calibrate_rms",
        "right_calibrate_rms",
        "sample_count",
        "baseline_mm",
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
    )
    metrics = dict(zip(metric_names, take(len(metric_names)), strict=True))
    return {
        "format": "stereo_calibration",
        "header": header,
        "payload": {
            "length": len(payload),
            "parser": "stereo_double_v1",
        },
        "calibration": calibration,
        "metrics": metrics,
    }


def _parse_kalibr_blob(
    payload: bytes,
    header: dict[str, Any],
) -> dict[str, Any]:
    if not payload:
        raise CalibrationError("Kalibr YAML payload 为空")
    if b"\x00" in payload:
        raise CalibrationError("Kalibr YAML payload 包含 NUL 字节")
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise CalibrationError("Kalibr YAML payload 不是有效 UTF-8") from exc
    missing = [field for field in KALIBR_REQUIRED_FIELDS if field not in text]
    if missing:
        raise CalibrationError(
            "Kalibr YAML payload 缺少必需字段：" + ", ".join(missing)
        )

    matrices: dict[str, dict[str, Any]] = {"cam0": {}, "cam1": {}}
    for section, name in (
        ("cam0", "T_cam_imu"),
        ("cam1", "T_cam_imu"),
        ("cam1", "T_cn_cnm1"),
    ):
        matrix = _extract_yaml_matrix(text, section, name)
        if matrix is not None:
            matrices[section][name] = matrix
    for section in matrices:
        resolution = _extract_yaml_resolution(text, section)
        if resolution is not None:
            matrices[section]["resolution"] = resolution
    return {
        "format": "kalibr_camchain_imucam",
        "header": header,
        "payload": {
            "length": len(payload),
            "parser": "kalibr_yaml_v2",
            "encoding": "utf-8",
            "sha256": hashlib.sha256(payload).hexdigest(),
            "required_fields": list(KALIBR_REQUIRED_FIELDS),
        },
        "kalibr_calibration": matrices,
        "kalibr_yaml": text,
    }


def _extract_yaml_resolution(text: str, section_name: str) -> list[int] | None:
    in_section = False
    section_indent = 0
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == f"{section_name}:":
            in_section = True
            section_indent = _indent(line)
            continue
        if not in_section:
            continue
        if _indent(line) <= section_indent:
            break
        if stripped.startswith("resolution:"):
            value = _vector_literal(stripped.split(":", 1)[1].split("#", 1)[0].strip())
            if value and len(value) == 2 and all(
                math.isfinite(v) and v > 0 and v == int(v) for v in value
            ):
                return [int(v) for v in value]
            return None
    return None


def _extract_yaml_matrix(
    text: str,
    section_name: str,
    matrix_name: str,
) -> list[list[float]] | None:
    lines = text.splitlines()
    section_index = next(
        (index for index, line in enumerate(lines) if line.strip() == f"{section_name}:"),
        None,
    )
    if section_index is None:
        return None
    section_indent = _indent(lines[section_index])
    section_end = len(lines)
    for index in range(section_index + 1, len(lines)):
        stripped = lines[index].strip()
        if (
            stripped
            and not stripped.startswith("#")
            and _indent(lines[index]) <= section_indent
        ):
            section_end = index
            break

    key_index = next(
        (
            index
            for index in range(section_index + 1, section_end)
            if lines[index].strip().startswith(f"{matrix_name}:")
        ),
        None,
    )
    if key_index is None:
        return None
    inline = lines[key_index].split(":", 1)[1].strip()
    if inline:
        fragments = [inline]
        balance = inline.count("[") - inline.count("]")
        index = key_index + 1
        while balance > 0 and index < section_end:
            fragment = lines[index].strip()
            fragments.append(fragment)
            balance += fragment.count("[") - fragment.count("]")
            index += 1
        return _matrix_literal(" ".join(fragments))

    rows: list[list[float]] = []
    for line in lines[key_index + 1 : section_end]:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("-"):
            row = _vector_literal(stripped[1:].strip())
            if row is None:
                return None
            rows.append(row)
            if len(rows) == 4:
                break
        elif rows:
            break
    return rows if len(rows) == 4 and all(len(row) == 4 for row in rows) else None


def _matrix_literal(value: str) -> list[list[float]] | None:
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return None
    if not isinstance(parsed, (list, tuple)) or len(parsed) != 4:
        return None
    rows = [_number_list(row) for row in parsed]
    if any(row is None or len(row) != 4 for row in rows):
        return None
    return [row for row in rows if row is not None]


def _vector_literal(value: str) -> list[float] | None:
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return None
    return _number_list(parsed)


def _number_list(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)):
        return None
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return None


def _indent(value: str) -> int:
    return len(value) - len(value.lstrip())


def _is_valid_serial_field(value: bytes) -> bool:
    has_text = False
    for byte in value:
        if byte == 0:
            return has_text
        if byte < 0x21 or byte > 0x7E:
            return False
        has_text = True
    return has_text


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _matrix(
    values: tuple[float, ...],
    rows: int,
    columns: int,
) -> list[list[float]]:
    return [
        [float(values[row * columns + column]) for column in range(columns)]
        for row in range(rows)
    ]


def _header_text(value: bytes) -> str:
    return value.split(b"\x00", 1)[0].decode("ascii", errors="replace")

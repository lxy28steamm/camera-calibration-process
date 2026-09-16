from __future__ import annotations

import ctypes
import re
import struct
from typing import Any
from urllib.parse import parse_qs, quote, urlparse

from ego_calibration.models import CalibrationError, CameraDevice


XU_GUID = "a29e7641-de04-47e3-8b2b-f4341aff003b"
SELECTOR_CALIB_INFO = 0x02
UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81


def scan_devices() -> tuple[CameraDevice, ...]:
    backend = _WindowsKsBackend()
    devices = []
    for target in backend.list_targets():
        serial = _serial_from_path(target["device_path"])
        identifier = _identifier(target)
        vid_pid = _vid_pid_text(target.get("vid"), target.get("pid"))
        label = f"Ego-Std · {serial or vid_pid or target['filter_index']}"
        devices.append(
            CameraDevice(
                kind="ego-std",
                identifier=identifier,
                label=label,
                model="ZXCZ/YCTC Stereo UVC",
                serial=serial,
                transport=f"Windows DirectShow/KS · {vid_pid}".rstrip(" ·"),
                path=target["device_path"],
            )
        )
    return tuple(sorted(devices, key=lambda item: item.label))


def query_for(identifier: str):
    return _WindowsKsBackend(identifier).query


def _identifier(target: dict[str, Any]) -> str:
    query = [f"node={target['node_id']}"]
    if target.get("device_path"):
        query.append(f"path={quote(target['device_path'], safe='')}")
    return f"ks://{target['filter_index']}?{'&'.join(query)}"


def _vid_pid_text(vid: int | None, pid: int | None) -> str:
    if vid is None or pid is None:
        return ""
    return f"USB {vid:04x}:{pid:04x}"


def _serial_from_path(device_path: str) -> str:
    match = re.search(
        r"vid_[0-9a-f]{4}&pid_[0-9a-f]{4}(?:&mi_[0-9a-f]{2})?#([^#]+)#",
        device_path,
        flags=re.IGNORECASE,
    )
    if not match:
        return ""
    candidate = match.group(1)
    if "&" in candidate or not re.fullmatch(r"[0-9A-Za-z._-]+", candidate):
        return ""
    return candidate


class _WindowsKsBackend:
    KSPROPERTY_TYPE_GET = 0x00000001
    KSPROPERTY_TYPE_SET = 0x00000002
    KSPROPERTY_TYPE_BASICSUPPORT = 0x00000200
    KSPROPERTY_TYPE_TOPOLOGY = 0x10000000

    def __init__(self, identifier: str = "") -> None:
        self.com = _load_com_types()
        self.xu_guid = self.com["GUID"]("{" + XU_GUID + "}")
        self.filter_index: int | None = None
        self.node_id: int | None = None
        self.device_path = ""
        self.target: dict[str, Any] | None = None
        if identifier:
            self._parse_identifier(identifier)

    def _parse_identifier(self, identifier: str) -> None:
        parsed = urlparse(identifier)
        if parsed.scheme != "ks":
            raise CalibrationError(f"Windows KS 设备标识无效：{identifier}")
        try:
            self.filter_index = int(parsed.netloc)
            values = parse_qs(parsed.query)
            self.node_id = int(values["node"][0])
            self.device_path = values.get("path", [""])[0]
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"Windows KS 设备标识无效：{identifier}") from exc

    def list_targets(self) -> list[dict[str, Any]]:
        targets = []
        try:
            for filter_index, moniker, display_name, vid, pid in self._monikers():
                targets.extend(
                    self._probe_filter(filter_index, moniker, display_name, vid, pid)
                )
        except Exception as exc:
            raise CalibrationError(f"Windows KS 相机检测失败：{_format_com_error(exc)}") from exc
        return targets

    def query(
        self,
        selector: int,
        request: int,
        size: int,
        payload: bytes | None,
    ) -> bytes:
        if self.target is None:
            self.target = self._resolve_target()
        flags = {
            UVC_GET_CUR: self.KSPROPERTY_TYPE_GET,
            UVC_SET_CUR: self.KSPROPERTY_TYPE_SET,
        }.get(request)
        if flags is None:
            raise CalibrationError(f"Windows KS 不支持 UVC 请求 0x{request:02x}")
        result = self._ks_property(
            self.target["ks"],
            self.target["node_id"],
            selector,
            flags,
            size,
            payload,
        )
        return b"" if payload is not None else result

    def _resolve_target(self) -> dict[str, Any]:
        targets = self.list_targets()
        for target in targets:
            device_matches = (
                target["device_path"] == self.device_path
                if self.device_path
                else target["filter_index"] == self.filter_index
            )
            if device_matches and target["node_id"] == self.node_id:
                return target
        raise CalibrationError("所选 Ego-Std Windows KS 相机已断开或 XU 节点已变化")

    def _probe_filter(
        self,
        filter_index: int,
        moniker: Any,
        display_name: str,
        vid: int | None,
        pid: int | None,
    ) -> list[dict[str, Any]]:
        try:
            unknown = moniker.BindToObject(
                None,
                None,
                ctypes.byref(self.com["IKsTopologyInfo"]._iid_),
            )
            topology = unknown.QueryInterface(self.com["IKsTopologyInfo"])
            ks = unknown.QueryInterface(self.com["IKsControl"])
        except Exception:
            return []

        targets = []
        for node_id in range(int(topology.get_NumNodes())):
            try:
                support = self._basic_support(ks, node_id, SELECTOR_CALIB_INFO)
            except Exception:
                continue
            if not support & self.KSPROPERTY_TYPE_GET:
                continue
            targets.append(
                {
                    "filter_index": filter_index,
                    "node_id": node_id,
                    "device_path": display_name,
                    "vid": vid,
                    "pid": pid,
                    "unknown": unknown,
                    "topology": topology,
                    "ks": ks,
                }
            )
        return targets

    def _monikers(self):
        device_enum = self.com["CreateObject"](
            self.com["CLSID_SystemDeviceEnum"],
            interface=self.com["ICreateDevEnum"],
        )
        enumerator = device_enum.CreateClassEnumerator(
            ctypes.byref(self.com["CLSID_VideoInputDeviceCategory"]),
            0,
        )
        index = 0
        while True:
            moniker, fetched = enumerator.Next(1)
            if fetched != 1 or not moniker:
                break
            display_name = self._moniker_display_name(moniker)
            vid, pid = _vid_pid_from_path(display_name)
            yield index, moniker, display_name, vid, pid
            index += 1

    def _moniker_display_name(self, moniker: Any) -> str:
        try:
            return str(moniker.GetDisplayName(self._create_bind_ctx(), None) or "")
        except Exception:
            return ""

    def _create_bind_ctx(self):
        bind_ctx = ctypes.POINTER(self.com["IBindCtx"])()
        result = ctypes.oledll.ole32.CreateBindCtx(0, ctypes.byref(bind_ctx))
        if result != 0:
            raise CalibrationError(
                f"CreateBindCtx 失败：HRESULT 0x{result & 0xFFFFFFFF:08x}"
            )
        return bind_ctx

    def _basic_support(self, ks: Any, node_id: int, selector: int) -> int:
        raw = self._ks_property(
            ks,
            node_id,
            selector,
            self.KSPROPERTY_TYPE_BASICSUPPORT,
            4,
            None,
        )
        return struct.unpack("<I", raw)[0]

    def _ks_property(
        self,
        ks: Any,
        node_id: int,
        selector: int,
        flags: int,
        size: int,
        payload: bytes | None,
    ) -> bytes:
        if payload is not None and len(payload) != size:
            raise CalibrationError("Windows KS payload 长度与请求不匹配")
        buffer = (ctypes.c_ubyte * max(size, 1))()
        for index, value in enumerate(payload or b""):
            buffer[index] = value
        node = self.com["KsNodeProperty"](
            self.com["KsProperty"](
                self.xu_guid,
                int(selector),
                int(flags) | self.KSPROPERTY_TYPE_TOPOLOGY,
            ),
            int(node_id),
            0,
        )
        try:
            ks.KsProperty(
                ctypes.cast(ctypes.byref(node), ctypes.POINTER(self.com["KsProperty"])),
                ctypes.sizeof(node),
                ctypes.cast(buffer, ctypes.c_void_p),
                int(size),
            )
        except Exception as exc:
            raise CalibrationError(
                f"Windows KS UVC XU 请求失败：{_format_com_error(exc)}"
            ) from exc
        return bytes(buffer[:size])


def _vid_pid_from_path(device_path: str) -> tuple[int | None, int | None]:
    match = re.search(
        r"vid_([0-9a-f]{4})&pid_([0-9a-f]{4})",
        device_path,
        flags=re.IGNORECASE,
    )
    if not match:
        return None, None
    return int(match.group(1), 16), int(match.group(2), 16)


def _format_com_error(exc: Exception) -> str:
    hresult = getattr(exc, "hresult", None)
    if hresult is None and getattr(exc, "args", None):
        hresult = exc.args[0]
    if isinstance(hresult, int):
        return f"HRESULT 0x{hresult & 0xFFFFFFFF:08x}: {exc}"
    return str(exc)


def _load_com_types() -> dict[str, Any]:
    try:
        from comtypes import COMMETHOD, GUID, HRESULT, IUnknown
        from comtypes.client import CreateObject
    except ImportError as exc:
        raise CalibrationError("Windows KS 后端缺少 comtypes 运行库") from exc

    pointer = ctypes.POINTER
    unsigned_long = ctypes.c_ulong

    class IBindCtx(IUnknown):
        _iid_ = GUID("{0000000e-0000-0000-C000-000000000046}")
        _methods_: list[Any] = []

    class IStream(IUnknown):
        _iid_ = GUID("{0000000c-0000-0000-C000-000000000046}")
        _methods_: list[Any] = []

    class IMoniker(IUnknown):
        _iid_ = GUID("{0000000f-0000-0000-C000-000000000046}")
        _methods_: list[Any] = []

    class IEnumMoniker(IUnknown):
        _iid_ = GUID("{00000102-0000-0000-C000-000000000046}")
        _methods_: list[Any] = []

    IEnumMoniker._methods_ = [
        COMMETHOD(
            [],
            HRESULT,
            "Next",
            (["in"], unsigned_long, "count"),
            (["out"], pointer(pointer(IMoniker)), "monikers"),
            (["out"], pointer(unsigned_long), "fetched"),
        ),
        COMMETHOD([], HRESULT, "Skip", (["in"], unsigned_long, "count")),
        COMMETHOD([], HRESULT, "Reset"),
        COMMETHOD(
            [],
            HRESULT,
            "Clone",
            (["out"], pointer(pointer(IEnumMoniker)), "enumerator"),
        ),
    ]
    IMoniker._methods_ = [
        COMMETHOD([], HRESULT, "GetClassID", (["out"], pointer(GUID), "class_id")),
        COMMETHOD([], HRESULT, "IsDirty"),
        COMMETHOD([], HRESULT, "Load", (["in"], pointer(IStream), "stream")),
        COMMETHOD(
            [],
            HRESULT,
            "Save",
            (["in"], pointer(IStream), "stream"),
            (["in"], ctypes.c_int, "clear_dirty"),
        ),
        COMMETHOD(
            [],
            HRESULT,
            "GetSizeMax",
            (["out"], pointer(ctypes.c_longlong), "size"),
        ),
        COMMETHOD(
            [],
            HRESULT,
            "BindToObject",
            (["in"], pointer(IBindCtx), "bind_context"),
            (["in"], pointer(IMoniker), "left_moniker"),
            (["in"], pointer(GUID), "interface_id"),
            (["out"], pointer(pointer(IUnknown)), "result"),
        ),
        COMMETHOD([], HRESULT, "BindToStorage"),
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
        COMMETHOD(
            [],
            HRESULT,
            "GetDisplayName",
            (["in"], pointer(IBindCtx), "bind_context"),
            (["in"], pointer(IMoniker), "left_moniker"),
            (["out"], pointer(ctypes.c_wchar_p), "display_name"),
        ),
    ]

    class ICreateDevEnum(IUnknown):
        _iid_ = GUID("{29840822-5B84-11D0-BD3B-00A0C911CE86}")
        _methods_ = [
            COMMETHOD(
                [],
                HRESULT,
                "CreateClassEnumerator",
                (["in"], pointer(GUID), "device_class"),
                (["out"], pointer(pointer(IEnumMoniker)), "enumerator"),
                (["in"], unsigned_long, "flags"),
            )
        ]

    class KsTopologyConnection(ctypes.Structure):
        _fields_ = [
            ("from_node", unsigned_long),
            ("from_node_pin", unsigned_long),
            ("to_node", unsigned_long),
            ("to_node_pin", unsigned_long),
        ]

    class IKsTopologyInfo(IUnknown):
        _iid_ = GUID("{720D4AC0-7533-11D0-A5D6-28DB04C10000}")
        _methods_ = [
            COMMETHOD([], HRESULT, "get_NumCategories", (["out"], pointer(unsigned_long), "count")),
            COMMETHOD([], HRESULT, "get_Category"),
            COMMETHOD([], HRESULT, "get_NumConnections", (["out"], pointer(unsigned_long), "count")),
            COMMETHOD([], HRESULT, "get_ConnectionInfo"),
            COMMETHOD([], HRESULT, "get_NodeName"),
            COMMETHOD([], HRESULT, "get_NumNodes", (["out"], pointer(unsigned_long), "count")),
            COMMETHOD(
                [],
                HRESULT,
                "get_NodeType",
                (["in"], unsigned_long, "node_id"),
                (["out"], pointer(GUID), "node_type"),
            ),
            COMMETHOD([], HRESULT, "CreateNodeInstance"),
        ]

    class KsProperty(ctypes.Structure):
        _fields_ = [("set", GUID), ("id", unsigned_long), ("flags", unsigned_long)]

    class KsNodeProperty(ctypes.Structure):
        _fields_ = [
            ("property", KsProperty),
            ("node_id", unsigned_long),
            ("reserved", unsigned_long),
        ]

    class IKsControl(IUnknown):
        _iid_ = GUID("{28F54685-06FD-11D2-B27A-00A0C9223196}")
        _methods_ = [
            COMMETHOD(
                [],
                HRESULT,
                "KsProperty",
                (["in"], pointer(KsProperty), "property"),
                (["in"], unsigned_long, "property_length"),
                (["in", "out"], ctypes.c_void_p, "property_data"),
                (["in"], unsigned_long, "data_length"),
                (["out"], pointer(unsigned_long), "bytes_returned"),
            ),
            COMMETHOD([], HRESULT, "KsMethod"),
            COMMETHOD([], HRESULT, "KsEvent"),
        ]

    return {
        "GUID": GUID,
        "IBindCtx": IBindCtx,
        "CreateObject": CreateObject,
        "ICreateDevEnum": ICreateDevEnum,
        "IKsTopologyInfo": IKsTopologyInfo,
        "IKsControl": IKsControl,
        "KsProperty": KsProperty,
        "KsNodeProperty": KsNodeProperty,
        "CLSID_SystemDeviceEnum": GUID("{62BE5D10-60EB-11d0-BD3B-00A0C911CE86}"),
        "CLSID_VideoInputDeviceCategory": GUID(
            "{860BB310-5D01-11d0-BD3B-00A0C911CE86}"
        ),
    }

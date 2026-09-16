from __future__ import annotations

from typing import Any

from ego_calibration.models import CalibrationError, CameraDevice


_CAMERAS = (
    ("rgb", "CAM_A"),
    ("left_mono", "CAM_B"),
    ("right_mono", "CAM_C"),
)


def scan_devices() -> tuple[CameraDevice, ...]:
    dai = _load_depthai()
    try:
        infos = tuple(dai.Device.getAllAvailableDevices())
    except Exception as exc:
        raise CalibrationError(f"Ego-Lite 检测失败：{exc}") from exc

    devices = {}
    for info in infos:
        mxid = _device_mxid(info)
        if not mxid:
            continue
        connection = _info_text(info, "protocol")
        path = _info_text(info, "name")
        devices[mxid] = CameraDevice(
            kind="ego-lite",
            identifier=mxid,
            label=f"Ego-Lite · {mxid}",
            model="Luxonis Ego-Lite",
            serial=mxid,
            transport=f"DepthAI {connection}".strip(),
            path=path or mxid,
        )
    return tuple(devices[mxid] for mxid in sorted(devices))


def read_calibration(mxid: str) -> dict[str, Any]:
    if not mxid.strip():
        raise CalibrationError("请先选择 Ego-Lite 相机")

    dai = _load_depthai()
    try:
        info = next(
            (
                item
                for item in dai.Device.getAllAvailableDevices()
                if _device_mxid(item) == mxid
            ),
            None,
        )
        if info is None:
            raise CalibrationError(f"未发现 Ego-Lite 相机：{mxid}")
        device = dai.Device(info)
        try:
            return _read_calibration_json(dai, device, mxid)
        finally:
            device.close()
    except CalibrationError:
        raise
    except Exception as exc:
        raise CalibrationError(f"DepthAI EEPROM 读取失败：{exc}") from exc


def _load_depthai() -> Any:
    try:
        import depthai as dai
    except ImportError as exc:
        raise CalibrationError("未安装 DepthAI 运行库") from exc
    return dai


def _device_mxid(info: Any) -> str:
    getter = getattr(info, "getMxId", None)
    value = getter() if callable(getter) else getattr(info, "mxid", "")
    return str(value).strip()


def _info_text(info: Any, name: str) -> str:
    getter = getattr(info, f"get{name.capitalize()}", None)
    value = getter() if callable(getter) else getattr(info, name, "")
    text = str(value).strip()
    if text.lower() == "none":
        return ""
    if text.startswith(("XLinkProtocol.", "XLinkDeviceState.")):
        return text.rsplit(".", 1)[-1]
    return text


def _read_calibration_json(
    dai: Any,
    device: Any,
    mxid: str,
    calibration: Any | None = None,
) -> dict[str, Any]:
    if calibration is None:
        calibration = device.readCalibration2()
    sockets = {
        name: getattr(dai.CameraBoardSocket, socket_name)
        for name, socket_name in _CAMERAS
    }
    sensors = {
        feature.socket: str(feature.sensorName)
        for feature in device.getConnectedCameraFeatures()
    }
    cameras = {
        name: _camera_payload(
            calibration,
            socket,
            socket_name,
            sensors.get(socket, ""),
        )
        for name, socket_name in _CAMERAS
        for socket in (sockets[name],)
    }
    camera_extrinsics = {
        f"T_{target}_from_{source}": _transform(
            calibration.getCameraExtrinsics(
                sockets[source],
                sockets[target],
                False,
            )
        )
        for source, target in (
            ("rgb", "left_mono"),
            ("right_mono", "left_mono"),
            ("left_mono", "rgb"),
            ("right_mono", "rgb"),
            ("left_mono", "right_mono"),
            ("rgb", "right_mono"),
        )
    }
    imu_extrinsics: dict[str, Any] = {}
    for name, _socket_name in _CAMERAS:
        socket = sockets[name]
        imu_extrinsics[f"T_imu_from_{name}"] = _optional_transform(
            lambda: calibration.getCameraToImuExtrinsics(socket, False)
        )
        imu_extrinsics[f"T_{name}_from_imu"] = _optional_transform(
            lambda: calibration.getImuToCameraExtrinsics(socket, False)
        )
    return {
        "camera_extrinsics": camera_extrinsics,
        "cameras": cameras,
        "depthai_version": str(getattr(dai, "__version__", "")),
        "device": {
            "imu_firmware": str(device.getIMUFirmwareVersion()),
            "imu_type": str(device.getConnectedIMU()),
            "mxid": mxid,
            "usb_speed": str(device.getUsbSpeed()),
        },
        "imu_extrinsics": imu_extrinsics,
        "raw_eeprom": calibration.eepromToJson(),
        "schema": {
            "extrinsic_translation_source": (
                "calibrated EEPROM values, not board specification values"
            ),
            "matrix_cm": "4x4 homogeneous transform; translation in centimeters",
            "matrix_m": "4x4 homogeneous transform; translation in meters",
            "transform_naming": (
                "T_target_from_source maps source-frame points into target frame"
            ),
        },
    }


def _camera_payload(
    calibration: Any,
    socket: Any,
    socket_name: str,
    sensor: str,
) -> dict[str, Any]:
    intrinsics, width, height = calibration.getDefaultIntrinsics(socket)
    return {
        "K": _matrix(intrinsics, 3, 3),
        "calibration_resolution": [int(width), int(height)],
        "distortion_coefficients": [
            float(value)
            for value in calibration.getDistortionCoefficients(socket)
        ],
        "distortion_model": str(calibration.getDistortionModel(socket)),
        "lens_position": int(calibration.getLensPosition(socket)),
        "sensor": sensor,
        "socket": socket_name,
    }


def _transform(value: Any) -> dict[str, Any]:
    matrix_cm = _matrix(value, 4, 4)
    matrix_m = [row.copy() for row in matrix_cm]
    for row in range(3):
        matrix_m[row][3] /= 100.0
    return {
        "available": True,
        "matrix_cm": matrix_cm,
        "matrix_m": matrix_m,
    }


def _optional_transform(getter: Any) -> dict[str, Any]:
    try:
        return _transform(getter())
    except Exception as exc:
        return {
            "available": False,
            "error": str(exc) or type(exc).__name__,
        }


def _matrix(value: Any, rows: int, columns: int) -> list[list[float]]:
    matrix = [[float(item) for item in row] for row in value]
    if len(matrix) != rows or any(len(row) != columns for row in matrix):
        raise CalibrationError(f"标定矩阵维度无效，应为 {rows}x{columns}")
    return matrix

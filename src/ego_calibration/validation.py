from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


_CAMERAS = ("rgb", "left_mono", "right_mono")
_CAMERA_TRANSFORMS = (
    "T_left_mono_from_rgb",
    "T_left_mono_from_right_mono",
    "T_rgb_from_left_mono",
    "T_rgb_from_right_mono",
    "T_right_mono_from_left_mono",
    "T_right_mono_from_rgb",
)
_IMU_TRANSFORMS = tuple(
    name
    for camera in _CAMERAS
    for name in (f"T_imu_from_{camera}", f"T_{camera}_from_imu")
)
_STD_MATRICES = {
    "K1": (3, 3),
    "D1": (1, 8),
    "K2": (3, 3),
    "D2": (1, 8),
    "R": (3, 3),
    "T": (3, 1),
    "R1": (3, 3),
    "R2": (3, 3),
    "P1": (3, 4),
    "P2": (3, 4),
    "Q": (4, 4),
}
_STD_REPROJECTION_METRICS = (
    ("左目内参 RMS", "left_calibrate_rms"),
    ("右目内参 RMS", "right_calibrate_rms"),
    ("双目内外参联合 RMS", "stereo_rms"),
)
_ORTHOGONAL_TOLERANCE = 0.05
_DETERMINANT_TOLERANCE = 0.05


@dataclass(frozen=True, slots=True)
class ValidationCheck:
    category: str
    name: str
    passed: bool | None  # None 表示缺少观测/误差数据，无法评估。
    detail: str
    informational: bool = False


@dataclass(frozen=True, slots=True)
class ValidationReport:
    checks: tuple[ValidationCheck, ...]

    @property
    def passed(self) -> bool:
        return all(check.passed is True for check in self.checks)

    @property
    def passed_count(self) -> int:
        return sum(check.passed is True and not check.informational for check in self.checks)

    @property
    def info_count(self) -> int:
        return sum(check.informational for check in self.checks)

    @property
    def failure_count(self) -> int:
        return sum(check.passed is False for check in self.checks)

    @property
    def skipped_count(self) -> int:
        return sum(check.passed is None for check in self.checks)

    @property
    def rotation_count(self) -> int:
        return sum(check.category == "旋转矩阵" for check in self.checks)


def validate_calibration(kind: str, payload: Any) -> ValidationReport:
    if not isinstance(payload, dict):
        return ValidationReport(
            (ValidationCheck("完整性", "标定数据", False, "标定结果不是 JSON 对象"),)
        )
    if kind == "uvc-stereo":
        from ego_calibration.inspection import StereoModel, InspectionSettings
        try:
            StereoModel.from_payload(kind, payload, InspectionSettings())
            checks = [ValidationCheck("完整性", "通用双目标定", True, "已检查内参、尺寸、旋转与以米为单位的平移；精度需另行复测")]
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            checks = [ValidationCheck("完整性", "通用双目标定", False, str(exc))]
    elif kind == "ego-lite":
        checks = _validate_ego_lite(payload)
    elif kind in ("ego-std", "ego-std-235"):
        checks = _validate_ego_std(payload)
    else:
        checks = [ValidationCheck("完整性", "相机型号", False, f"未知型号：{kind}")]
    return ValidationReport(tuple(checks))


def _validate_ego_lite(payload: dict[str, Any]) -> list[ValidationCheck]:
    checks: list[ValidationCheck] = []
    device = payload.get("device")
    base_ok = (
        isinstance(device, dict)
        and bool(str(device.get("mxid", "")).strip())
        and bool(str(device.get("imu_type", "")).strip())
        and isinstance(payload.get("raw_eeprom"), dict)
        and isinstance(payload.get("schema"), dict)
    )
    checks.append(
        ValidationCheck(
            "完整性",
            "设备与 EEPROM 信息",
            base_ok,
            "MXID、IMU、原始 EEPROM 和 schema 均存在" if base_ok else "缺少 MXID、IMU、原始 EEPROM 或 schema",
        )
    )

    cameras = payload.get("cameras")
    for name in _CAMERAS:
        camera = cameras.get(name) if isinstance(cameras, dict) else None
        ok, detail = _validate_lite_camera(camera)
        checks.append(ValidationCheck("完整性", f"相机 {name}", ok, detail))

    for section_name, section, transforms in (
        ("相机外参", payload.get("camera_extrinsics"), _CAMERA_TRANSFORMS),
        ("IMU 外参", payload.get("imu_extrinsics"), _IMU_TRANSFORMS),
    ):
        for name in transforms:
            transform = section.get(name) if isinstance(section, dict) else None
            ok, detail = _validate_transform_payload(transform)
            checks.append(ValidationCheck("完整性", f"{section_name} {name}", ok, detail))
            matrix = transform.get("matrix_cm") if isinstance(transform, dict) else None
            rotation_ok, rotation_detail = _validate_rotation(matrix)
            checks.append(
                ValidationCheck(
                    "旋转矩阵",
                    f"{section_name} {name}",
                    rotation_ok,
                    rotation_detail,
                )
            )
    names = [f"相机 {camera} 内参 RMS" for camera in _CAMERAS]
    for name in (*names, "相机间内外参联合 RMS"):
        checks.append(_unavailable_reprojection(name, "当前读取的 Lite EEPROM 未提供 RMS"))
    return checks


def _validate_ego_std(payload: dict[str, Any]) -> list[ValidationCheck]:
    checks: list[ValidationCheck] = []
    identity = payload.get("device_identity")
    if (
        isinstance(identity, dict)
        and identity.get("usb_serial_number")
        and identity.get("calibration_serial_number")
    ):
        serials_match = bool(identity.get("serials_match"))
        comparison = str(identity.get("serial_comparison", "")).strip()
        if serials_match or comparison == "match":
            detail = "USB 序列号与标定 SN 一致"
        else:
            detail = (
                "USB 序列号与标定 SN 均已读取，但厂家协议未提供两套编号的映射；"
                "仅凭字符串不同不能判定标定 blob 不属于当前设备"
            )
        checks.append(
            ValidationCheck(
                "设备身份",
                "USB 与标定序列号",
                True,
                detail,
            )
        )
    header = payload.get("header")
    payload_format = payload.get("format")
    header_identity_ok = isinstance(header, dict) and (
        bool(str(header.get("serial_number", "")).strip())
        or (
            header.get("header_format") == "legacy_metadata"
            and bool(str(header.get("device_model", "")).strip())
        )
    )
    header_ok = (
        payload_format in ("stereo_calibration", "kalibr_camchain_imucam")
        and isinstance(header, dict)
        and header.get("magic") == "ZXCZ"
        and header.get("schema_version") in (1, 2)
        and isinstance(header.get("payload_length"), int)
        and header_identity_ok
    )
    checks.append(
        ValidationCheck(
            "完整性",
            "标定头信息",
            header_ok,
            "格式、schema、长度和序列号均存在" if header_ok else "格式、schema、长度或序列号缺失",
        )
    )

    if payload_format == "kalibr_camchain_imucam":
        _validate_kalibr_payload(payload, checks)
    else:
        _validate_stereo_payload(payload, checks)

    return checks


def _validate_stereo_payload(
    payload: dict[str, Any],
    checks: list[ValidationCheck],
) -> None:
    calibration = payload.get("calibration")
    for name, (rows, columns) in _STD_MATRICES.items():
        value = calibration.get(name) if isinstance(calibration, dict) else None
        if rows == 1:
            ok, detail = _validate_vector(value, columns)
        else:
            ok, detail = _validate_matrix(value, rows, columns)
        checks.append(ValidationCheck("完整性", f"双目标定 {name}", ok, detail))

    metrics = payload.get("metrics")
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
    metrics_ok = isinstance(metrics, dict) and all(
        _is_finite_number(metrics.get(name)) for name in metric_names
    )
    checks.append(
        ValidationCheck(
            "完整性",
            "标定质量指标",
            metrics_ok,
            "8 项质量指标完整" if metrics_ok else "质量指标缺失或包含非有限数值",
        )
    )
    for name, field in _STD_REPROJECTION_METRICS:
        if not isinstance(metrics, dict) or field not in metrics:
            checks.append(_unavailable_reprojection(name, f"标定 blob 缺少 metrics.{field}"))
        else:
            checks.append(_validate_reprojection_rms(name, metrics[field], f"metrics.{field}"))

    if isinstance(calibration, dict):
        for name in ("R", "R1", "R2"):
            rotation_ok, rotation_detail = _validate_rotation(calibration.get(name))
            checks.append(
                ValidationCheck(
                    "旋转矩阵",
                    f"双目标定 {name}",
                    rotation_ok,
                    rotation_detail,
                )
            )
    else:
        for name in ("R", "R1", "R2"):
            checks.append(
                ValidationCheck("旋转矩阵", f"双目标定 {name}", False, "矩阵缺失")
            )


def _validate_kalibr_payload(
    payload: dict[str, Any],
    checks: list[ValidationCheck],
) -> None:
    metadata = payload.get("payload")
    yaml_text = payload.get("kalibr_yaml")
    required = metadata.get("required_fields") if isinstance(metadata, dict) else None
    payload_ok = (
        isinstance(metadata, dict)
        and metadata.get("parser") == "kalibr_yaml_v2"
        and metadata.get("encoding") == "utf-8"
        and isinstance(metadata.get("length"), int)
        and metadata["length"] > 0
        and isinstance(required, list)
        and all(isinstance(field, str) and field in str(yaml_text) for field in required)
    )
    checks.append(
        ValidationCheck(
            "完整性",
            "Kalibr YAML 数据",
            payload_ok,
            "UTF-8 原始 YAML 和必需字段完整" if payload_ok else "YAML、编码或必需字段缺失",
        )
    )

    calibration = payload.get("kalibr_calibration")
    for display_name, section_name, matrix_name in (
        ("Kalibr cam0.T_cam_imu", "cam0", "T_cam_imu"),
        ("Kalibr cam1.T_cam_imu", "cam1", "T_cam_imu"),
        ("Kalibr cam1.T_cn_cnm1", "cam1", "T_cn_cnm1"),
    ):
        section = calibration.get(section_name) if isinstance(calibration, dict) else None
        matrix = section.get(matrix_name) if isinstance(section, dict) else None
        ok, detail = _validate_homogeneous_matrix(matrix)
        checks.append(ValidationCheck("完整性", display_name, ok, detail))
        rotation_ok, rotation_detail = _validate_rotation(matrix)
        checks.append(
            ValidationCheck("旋转矩阵", display_name, rotation_ok, rotation_detail)
        )

    for name, _field in _STD_REPROJECTION_METRICS:
        checks.append(_unavailable_reprojection(name, "当前读取的 Kalibr 参数未提供 RMS"))


def _unavailable_reprojection(name: str, reason: str) -> ValidationCheck:
    return ValidationCheck(
        "二维重投影误差",
        name,
        None,
        f"{reason}；缺少标定板三维点、二维角点观测和对应位姿，无法重算像素误差",
    )


def _validate_reprojection_rms(name: str, value: Any, source: str) -> ValidationCheck:
    if not _is_finite_number(value) or value < 0:
        return ValidationCheck(
            "二维重投影误差", name, False, f"{source} 无效：RMS 必须是有限且非负的像素值"
        )
    detail = (
        f"RMS={value:.6g} px；来源：设备保存的 {source}（标定时误差）；"
        "仅展示，未设置合格阈值"
    )
    return ValidationCheck("二维重投影误差", name, True, detail, informational=True)


def _validate_lite_camera(value: Any) -> tuple[bool, str]:
    if not isinstance(value, dict):
        return False, "相机标定缺失"
    k_ok, _detail = _validate_matrix(value.get("K"), 3, 3)
    resolution = value.get("calibration_resolution")
    resolution_ok = (
        isinstance(resolution, list)
        and len(resolution) == 2
        and all(_is_finite_number(item) and item > 0 for item in resolution)
    )
    distortion = value.get("distortion_coefficients")
    distortion_ok = (
        isinstance(distortion, list)
        and bool(distortion)
        and all(_is_finite_number(item) for item in distortion)
    )
    metadata_ok = bool(str(value.get("distortion_model", "")).strip()) and bool(
        str(value.get("socket", "")).strip()
    )
    if k_ok and resolution_ok and distortion_ok and metadata_ok:
        return True, "内参、分辨率、畸变和相机信息完整"
    return False, "内参、分辨率、畸变或相机信息缺失/无效"


def _validate_transform_payload(value: Any) -> tuple[bool, str]:
    if not isinstance(value, dict):
        return False, "外参缺失"
    cm_ok, _cm_detail = _validate_homogeneous_matrix(value.get("matrix_cm"))
    m_ok, _m_detail = _validate_homogeneous_matrix(value.get("matrix_m"))
    if value.get("available") is True and cm_ok and m_ok:
        return True, "cm/m 两套 4x4 齐次矩阵完整"
    return False, "available、matrix_cm 或 matrix_m 缺失/无效"


def _validate_homogeneous_matrix(value: Any) -> tuple[bool, str]:
    ok, detail = _validate_matrix(value, 4, 4)
    if not ok:
        return ok, detail
    bottom = value[3]
    if any(abs(float(bottom[index])) > 1e-6 for index in range(3)) or abs(
        float(bottom[3]) - 1.0
    ) > 1e-6:
        return False, "齐次矩阵末行为非法值"
    return True, "4x4 齐次矩阵完整"


def _validate_matrix(value: Any, rows: int, columns: int) -> tuple[bool, str]:
    if not isinstance(value, list) or len(value) != rows:
        return False, f"矩阵应为 {rows}x{columns}"
    if any(not isinstance(row, list) or len(row) != columns for row in value):
        return False, f"矩阵应为 {rows}x{columns}"
    if any(not _is_finite_number(item) for row in value for item in row):
        return False, "矩阵包含非有限数值"
    return True, f"{rows}x{columns} 矩阵完整"


def _validate_vector(value: Any, length: int) -> tuple[bool, str]:
    if not isinstance(value, list) or len(value) != length:
        return False, f"向量长度应为 {length}"
    if any(not _is_finite_number(item) for item in value):
        return False, "向量包含非有限数值"
    return True, f"长度 {length} 向量完整"


def _validate_rotation(value: Any) -> tuple[bool, str]:
    if not isinstance(value, list) or len(value) < 3:
        return False, "矩阵缺失"
    if any(not isinstance(row, list) or len(row) < 3 for row in value[:3]):
        return False, "无法提取 3x3 旋转块"
    rotation = [row[:3] for row in value[:3]]
    if any(not _is_finite_number(item) for row in rotation for item in row):
        return False, "旋转矩阵包含非有限数值"
    if all(abs(float(item)) <= 1e-9 for row in rotation for item in row):
        return False, "旋转矩阵全为 0"

    orthogonal_error = max(
        abs(
            sum(float(rotation[k][row]) * float(rotation[k][column]) for k in range(3))
            - (1.0 if row == column else 0.0)
        )
        for row in range(3)
        for column in range(3)
    )
    determinant = _determinant_3x3(rotation)
    detail = f"det={determinant:.6f}，正交误差={orthogonal_error:.3g}"
    passed = (
        abs(determinant - 1.0) <= _DETERMINANT_TOLERANCE
        and orthogonal_error <= _ORTHOGONAL_TOLERANCE
    )
    if not passed:
        detail += "（应接近 det=1 且 RᵀR=I）"
    return passed, detail


def _determinant_3x3(matrix: list[list[Any]]) -> float:
    a, b, c = (float(item) for item in matrix[0])
    d, e, f = (float(item) for item in matrix[1])
    g, h, i = (float(item) for item in matrix[2])
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

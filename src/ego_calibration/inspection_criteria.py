"""Explicit inspection limits and descriptive comparisons with stored calibration."""
from __future__ import annotations

import math


def inspection_criteria(settings, stats):
    criteria = []
    for name, key, statistic, limit in (
        ("左目重投影", "left", "rms", settings.mono_rms_limit),
        ("右目重投影", "right", "rms", settings.mono_rms_limit),
        ("左预测右", "left_to_right", "rms", settings.stereo_rms_limit),
        ("右预测左", "right_to_left", "rms", settings.stereo_rms_limit),
        ("极线偏差", "epipolar", "p95", settings.epipolar_p95_limit),
    ):
        value = (stats.get(key) or {}).get(statistic)
        criteria.append({
            "key": key, "name": name, "statistic": statistic, "unit": "px",
            "limit": limit, "value": value,
            "status": "incomplete" if value is None else "pass" if value <= limit else "fail",
        })
    defaults = (settings.mono_rms_limit, settings.stereo_rms_limit, settings.epipolar_p95_limit) == (1.0, 1.5, 1.0)
    return {
        "basis": "software_reference" if defaults else "user_configured",
        "label": "软件默认参考阈值" if defaults else "用户配置阈值",
        "reference": settings.criteria_reference.strip() or "未提供厂商或项目验收文件",
        "standards_compliance": "not_assessed",
        "criteria": criteria,
        "notes": [
            "通过表示满足本次配置及采样条件，不代表通过厂商、国标或 ISO 认证；依据备注由操作者填写。",
            "重投影 / 跨目预测 RMS 按所有有效角点的二维距离汇总，使用原始单目像素；极线 P95 是校正后对应角点垂直于极线方向的偏差的第 95 百分位，使用本次校正投影的像素尺度。",
            "误差项达标后，仍须满足有效样本数、覆盖范围、距离与倾角变化、分辨率确认和数据完整性要求。",
        ],
    }


def calibration_comparison(payload, acceptance, *, resolution_confirmed):
    # Only consume fields with a known meaning in the device blob schema.
    metrics = payload.get("metrics", {}) if payload.get("format") == "stereo_calibration" else {}

    def historical(field):
        value = metrics.get(field)
        return float(value) if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None

    rows = []
    for criterion in acceptance["criteria"]:
        key = criterion["key"]
        old = historical({"left": "left_calibrate_rms", "right": "right_calibrate_rms"}.get(key, ""))
        new = criterion["value"]
        delta = new - old if new is not None and old is not None and resolution_confirmed else None
        if key in ("left", "right"):
            note = "历史拟合与本次固定参数复测使用不同图像和位姿估计；差值仅作参考，不作为退化或合格判据。"
            if not resolution_confirmed:
                note = "标定分辨率未确认，暂不计算差值。" + note
        elif key == "epipolar":
            note = "当前设备标定格式未提供同口径极线 P95，不能用历史 RMS 代替。"
        else:
            note = "历史双目联合 RMS 与跨目预测 RMS 算法不同，不计算差值。"
        rows.append({**criterion, "historical_value": old, "delta_px": delta, "note": note})
    rows.append({
        "key": "historical_stereo", "name": "设备双目联合", "statistic": "rms", "unit": "px",
        "historical_value": historical("stereo_rms"), "value": None, "delta_px": None,
        "limit": None, "status": "reference", "note": "设备保存的联合标定拟合误差；本次未重新联合标定，无同口径复测值。",
    })
    return {
        "reference": "本次使用的设备 / 导入标定记录",
        "calibration_serial": payload.get("header", {}).get("serial_number") or payload.get("device", {}).get("mxid"),
        "historical_sample_count": historical("sample_count"),
        "parameters_fixed": True,
        "note": "本次固定已有 K、D、R、T 和基线，仅估计标定板位姿；没有求解新的相机参数，不能据此报告内参、外参或基线漂移。",
        "rows": rows,
    }

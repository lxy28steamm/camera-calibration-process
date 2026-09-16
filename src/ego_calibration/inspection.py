"""Read-only verification of stored stereo calibration against fresh AprilGrid images."""
from __future__ import annotations

import ast
import csv
import html
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ego_calibration.models import CalibrationError
from ego_calibration.inspection_criteria import calibration_comparison, inspection_criteria
from ego_calibration.validation import validate_calibration


@dataclass(frozen=True)
class InspectionSettings:
    rows: int = 6
    columns: int = 6
    tag_size_m: float = 0.055
    tag_spacing_m: float = 0.0165
    width: int = 1920
    height: int = 1080
    resolution_confirmed: bool = False
    duration_s: int = 120
    interval_s: float = 1.0
    min_views: int = 30
    min_tags: int = 6
    min_cells: int = 7
    min_sharpness: float = 40.0
    mono_rms_limit: float = 1.0
    stereo_rms_limit: float = 1.5
    epipolar_p95_limit: float = 1.0
    criteria_reference: str = ""

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> InspectionSettings:
        if not isinstance(values, dict):
            raise CalibrationError("检测设置必须为对象")
        allowed = cls.__dataclass_fields__
        if set(values) - allowed.keys():
            raise CalibrationError("检测设置包含未知字段")
        result = cls(**values)
        for name, low, high in (
            ("rows", 2, 20), ("columns", 2, 20), ("width", 64, 8192),
            ("height", 64, 8192), ("duration_s", 5, 1800),
            ("min_views", 3, 300), ("min_tags", 2, 100), ("min_cells", 1, 9),
        ):
            value = getattr(result, name)
            if type(value) is not int or not low <= value <= high:
                raise CalibrationError(f"{name} 必须为 {low}–{high} 范围内的整数")
        for name, low, high in (
            ("tag_size_m", 0.005, 1), ("tag_spacing_m", 0.001, 1),
            ("interval_s", 0.3, 10), ("min_sharpness", 0, 10000),
            ("mono_rms_limit", 0.05, 20), ("stereo_rms_limit", 0.05, 30),
            ("epipolar_p95_limit", 0.05, 20),
        ):
            value = getattr(result, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise CalibrationError(f"{name} 数值超出允许范围")
        if type(result.resolution_confirmed) is not bool:
            raise CalibrationError("分辨率确认必须为布尔值")
        if not isinstance(result.criteria_reference, str) or len(result.criteria_reference) > 200:
            raise CalibrationError("判定依据备注必须为不超过 200 字的文字")
        if result.min_tags > result.rows * result.columns:
            raise CalibrationError("最少标签数不能大于标定板标签总数")
        return result

    def points(self, ids: list[int], corner_order=(0, 1, 2, 3)) -> np.ndarray:
        points = []
        step = self.tag_size_m + self.tag_spacing_m
        for tag in ids:
            x, y = (tag % self.columns) * step, (tag // self.columns) * step
            size = self.tag_size_m
            # OpenCV ArUco corners: top-left, top-right, bottom-right, bottom-left.
            corners = ((x, y, 0), (x + size, y, 0), (x + size, y + size, 0), (x, y + size, 0))
            points.extend(corners[index] for index in corner_order)
        return np.asarray(points, dtype=np.float64)


@dataclass
class CameraModel:
    K: np.ndarray
    D: np.ndarray
    resolution: tuple[int, int]
    model: str = "radtan"

    def normalized(self, pixels: np.ndarray, R=None, P=None) -> np.ndarray:
        points = np.ascontiguousarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
        function = cv2.fisheye.undistortPoints if self.model == "equidistant" else cv2.undistortPoints
        return function(points, self.K, self.D, R=R, P=P).reshape(-1, 2)

    def project(self, points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        function = cv2.fisheye.projectPoints if self.model == "equidistant" else cv2.projectPoints
        projected, _ = function(np.ascontiguousarray(points, dtype=np.float64).reshape(-1, 1, 3), cv2.Rodrigues(rotation)[0], translation, self.K, self.D)
        return projected.reshape(-1, 2)

    def pose(self, points: np.ndarray, pixels: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        success, rotation, translation = cv2.solvePnP(
            np.ascontiguousarray(points, dtype=np.float64), np.ascontiguousarray(self.normalized(pixels)), np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE
        )
        matrix = cv2.Rodrigues(rotation)[0]
        if not success or not np.isfinite(translation).all() or np.min((points @ matrix.T + translation.reshape(1, 3))[:, 2]) <= 0:
            raise CalibrationError("标定板位姿无效或位于相机后方")
        return matrix, translation.reshape(3, 1)


def _array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{name} 数据无效") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise CalibrationError(f"{name} 维度或数值无效")
    return result


def _camera(K, D, resolution, model="radtan") -> CameraModel:
    matrix = _array(K, (3, 3), "相机内参")
    distortion = np.asarray(D, dtype=np.float64).reshape(-1)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
        raise CalibrationError("焦距或内参末行无效")
    if model not in ("radtan", "equidistant", "none"):
        raise CalibrationError(f"暂不支持此镜头模型：{model}")
    lengths = (4,) if model == "equidistant" else (4, 5, 8, 12, 14)
    if model == "none":
        distortion = np.zeros(5)
    if len(distortion) not in lengths or not np.isfinite(distortion).all():
        raise CalibrationError("畸变参数数量或数值无效")
    if not isinstance(resolution, (list, tuple)) or len(resolution) != 2 or any(type(v) is not int or v <= 0 for v in resolution):
        raise CalibrationError("标定分辨率缺失或无效")
    return CameraModel(matrix, distortion, tuple(resolution), model)


def _yaml_camera(text: str, section: str) -> dict[str, Any]:
    """Read Kalibr's scalar/inline-vector camera fields without a YAML dependency."""
    fields: dict[str, Any] = {}
    active = False
    for line in text.splitlines():
        stripped = line.split("#", 1)[0].strip()
        if stripped == f"{section}:":
            active = True
            continue
        if not stripped or not active:
            continue
        if not line[0].isspace():
            break
        if ":" in stripped:
            key, value = stripped.split(":", 1)
            if key in ("intrinsics", "distortion_coeffs", "resolution"):
                try:
                    fields[key] = ast.literal_eval(value.strip())
                except (ValueError, SyntaxError):
                    raise CalibrationError(f"Kalibr {section}.{key} 必须为有效的行内数组") from None
            elif key in ("camera_model", "distortion_model"):
                fields[key] = value.strip().strip("'\"")
    return fields


@dataclass
class StereoModel:
    left: CameraModel
    right: CameraModel
    R: np.ndarray
    T: np.ndarray
    resolution_source: str

    @classmethod
    def from_payload(cls, kind: str, payload: dict[str, Any], settings: InspectionSettings) -> StereoModel:
        if payload.get("format") == "generic_stereo":
            size = payload.get("resolution")
            left = _camera(payload["K1"], payload["D1"], size, payload.get("distortion_model", "radtan"))
            right = _camera(payload["K2"], payload["D2"], size, payload.get("distortion_model", "radtan"))
            rotation = _array(payload["R"], (3,3), "左到右旋转")
            translation = _array(payload["T_m"], (3,1), "左到右平移（米）")
            source = "file"
        elif kind == "ego-lite":
            cameras = []
            for name in ("left_mono", "right_mono"):
                data = payload["cameras"][name]
                lens = data.get("distortion_model", "")
                if "Perspective" not in lens:
                    raise CalibrationError(f"OAK 镜头模型尚不支持复测：{lens}")
                cameras.append(_camera(data["K"], data["distortion_coefficients"], data["calibration_resolution"]))
            transform = _array(payload["camera_extrinsics"]["T_right_mono_from_left_mono"]["matrix_m"], (4, 4), "左到右外参")
            left, right = cameras
            rotation, translation, source = transform[:3, :3], transform[:3, 3:4], "device"
        elif payload.get("format") == "stereo_calibration":
            data = payload["calibration"]
            size = (settings.width, settings.height)
            left, right = _camera(data["K1"], data["D1"], size), _camera(data["K2"], data["D2"], size)
            rotation = _array(data["R"], (3, 3), "双目旋转")
            translation = _array(data["T"], (3, 1), "双目平移") / 1000.0
            baseline = payload.get("metrics", {}).get("baseline_mm")
            if baseline and not math.isclose(float(np.linalg.norm(translation)) * 1000, baseline, rel_tol=0.05):
                raise CalibrationError("设备平移与 baseline_mm 不一致，需核实平移单位")
            source = "user_confirmed" if settings.resolution_confirmed else "unconfirmed"
        elif payload.get("format") == "kalibr_camchain_imucam":
            cameras = []
            for name in ("cam0", "cam1"):
                data = _yaml_camera(payload.get("kalibr_yaml", ""), name)
                if data.get("camera_model") != "pinhole":
                    raise CalibrationError("Kalibr 复测当前支持 pinhole 镜头；此设备模型不受支持")
                fx, fy, cx, cy = _array(data.get("intrinsics"), (4,), "Kalibr 内参")
                cameras.append(_camera([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], data.get("distortion_coeffs", []), data.get("resolution"), data.get("distortion_model")))
            left, right = cameras
            transform = _array(payload["kalibr_calibration"]["cam1"]["T_cn_cnm1"], (4, 4), "Kalibr 左到右外参")
            rotation, translation, source = transform[:3, :3], transform[:3, 3:4], "device"
        else:
            raise CalibrationError("此设备标定格式暂不支持画面复测")
        if left.resolution != right.resolution:
            raise CalibrationError("当前流程要求左右标定分辨率一致")
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=0.01) or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=0.01):
            raise CalibrationError("双目旋转矩阵无效")
        if not 0.0001 < float(np.linalg.norm(translation)) < 10:
            raise CalibrationError("双目基线无效，请核实外参和单位")
        return cls(left, right, rotation, translation, source)


def error_stats(values: list[float] | np.ndarray) -> dict[str, Any] | None:
    array = np.asarray(values, dtype=float)
    if not array.size:
        return None
    if not np.isfinite(array).all():
        raise CalibrationError("误差计算产生非有限数值")
    return {"count": int(array.size), "rms": float(np.sqrt(np.mean(array**2))), "mean": float(np.mean(array)), "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}


def image_quality(image: np.ndarray, points: np.ndarray | None = None) -> dict[str, float]:
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    roi = gray
    if points is not None and len(points):
        x, y, w, h = cv2.boundingRect(np.asarray(points, np.float32))
        roi = gray[max(0, y):min(gray.shape[0], y+h), max(0, x):min(gray.shape[1], x+w)]
    if not roi.size:
        raise CalibrationError("标定板区域无效")
    return {"sharpness": float(cv2.Laplacian(roi, cv2.CV_64F).var()), "dark_fraction": float(np.mean(roi < 5)), "bright_fraction": float(np.mean(roi > 250)), "mean_brightness": float(np.mean(roi))}


class StereoInspector:
    def __init__(self, kind: str, payload: dict[str, Any], settings: InspectionSettings) -> None:
        self.kind, self.payload, self.settings = kind, payload, settings
        self.model = StereoModel.from_payload(kind, payload, settings)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
        # Kalibr targets can use a two-cell black border; ordinary AprilTags use one.
        params.markerBorderBits = 2
        self.wide_border_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
        self.corner_order = None
        self.samples: list[dict[str, Any]] = []
        self.coverage = [set(), set()]
        self.signatures: list[np.ndarray] = []
        self.errors: dict[str, list[float]] = {key: [] for key in ("left", "right", "left_to_right", "right_to_left", "epipolar", "tag_scale_percent")}
        self.normals: list[np.ndarray] = []
        self.depths: list[float] = []
        # Rectification acts on already undistorted rays; it works with both lens models.
        m = self.model
        self.R1, self.R2, self.P1, self.P2, *_ = cv2.stereoRectify(m.left.K, np.zeros(5), m.right.K, np.zeros(5), m.left.resolution, m.R, m.T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.epipolar_axis = 1 if abs(self.P2[0, 3]) > abs(self.P2[1, 3]) else 0

    def detect(self, frame: np.ndarray) -> dict[int, np.ndarray]:
        corners, ids, _ = self.detector.detectMarkers(frame)
        wide_corners, wide_ids, _ = self.wide_border_detector.detectMarkers(frame)
        if wide_ids is not None and (ids is None or len(wide_ids) > len(ids)):
            corners, ids = wide_corners, wide_ids
        if ids is None:
            return {}
        result = {}
        for tag, points in zip(ids.reshape(-1), corners):
            if int(tag) in result:
                raise CalibrationError("画面中出现重复 AprilTag ID，请移开其他标定板")
            if 0 <= int(tag) < self.settings.rows * self.settings.columns:
                result[int(tag)] = np.asarray(points, dtype=np.float64).reshape(4, 2)
        return result

    def inspect(self, left: np.ndarray, right: np.ndarray, timestamp: float, detections=None) -> tuple[dict[str, Any], np.ndarray]:
        expected = self.model.left.resolution
        for frame in (left, right):
            if (frame.shape[1], frame.shape[0]) != expected:
                raise CalibrationError(f"画面为 {frame.shape[1]}×{frame.shape[0]}，标定按 {expected[0]}×{expected[1]} 配置；请核实分辨率，未自动缩放内参")
        found = detections if detections is not None else (self.detect(left), self.detect(right))
        ids = sorted(set(found[0]) & set(found[1]))
        sample: dict[str, Any] = {"index": len(self.samples), "timestamp_s": timestamp, "tags_left": len(found[0]), "tags_right": len(found[1]), "common_tags": len(ids), "accepted": False, "reason": "", "metrics": {}}
        views = [cv2.cvtColor(f, cv2.COLOR_GRAY2BGR) if f.ndim == 2 else f.copy() for f in (left, right)]
        for view, markers in zip(views, found):
            if markers:
                cv2.aruco.drawDetectedMarkers(view, [p.astype(np.float32).reshape(1, 4, 2) for p in markers.values()], np.array(list(markers), np.int32).reshape(-1, 1))
        pixels = [np.concatenate([markers[i] for i in ids]) if ids else None for markers in found]
        sample["quality"] = [image_quality(frame, p) for frame, p in zip((left, right), pixels)]
        if len(ids) < self.settings.min_tags:
            sample["reason"] = f"共同标签不足：{len(ids)}/{self.settings.min_tags}，请让左右相机同时看到标定板"
        elif any(q["sharpness"] < self.settings.min_sharpness for q in sample["quality"]):
            sample["reason"] = "标定板区域清晰度不足，请稍停并改善对焦或照明"
        elif any(max(q["dark_fraction"], q["bright_fraction"]) > 0.8 for q in sample["quality"]):
            sample["reason"] = "标定板区域严重过暗或过曝"
        else:
            try:
                self._measure(sample, ids, pixels, views)
            except (cv2.error, CalibrationError) as exc:
                sample["reason"] = f"无法估计标定板位姿：{exc}"
        self.samples.append(sample)
        combined = np.concatenate(views, axis=1)
        scale = min(1.0, 1600 / combined.shape[1])
        if scale < 1:
            combined = cv2.resize(combined, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return sample, combined

    def _measure(self, sample, ids, pixels, views) -> None:
        m, settings = self.model, self.settings
        if self.corner_order is None:
            if len({tag // settings.columns for tag in ids}) < 2 or len({tag % settings.columns for tag in ids}) < 2:
                raise CalibrationError("首次识别至少需要覆盖两行和两列标签，以确认标定板排列")
            # Infer only the board's discrete corner convention, never camera parameters.
            # This supports both OpenCV top-down grids and Kalibr bottom-up PDFs.
            candidates = [tuple(np.roll(order, shift)) for order in ((0, 1, 2, 3), (0, 3, 2, 1)) for shift in range(4)]
            normalized = m.left.normalized(pixels[0])
            scores = []
            for order in candidates:
                xy = settings.points(ids, order)[:, :2].reshape(-1, 1, 2)
                homography, _ = cv2.findHomography(xy, normalized, 0)
                score = float("inf") if homography is None else np.mean(np.linalg.norm(cv2.perspectiveTransform(xy, homography).reshape(-1, 2)-normalized, axis=1))
                scores.append(score)
            if not np.isfinite(scores).any():
                raise CalibrationError("无法确定标定板排列")
            self.corner_order = candidates[int(np.argmin(scores))]
        points = settings.points(ids, self.corner_order)
        lr, lt = m.left.pose(points, pixels[0])
        rr, rt = m.right.pose(points, pixels[1])
        lp, rp = m.left.project(points, lr, lt), m.right.project(points, rr, rt)
        cross_r = m.right.project(points, m.R @ lr, m.R @ lt + m.T)
        cross_l = m.left.project(points, m.R.T @ rr, m.R.T @ (rt - m.T))
        rect_l = m.left.normalized(pixels[0], self.R1, self.P1)
        rect_r = m.right.normalized(pixels[1], self.R2, self.P2)
        values = {
            "left": np.linalg.norm(lp - pixels[0], axis=1),
            "right": np.linalg.norm(rp - pixels[1], axis=1),
            "left_to_right": np.linalg.norm(cross_r - pixels[1], axis=1),
            "right_to_left": np.linalg.norm(cross_l - pixels[0], axis=1),
            "epipolar": np.abs(rect_l[:, self.epipolar_axis] - rect_r[:, self.epipolar_axis]),
        }
        homogeneous = cv2.triangulatePoints(np.eye(3, 4), np.column_stack((m.R, m.T)), m.left.normalized(pixels[0]).T, m.right.normalized(pixels[1]).T)
        if np.any(np.abs(homogeneous[3]) < 1e-12):
            raise CalibrationError("双目三角测量退化")
        xyz = (homogeneous[:3] / homogeneous[3]).T
        sample["positive_depth_fraction"] = float(np.mean((xyz[:, 2] > 0) & ((xyz @ m.R.T + m.T.ravel())[:, 2] > 0)))
        tags = xyz.reshape(-1, 4, 3)
        lengths = np.linalg.norm(tags - np.roll(tags, -1, axis=1), axis=2)
        values["tag_scale_percent"] = (np.abs(lengths / settings.tag_size_m - 1) * 100).reshape(-1)
        sample["metrics"] = {key: error_stats(value) for key, value in values.items()}
        sample["board_depth_m"] = float(lt[2, 0])
        sample["board_normal"] = lr[:, 2].tolist()
        sample["tag_ids"] = ids
        sample["observed_corners"] = [p.tolist() for p in pixels]
        for view, observed, projected in zip(views, pixels, (cross_l, cross_r)):
            for a, b in zip(observed, projected):
                if np.isfinite(b).all() and np.max(np.abs(b)) < 100000:
                    cv2.line(view, tuple(np.rint(a).astype(int)), tuple(np.rint(b).astype(int)), (0, 80, 255), 2)
                    cv2.circle(view, tuple(np.rint(b).astype(int)), 3, (0, 200, 255), -1)
        center_world = settings.points(list(range(settings.rows * settings.columns))).mean(axis=0).reshape(1, 3)
        center = m.left.project(center_world, lr, lt)[0] / np.asarray(m.left.resolution)
        signature = np.r_[center, math.log(max(float(lt[2, 0]), 1e-6)), lr[:, 2]]
        duplicate = any(np.linalg.norm(signature[:2] - prior[:2]) < 0.035 and abs(signature[2] - prior[2]) < 0.12 and np.linalg.norm(signature[3:] - prior[3:]) < 0.12 for prior in self.signatures)
        if duplicate:
            sample["reason"] = "位置与已有样本接近，请改变位置、距离或倾斜角度"
            return
        sample["accepted"] = True
        sample["reason"] = "已采纳"
        self.signatures.append(signature)
        self.normals.append(lr[:, 2])
        self.depths.append(float(lt[2, 0]))
        for key, value in values.items():
            self.errors[key].extend(value.tolist())
        width, height = m.left.resolution
        for cells, observed in zip(self.coverage, pixels):
            for x, y in observed:
                if 0 <= x < width and 0 <= y < height:
                    cells.add(min(2, int(y * 3 / height)) * 3 + min(2, int(x * 3 / width)))

    def report(self, *, finished: bool, source: dict[str, Any], failure: str = "") -> dict[str, Any]:
        settings, m = self.settings, self.model
        accepted = sum(s["accepted"] for s in self.samples)
        spread = max((math.degrees(math.acos(float(np.clip(a @ b, -1, 1)))) for a in self.normals for b in self.normals), default=0.0)
        depth_ratio = max(self.depths) / min(self.depths) if self.depths else 1.0
        stats = {key: error_stats(values) for key, values in self.errors.items()}
        checks = []

        def check(name, ok, detail):
            checks.append({"name": name, "status": "pass" if ok else "incomplete", "detail": detail})

        check("有效样本", accepted >= settings.min_views, f"{accepted}/{settings.min_views} 对不同姿态的有效样本")
        check("画面覆盖", all(len(c) >= settings.min_cells for c in self.coverage), f"左 {len(self.coverage[0])}/9 格，右 {len(self.coverage[1])}/9 格；要求各至少 {settings.min_cells} 格")
        check("倾斜角度变化", spread >= 20, f"标定板法向变化 {spread:.1f}°；要求至少 20°")
        check("距离变化", depth_ratio >= 1.25, f"最远/最近距离比 {depth_ratio:.2f}；要求至少 1.25")
        check("标定分辨率依据", m.resolution_source != "unconfirmed", {"file": "来自导入标定文件的明确尺寸", "device": "来自设备标定数据", "user_confirmed": "操作者确认；设备未记录此字段", "unconfirmed": "设备未记录；当前按填写尺寸试算，需核实后才能作通过判定"}[m.resolution_source])
        basic = validate_calibration(self.kind, self.payload)
        checks.append({"name": "设备标定数据完整性", "status": "fail" if basic.failure_count else "pass", "detail": f"异常 {basic.failure_count} 项；历史 RMS 不参与本次精度判定"})
        acceptance = inspection_criteria(settings, stats)
        for criterion in acceptance["criteria"]:
            value, field, limit = criterion["value"], criterion["statistic"], criterion["limit"]
            checks.append({"name": criterion["name"], "status": criterion["status"], "detail": f"{field.upper()} {value:.3f} px / 要求 ≤ {limit:g} px" if value is not None else f"尚无有效观测 / 要求 {field.upper()} ≤ {limit:g} px"})
        if accepted and any(s["accepted"] and s.get("positive_depth_fraction", 0) < 0.95 for s in self.samples):
            checks.append({"name": "双目三角测量", "status": "fail", "detail": "部分样本超过 5% 的角点位于相机后方，请检查左右顺序和外参方向"})
        if failure:
            checks.append({"name": "采集过程", "status": "incomplete", "detail": failure})
        if not finished:
            status = "running"
        elif failure or any(c["status"] == "incomplete" for c in checks):
            status = "incomplete"
        elif any(c["status"] == "fail" for c in checks):
            status = "fail"
        else:
            status = "pass"
        return {
            "schema_version": 2, "created_at": datetime.now(timezone.utc).isoformat(),
            "status": status, "summary": {"running": "检测进行中", "incomplete": "检测未完成 / 条件待确认", "fail": "超出本次设置阈值", "pass": "通过本次设置阈值"}[status],
            "settings": asdict(settings), "source": source, "device_kind": self.kind,
            "resolution": list(m.left.resolution), "resolution_source": m.resolution_source,
            "baseline_mm": float(np.linalg.norm(m.T) * 1000),
            "counts": {"sampled": len(self.samples), "accepted": accepted, "rejected": len(self.samples)-accepted},
            "coverage": [sorted(c) for c in self.coverage], "orientation_spread_deg": spread,
            "board_corner_order": [int(v) for v in self.corner_order] if self.corner_order else None,
            "depth_ratio": depth_ratio, "metrics": stats, "checks": checks,
            "acceptance": acceptance,
            "calibration_comparison": calibration_comparison(self.payload, acceptance, resolution_confirmed=m.resolution_source != "unconfirmed"),
            "notes": ["本次固定设备内参、畸变与双目外参，仅估计每帧标定板位姿；未重新标定或写入设备。", "阈值为可修改的检测参考值，不是厂商验收标准。", "清晰度为标定板区域拉普拉斯方差，受纹理、光照和图像模式影响。", "标签边长误差为三角测量辅助指标，未设置合格阈值。", "范围：左右相机几何、采样与画面质量；不包括 RGB、IMU 外参、硬件曝光同步、温漂或长期稳定性验收。"],
            "samples": self.samples, "calibration": self.payload,
        }


def write_report(directory: Path, report: dict[str, Any]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    temporary = directory / "report.json.tmp"
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(directory / "report.json")
    with (directory / "samples.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["序号", "时间秒", "有效", "原因", "共同标签", "左目RMS_px", "右目RMS_px", "左预测右RMS_px", "右预测左RMS_px", "极线P95_px"])
        for sample in report["samples"]:
            metrics = sample["metrics"]
            writer.writerow([sample["index"], sample["timestamp_s"], sample["accepted"], sample["reason"], sample["common_tags"], *[(metrics.get(k) or {}).get("p95" if k == "epipolar" else "rms", "") for k in ("left", "right", "left_to_right", "right_to_left", "epipolar")]])
    def e(value):
        return html.escape(str(value))
    labels = {"pass": "通过", "fail": "超出阈值", "incomplete": "未完成"}
    rows = "".join(f'<tr><td>{e(c["name"])}</td><td class="{e(c["status"])}">{labels[c["status"]]}</td><td>{e(c["detail"])}</td></tr>' for c in report["checks"])
    notes = "".join(f"<li>{e(note)}</li>" for note in report["notes"])
    evidence = "".join(f'<figure><img src="{e(s["preview"])}" alt="样本 {s["index"]}"><figcaption>#{s["index"]} · {e(s["reason"])}</figcaption></figure>' for s in report["samples"] if s.get("preview"))
    settings = report["settings"]
    comparison_html = ""
    if report.get("acceptance") and report.get("calibration_comparison"):
        acceptance, comparison = report["acceptance"], report["calibration_comparison"]
        comparison_rows = []
        for row in comparison["rows"]:
            old = "未提供" if row["historical_value"] is None else f'{row["historical_value"]:.3f}'
            current = ("未复算" if row["status"] == "reference" else "待采样") if row["value"] is None else f'{row["value"]:.3f}'
            delta = "—" if row["delta_px"] is None else f'{row["delta_px"]:+.3f}'
            limit = "—" if row["limit"] is None else f'≤ {row["limit"]:g}'
            result = "历史记录" if row["status"] == "reference" else labels[row["status"]]
            comparison_rows.append(f'<tr><td>{e(row["name"])} {e(row["statistic"].upper())}</td><td>{old}</td><td>{current}</td><td>{delta}</td><td>{limit}</td><td class="{e(row["status"])}">{result}</td></tr>')
        explanations = list(dict.fromkeys([row["note"] for row in comparison["rows"]]))
        comparison_notes = "".join(f'<li>{e(note)}</li>' for note in [*acceptance["notes"], *explanations, comparison["note"]])
        comparison_html = f'''<h2>判定要求与已有标定对比</h2><p>{e(acceptance['label'])} · 依据备注：{e(acceptance['reference'])}</p><p>{e(comparison['reference'])}；标定 SN：{e(comparison['calibration_serial'] or '未提供')}；历史样本数：{e(comparison['historical_sample_count'] if comparison['historical_sample_count'] is not None else '未提供')}。数值单位 px；差值 = 本次 − 历史，仅作参考。</p><div style="overflow:auto"><table><thead><tr><th>指标</th><th>已有标定</th><th>本次复测</th><th>参考差值</th><th>通过要求</th><th>本次结果</th></tr></thead><tbody>{''.join(comparison_rows)}</tbody></table></div><ul>{comparison_notes}</ul>'''
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>双目相机检测报告</title><style>body{{font:16px/1.7 system-ui,sans-serif;color:#172b4d;background:#f3f6fa;margin:0;padding:32px}}main{{max-width:1100px;margin:auto;background:white;padding:28px}}h1{{font-size:28px}}table{{border-collapse:collapse;width:100%}}td,th{{padding:10px;text-align:left;border-bottom:1px solid #d8e0ea}}.pass{{color:#15803d}}.fail{{color:#b91c1c}}.incomplete{{color:#a16207}}.evidence{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}}figure{{margin:0}}img{{width:100%}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}@media print{{body{{background:white;padding:0}}figure{{break-inside:avoid}}}}</style><main><h1>双目相机检测报告</h1><p><strong>{e(report['summary'])}</strong> · {e(report['created_at'])}</p><p>有效样本 {report['counts']['accepted']} / 采样 {report['counts']['sampled']} · 单目 {e(report['resolution'])} · 已有标定基线 {report['baseline_mm']:.3f} mm</p><p>AprilGrid：{settings['rows']}×{settings['columns']}，tag36h11，标签 {settings['tag_size_m']*1000:g} mm，间隙 {settings['tag_spacing_m']*1000:g} mm</p><table><thead><tr><th>检查项</th><th>结果</th><th>观测与判定依据</th></tr></thead><tbody>{rows}</tbody></table>{comparison_html}<h2>说明</h2><ul>{notes}</ul><h2>完整统计</h2><pre>{e(json.dumps(report['metrics'], ensure_ascii=False, indent=2))}</pre><h2>样本画面</h2><div class="evidence">{evidence}</div></main></html>'''
    (directory / "report.html").write_text(document, encoding="utf-8")

from __future__ import annotations

import csv
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from importlib import resources
from pathlib import Path
from typing import Any

from ego_calibration import ego_lite
from ego_calibration.ego_std import _extract_yaml_matrix
from ego_calibration.models import CalibrationError


CAMERA_FPS = 20
IMU_HZ = 200
STEREO_QUEUE_SIZE = 32
STEREO_PENDING_LIMIT = 64
STEREO_TIMESTAMP_TOLERANCE_NS = 1_000_000
MIN_STEREO_MATCH_RATIO = 0.95
FLASH_BACKENDS = ("v2", "v3")
MEASURED_BNO086_IMU_YAML = Path(os.environ.get("CAMERA_IMU_NOISE_YAML", str(Path.home() / ".config/camera-workbench/imu-noise.yaml")))
IMU_CSV_HEADER = (
    "timestamp",
    "omega_x",
    "omega_y",
    "omega_z",
    "alpha_x",
    "alpha_y",
    "alpha_z",
)


def default_aprilgrid_path() -> Path:
    return Path(
        str(
            resources.files("ego_calibration.resources").joinpath(
                "ego_lite_aprilgrid_6x6.yaml"
            )
        )
    )


def write_default_aprilgrid_yaml(directory: Path) -> Path:
    """将默认 6x6 tag36H11 配置放入数据集根目录。"""
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / "target.yaml"
    if not target.exists():
        source = default_aprilgrid_path()
        try:
            target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            raise CalibrationError(f"无法写入默认 AprilGrid 配置：{exc}") from exc
    return target


def default_imu_path() -> Path:
    if MEASURED_BNO086_IMU_YAML.is_file():
        return MEASURED_BNO086_IMU_YAML
    return Path(
        str(
            resources.files("ego_calibration.resources").joinpath(
                "kalibr_default_imu_200hz.yaml"
            )
        )
    )


@dataclass(frozen=True, slots=True)
class CaptureSummary:
    directory: Path
    left_frames: int
    right_frames: int
    imu_samples: int
    elapsed_seconds: float
    stopped: bool
    unpaired_left_frames: int = 0
    unpaired_right_frames: int = 0


@dataclass(frozen=True, slots=True)
class UndistortionMap:
    """一对与 OAK 灰度输出分辨率匹配的 OpenCV 去畸变映射表。"""

    map_x: Any
    map_y: Any
    resolution: tuple[int, int]


@dataclass(frozen=True, slots=True)
class ImuNoiseCaptureSummary:
    directory: Path
    csv_path: Path
    imu_samples: int
    elapsed_seconds: float
    average_rate_hz: float
    stopped: bool


@dataclass(frozen=True, slots=True)
class KalibrResult:
    source: Path
    matrix_m: list[list[float]]
    stereo_matrix_m: list[list[float]]
    determinant: float
    orthogonal_error: float
    translation_m: tuple[float, float, float]
    timeshift_seconds: float | None


@dataclass(frozen=True, slots=True)
class FlashSummary:
    backup: Path
    written_calibration: Path
    final_report: Path
    result: KalibrResult
    readback_cm: list[list[float]]
    maximum_error: float
    backend: str
    depthai_version: str
    flash_api: str


@dataclass(frozen=True, slots=True)
class KalibrEnvironment:
    """可运行 Kalibr 的命令及其需要加载的 shell 环境。"""

    setup_script: Path | None
    commands: dict[str, Path]
    missing: tuple[str, ...]
    runtime_missing: tuple[str, ...] = ()

    @property
    def ready(self) -> bool:
        return not self.missing and not self.runtime_missing

    @property
    def label(self) -> str:
        if self.setup_script is not None:
            return str(self.setup_script)
        return "当前进程环境"


def create_session_directory(parent: Path, mxid: str) -> Path:
    return _create_unique_session_directory(parent, mxid, "oak-cam-imu")


def create_imu_noise_session_directory(parent: Path, mxid: str) -> Path:
    return _create_unique_session_directory(parent, mxid, "oak-imu-noise")


def _create_unique_session_directory(parent: Path, mxid: str, prefix: str) -> Path:
    parent = Path(parent).expanduser()
    parent.mkdir(parents=True, exist_ok=True)
    safe_mxid = "".join(character for character in mxid if character.isalnum())[-12:]
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    base_name = f"{prefix}-{safe_mxid or 'device'}-{stamp}"
    candidate = parent / base_name
    suffix = 1
    while candidate.exists():
        candidate = parent / f"{base_name}-{suffix}"
        suffix += 1
    candidate.mkdir()
    return candidate


def capture_imu_noise_dataset(
    mxid: str,
    directory: Path,
    duration_seconds: int,
    *,
    should_stop: Callable[[], bool] | None = None,
    progress: Callable[[float, int], None] | None = None,
) -> ImuNoiseCaptureSummary:
    if duration_seconds <= 0:
        raise CalibrationError("IMU 静置采集时长必须大于 0 秒")
    dai = _load_depthai()
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    imu_path = directory / "imu0.csv"
    if imu_path.exists() and imu_path.stat().st_size:
        raise CalibrationError("IMU 静置数据目录中已经存在 imu0.csv")

    pipeline = _create_imu_capture_pipeline(dai)
    info = _find_device_info(dai, mxid)
    stop_requested = should_stop or (lambda: False)
    imu_count = 0
    stopped = False
    started = time.monotonic()
    imu_type = ""

    try:
        device = dai.Device(pipeline, info)
    except Exception as exc:
        raise CalibrationError(
            f"DepthAI IMU 静置采集启动失败：{exc}；"
            "请关闭相机预览或其他占用程序，必要时重新插拔 USB 后重试"
        ) from exc
    try:
        imu_type = str(device.getConnectedIMU())
        imu_queue = device.getOutputQueue("calib_imu", maxSize=200, blocking=False)
        timestamp_offset_ns = time.time_ns()
        last_progress = 0.0
        last_flush = 0.0
        started = time.monotonic()
        with imu_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(IMU_CSV_HEADER)
            while True:
                elapsed = time.monotonic() - started
                if stop_requested():
                    stopped = True
                    break
                if elapsed >= duration_seconds:
                    break
                imu_data = imu_queue.tryGet()
                if imu_data is None:
                    time.sleep(0.002)
                else:
                    for packet in imu_data.packets:
                        writer.writerow(_imu_csv_row(packet, timestamp_offset_ns))
                        imu_count += 1
                if elapsed - last_flush >= 1.0:
                    stream.flush()
                    last_flush = elapsed
                if progress is not None and elapsed - last_progress >= 0.25:
                    progress(elapsed, imu_count)
                    last_progress = elapsed
    except CalibrationError:
        raise
    except Exception as exc:
        raise CalibrationError(f"OAK 原始 IMU 静置采集失败：{exc}") from exc
    finally:
        device.close()

    elapsed = min(time.monotonic() - started, float(duration_seconds))
    if imu_count == 0:
        raise CalibrationError("IMU 静置采集没有收到任何原始数据")
    average_rate_hz = imu_count / elapsed if elapsed > 0 else 0.0
    _write_imu_noise_metadata(
        directory,
        mxid,
        duration_seconds,
        elapsed,
        imu_count,
        average_rate_hz,
        imu_type,
        stopped,
    )
    if progress is not None:
        progress(elapsed, imu_count)
    return ImuNoiseCaptureSummary(
        directory=directory,
        csv_path=imu_path,
        imu_samples=imu_count,
        elapsed_seconds=elapsed,
        average_rate_hz=average_rate_hz,
        stopped=stopped,
    )


def capture_kalibr_dataset(
    mxid: str,
    directory: Path,
    duration_seconds: int,
    *,
    should_stop: Callable[[], bool] | None = None,
    progress: Callable[[float, int, int, int], None] | None = None,
    preview: Callable[[Any, Any, int, int], None] | None = None,
) -> CaptureSummary:
    if duration_seconds <= 0:
        raise CalibrationError("采集时长必须大于 0 秒")
    dai = _load_depthai()
    try:
        import cv2
    except ImportError as exc:
        raise CalibrationError("未安装 OpenCV，无法保存标定图像") from exc

    directory = Path(directory)
    left_directory = directory / "cam0"
    right_directory = directory / "cam1"
    if any(path.exists() and any(path.iterdir()) for path in (left_directory, right_directory)):
        raise CalibrationError("数据目录中已经存在相机图像，请选择新的空目录")
    left_directory.mkdir(parents=True, exist_ok=True)
    right_directory.mkdir(parents=True, exist_ok=True)
    write_default_aprilgrid_yaml(directory)

    pipeline = _create_capture_pipeline(dai)
    info = _find_device_info(dai, mxid)
    stop_requested = should_stop or (lambda: False)
    left_count = right_count = imu_count = 0
    discarded_left = discarded_right = 0
    pending_left: dict[int, Any] = {}
    pending_right: dict[int, Any] = {}
    started = time.monotonic()
    stopped = False
    imu_path = directory / "imu0.csv"
    latest_left_frame = latest_right_frame = None
    last_preview = 0.0

    try:
        device = dai.Device(pipeline, info)
    except Exception as exc:
        raise CalibrationError(f"DepthAI 标定采集启动失败：{exc}") from exc
    try:
        calibration = device.readCalibration2()
        (directory / "camchain.yaml").write_text(
            build_undistorted_camchain_yaml(dai, calibration), encoding="utf-8"
        )
        left_socket = dai.CameraBoardSocket.CAM_B
        right_socket = dai.CameraBoardSocket.CAM_C
        left_undistortion = _create_undistortion_map(
            cv2, calibration, left_socket
        )
        right_undistortion = _create_undistortion_map(
            cv2, calibration, right_socket
        )
        _write_capture_metadata(
            directory,
            mxid,
            duration_seconds,
            device,
            image_undistorted=True,
        )
        left_queue = device.getOutputQueue(
            "calib_left", maxSize=STEREO_QUEUE_SIZE, blocking=False
        )
        right_queue = device.getOutputQueue(
            "calib_right", maxSize=STEREO_QUEUE_SIZE, blocking=False
        )
        imu_queue = device.getOutputQueue("calib_imu", maxSize=100, blocking=False)

        with imu_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(IMU_CSV_HEADER)
            last_progress = 0.0
            timestamp_offset_ns = time.time_ns()
            started = time.monotonic()
            while True:
                elapsed = time.monotonic() - started
                stop_now = stop_requested()
                deadline_reached = elapsed >= duration_seconds

                received = False
                left_packets = _queue_try_get_all(left_queue)
                right_packets = _queue_try_get_all(right_queue)
                received = bool(left_packets or right_packets)
                for packet in left_packets:
                    pending_left[_timestamp_ns(packet)] = packet
                for packet in right_packets:
                    pending_right[_timestamp_ns(packet)] = packet

                stereo_pairs, unmatched_left, unmatched_right = _pop_stereo_pairs(
                    pending_left, pending_right
                )
                discarded_left += unmatched_left
                discarded_right += unmatched_right
                for left_packet, right_packet in stereo_pairs:
                    latest_left_frame = _undistort_frame(
                        cv2, left_packet.getCvFrame(), left_undistortion
                    )
                    latest_right_frame = _undistort_frame(
                        cv2, right_packet.getCvFrame(), right_undistortion
                    )
                    _save_frame(
                        cv2,
                        left_packet,
                        left_directory,
                        timestamp_offset_ns,
                        latest_left_frame,
                    )
                    _save_frame(
                        cv2,
                        right_packet,
                        right_directory,
                        timestamp_offset_ns,
                        latest_right_frame,
                    )
                    left_count += 1
                    right_count += 1
                dropped_left, dropped_right = _trim_pending_stereo_packets(
                    pending_left, pending_right
                )
                discarded_left += dropped_left
                discarded_right += dropped_right

                imu_batches = _queue_try_get_all(imu_queue)
                if imu_batches:
                    received = True
                for imu_data in imu_batches:
                    for packet in imu_data.packets:
                        writer.writerow(_imu_csv_row(packet, timestamp_offset_ns))
                        imu_count += 1
                now = time.monotonic()
                if (
                    preview is not None
                    and latest_left_frame is not None
                    and latest_right_frame is not None
                    and now - last_preview >= 0.1
                ):
                    preview(
                        latest_left_frame.copy(),
                        latest_right_frame.copy(),
                        left_count,
                        right_count,
                    )
                    last_preview = now
                if progress is not None and elapsed - last_progress >= 0.25:
                    progress(elapsed, left_count, right_count, imu_count)
                    last_progress = elapsed
                if stop_now or deadline_reached:
                    stopped = stop_now
                    break
                if not received:
                    time.sleep(0.002)
    except CalibrationError:
        raise
    except Exception as exc:
        raise CalibrationError(f"OAK 相机/IMU 数据采集失败：{exc}") from exc
    finally:
        device.close()

    elapsed = min(time.monotonic() - started, float(duration_seconds))
    if left_count == 0 or right_count == 0 or imu_count == 0:
        raise CalibrationError(
            f"采集数据不完整：左目 {left_count} 帧，右目 {right_count} 帧，"
            f"IMU {imu_count} 条"
        )
    return CaptureSummary(
        directory=directory,
        left_frames=left_count,
        right_frames=right_count,
        imu_samples=imu_count,
        elapsed_seconds=elapsed,
        stopped=stopped,
        unpaired_left_frames=discarded_left + len(pending_left),
        unpaired_right_frames=discarded_right + len(pending_right),
    )


def build_camchain_yaml(dai: Any, calibration: Any) -> str:
    """生成与当前 OAK 采集图像匹配的 Kalibr camchain。

    采集图像会按 EEPROM 的完整 Perspective 参数去畸变，因此 camchain
    必须声明零畸变 pinhole 模型，不能再把 14 参数截断为 4 参数 radtan。
    """

    return _build_camchain_yaml(dai, calibration, undistorted=True)


def build_undistorted_camchain_yaml(dai: Any, calibration: Any) -> str:
    """为主机端去畸变后的灰度图生成 Kalibr camchain。

    输出图像仍保持 EEPROM 的原始分辨率和内参，但已经按完整的 DepthAI
    Perspective 畸变参数重映射，因此 Kalibr 看到的是零畸变 pinhole 图像。
    """

    return build_camchain_yaml(dai, calibration)


def _build_camchain_yaml(
    dai: Any,
    calibration: Any,
    *,
    undistorted: bool,
) -> str:
    left = _camera_yaml(
        calibration, dai.CameraBoardSocket.CAM_B, undistorted=undistorted
    )
    right = _camera_yaml(
        calibration, dai.CameraBoardSocket.CAM_C, undistorted=undistorted
    )
    stereo = _matrix(calibration.getCameraExtrinsics(
        dai.CameraBoardSocket.CAM_B,
        dai.CameraBoardSocket.CAM_C,
        False,
    ), 4, 4)
    for row in range(3):
        stereo[row][3] /= 100.0
    return "\n".join(
        [
            "cam0:",
            *_camera_yaml_lines(left),
            "  rostopic: /cam0/image_raw",
            "cam1:",
            "  T_cn_cnm1:",
            *[f"    - {_yaml_vector(row)}" for row in stereo],
            *_camera_yaml_lines(right),
            "  rostopic: /cam1/image_raw",
            "",
        ]
    )


def load_kalibr_result(path: Path) -> KalibrResult:
    source = Path(path).expanduser()
    try:
        text = source.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise CalibrationError(f"无法读取 Kalibr 结果：{exc}") from exc
    matrix = _extract_yaml_matrix(text, "cam0", "T_cam_imu")
    if matrix is None:
        raise CalibrationError("Kalibr 结果缺少 cam0.T_cam_imu 4x4 矩阵")
    _validate_homogeneous(matrix)
    determinant, orthogonal_error = _rotation_metrics(matrix)
    if abs(determinant - 1.0) > 0.05 or orthogonal_error > 0.05:
        raise CalibrationError(
            "cam0.T_cam_imu 旋转矩阵无效："
            f"det={determinant:.6f}，正交误差={orthogonal_error:.3g}"
        )
    stereo = _extract_yaml_matrix(text, "cam1", "T_cn_cnm1")
    if stereo is None:
        raise CalibrationError("Kalibr 结果缺少 cam1.T_cn_cnm1，无法核对双目链")
    _validate_homogeneous(stereo, "cam1.T_cn_cnm1")
    stereo_determinant, stereo_orthogonal_error = _rotation_metrics(stereo)
    if abs(stereo_determinant - 1.0) > 0.05 or stereo_orthogonal_error > 0.05:
        raise CalibrationError("cam1.T_cn_cnm1 旋转矩阵无效")
    translation = tuple(float(matrix[row][3]) for row in range(3))
    if math.sqrt(sum(value * value for value in translation)) > 1.0:
        raise CalibrationError("cam0.T_cam_imu 平移超过 1 米，不符合 OAK 设备物理尺寸")
    timeshift = _extract_yaml_scalar(text, "cam0", "timeshift_cam_imu")
    return KalibrResult(
        source=source,
        matrix_m=matrix,
        stereo_matrix_m=stereo,
        determinant=determinant,
        orthogonal_error=orthogonal_error,
        translation_m=translation,
        timeshift_seconds=timeshift,
    )


def flash_kalibr_result(
    mxid: str,
    result_path: Path,
    backup_directory: Path,
    backend: str = "v2",
) -> FlashSummary:
    if backend not in FLASH_BACKENDS:
        raise CalibrationError(f"未知 DepthAI 写入后端：{backend}")
    result = load_kalibr_result(result_path)
    dai = _load_depthai()
    depthai_version = str(getattr(dai, "__version__", ""))
    flash_api = _flash_api_name(backend)
    backup_directory = Path(backup_directory).expanduser()
    backup_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    safe_mxid = "".join(character for character in mxid if character.isalnum())
    backup = backup_directory / f"oak-eeprom-{safe_mxid or 'device'}-{stamp}.json"
    written_calibration = backup_directory / (
        f"oak-calibration-{backend}-{safe_mxid or 'device'}-{stamp}.json"
    )
    final_report = backup_directory / f"ego-lite-calibration-{stamp}.json"
    info = _find_device_info(dai, mxid)
    try:
        device = dai.Device(info)
        try:
            calibration = _read_device_calibration(device)
            if not calibration.eepromToJsonFile(backup):
                raise CalibrationError(f"EEPROM 备份写入失败：{backup}")
            _verify_stereo_chain(dai, calibration, result)
            rotation = [row[:3] for row in result.matrix_m[:3]]
            translation_cm = [value * 100.0 for value in result.translation_m]
            calibration.setImuExtrinsics(
                dai.CameraBoardSocket.CAM_B,
                rotation,
                translation_cm,
                translation_cm,
            )
            if not calibration.eepromToJsonFile(written_calibration):
                raise CalibrationError(
                    f"待写入标定 JSON 生成失败：{written_calibration}"
                )
            final_payload = ego_lite._read_calibration_json(
                dai,
                device,
                mxid,
                calibration,
            )
            calibration_to_flash = dai.CalibrationHandler(str(written_calibration))
            flash_calibration_handler(device, calibration_to_flash, backend)
        finally:
            device.close()
        readback = _readback_imu_to_left(dai, mxid)
    except Exception as exc:
        raise CalibrationError(
            f"EEPROM 写入或回读失败：{exc}；写入前备份位于 {backup}"
        ) from exc

    expected_cm = [row.copy() for row in result.matrix_m]
    for row in range(3):
        expected_cm[row][3] *= 100.0
    maximum_error = max(
        abs(readback[row][column] - expected_cm[row][column])
        for row in range(4)
        for column in range(4)
    )
    if maximum_error > 1e-4:
        raise CalibrationError(
            f"EEPROM 回读值与写入值不一致（最大误差 {maximum_error:.3g}）；"
            f"写入前备份位于 {backup}"
        )
    try:
        write_ego_lite_report(final_payload, final_report)
    except OSError as exc:
        raise CalibrationError(
            "EEPROM 已写入且回读一致，但完整标定报告保存失败："
            f"{exc}；厂家原生写入 JSON 位于 {written_calibration}"
        ) from exc
    return FlashSummary(
        backup,
        written_calibration,
        final_report,
        result,
        readback,
        maximum_error,
        backend,
        depthai_version,
        flash_api,
    )


def write_ego_lite_report(payload: dict[str, Any], path: Path) -> None:
    Path(path).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def flash_backend_capabilities(dai: Any | None = None) -> dict[str, bool]:
    runtime = dai or _load_depthai()
    device_type = getattr(runtime, "Device", None)
    return {
        backend: callable(getattr(device_type, _flash_api_name(backend), None))
        for backend in FLASH_BACKENDS
    }


def flash_calibration_handler(device: Any, calibration: Any, backend: str) -> None:
    if backend not in FLASH_BACKENDS:
        raise CalibrationError(f"未知 DepthAI 写入后端：{backend}")
    api_name = _flash_api_name(backend)
    writer = getattr(device, api_name, None)
    if not callable(writer):
        version = _device_runtime_version(device)
        raise CalibrationError(
            f"当前 DepthAI {version or '运行库'} 不支持 {api_name}；"
            f"请选择可用的写入版本"
        )
    result = writer(calibration)
    if result is False:
        raise CalibrationError(f"{api_name} 返回写入失败")


def kalibr_commands(
    directory: Path,
    target_yaml: Path,
    imu_yaml: Path,
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    directory = Path(directory).resolve()
    bag = directory / "oak-cam-imu.bag"
    return (
        (
            "kalibr_bagcreater",
            "--folder",
            f"{directory}/",
            "--output-bag",
            str(bag),
        ),
        (
            "kalibr_calibrate_imu_camera",
            "--bag",
            str(bag),
            "--cams",
            str((directory / "camchain.yaml").resolve()),
            "--imu",
            str(Path(imu_yaml).resolve()),
            "--target",
            str(Path(target_yaml).resolve()),
            "--dont-show-report",
        ),
    )


KALIBR_COMMAND_NAMES = ("kalibr_bagcreater", "kalibr_calibrate_imu_camera")


def kalibr_setup_candidates() -> tuple[Path, ...]:
    """返回常见的 ROS/Kalibr 环境脚本，顺序按用户配置优先。"""
    candidates: list[Path] = []
    configured = os.environ.get("EGO_KALIBR_SETUP", "").strip()
    if configured:
        candidates.append(Path(configured).expanduser())
    home = Path.home()
    project_script = Path(__file__).resolve().parents[2] / "scripts" / "kalibr-env.sh"
    bundled_script = Path(getattr(sys, "_MEIPASS", "")) / "scripts" / "kalibr-env.sh"
    candidates.extend(
        [
            project_script,
            project_script.with_name("kalibr-env-docker.sh"),
            bundled_script,
            bundled_script.with_name("kalibr-env-docker.sh"),
            home / "kalibr_ws" / "devel" / "setup.bash",
            home / "kalibr_ws" / "install" / "setup.bash",
            home / "catkin_ws" / "devel" / "setup.bash",
            home / "catkin_ws" / "install" / "setup.bash",
        ]
    )
    candidates.extend(sorted(Path("/opt/ros").glob("*/setup.bash")))
    unique: list[Path] = []
    for path in candidates:
        path = path.expanduser()
        if path not in unique and path.is_file():
            unique.append(path)
    return tuple(unique)


def _commands_from_path(path: str | None = None) -> dict[str, Path]:
    return {
        command: Path(found)
        for command in KALIBR_COMMAND_NAMES
        if (found := shutil.which(command, path=path)) is not None
    }


def _commands_from_setup(setup_script: Path) -> dict[str, Path]:
    source = shlex.quote(str(setup_script))
    query = "; ".join(
        f"printf '__EGO_KALIBR_{command}__ '; "
        f"command -v {shlex.quote(command)} || true"
        for command in KALIBR_COMMAND_NAMES
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", f"source {source} >/dev/null 2>&1 && {query}"],
            check=False,
            capture_output=True,
            text=True,
            timeout=8,
        )
    except (OSError, subprocess.SubprocessError):
        return {}
    found: dict[str, Path] = {}
    markers = {
        f"__EGO_KALIBR_{command}__": command
        for command in KALIBR_COMMAND_NAMES
    }
    for line in result.stdout.splitlines():
        line = line.strip()
        marker = next((key for key in markers if line.startswith(key)), None)
        if marker is None:
            continue
        command = markers[marker]
        path_text = line[len(marker):].strip()
        if not path_text:
            continue
        path = Path(path_text)
        if path.is_file() and os.access(path, os.X_OK):
            found[command] = path
    return found


def _cv_bridge_available(setup_script: Path | None = None) -> bool:
    """检查 Kalibr 使用的 Python 环境是否包含 ROS cv_bridge。"""
    probe = "import cv_bridge"
    try:
        if setup_script is None:
            python = shutil.which("python3") or shutil.which("python")
            if python is None:
                return False
            result = subprocess.run(
                [python, "-c", probe],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
            )
        else:
            source = shlex.quote(str(setup_script))
            result = subprocess.run(
                [
                    "bash",
                    "-lc",
                    f"source {source} >/dev/null 2>&1 && "
                    f"python3 -c {shlex.quote(probe)}",
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
            )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


def detect_kalibr_environment(setup_script: Path | None = None) -> KalibrEnvironment:
    """检测当前进程或指定 setup.bash 是否能提供两个 Kalibr 命令。"""
    direct = _commands_from_path()
    if len(direct) == len(KALIBR_COMMAND_NAMES) and setup_script is None:
        runtime_missing = () if _cv_bridge_available() else ("cv_bridge",)
        return KalibrEnvironment(None, direct, (), runtime_missing)

    scripts = (Path(setup_script).expanduser(),) if setup_script else kalibr_setup_candidates()
    for script in scripts:
        if not script.is_file():
            continue
        commands = _commands_from_setup(script)
        if len(commands) == len(KALIBR_COMMAND_NAMES):
            runtime_missing = () if _cv_bridge_available(script) else ("cv_bridge",)
            return KalibrEnvironment(script, commands, (), runtime_missing)

    missing = tuple(command for command in KALIBR_COMMAND_NAMES if command not in direct)
    return KalibrEnvironment(None, direct, missing)


def missing_kalibr_commands() -> tuple[str, ...]:
    """兼容旧调用：只报告当前进程 PATH 中缺少的命令。"""
    return detect_kalibr_environment().missing


def kalibr_process_spec(
    command: tuple[str, ...], environment: KalibrEnvironment
) -> tuple[str, list[str]]:
    """生成 QProcess 可用的程序和参数，必要时先 source setup.bash。"""
    if environment.setup_script is None:
        return command[0], list(command[1:])
    source = shlex.quote(str(environment.setup_script))
    script = f"source {source} && exec {shlex.join(command)}"
    return "bash", ["-lc", script]


def validate_kalibr_dataset(
    directory: Path,
    target_yaml: Path,
    imu_yaml: Path,
) -> str:
    """在启动 Kalibr 前检查文件、话题和时间戳，返回可读摘要。"""
    directory = Path(directory).expanduser()
    target_yaml = Path(target_yaml).expanduser()
    imu_yaml = Path(imu_yaml).expanduser()
    if not directory.is_dir():
        raise ValueError(f"数据集目录不存在：{directory}")

    image_sets: dict[str, list[Path]] = {}
    for name in ("cam0", "cam1"):
        folder = directory / name
        if not folder.is_dir():
            raise ValueError(f"数据集缺少目录：{folder}")
        images = sorted(folder.glob("*.png"))
        if not images:
            raise ValueError(f"{name} 中没有 PNG 图像")
        image_sets[name] = images
    left_count = len(image_sets["cam0"])
    right_count = len(image_sets["cam1"])
    left_timestamps = _image_timestamps(image_sets["cam0"], "cam0")
    right_timestamps = _image_timestamps(image_sets["cam1"], "cam1")
    paired_count, unpaired_left, unpaired_right = _match_stereo_timestamps(
        left_timestamps, right_timestamps
    )
    match_ratio = paired_count / max(left_count, right_count)
    if match_ratio < MIN_STEREO_MATCH_RATIO:
        raise ValueError(
            "左右灰度图时间戳匹配率过低："
            f"有效配对 {paired_count}，左未配对 {unpaired_left}，"
            f"右未配对 {unpaired_right}，匹配率 {match_ratio:.1%}"
        )

    imu_path = directory / "imu0.csv"
    if not imu_path.is_file():
        raise ValueError(f"数据集缺少：{imu_path}")
    imu_count = 0
    first_imu = last_imu = None
    with imu_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != IMU_CSV_HEADER:
            raise ValueError(
                "imu0.csv 表头必须为：" + ",".join(IMU_CSV_HEADER)
            )
        for row in reader:
            try:
                timestamp = int(row["timestamp"])
                values = [
                    float(row[key])
                    for key in IMU_CSV_HEADER[1:]
                ]
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"imu0.csv 包含无效数据（第 {imu_count + 2} 行）") from exc
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"imu0.csv 包含非有限数值（第 {imu_count + 2} 行）")
            if last_imu is not None and timestamp <= last_imu:
                raise ValueError(f"imu0.csv 时间戳不递增（第 {imu_count + 2} 行）")
            first_imu = timestamp if first_imu is None else first_imu
            last_imu = timestamp
            imu_count += 1
    if imu_count < 10 or first_imu is None or last_imu is None:
        raise ValueError("imu0.csv 数据量过少，至少需要 10 条原始 IMU")

    camchain_path = directory / "camchain.yaml"
    if not camchain_path.is_file():
        raise ValueError(f"数据集缺少：{camchain_path}")
    camchain = camchain_path.read_text(encoding="utf-8", errors="replace")
    for marker in ("cam0:", "cam1:", "/cam0/image_raw", "/cam1/image_raw"):
        if marker not in camchain:
            raise ValueError(f"camchain.yaml 缺少 {marker}")

    if not target_yaml.is_file():
        raise ValueError(f"AprilGrid 配置不存在：{target_yaml}")
    target = target_yaml.read_text(encoding="utf-8", errors="replace")
    if not re.search(r"^\s*target_type\s*:\s*aprilgrid\s*$", target, re.MULTILINE | re.IGNORECASE):
        raise ValueError("target.yaml 必须声明 target_type: aprilgrid")
    for key in ("tagCols", "tagRows", "tagSize", "tagSpacing"):
        if not re.search(rf"^\s*{re.escape(key)}\s*:\s*[-+]?\d", target, re.MULTILINE):
            raise ValueError(f"AprilGrid 配置缺少 {key}")

    if not imu_yaml.is_file():
        raise ValueError(f"IMU 噪声配置不存在：{imu_yaml}")
    imu_config = imu_yaml.read_text(encoding="utf-8", errors="replace")
    for key in (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
    ):
        if not re.search(rf"^\s*{re.escape(key)}\s*:\s*[-+]?\d", imu_config, re.MULTILINE):
            raise ValueError(f"IMU 噪声 YAML 缺少 {key}")

    duration = (last_imu - first_imu) / 1_000_000_000
    rate = (imu_count - 1) / duration if duration > 0 else 0.0
    stereo_summary = f"左右各 {paired_count} 帧"
    if unpaired_left or unpaired_right:
        stereo_summary = (
            f"左右有效配对 {paired_count} 对"
            f"（原始左 {left_count}、右 {right_count}；"
            f"未配对左 {unpaired_left}、右 {unpaired_right}）"
        )
    return (
        f"输入检查通过：{stereo_summary}，IMU {imu_count} 条，"
        f"IMU 时长 {duration:.2f} 秒、平均 {rate:.2f} Hz；"
        f"AprilGrid：{target_yaml.name}；噪声：{imu_yaml.name}"
    )


def _create_capture_pipeline(dai: Any) -> Any:
    pipeline = dai.Pipeline()
    for name, socket in (
        ("calib_left", dai.CameraBoardSocket.CAM_B),
        ("calib_right", dai.CameraBoardSocket.CAM_C),
    ):
        camera = pipeline.createMonoCamera()
        camera.setBoardSocket(socket)
        camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        camera.setFps(CAMERA_FPS)
        output = pipeline.createXLinkOut()
        output.setStreamName(name)
        camera.out.link(output.input)
    _add_raw_imu_stream(dai, pipeline)
    return pipeline


def _create_imu_capture_pipeline(dai: Any) -> Any:
    pipeline = dai.Pipeline()
    _add_raw_imu_stream(dai, pipeline)
    return pipeline


def _add_raw_imu_stream(dai: Any, pipeline: Any) -> None:
    imu = pipeline.createIMU()
    imu.enableIMUSensor(
        [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], IMU_HZ
    )
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(20)
    imu_output = pipeline.createXLinkOut()
    imu_output.setStreamName("calib_imu")
    imu.out.link(imu_output.input)


def _camera_yaml(
    calibration: Any,
    socket: Any,
    *,
    undistorted: bool = False,
) -> dict[str, Any]:
    intrinsics, width, height = calibration.getDefaultIntrinsics(socket)
    k = _matrix(intrinsics, 3, 3)
    coefficients = [float(value) for value in calibration.getDistortionCoefficients(socket)]
    model = str(calibration.getDistortionModel(socket)).lower()
    distortion_model = "equidistant" if "fisheye" in model else "radtan"
    if undistorted:
        distortion_model = "none"
        coefficients = []
    return {
        "intrinsics": [k[0][0], k[1][1], k[0][2], k[1][2]],
        "distortion_model": distortion_model,
        "distortion_coeffs": coefficients[:4],
        "resolution": [int(width), int(height)],
    }


def _create_undistortion_map(
    cv2: Any,
    calibration: Any,
    socket: Any,
) -> UndistortionMap:
    """使用完整 EEPROM Perspective 参数创建主机端去畸变映射。

    DepthAI 2.x 的 ``MonoCamera`` 输出没有设备端 ``enableUndistortion``
    接口，因此不能只截取前 4 个系数冒充 Kalibr radtan；OpenCV 的
    ``initUndistortRectifyMap`` 能直接处理 Perspective 的 14 参数格式。
    """

    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - opencv normally supplies numpy
        raise CalibrationError("未安装 NumPy，无法执行 OAK 灰度图去畸变") from exc

    intrinsics, width, height = calibration.getDefaultIntrinsics(socket)
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise CalibrationError(f"OAK 标定分辨率无效：{width}x{height}")
    matrix = np.asarray(_matrix(intrinsics, 3, 3), dtype=np.float64)
    coefficients = [
        float(value) for value in calibration.getDistortionCoefficients(socket)
    ]
    model = str(calibration.getDistortionModel(socket)).lower()
    if "fisheye" in model:
        raise CalibrationError(
            "当前 Kalibr 采集链路不支持 OAK fisheye 灰度相机；请改用 Perspective 模型"
        )
    # OpenCV 允许 4/5/8/12/14 项；空列表表示没有畸变，补成 5 个零值。
    distortion = np.asarray(coefficients or [0.0] * 5, dtype=np.float64).reshape(-1, 1)
    try:
        map_x, map_y = cv2.initUndistortRectifyMap(
            matrix,
            distortion,
            None,
            matrix,
            (width, height),
            cv2.CV_32FC1,
        )
    except Exception as exc:
        raise CalibrationError(f"创建 OAK 灰度图去畸变映射失败：{exc}") from exc
    return UndistortionMap(map_x, map_y, (width, height))


def _undistort_frame(cv2: Any, frame: Any, mapping: UndistortionMap) -> Any:
    shape = getattr(frame, "shape", ())
    actual = (int(shape[1]), int(shape[0])) if len(shape) >= 2 else None
    if actual != mapping.resolution:
        expected = f"{mapping.resolution[0]}x{mapping.resolution[1]}"
        found = f"{actual[0]}x{actual[1]}" if actual else "未知"
        raise CalibrationError(
            f"OAK 灰度图分辨率与 EEPROM 不一致：收到 {found}，期望 {expected}"
        )
    try:
        return cv2.remap(
            frame,
            mapping.map_x,
            mapping.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
    except Exception as exc:
        raise CalibrationError(f"OAK 灰度图去畸变失败：{exc}") from exc


def _camera_yaml_lines(camera: dict[str, Any]) -> list[str]:
    return [
        "  camera_model: pinhole",
        f"  intrinsics: {_yaml_vector(camera['intrinsics'])}",
        f"  distortion_model: {camera['distortion_model']}",
        f"  distortion_coeffs: {_yaml_vector(camera['distortion_coeffs'])}",
        f"  resolution: {_yaml_vector(camera['resolution'])}",
    ]


def _yaml_vector(values: Any) -> str:
    return "[" + ", ".join(_yaml_number(value) for value in values) + "]"


def _yaml_number(value: Any) -> str:
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.12g}"


def _save_frame(
    cv2: Any,
    packet: Any,
    directory: Path,
    timestamp_offset_ns: int,
    frame: Any | None = None,
) -> None:
    timestamp = _timestamp_ns(packet) + timestamp_offset_ns
    path = directory / f"{timestamp}.png"
    if not cv2.imwrite(str(path), packet.getCvFrame() if frame is None else frame):
        raise CalibrationError(f"图像写入失败：{path}")


def _queue_try_get_all(queue: Any) -> list[Any]:
    getter = getattr(queue, "tryGetAll", None)
    if callable(getter):
        return list(getter())
    packet = queue.tryGet()
    return [] if packet is None else [packet]


def _pop_stereo_pairs(
    pending_left: dict[int, Any],
    pending_right: dict[int, Any],
    tolerance_ns: int = STEREO_TIMESTAMP_TOLERANCE_NS,
) -> tuple[list[tuple[Any, Any]], int, int]:
    pairs: list[tuple[Any, Any]] = []
    discarded_left = discarded_right = 0
    while pending_left and pending_right:
        left_timestamp = min(pending_left)
        right_timestamp = min(pending_right)
        delta = right_timestamp - left_timestamp
        if abs(delta) <= tolerance_ns:
            pairs.append(
                (
                    pending_left.pop(left_timestamp),
                    pending_right.pop(right_timestamp),
                )
            )
        elif delta > 0:
            pending_left.pop(left_timestamp)
            discarded_left += 1
        else:
            pending_right.pop(right_timestamp)
            discarded_right += 1
    return pairs, discarded_left, discarded_right


def _trim_pending_stereo_packets(
    pending_left: dict[int, Any],
    pending_right: dict[int, Any],
    limit: int = STEREO_PENDING_LIMIT,
) -> tuple[int, int]:
    discarded: list[int] = []
    for pending in (pending_left, pending_right):
        count = max(len(pending) - limit, 0)
        for sequence in sorted(pending)[:count]:
            pending.pop(sequence)
        discarded.append(count)
    return discarded[0], discarded[1]


def _image_timestamps(images: list[Path], camera_name: str) -> list[int]:
    timestamps: list[int] = []
    for image in images:
        try:
            timestamp = int(image.stem)
        except ValueError as exc:
            raise ValueError(
                f"{camera_name} 图像文件名不是纳秒时间戳：{image.name}"
            ) from exc
        timestamps.append(timestamp)
    return sorted(timestamps)


def _match_stereo_timestamps(
    left_timestamps: list[int],
    right_timestamps: list[int],
    tolerance_ns: int = STEREO_TIMESTAMP_TOLERANCE_NS,
) -> tuple[int, int, int]:
    left_index = right_index = paired = 0
    unpaired_left = unpaired_right = 0
    while left_index < len(left_timestamps) and right_index < len(right_timestamps):
        delta = right_timestamps[right_index] - left_timestamps[left_index]
        if abs(delta) <= tolerance_ns:
            paired += 1
            left_index += 1
            right_index += 1
        elif delta > 0:
            unpaired_left += 1
            left_index += 1
        else:
            unpaired_right += 1
            right_index += 1
    unpaired_left += len(left_timestamps) - left_index
    unpaired_right += len(right_timestamps) - right_index
    return paired, unpaired_left, unpaired_right


def _timestamp_ns(value: Any) -> int:
    timestamp = value.getTimestampDevice()
    return int(round(timestamp.total_seconds() * 1_000_000_000))


def _imu_csv_row(packet: Any, timestamp_offset_ns: int) -> list[float | int]:
    gyro = packet.gyroscope
    accel = packet.acceleroMeter
    return [
        _timestamp_ns(gyro) + timestamp_offset_ns,
        float(gyro.x),
        float(gyro.y),
        float(gyro.z),
        float(accel.x),
        float(accel.y),
        float(accel.z),
    ]


def _write_capture_metadata(
    directory: Path,
    mxid: str,
    duration_seconds: int,
    device: Any,
    *,
    image_undistorted: bool = False,
) -> None:
    payload = {
        "format": "Kalibr folder dataset",
        "mxid": mxid,
        "camera_mapping": {"cam0": "CAM_B left mono", "cam1": "CAM_C right mono"},
        "camera_fps": CAMERA_FPS,
        "imu_hz": IMU_HZ,
        "imu_type": str(device.getConnectedIMU()),
        "duration_seconds": duration_seconds,
        "target_yaml": "target.yaml",
        "image_undistorted": image_undistorted,
        "distortion_handling": (
            "OpenCV full DepthAI Perspective coefficients (14 parameters)"
            if image_undistorted
            else "raw DepthAI camera output"
        ),
        "timestamp_clock": (
            "DepthAI device monotonic clock plus one common host epoch offset, "
            "nanoseconds"
        ),
    }
    (directory / "capture.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def _write_imu_noise_metadata(
    directory: Path,
    mxid: str,
    requested_duration_seconds: int,
    elapsed_seconds: float,
    imu_samples: int,
    average_rate_hz: float,
    imu_type: str,
    stopped: bool,
) -> None:
    payload = {
        "format": "Kalibr IMU CSV dataset",
        "capture_mode": "stationary_imu_noise",
        "mxid": mxid,
        "imu_hz": IMU_HZ,
        "imu_type": imu_type,
        "requested_duration_seconds": requested_duration_seconds,
        "elapsed_seconds": elapsed_seconds,
        "imu_samples": imu_samples,
        "average_rate_hz": average_rate_hz,
        "stopped_early": stopped,
        "stationary_required": True,
        "csv_file": "imu0.csv",
        "csv_columns": list(IMU_CSV_HEADER),
        "units": {
            "timestamp": "nanoseconds",
            "omega": "rad/s",
            "alpha": "m/s^2",
        },
        "next_step": (
            "Use Allan variance or imu_utils to estimate Kalibr noise density "
            "and random walk parameters; this capture does not create imu.yaml."
        ),
    }
    (directory / "capture.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def _find_device_info(dai: Any, mxid: str) -> Any:
    info = next(
        (item for item in dai.Device.getAllAvailableDevices() if item.getMxId() == mxid),
        None,
    )
    if info is None:
        raise CalibrationError(f"未发现 Ego-Lite 相机：{mxid}")
    return info


def _readback_imu_to_left(dai: Any, mxid: str) -> list[list[float]]:
    deadline = time.monotonic() + 10.0
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            info = _find_device_info(dai, mxid)
            device = dai.Device(info)
            try:
                value = _read_device_calibration(device).getImuToCameraExtrinsics(
                    dai.CameraBoardSocket.CAM_B, False
                )
                return _matrix(value, 4, 4)
            finally:
                device.close()
        except Exception as exc:
            last_error = exc
            time.sleep(0.5)
    raise CalibrationError(f"写入后无法重新连接并回读相机：{last_error}")


def _read_device_calibration(device: Any) -> Any:
    reader = getattr(device, "readCalibration", None)
    if not callable(reader):
        raise CalibrationError("当前 DepthAI 运行库缺少 readCalibration()")
    return reader()


def _flash_api_name(backend: str) -> str:
    return "flashCalibration2" if backend == "v2" else "flashCalibration"


def _device_runtime_version(device: Any) -> str:
    module = type(device).__module__.split(".", 1)[0]
    try:
        runtime = __import__(module)
    except ImportError:
        return ""
    return str(getattr(runtime, "__version__", ""))


def _load_depthai() -> Any:
    try:
        import depthai as dai
    except ImportError as exc:
        raise CalibrationError("未安装 DepthAI 运行库") from exc
    return dai


def _matrix(value: Any, rows: int, columns: int) -> list[list[float]]:
    try:
        matrix = [[float(item) for item in row] for row in value]
    except (TypeError, ValueError) as exc:
        raise CalibrationError("标定矩阵包含无效数值") from exc
    if len(matrix) != rows or any(len(row) != columns for row in matrix):
        raise CalibrationError(f"标定矩阵维度无效，应为 {rows}x{columns}")
    if any(not math.isfinite(item) for row in matrix for item in row):
        raise CalibrationError("标定矩阵包含非有限数值")
    return matrix


def _validate_homogeneous(
    matrix: list[list[float]],
    name: str = "cam0.T_cam_imu",
) -> None:
    matrix = _matrix(matrix, 4, 4)
    if any(abs(matrix[3][column]) > 1e-6 for column in range(3)) or abs(
        matrix[3][3] - 1.0
    ) > 1e-6:
        raise CalibrationError(f"{name} 不是有效的 4x4 齐次矩阵")


def _verify_stereo_chain(dai: Any, calibration: Any, result: KalibrResult) -> None:
    current = _matrix(
        calibration.getCameraExtrinsics(
            dai.CameraBoardSocket.CAM_B,
            dai.CameraBoardSocket.CAM_C,
            False,
        ),
        4,
        4,
    )
    for row in range(3):
        current[row][3] /= 100.0
    rotation_error = max(
        abs(current[row][column] - result.stereo_matrix_m[row][column])
        for row in range(3)
        for column in range(3)
    )
    translation_error = max(
        abs(current[row][3] - result.stereo_matrix_m[row][3])
        for row in range(3)
    )
    if rotation_error > 0.02 or translation_error > 0.005:
        raise CalibrationError(
            "Kalibr 结果中的左目→右目外参与当前 EEPROM 不一致，"
            "不能确认该结果属于当前相机（"
            f"旋转元素误差 {rotation_error:.3g}，平移误差 {translation_error:.3g} m）"
        )


def _rotation_metrics(matrix: list[list[float]]) -> tuple[float, float]:
    rotation = [row[:3] for row in matrix[:3]]
    orthogonal_error = max(
        abs(
            sum(rotation[k][row] * rotation[k][column] for k in range(3))
            - (1.0 if row == column else 0.0)
        )
        for row in range(3)
        for column in range(3)
    )
    a, b, c = rotation[0]
    d, e, f = rotation[1]
    g, h, i = rotation[2]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return determinant, orthogonal_error


def _extract_yaml_scalar(text: str, section: str, key: str) -> float | None:
    lines = text.splitlines()
    section_index = next(
        (index for index, line in enumerate(lines) if line.strip() == f"{section}:"),
        None,
    )
    if section_index is None:
        return None
    section_indent = len(lines[section_index]) - len(lines[section_index].lstrip())
    for line in lines[section_index + 1 :]:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if stripped and not stripped.startswith("#") and indent <= section_indent:
            break
        if stripped.startswith(f"{key}:"):
            try:
                value = float(stripped.split(":", 1)[1].strip())
            except ValueError:
                return None
            return value if math.isfinite(value) else None
    return None

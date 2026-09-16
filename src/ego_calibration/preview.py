from __future__ import annotations

import platform
import threading
import time
from typing import Any

from PySide6.QtCore import QThread, Qt, Signal
from PySide6.QtGui import QCloseEvent, QImage, QPixmap, QShowEvent
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from ego_calibration.models import CameraDevice


DEPTHAI_PREVIEW_SOURCES = (
    ("RGB", "rgb"),
    ("双目灰度（左 | 右）", "stereo"),
    ("左目灰度", "left"),
    ("右目灰度", "right"),
)

from ego_calibration.capture import (EGO_STD_SINGLE_RESOLUTIONS, EGO_STD_STEREO_RESOLUTIONS, uvc_sources, _windows_ks_video_index, _open_video_capture, _open_windows_video_capture, _read_first_frame, _is_video_frame, read_uvc_resolution)

_PREVIEW_STYLE = """
QDialog { background: #F3F6FA; color: #172B4D; }
QLabel#previewTitle { font-size: 18px; font-weight: 700; }
QLabel#previewHint { color: #5E6C84; }
QLabel#previewInfo { background: #EFF6FF; color: #1D4ED8; padding: 6px 10px; font-weight: 700; }
QLabel#previewError { background: #FEF2F2; color: #B91C1C; padding: 6px 10px; font-weight: 700; }
QLabel#previewImage { background: #111827; color: #C9D5E6; border: 1px solid #344563; }
QComboBox { background: #FFFFFF; border: 1px solid #D8E0EA; padding: 7px; min-width: 190px; }
QPushButton { background: #E9EEF5; color: #253858; border: 0; padding: 9px 18px; font-weight: 700; }
QPushButton:hover { background: #DDE5EF; }
"""


class AprilGridPreview:
    """Annotate the live mono pair without changing the saved calibration data."""

    def __init__(self) -> None:
        self._cv2 = None
        self._detector = None
        try:
            import cv2

            self._cv2 = cv2
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
            if hasattr(aruco, "ArucoDetector"):
                self._detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            else:
                self._dictionary = dictionary
                self._parameters = aruco.DetectorParameters_create()
        except (AttributeError, ImportError):
            self._cv2 = None

    def render(self, left_frame: Any, right_frame: Any) -> tuple[QImage, str]:
        left, left_count = self._annotate(left_frame, "CAM_B 左")
        right, right_count = self._annotate(right_frame, "CAM_C 右")
        combined = _combine_stereo_grayscale(left, right)
        image = _frame_to_image(combined)
        if self._detector is None:
            status = "实时预览 · AprilTag 检测不可用，仍可观察标定板覆盖范围"
        else:
            status = (
                "实时预览 · AprilTag 36H11："
                f"左 {left_count}/36，右 {right_count}/36 · "
                "仅用于观察，不会改变保存的原始图像"
            )
        return image, status

    def _annotate(self, frame: Any, label: str) -> tuple[Any, int | None]:
        view = frame.copy()
        if self._cv2 is None or self._detector is None:
            return view, None
        aruco = self._cv2.aruco
        if hasattr(self._detector, "detectMarkers"):
            corners, ids, _rejected = self._detector.detectMarkers(frame)
        else:
            corners, ids, _rejected = aruco.detectMarkers(
                frame, self._dictionary, parameters=self._parameters
            )
        if ids is not None:
            aruco.drawDetectedMarkers(view, corners, ids)
            count = len(ids)
        else:
            count = 0
        self._cv2.putText(
            view,
            label,
            (12, 28),
            self._cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            255,
            2,
            self._cv2.LINE_AA,
        )
        return view, count


class _PreviewThread(QThread):
    frame_ready = Signal(QImage)
    resolution_ready = Signal(int, int)
    info_ready = Signal(str)
    failed = Signal(str)

    def __init__(self, device: CameraDevice, source: Any) -> None:
        super().__init__()
        self.device = device
        self.source = source
        self._stop_requested = threading.Event()

    def stop(self) -> None:
        self._stop_requested.set()

    def run(self) -> None:
        try:
            if self.device.kind == "ego-lite":
                self._run_depthai(str(self.source))
            else:
                self._run_uvc(self.source)
        except Exception as exc:
            if not self._stop_requested.is_set():
                self.failed.emit(str(exc) or type(exc).__name__)

    def _run_depthai(self, channel: str) -> None:
        try:
            import depthai as dai
        except ImportError as exc:
            raise RuntimeError("未安装 DepthAI 运行库") from exc

        pipeline = dai.Pipeline()
        channel_label = {
            "rgb": "RGB",
            "stereo": "双目灰度（左 | 右）",
            "left": "左目灰度",
            "right": "右目灰度",
        }[channel]
        if channel == "stereo":
            for name, socket in (
                ("preview_left", dai.CameraBoardSocket.CAM_B),
                ("preview_right", dai.CameraBoardSocket.CAM_C),
            ):
                output = pipeline.createXLinkOut()
                output.setStreamName(name)
                camera = pipeline.createMonoCamera()
                camera.setBoardSocket(socket)
                camera.setResolution(
                    dai.MonoCameraProperties.SensorResolution.THE_800_P
                )
                camera.setFps(30)
                camera.out.link(output.input)
        else:
            output = pipeline.createXLinkOut()
            output.setStreamName("preview")
        if channel == "rgb":
            camera = pipeline.createColorCamera()
            camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camera.setFps(30)
            camera.video.link(output.input)
        elif channel in ("left", "right"):
            camera = pipeline.createMonoCamera()
            socket = (
                dai.CameraBoardSocket.CAM_B
                if channel == "left"
                else dai.CameraBoardSocket.CAM_C
            )
            camera.setBoardSocket(socket)
            camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            camera.setFps(30)
            camera.out.link(output.input)

        try:
            device = dai.Device(pipeline, dai.DeviceInfo(self.device.identifier))
        except Exception as exc:
            raise RuntimeError(f"DepthAI 实时画面打开失败：{exc}") from exc
        try:
            if channel == "stereo":
                left_queue = device.getOutputQueue(
                    name="preview_left", maxSize=1, blocking=False
                )
                right_queue = device.getOutputQueue(
                    name="preview_right", maxSize=1, blocking=False
                )
                self._read_depthai_stereo_frames(left_queue, right_queue)
            else:
                queue = device.getOutputQueue(
                    name="preview", maxSize=1, blocking=False
                )
                self._read_depthai_frames(queue, channel_label)
        finally:
            device.close()

    def _read_depthai_frames(self, queue: Any, channel_label: str) -> None:
        frame_count = 0
        stats_started = time.monotonic()
        while not self._stop_requested.is_set():
            packet = queue.tryGet()
            if packet is None:
                self.msleep(10)
                continue
            frame = packet.getCvFrame()
            self.frame_ready.emit(_frame_to_image(frame))
            frame_count += 1
            stats_started, frame_count = self._update_stats(
                channel_label,
                frame,
                stats_started,
                frame_count,
            )

    def _read_depthai_stereo_frames(
        self,
        left_queue: Any,
        right_queue: Any,
    ) -> None:
        left_frame = right_frame = None
        frame_count = 0
        stats_started = time.monotonic()
        while not self._stop_requested.is_set():
            left_packet = left_queue.tryGet()
            right_packet = right_queue.tryGet()
            if left_packet is not None:
                left_frame = left_packet.getCvFrame()
            if right_packet is not None:
                right_frame = right_packet.getCvFrame()
            if left_frame is None or right_frame is None:
                self.msleep(5)
                continue
            frame = _combine_stereo_grayscale(left_frame, right_frame)
            left_frame = right_frame = None
            self.frame_ready.emit(_frame_to_image(frame))
            frame_count += 1
            stats_started, frame_count = self._update_stats(
                "双目灰度（左 | 右）",
                frame,
                stats_started,
                frame_count,
            )

    def _run_uvc(self, source: Any) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("未安装 OpenCV 视频运行库") from exc

        capture = _open_video_capture(cv2, source, EGO_STD_STEREO_RESOLUTIONS)
        try:
            frame_count = 0
            failed_reads = 0
            resolution = None
            stats_started = time.monotonic()
            while not self._stop_requested.is_set():
                try:
                    success, frame = capture.read()
                except cv2.error:
                    success, frame = False, None
                if not success or not _is_video_frame(frame):
                    failed_reads += 1
                    if failed_reads >= 30:
                        raise RuntimeError("UVC 相机已打开，但没有收到视频帧")
                    self.msleep(20)
                    continue
                failed_reads = 0
                actual = (int(frame.shape[1]), int(frame.shape[0]))
                if actual != resolution:
                    resolution = actual
                    self.resolution_ready.emit(*actual)
                self.frame_ready.emit(_frame_to_image(frame))
                frame_count += 1
                stats_started, frame_count = self._update_stats(
                    "UVC",
                    frame,
                    stats_started,
                    frame_count,
                )
        finally:
            capture.release()

    def _update_stats(
        self,
        label: str,
        frame: Any,
        started: float,
        frame_count: int,
    ) -> tuple[float, int]:
        elapsed = time.monotonic() - started
        if elapsed < 1.0:
            return started, frame_count
        height, width = frame.shape[:2]
        self.info_ready.emit(
            f"{label} · {width}×{height} · {frame_count / elapsed:.1f} FPS"
        )
        return time.monotonic(), 0


class CameraPreviewDialog(QDialog):
    resolution_ready = Signal(str, int, int)

    def __init__(self, device: CameraDevice, parent: Any = None) -> None:
        super().__init__(parent)
        self.device = device
        self._thread: _PreviewThread | None = None
        self._started = False
        self.setWindowTitle(f"实时画面 · {device.label}")
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.resize(1100, 760)
        self.setMinimumSize(780, 560)
        self.setStyleSheet(_PREVIEW_STYLE)

        title = QLabel(f"实时画面 · {device.label}")
        title.setObjectName("previewTitle")
        self.source = QComboBox()
        self._add_sources()
        self.source.currentIndexChanged.connect(self._restart_stream)

        self.info = QLabel("正在连接相机…")
        self.info.setObjectName("previewInfo")
        self.image = QLabel("正在等待视频帧…")
        self.image.setObjectName("previewImage")
        self.image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image.setMinimumSize(640, 420)

        hint = QLabel(
            "将镜头对准有文字、边缘或纹理的物体，观察细节是否清晰；"
            "Ego-Lite 可切换 RGB、双目灰度、左目灰度和右目灰度。"
        )
        hint.setObjectName("previewHint")
        hint.setWordWrap(True)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.close)

        heading = QHBoxLayout()
        heading.addWidget(title)
        heading.addStretch()
        heading.addWidget(QLabel("画面通道" if device.kind == "ego-lite" else "视频源"))
        heading.addWidget(self.source)
        heading.addWidget(self.info)

        footer = QHBoxLayout()
        footer.addWidget(hint, 1)
        footer.addWidget(close_button)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(12)
        layout.addLayout(heading)
        layout.addWidget(self.image, 1)
        layout.addLayout(footer)

    def _add_sources(self) -> None:
        if self.device.kind == "ego-lite":
            for label, channel in DEPTHAI_PREVIEW_SOURCES:
                self.source.addItem(label, channel)
            return
        for label, source in uvc_sources(self.device.identifier):
            self.source.addItem(label, source)

    def showEvent(self, event: QShowEvent) -> None:
        super().showEvent(event)
        if not self._started:
            self._started = True
            self._start_stream()

    def closeEvent(self, event: QCloseEvent) -> None:
        if not self._stop_stream():
            event.ignore()
            return
        super().closeEvent(event)

    def _restart_stream(self) -> None:
        if not self._started:
            return
        if not self._stop_stream():
            return
        self._start_stream()

    def _start_stream(self) -> None:
        source = self.source.currentData()
        if source is None:
            self._show_error("没有可用的视频源")
            return
        self.image.clear()
        self.image.setText("正在等待视频帧…")
        self.info.setObjectName("previewInfo")
        self.info.setText("正在连接相机…")
        self.info.style().unpolish(self.info)
        self.info.style().polish(self.info)
        thread = _PreviewThread(self.device, source)
        thread.frame_ready.connect(self._show_frame)
        thread.resolution_ready.connect(self._report_resolution)
        thread.info_ready.connect(self.info.setText)
        thread.failed.connect(self._show_error)
        self._thread = thread
        thread.start()

    def _report_resolution(self, width: int, height: int) -> None:
        # 手动切到其他相机时，不把它的尺寸记到当前标定设备上。
        thread = self.sender()
        if thread is not self._thread:
            return
        selected_source = uvc_sources(self.device.identifier)[0][1]
        if (
            not self.device.identifier.startswith("usb://")
            and thread.source == selected_source
        ):
            self.resolution_ready.emit(self.device.identifier, width, height)

    def _stop_stream(self) -> bool:
        if self._thread is None:
            return True
        thread = self._thread
        thread.stop()
        if thread.wait(5000):
            self._thread = None
            return True
        self._show_error("相机仍在关闭，请稍后再试")
        return False

    def _show_frame(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image).scaled(
            self.image.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image.setPixmap(pixmap)

    def _show_error(self, message: str) -> None:
        self.info.setObjectName("previewError")
        self.info.setText("打开失败")
        self.info.style().unpolish(self.info)
        self.info.style().polish(self.info)
        self.image.clear()
        self.image.setText(message)


def _frame_to_image(frame: Any) -> QImage:
    height, width = frame.shape[:2]
    if len(frame.shape) == 2:
        image_format = QImage.Format.Format_Grayscale8
    elif frame.shape[2] == 3:
        image_format = QImage.Format.Format_BGR888
    else:
        image_format = QImage.Format.Format_ARGB32
    return QImage(
        frame.data,
        width,
        height,
        int(frame.strides[0]),
        image_format,
    ).copy()


def _combine_stereo_grayscale(left_frame: Any, right_frame: Any) -> Any:
    import numpy

    if left_frame.ndim != 2 or right_frame.ndim != 2:
        raise RuntimeError("双目灰度画面格式无效")
    if left_frame.shape != right_frame.shape:
        raise RuntimeError("左右目灰度画面分辨率不一致")
    divider = numpy.zeros((left_frame.shape[0], 4), dtype=left_frame.dtype)
    return numpy.concatenate((left_frame, divider, right_frame), axis=1)

"""Camera capture helpers independent of the optional Qt interface."""
from __future__ import annotations
import platform
import time
from typing import Any
from urllib.parse import urlparse

# EGO-Std 的 UVC 视频是一帧左右拼接图。这些尺寸仅用于协商，
# 实际画面尺寸必须从解码后的帧读取，不能据此推断标定内参的分辨率。
EGO_STD_SINGLE_RESOLUTIONS = ((1920, 1080), (1600, 1200))
EGO_STD_STEREO_RESOLUTIONS = tuple(
    (single_width * 2, single_height)
    for single_width, single_height in EGO_STD_SINGLE_RESOLUTIONS
)

def uvc_sources(identifier: str) -> tuple[tuple[str, Any], ...]:
    sources: list[tuple[str, Any]] = []
    windows_index = _windows_ks_video_index(identifier)
    if windows_index is not None:
        sources.append((f"已选 Ego-Std · 视频设备 {windows_index}", windows_index))
    elif identifier and not identifier.startswith(("usb://", "ks://")):
        source: str | int = int(identifier) if identifier.isdecimal() else identifier
        sources.append((f"已选设备 · {identifier}", source))
    for index in range(10):
        if all(source != index for _label, source in sources):
            sources.append((f"视频设备 {index}", index))
    return tuple(sources)


def _windows_ks_video_index(identifier: str) -> int | None:
    """Return the DirectShow filter index embedded in a Windows KS identifier."""
    if not identifier:
        return None
    parsed = urlparse(identifier)
    if parsed.scheme.lower() != "ks":
        return None
    try:
        index = int(parsed.netloc)
    except (TypeError, ValueError):
        return None
    return index if index >= 0 else None


def _open_video_capture(
    cv2: Any,
    source: Any,
    resolutions: tuple[tuple[int, int], ...] = ((1920, 1080),),
) -> Any:
    system = platform.system()
    if system == "Windows" and isinstance(source, int):
        return _open_windows_video_capture(cv2, source, resolutions)

    backend = {
        "Linux": cv2.CAP_V4L2,
        "Darwin": cv2.CAP_AVFOUNDATION,
    }.get(system, cv2.CAP_ANY)
    failures: list[str] = []
    # V4L2 不负责解码 H.264；将压缩字节当图像读取会触发 reshape 异常。
    # 优先使用 OpenCV 能直接解码的 MJPEG，再尝试设备默认格式。
    for width, height in resolutions:
        for codec in ("MJPG", None):
            capture = cv2.VideoCapture()
            ready = False
            label = f"{width}×{height} {codec or '默认格式'}"
            try:
                capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
                capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
                if not capture.open(source, backend) and backend != cv2.CAP_ANY:
                    capture.release()
                    capture = cv2.VideoCapture(source)
                if not capture.isOpened():
                    failures.append(f"{label} 无法打开")
                    continue
                if codec is not None:
                    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*codec))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
                capture.set(cv2.CAP_PROP_FPS, 30)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if _read_first_frame(capture) is None:
                    failures.append(f"{label} 已打开但没有收到可解码的图像")
                    continue
                ready = True
                return capture
            except Exception as exc:
                failures.append(f"{label}：{exc}")
            finally:
                if not ready:
                    capture.release()
    raise RuntimeError(f"无法打开 UVC 视频源 {source}：{'；'.join(failures)}")


def _open_windows_video_capture(
    cv2: Any,
    source: int,
    resolutions: tuple[tuple[int, int], ...],
) -> Any:
    """Open the SC233HGS compressed stereo stream with explicit negotiation."""
    attempts = (
        ("DirectShow MJPEG", cv2.CAP_DSHOW, "MJPG"),
        ("DirectShow 默认格式", cv2.CAP_DSHOW, None),
        ("Media Foundation MJPEG", cv2.CAP_MSMF, "MJPG"),
        ("Media Foundation H.264", cv2.CAP_MSMF, "H264"),
        ("Media Foundation 默认格式", cv2.CAP_MSMF, None),
        ("自动后端", cv2.CAP_ANY, None),
    )
    failures: list[str] = []
    for width, height in resolutions:
        for label, backend, codec in attempts:
            capture = cv2.VideoCapture()
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
            params = [
                cv2.CAP_PROP_FRAME_WIDTH,
                int(width),
                cv2.CAP_PROP_FRAME_HEIGHT,
                int(height),
                cv2.CAP_PROP_FPS,
                30,
            ]
            if codec is not None:
                params.extend(
                    [cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*codec)]
                )
            try:
                opened = bool(capture.open(source, backend, params))
            except Exception as exc:
                failures.append(f"{width}×{height} {label} 参数打开异常：{exc}")
                capture.release()
                continue
            if not opened or not capture.isOpened():
                failures.append(f"{width}×{height} {label} 无法打开")
                capture.release()
                continue

            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            try:
                frame = _read_first_frame(capture)
            except Exception as exc:
                failures.append(f"{width}×{height} {label} 读帧异常：{exc}")
                capture.release()
                continue
            if frame is None:
                failures.append(f"{width}×{height} {label} 已打开但没有收到帧")
                capture.release()
                continue
            return capture

    detail = "；".join(failures)
    modes = " / ".join(f"{width}×{height}" for width, height in resolutions)
    raise RuntimeError(
        f"无法打开 EGO-Std UVC 视频源 {source}（请求 {modes}@30，"
        f"已尝试 H.264/MJPEG 与 DirectShow/MSMF）：{detail}。"
        "请关闭占用相机的应用，并确认 Windows 相机权限和 HEVC/H.264 解码组件。"
    )


def _read_first_frame(capture: Any) -> Any | None:
    for _attempt in range(5):
        success, frame = capture.read()
        if success and _is_video_frame(frame):
            return frame
        time.sleep(0.03)
    return None


def _is_video_frame(frame: Any) -> bool:
    return (
        frame is not None
        and frame.dtype.name == "uint8"
        and frame.ndim in (2, 3)
        and frame.shape[0] > 1
        and frame.shape[1] > 1
        and (frame.ndim == 2 or frame.shape[2] in (3, 4))
    )


def read_uvc_resolution(identifier: str) -> list[int]:
    """Measure the selected device's decoded image, never the requested mode."""
    import cv2

    if identifier.startswith("usb://"):
        raise RuntimeError("USB 标识未提供可确认的 UVC 视频路径")
    source = uvc_sources(identifier)[0][1]
    capture = _open_video_capture(cv2, source, EGO_STD_STEREO_RESOLUTIONS)
    try:
        frame = _read_first_frame(capture)
        if frame is None:
            raise RuntimeError("没有收到可解码的 UVC 图像")
        return [int(frame.shape[1]), int(frame.shape[0])]
    finally:
        capture.release()


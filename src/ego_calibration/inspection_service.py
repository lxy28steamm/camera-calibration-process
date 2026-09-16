from __future__ import annotations

import copy
import json
import threading
import time
import uuid
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ego_calibration import ego_lite, ego_std
from ego_calibration.mono_service import MonoService as DexService, MonoSettings as DexSettings
from ego_calibration.devices import scan_uvc_devices, capabilities
from ego_calibration.inspection import InspectionSettings, StereoInspector, write_report
from ego_calibration.models import CalibrationError, CameraDevice
from ego_calibration.capture import EGO_STD_STEREO_RESOLUTIONS, _is_video_frame, _open_video_capture, uvc_sources
from ego_calibration.validation import validate_calibration
from ego_calibration.lite_service import LiteService
from ego_calibration.std_service import StdService


def jpeg(frame: np.ndarray, width: int = 1600) -> bytes:
    if frame.shape[1] > width:
        frame = cv2.resize(frame, (width, max(1, round(frame.shape[0] * width / frame.shape[1]))), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise CalibrationError("预览图像编码失败")
    return encoded.tobytes()


def split_stereo(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if not _is_video_frame(frame) or frame.shape[1] % 2:
        raise CalibrationError("输入必须为左右等宽拼接的双目图像")
    half = frame.shape[1] // 2
    return frame[:, :half].copy(), frame[:, half:].copy()


class LiveCamera:
    def __init__(self, device: CameraDevice, resolution: tuple[int, int] | None = None):
        self.device, self.resolution = device, resolution
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.changed = threading.Condition(self.lock)
        self.latest = None
        self.preview_source = None
        self.preview = b""
        self.stats: dict[str, Any] = {"frames": 0, "fps": 0.0, "read_errors": 0, "error": "", "running": True, "resolution": None}
        self.thread = threading.Thread(target=self._run, daemon=True, name="camera-stream")
        self.preview_thread = threading.Thread(target=self._encode_preview, daemon=True, name="camera-preview")
        self.thread.start()
        self.preview_thread.start()

    def _run(self):
        try:
            if self.device.kind == "ego-lite":
                self._oak()
            else:
                self._uvc()
        except Exception as exc:
            with self.lock:
                self.stats["error"] = str(exc)
        finally:
            with self.lock:
                self.stats["running"] = False
                self.changed.notify_all()

    def _uvc(self):
        if self.device.identifier.startswith("usb://"):
            raise CalibrationError("当前 USB 标识没有视频路径，请选择系统视频设备路径或索引")
        modes = ((self.resolution[0] * 2, self.resolution[1]),) if self.resolution else EGO_STD_STEREO_RESOLUTIONS
        capture = _open_video_capture(cv2, uvc_sources(self.device.identifier)[0][1], modes)
        started, count, failures = time.monotonic(), 0, 0
        try:
            while not self.stop_event.is_set():
                try:
                    success, frame = capture.read()
                except cv2.error:
                    success, frame = False, None
                if not success or not _is_video_frame(frame):
                    failures += 1
                    with self.lock:
                        self.stats["read_errors"] += 1
                    if failures >= 30:
                        raise CalibrationError("连续无法读取视频帧，请检查设备连接或占用情况")
                    self.stop_event.wait(0.02)
                    continue
                failures = 0
                count += 1
                now = time.monotonic()
                left, right = split_stereo(frame)
                self._store(left, right, now, count, started, frame, "同一 UVC 帧左右拆分；未独立验证硬件曝光同步")
        finally:
            capture.release()

    def _oak(self):
        import depthai as dai

        size = self.resolution or (1280, 800)
        modes = {(1280, 800): "THE_800_P", (1280, 720): "THE_720_P", (640, 400): "THE_400_P"}
        if size not in modes:
            raise CalibrationError(f"OAK 灰度采集不支持此尺寸 {size}；未自动缩放标定内参")
        pipeline = dai.Pipeline()
        for name, socket in (("left", dai.CameraBoardSocket.CAM_B), ("right", dai.CameraBoardSocket.CAM_C)):
            camera = pipeline.createMonoCamera()
            camera.setBoardSocket(socket)
            camera.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, modes[size]))
            camera.setFps(30)
            output = pipeline.createXLinkOut()
            output.setStreamName(name)
            camera.out.link(output.input)
        device = dai.Device(pipeline, dai.DeviceInfo(self.device.identifier))
        try:
            queues = [device.getOutputQueue(name=n, maxSize=4, blocking=False) for n in ("left", "right")]
            pending = [None, None]
            started, count = time.monotonic(), 0
            last_pair = started
            while not self.stop_event.is_set():
                for index, queue in enumerate(queues):
                    packet = queue.tryGet()
                    if packet is not None:
                        pending[index] = packet
                if all(p is not None for p in pending):
                    times = [p.getTimestampDevice().total_seconds() for p in pending]
                    if abs(times[0]-times[1]) <= 0.005:
                        left, right = [p.getCvFrame() for p in pending]
                        pending = [None, None]
                        count += 1
                        now = time.monotonic()
                        self._store(left, right, now, count, started, None, f"DepthAI 设备时间戳配对，差值 {abs(times[0]-times[1])*1000:.2f} ms")
                        last_pair = now
                    else:
                        pending[0 if times[0] < times[1] else 1] = None
                if time.monotonic() - last_pair > 8:
                    raise CalibrationError("OAK 未收到时间戳匹配的左右图像")
                self.stop_event.wait(0.002)
        finally:
            device.close()

    def _store(self, left, right, now, count, started, frame, sync):
        with self.lock:
            self.latest = (count, now, left, right)
            self.preview_source = frame
            self.stats.update(frames=count, fps=count/max(0.001, now-started), resolution=[left.shape[1]*2, left.shape[0]], sync=sync)
            self.changed.notify_all()

    def _encode_preview(self):
        # Only retain the newest frame. JPEG encoding must not block capture or
        # build a queue of old frames on slower hosts.
        sequence, deadline = 0, 0.0
        try:
            while not self.stop_event.is_set():
                if self.stop_event.wait(max(0.0, deadline - time.monotonic())):
                    break
                with self.changed:
                    self.changed.wait_for(lambda: self.stop_event.is_set() or not self.stats["running"] or (self.latest is not None and self.latest[0] != sequence), timeout=1)
                    if self.stop_event.is_set():
                        break
                    if self.latest is None or self.latest[0] == sequence:
                        if not self.stats["running"]:
                            break
                        continue
                    sequence, _, left, right = self.latest
                    frame = self.preview_source
                deadline = time.monotonic() + 1 / 30
                encoded = jpeg(frame if frame is not None else np.concatenate((left, right), axis=1))
                with self.lock:
                    self.preview = encoded
        except Exception as exc:
            with self.lock:
                self.stats["preview_error"] = str(exc)

    def snapshot(self):
        with self.lock:
            return self.latest

    def state(self):
        with self.lock:
            return dict(self.stats)

    def close(self):
        self.stop_event.set()
        with self.changed:
            self.changed.notify_all()
        self.thread.join(5)
        self.preview_thread.join(5)
        if self.thread.is_alive() or self.preview_thread.is_alive():
            raise CalibrationError("相机仍在关闭，请稍后重试")


class InspectionService:
    def __init__(self, directory: Path, dex_project: Path | None = None):
        self.directory = directory.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.devices: list[CameraDevice] = []
        self.selected: CameraDevice | None = None
        self.payload: dict[str, Any] | None = None
        self.live: LiveCamera | None = None
        self.job: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.operation = ""
        self.error = ""
        self.notice = "先扫描并选择相机，再读取设备标定"
        self.report: dict[str, Any] | None = None
        self.session_id = ""
        self.annotated = b""
        self.sample_preview = b""
        self.elapsed = 0.0
        self.health = None
        self.dex = DexService(self.directory, dex_project)
        self.lite = LiteService(self.directory)
        self.std = StdService(self.directory)

    def state(self) -> dict[str, Any]:
        with self.lock:
            report = None
            if self.report:
                report = {k: v for k, v in self.report.items() if k not in ("samples", "calibration")}
                report["latest_sample"] = self.report["samples"][-1] if self.report["samples"] else None
            validation = validate_calibration(self.selected.kind, self.payload) if self.selected and self.payload else None
            return copy.deepcopy({
                "devices": [dict(asdict(d), capabilities=capabilities(d)) for d in self.devices],
                "capabilities": capabilities(self.selected), "health": self.health,
                "selected": asdict(self.selected) if self.selected else None,
                "calibration": self.payload,
                "validation": [asdict(c) for c in validation.checks] if validation else [],
                "busy": bool(self.operation), "operation": self.operation,
                "error": self.error, "notice": self.notice,
                "stream": self.live.state() if self.live else None,
                "frame_mode": "sample" if self.annotated else "live",
                "sample_available": bool(self.sample_preview and self.report and self.report["samples"]),
                "report": report, "session_id": self.session_id, "elapsed_s": self.elapsed,
                "dex": self.dex.state(),
                "lite": self.lite.state(), "std": self.std.state(),
            })

    def dispatch(self, action: str, data: dict[str, Any]) -> None:
        if action == "stop":
            if self.operation in ("dex_flash_write", "lite_flash_write"):
                raise CalibrationError("Flash 写入正在备份、写入或回读，请等待完成后再退出")
            self.stop_event.set()
            return
        functions = {"scan": self._scan, "health": self._health, "layout": self._layout, "select": self._select, "read": self._read, "preview": self._preview, "close_preview": self._close_preview, "inspect": self._inspect, "video": self._video, "replay": self._replay, "load_calibration": self._load_calibration}
        for name in ("preview", "record", "close", "solve", "import_video", "inspect_yaml", "flash_probe", "flash_read", "flash_write"):
            functions["dex_" + name] = lambda data, name=name: self._dex_action(name, data)
        functions["mono_verify"] = lambda data: self._dex_action("solve", {**data, "verify": True})
        for prefix, names in (("lite", ("environment", "capture", "noise", "import_dataset", "solve", "inspect_result", "flash_write")),
                              ("std", ("environment", "import_video", "solve", "load_result"))):
            for name in names:
                functions[prefix + '_' + name] = lambda data, prefix=prefix, name=name: self._calibration_action(prefix, name, data)
        if action not in functions:
            raise CalibrationError("未知操作")
        with self.lock:
            if self.operation:
                raise CalibrationError("当前任务尚未结束，请先停止或等待完成")
            self.operation, self.error = action, ""
            self.stop_event.clear()
            self.job = threading.Thread(target=self._job, args=(functions[action], data), daemon=True, name="inspection-job")
            self.job.start()

    def _job(self, function, data):
        try:
            function(data)
        except Exception as exc:
            with self.lock:
                self.error = str(exc) or type(exc).__name__
                self.notice = "操作未完成，请查看提示"
        finally:
            with self.lock:
                self.operation = ""

    def _scan(self, _data):
        found, warnings = [], []
        for scanner in (ego_std.scan_devices, ego_lite.scan_devices):
            try:
                found.extend(scanner())
            except Exception as exc:
                warnings.append(str(exc))
        found.extend(scan_uvc_devices(found))
        with self.lock:
            self.devices = found
            if self.selected is None and found:
                self.selected = found[0]
            self.notice = f"发现 {len(found)} 台设备" + ("；" + "；".join(warnings) if warnings else "")

    def _select(self, data):
        selected = next((d for d in self.devices if d.identifier == data.get("identifier")), None)
        if selected is None:
            raise CalibrationError("设备不在扫描结果中，请重新扫描")
        self._close_preview({})
        self.dex.invalidate_review()
        self.lite.invalidate_review()
        with self.lock:
            self.selected, self.payload, self.report = selected, None, None
            self.annotated, self.session_id = b"", ""
            self.health = None
            self.notice = "已选择设备，可检查画面与帧率，或按工作流采集标定板"
            if selected.kind == "dex-mono":
                self.notice = "已选择 Dex 单目，可预览、录制并求解内参，或读取 Flash 中的标定"

    def _read(self, _data):
        if self.selected is None:
            raise CalibrationError("请先选择相机")
        if self.selected.kind == "dex-mono":
            return self._dex_action("flash_read", {})
        if not capabilities(self.selected)["read_calibration"]:
            raise CalibrationError("此设备未提供存储读取适配器，请导入标定文件或采集后重新标定")
        self._close_preview({})
        reader = ego_lite.read_calibration if self.selected.kind == "ego-lite" else ego_std.read_calibration
        payload = reader(self.selected.identifier)
        with self.lock:
            self.payload = payload
            self.report, self.annotated = None, b""
            self.notice = "设备标定已读取，可查看参数或打开画面进行复测"

    def _load_calibration(self, data):
        payload, kind = data.get("payload"), data.get("kind")
        if data.get("yaml_text"):
            import yaml
            chain = yaml.safe_load(data["yaml_text"])
            if not isinstance(chain, dict) or not all(name in chain for name in ("cam0", "cam1")):
                raise CalibrationError("双目 YAML 需要 cam0、cam1；单目 YAML 请在单目工作流导入")
            payload = {"format": "kalibr_camchain_imucam", "kalibr_yaml": data["yaml_text"], "kalibr_calibration": chain}
            kind = "uvc-stereo"
        if isinstance(payload, dict) and payload.get("format") == "generic_stereo":
            kind = "uvc-stereo"
        if kind not in ("ego-lite", "ego-std", "ego-std-235", "uvc-stereo") or not isinstance(payload, dict):
            raise CalibrationError("请选择设备标定 JSON、通用双目 JSON 或 Kalibr 双目 YAML")
        result = validate_calibration(kind, payload)
        if result.failure_count:
            raise CalibrationError(f"导入的标定数据存在 {result.failure_count} 项异常：" + "；".join(c.detail for c in result.checks if c.passed is False))
        self._close_preview({})
        with self.lock:
            if kind == "uvc-stereo" and self.selected and self.selected.kind in ("uvc", "uvc-stereo"):
                self.selected = replace(self.selected, kind=kind)
            else:
                self.selected = CameraDevice(kind, "offline", "离线标定文件", transport="导入标定")
            self.payload, self.report, self.annotated = payload, None, b""
            self.notice = "标定已导入，可进行匹配尺寸的双目画面复测"

    def _layout(self, data):
        if not capabilities(self.selected)["select_layout"] or data.get("layout") not in ("mono", "stereo"):
            raise CalibrationError("仅普通 UVC 支持手动选择单目 / 左右拼接布局")
        self._close_preview({})
        self.selected = replace(self.selected, kind="uvc" if data["layout"]=="mono" else "uvc-stereo")
        self.devices = [self.selected if d.identifier==self.selected.identifier else d for d in self.devices]
        self.payload, self.report, self.annotated = None, None, b""
        self.notice = "画面布局已更新，请核实实际输出排列"

    def _health(self, data):
        from ego_calibration.camera_health import check_camera
        if not capabilities(self.selected)["health"] or self.selected.identifier == "offline":
            raise CalibrationError("基础检测需要选择已连接的 V4L2 相机")
        self._close_preview({})
        settings = DexSettings.from_dict(data.get("settings", {}))
        directory = self.dex.new_directory("health")
        self.health = {"status": "running"}
        def progress(report, frame):
            report["report_id"] = self.dex.file_id(directory / "health.json")
            with self.lock:
                self.health = report
                if frame is not None:
                    self.annotated = jpeg(frame)
        try:
            check_camera(self.selected, settings, directory, self.stop_event, progress)
        except Exception as exc:
            self.health.update(status="failed", error=str(exc))
            (directory / "health.json").write_text(json.dumps(self.health, ensure_ascii=False, indent=2), encoding="utf-8")
            raise
        self.notice = "基础检测已保存；请结合实际尺寸、帧率和检查项判断"

    def _preview(self, data):
        if self.selected is None or self.selected.identifier == "offline":
            raise CalibrationError("实时预览需要选择已连接的相机")
        if capabilities(self.selected)["mono"]:
            return self._dex_action("preview", data)
        self._close_preview({})
        resolution = None
        if "settings" in data:
            settings = InspectionSettings.from_dict(data["settings"])
            resolution = (settings.width, settings.height)
        elif self.selected.kind == "ego-lite" and self.payload:
            resolution = tuple(self.payload["cameras"]["left_mono"]["calibration_resolution"])
        with self.lock:
            self.live = LiveCamera(self.selected, resolution)
            self.annotated = b""
            self.notice = "正在打开双目画面"

    def _close_preview(self, _data):
        self.dex.close_preview()
        if self.live:
            self.live.close()
            with self.lock:
                self.live = None

    def _dex_action(self, action, data):
        self._close_preview({})
        if action == "close":
            self.notice = "单目画面已关闭，相机已释放"
        elif action in ("preview", "record"):
            stream = self.dex.capture(self.selected, DexSettings.from_dict(data.get("settings", {})), recording=action == "record")
            self.notice = "单目实时预览" if action == "preview" else "正在录制单目原始视频"
            if action == "record":
                while stream.state()["running"]:
                    if self.stop_event.wait(.1):
                        stream.close()
                        break
                if stream.state()["error"]:
                    raise CalibrationError(stream.state()["error"])
                self.dex.video_id = self.dex.file_id(stream.video)
                self.notice = "单目视频已保存，可以求解或复测单目内参"
        elif action == "solve":
            self.notice = "正在按视频时间戳分析单目标定板"
            self.dex.solve(data, self.directory / "uploads", self.stop_event)
            self.notice = "单目复测完成，请查看实测误差与条件判定" if data.get('verify') else "单目标定求解完成，请检查 PDF 报告和独立画面；求解完成不代表精度已验收"
        elif action == "import_video":
            self.dex.import_video(data.get("upload"), self.directory / "uploads")
            self.notice = "单目视频已导入，确认标定板、模型与求解器后继续"
        elif action == "inspect_yaml":
            result = self.dex.inspect_flash(data, self.selected, self.stop_event)
            self.notice = "Dex YAML 和区域配置已校验；文件格式有效不代表精度已验证" if result.get("ready_to_read") else result.get("reservation_error", "YAML 已校验")
        elif action.startswith("flash_"):
            self.notice = "正在执行 Dex Flash " + action.removeprefix("flash_")
            self.dex.flash(action.removeprefix("flash_"), data, self.selected, self.stop_event)
            self.notice = {"flash_probe": "Dex 设备与 Flash 信息已读取", "flash_read": "Dex 标定已读回并通过 CRC 校验", "flash_write": "Dex 标定写入及逐字节回读校验完成"}[action]

    def _calibration_action(self, prefix, name, data):
        service = self.lite if prefix == 'lite' else self.std
        if name == 'environment':
            service.check_environment(data)
            return
        self._close_preview({})
        if prefix == 'lite':
            if name in ('capture', 'noise'):
                service.capture(data, self.selected, self.stop_event, noise=name == 'noise')
            elif name == 'import_dataset':
                service.import_dataset(data, self.stop_event)
            elif name == 'solve':
                service.solve(data, self.stop_event)
            elif name == 'inspect_result':
                service.inspect_result(data, self.selected)
            elif name == 'flash_write':
                service.flash(data, self.selected, self.stop_event)
        elif name == 'import_video':
            service.import_video(data, self.directory/'uploads', self.stop_event)
        elif name == 'solve':
            service.solve(data, self.stop_event)
        elif name == 'load_result':
            self._load_calibration({'yaml_text': service.calibration_text(data.get('yaml_id'))})
        self.notice = '任务结束，数据和结果保存在对应标定页面'

    def _prepare(self, data, source):
        if not self.payload or not self.selected:
            raise CalibrationError("请先读取或导入设备标定")
        settings = InspectionSettings.from_dict(data.get("settings", {}))
        inspector = StereoInspector(self.selected.kind, copy.deepcopy(self.payload), settings)
        session = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8]
        directory = self.directory / session
        for name in ("cam0", "cam1", "previews"):
            (directory / name).mkdir(parents=True)
        source.update(device=asdict(self.selected), timestamp_basis="相机到达主机的单调时钟 / 视频时间；详见模式")
        metadata = {"settings": asdict(settings), "source": source, "calibration": inspector.payload, "device_kind": inspector.kind}
        (directory / "capture.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        with self.lock:
            self.session_id, self.report, self.elapsed, self.annotated = session, None, 0.0, b""
        return inspector, directory, source

    def _sample(self, inspector, directory, source, left, right, timestamp):
        source["frame_resolution"] = [left.shape[1] + right.shape[1], left.shape[0]]
        sample, annotated = inspector.inspect(left, right, timestamp)
        stem = f"{sample['index']:05d}"
        # Preserve every sampled pair, including quality rejections, for review/replay.
        for name, frame in (("cam0", left), ("cam1", right)):
            ok, encoded = cv2.imencode(".png", frame)
            if not ok:
                raise CalibrationError("无法编码原始图像")
            (directory / name / f"{stem}.png").write_bytes(encoded.tobytes())
        sample["images"] = [f"cam0/{stem}.png", f"cam1/{stem}.png"]
        encoded = jpeg(annotated)
        # A bounded contact sheet keeps exported HTML usable for long runs.
        if sample["accepted"] or sample["index"] < 3:
            sample["preview"] = f"previews/{stem}.jpg"
            (directory / sample["preview"]).write_bytes(encoded)
        with (directory / "observations.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
        with self.lock:
            self.annotated = encoded
            self.sample_preview = encoded
            self.elapsed = timestamp
            self.report = inspector.report(finished=False, source=source)
            self.notice = sample["reason"]

    def _finish(self, inspector, directory, source, failure=""):
        report = inspector.report(finished=True, source=source, failure=failure)
        write_report(directory, report)
        with self.lock:
            self.report, self.notice = report, report["summary"]

    def _inspect(self, data):
        inspector, directory, source = self._prepare(data, {"mode": "live"})
        failure = ""
        try:
            self._preview({"settings": {**asdict(inspector.settings), "width": inspector.model.left.resolution[0], "height": inspector.model.left.resolution[1]}})
            started, next_sample, last_seq = time.monotonic(), 0.0, -1
            while not self.stop_event.is_set():
                elapsed = time.monotonic() - started
                if elapsed >= inspector.settings.duration_s or len(inspector.samples) >= 300:
                    break
                state, snapshot = self.live.state(), self.live.snapshot()
                if state["error"]:
                    raise CalibrationError(state["error"])
                if elapsed > 10 and snapshot is None:
                    raise CalibrationError("相机打开后未收到图像")
                if snapshot and snapshot[0] != last_seq and elapsed >= next_sample:
                    sequence, arrival, left, right = snapshot
                    source["stream"] = state
                    self._sample(inspector, directory, source, left, right, max(0.0, arrival-started))
                    next_sample = elapsed + inspector.settings.interval_s
                    last_seq = sequence
                self.stop_event.wait(0.03)
            source["stopped_by_user"] = self.stop_event.is_set()
            if self.live:
                source["stream"] = self.live.state()
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            self._finish(inspector, directory, source, failure)

    def _video(self, data):
        upload = str(data.get("upload", ""))
        if not upload or Path(upload).name != upload:
            raise CalibrationError("视频上传标识无效")
        path = self.directory / "uploads" / upload
        if not path.is_file():
            raise CalibrationError("上传视频不存在")
        self._close_preview({})
        inspector, directory, source = self._prepare(data, {"mode": "video", "file": upload, "identity_note": "离线视频的设备身份和左右同步由操作者确认，未通过文件独立验证"})
        capture, failure = cv2.VideoCapture(str(path)), ""
        try:
            if not capture.isOpened():
                raise CalibrationError("视频无法打开，请使用原始双目拼接视频")
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not math_is_positive(fps):
                raise CalibrationError("视频未提供有效帧率，无法按时间采样")
            source["fps"] = fps
            index, next_sample = 0, 0.0
            while not self.stop_event.is_set() and len(inspector.samples) < 300:
                success, frame = capture.read()
                if not success:
                    expected = capture.get(cv2.CAP_PROP_FRAME_COUNT)
                    if index == 0 or (expected > 0 and index < expected - 2):
                        raise CalibrationError("视频为空或在文件结束前解码失败")
                    break
                timestamp = index / fps
                index += 1
                if timestamp >= inspector.settings.duration_s:
                    break
                if timestamp >= next_sample:
                    left, right = split_stereo(frame)
                    self._sample(inspector, directory, source, left, right, timestamp)
                    next_sample = timestamp + inspector.settings.interval_s
            source["decoded_frames"] = index
            source["stopped_by_user"] = self.stop_event.is_set()
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            capture.release()
            self._finish(inspector, directory, source, failure)

    def _replay(self, data):
        session = str(data.get("session", ""))
        if Path(session).name != session or not session:
            raise CalibrationError("记录标识无效")
        original = self.directory / session
        metadata = json.loads((original / "capture.json").read_text(encoding="utf-8"))
        observations = [json.loads(line) for line in (original / "observations.jsonl").read_text(encoding="utf-8").splitlines()]
        self._close_preview({})
        with self.lock:
            self.payload = metadata["calibration"]
            self.selected = CameraDevice(**metadata["source"]["device"])
        data = {"settings": data.get("settings", metadata["settings"])}
        inspector, directory, source = self._prepare(data, {"mode": "replay", "original_session": session, "original_source": metadata["source"]})
        failure = ""
        try:
            for sample in observations:
                if self.stop_event.is_set():
                    break
                paths = [(original / name).resolve() for name in sample["images"]]
                if len(paths) != 2 or any(not path.is_relative_to(original.resolve()) for path in paths):
                    raise CalibrationError("记录中的原始样本路径无效")
                frames = [cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_UNCHANGED) for path in paths]
                if any(frame is None for frame in frames):
                    raise CalibrationError("原始样本无法读取")
                self._sample(inspector, directory, source, *frames, sample["timestamp_s"])
            source["stopped_by_user"] = self.stop_event.is_set()
        except Exception as exc:
            failure = str(exc)
            raise
        finally:
            self._finish(inspector, directory, source, failure)

    def records(self):
        records = []
        for path in sorted(self.directory.glob("*/report.json"), reverse=True)[:100]:
            try:
                report = json.loads(path.read_text(encoding="utf-8"))
                records.append({"id": path.parent.name, "created_at": report["created_at"], "summary": report["summary"], "status": report["status"], "counts": report["counts"], "source": report["source"], "resolution": report["resolution"]})
            except (OSError, ValueError, KeyError):
                continue
        return records

    def sample_frame(self):
        with self.lock:
            return self.sample_preview if self.report and self.report["samples"] else b""

    def frame(self, *, live_only=False):
        with self.lock:
            if not live_only and (self.operation in ("inspect", "video", "replay", "health") or self.report or self.health):
                if self.annotated:
                    return self.annotated
            if self.live:
                with self.live.lock:
                    return self.live.preview
            return b""

    def close(self):
        if self.operation in ("dex_flash_write", "lite_flash_write") and self.job:
            self.job.join()
        self.stop_event.set()
        if self.job:
            self.job.join(10)
        self._close_preview({})


def math_is_positive(value):
    return bool(np.isfinite(value) and value > 0)

"""Shared mono workflow with optional Kalibr and Sunplus storage adapters."""
from __future__ import annotations

import copy
import json
import math
import os
import platform
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from ego_calibration.models import CalibrationError, CameraDevice


def scan_dex_devices() -> list[CameraDevice]:
    """Compatibility wrapper; discovery is shared with all V4L2 cameras."""
    from ego_calibration.devices import scan_uvc_devices
    return [d for d in scan_uvc_devices() if d.kind == "dex-mono"]


def child_environment() -> dict[str, str]:
    env = dict(os.environ, MPLBACKEND="Agg", PYTHONUNBUFFERED="1", PYTHONFAULTHANDLER="1")
    # Frozen Python's bundled libraries must not replace FFmpeg / Conda libraries.
    if getattr(sys, "frozen", False):
        original = env.pop("LD_LIBRARY_PATH_ORIG", None)
        if original:
            env["LD_LIBRARY_PATH"] = original
        else:
            env.pop("LD_LIBRARY_PATH", None)
        env.pop("PYTHONHOME", None)
        env.pop("PYTHONPATH", None)
    return env


@dataclass(frozen=True)
class MonoSettings:
    width: int = 1920
    height: int = 1080
    fps: int = 30
    duration_s: int = 120
    board: str = "aprilgrid"
    rows: int = 6
    columns: int = 6
    size_mm: float = 55.0
    gap_mm: float = 16.5
    model: str = "pinhole-equi"
    sample_hz: float = 2.0
    engine: str = "opencv"
    input_format: str = "mjpeg"

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict) or set(values) - cls.__dataclass_fields__.keys():
            raise CalibrationError("单目设置格式无效")
        result = cls(**values)
        for key, low, high in (("width", 64, 8192), ("height", 64, 8192), ("fps", 1, 120), ("duration_s", 1, 1800), ("rows", 2, 30), ("columns", 2, 30)):
            value = getattr(result, key)
            if type(value) is not int or not low <= value <= high:
                raise CalibrationError(f"{key} 必须是 {low}–{high} 的整数")
        for key, low, high in (("size_mm", .1, 1000), ("gap_mm", 0, 1000), ("sample_hz", .1, 30)):
            value = getattr(result, key)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise CalibrationError(f"{key} 数值无效")
        if result.board not in ("aprilgrid", "checkerboard") or result.model not in ("pinhole-equi", "pinhole-radtan", "omni-radtan", "ds-none"):
            raise CalibrationError("标定板或相机模型不受支持")
        if result.engine not in ("opencv", "kalibr") or result.input_format not in ("mjpeg", "yuyv422", "h264"):
            raise CalibrationError("求解器或采集格式无效")
        return result

    def target_yaml(self):
        if self.board == "aprilgrid":
            return f"target_type: aprilgrid\ntagCols: {self.columns}\ntagRows: {self.rows}\ntagSize: {self.size_mm/1000:.9g}\ntagSpacing: {self.gap_mm/self.size_mm:.9g}\n"
        return f"target_type: checkerboard\ntargetCols: {self.columns}\ntargetRows: {self.rows}\nrowSpacingMeters: {self.size_mm/1000:.9g}\ncolSpacingMeters: {self.size_mm/1000:.9g}\n"


class MonoStream:
    def __init__(self, pipeline, device, settings, directory, *, recording=False):
        self.settings, self.directory = settings, directory
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.preview = b""
        self.process = None
        self.video = directory / "recording.mkv" if recording else None
        self.log_path = directory / "recording.log"
        self.stats = {"running": True, "recording": recording, "resolution": None, "preview_frames": 0, "elapsed_s": 0.0, "error": "", "detection": "等待画面"}
        self.metadata = {"status": "recording" if recording else "preview", "device": asdict(device), "settings": asdict(settings), "requested_resolution": [settings.width, settings.height], "requested_fps": settings.fps, "codec": "camera-native MJPEG stream copy", "started_at_ns": time.time_ns()}
        self.height = max(2, round(960 * settings.height / settings.width / 2) * 2)
        self.command = ["ffmpeg", *pipeline.ffmpeg_command(device.identifier, settings.width, settings.height, settings.fps, self.height, self.video, settings.duration_s)]
        self.command[self.command.index("-loglevel") + 1] = "info"
        if "-input_format" in self.command:
            self.command[self.command.index("-input_format")+1] = settings.input_format
            if recording and settings.input_format == "yuyv422":
                self.command[self.command.index("-c:v")+1] = "ffv1"
        self.metadata["codec"] = "FFV1 lossless" if settings.input_format == "yuyv422" else settings.input_format + " native stream copy"
        self.thread = threading.Thread(target=self._run, daemon=True, name="dex-video")
        self.thread.start()

    def _run(self):
        started = time.monotonic()
        try:
            try:
                controls = subprocess.run(["v4l2-ctl", "-d", self.metadata["device"]["identifier"], "--list-ctrls"], capture_output=True, text=True, timeout=3, env=child_environment())
                self.metadata["v4l2_controls"] = controls.stdout
            except (OSError, subprocess.SubprocessError):
                pass
            self._save_metadata()
            with self.log_path.open("wb") as log:
                self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, env=child_environment(), start_new_session=True)
                reader = threading.Thread(target=self._read_frames, daemon=True)
                reader.start()
                stopping_at = None
                while self.process.poll() is None:
                    if self.stop_event.wait(.1) and stopping_at is None:
                        stopping_at = time.monotonic()
                        try:
                            self.process.stdin.write(b"q\n")
                            self.process.stdin.flush()
                        except (BrokenPipeError, OSError):
                            pass
                    if stopping_at and time.monotonic() - stopping_at > 6:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    with self.lock:
                        self.stats["elapsed_s"] = time.monotonic() - started
                    if time.monotonic() - started > 15 and not self.stats["preview_frames"] and stopping_at is None:
                        self.stats["error"] = "相机未收到可解码画面，请检查设备、分辨率或占用情况"
                        self.stop_event.set()
                code = self.process.wait()
                reader.join(3)
                self.process.stdin.close()
                self.process.stdout.close()
            if code != 0 or self.stats["error"]:
                raise CalibrationError(self.stats["error"] or f"FFmpeg 退出码 {code}，请查看采集日志")
            if self.video:
                probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,width,height,avg_frame_rate:format=duration,size", "-of", "json", str(self.video)], capture_output=True, text=True, check=True, timeout=20, env=child_environment())
                info = json.loads(probe.stdout)
                stream = info["streams"][0]
                actual = [stream["width"], stream["height"]]
                with self.lock:
                    self.stats["resolution"] = actual
                self.metadata["video_info"] = info
                if actual != self.metadata["requested_resolution"] or float(info["format"].get("duration", 0)) <= 0:
                    raise CalibrationError("录制尺寸与请求不一致或视频时长无效，请查看 capture.json")
            self.metadata["status"] = "complete"
        except Exception as exc:
            with self.lock:
                self.stats["error"] = str(exc)
            self.metadata.update(status="failed", error=str(exc))
        finally:
            if self.process and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
            self.metadata["finished_at_ns"] = time.time_ns()
            try:
                self._save_metadata()
            except OSError as exc:
                with self.lock:
                    self.stats["error"] = f"无法保存采集元数据：{exc}"
            with self.lock:
                self.stats["running"] = False

    def _save_metadata(self):
        (self.directory / "capture.json").write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    def _read_frames(self):
        size = 960 * self.height * 3
        buffer = bytearray()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        detectors = []
        for border in (1, 2):
            params = cv2.aruco.DetectorParameters()
            params.markerBorderBits = border
            detectors.append(cv2.aruco.ArucoDetector(dictionary, params))
        try:
            while True:
                block = self.process.stdout.read(size - len(buffer))
                if not block:
                    break
                buffer.extend(block)
                if len(buffer) < size:
                    continue
                frame = cv2.cvtColor(np.frombuffer(buffer, np.uint8).reshape(self.height, 960, 3), cv2.COLOR_RGB2BGR)
                buffer.clear()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if self.settings.board == "aprilgrid":
                    observations = [detector.detectMarkers(gray)[:2] for detector in detectors]
                    corners, ids = max(observations, key=lambda item: 0 if item[1] is None else len(item[1]))
                    count = 0 if ids is None else sum(0 <= int(i) < self.settings.rows * self.settings.columns for i in ids.flatten())
                    if ids is not None:
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    detection = f"AprilTag {count}/{self.settings.rows*self.settings.columns}（仅预览；求解重新检测原图）"
                else:
                    pattern = (self.settings.columns, self.settings.rows)
                    found, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_FAST_CHECK)
                    if found:
                        cv2.drawChessboardCorners(frame, pattern, corners, found)
                    detection = "棋盘格：" + ("已检测到完整内角点" if found else "尚未检测到完整内角点")
                ok, encoded = cv2.imencode(".jpg", frame)
                match = re.search(r"Stream #0:0.*?Video:.*?\b(\d{2,5})x(\d{2,5})\b", self.log_path.read_text(errors="replace"))
                with self.lock:
                    if ok:
                        self.preview = encoded.tobytes()
                    self.stats["preview_frames"] += 1
                    self.stats["detection"] = detection
                    if match:
                        self.stats["resolution"] = [int(v) for v in match.groups()]
        except Exception as exc:
            with self.lock:
                self.stats["error"] = f"预览解码失败：{exc}"
            self.stop_event.set()

    def state(self):
        with self.lock:
            return dict(self.stats)

    def close(self):
        self.stop_event.set()
        self.thread.join(12)
        if self.thread.is_alive():
            raise CalibrationError("相机正在保存视频，请稍后重试")


class MonoService:
    def __init__(self, directory: Path, project: Path | None = None):
        self.directory = (directory / ("dex" if (directory / "dex").exists() and not (directory / "mono").exists() else "mono")).resolve()
        self.project = Path(__file__).with_name("backends")
        self.legacy = project.resolve() if project else None
        self.lock = threading.RLock()
        self.stream = None
        self.log_path = None
        self.video_id = ""
        self.result = None
        self.review = None
        self.flash_result = None
        self.review_device = ""
        self.review_token = ""
        self.yaml_path = None
        self.profile_path = None

    def new_directory(self, category):
        path = self.directory / category / (time.strftime("%Y%m%d-%H%M%S-") + secrets.token_hex(4))
        path.mkdir(parents=True)
        return path

    def environment(self):
        return {"project": str(self.project), "bundled": True, "project_ready": all((self.project / name).is_file() for name in ("run.sh", "video_pipeline.py", "kalibr_runner.py", "calibration_flash.py")), "ffmpeg": bool(shutil.which("ffmpeg") and shutil.which("ffprobe")), "flash_sdk": (self.project / "flash_tools/libcamera_flash.so").is_file(), "platform": platform.system()}

    def state(self):
        with self.lock:
            log = ""
            if self.log_path and self.log_path.is_file():
                with self.log_path.open("rb") as stream:
                    stream.seek(max(0, self.log_path.stat().st_size - 18000))
                    log = stream.read().decode(errors="replace")
            return copy.deepcopy({"environment": self.environment(), "stream": self.stream.state() if self.stream else None, "video_id": self.video_id, "result": self.result, "review": self.review, "review_token": self.review_token, "flash_result": self.flash_result, "log": log})

    def file_id(self, path):
        path = path.resolve()
        if path.is_relative_to(self.directory):
            return "local/" + path.relative_to(self.directory).as_posix()
        if self.legacy and path.is_relative_to(self.legacy):
            return "project/" + path.relative_to(self.legacy).as_posix()
        raise CalibrationError("文件不在 单目数据目录")

    def file(self, identifier):
        if not isinstance(identifier, str):
            raise CalibrationError("单目文件标识无效")
        parts = identifier.split("/", 1)
        roots = {"local": self.directory}
        if self.legacy:
            roots["project"] = self.legacy
        if len(parts) != 2 or parts[0] not in roots:
            raise CalibrationError("单目文件标识无效")
        root = roots[parts[0]]
        path = (root / parts[1]).resolve()
        allowed = {"data", "results", "flash_backups", "imports", "preview", "health"} if parts[0] == "local" else {"data", "results", "flash_backups"}
        if not path.is_relative_to(root) or not path.is_file() or path.relative_to(root).parts[0] not in allowed:
            raise CalibrationError("文件不在允许的 单目数据目录")
        if path.suffix.lower() not in {".yaml", ".yml", ".json", ".jsonl", ".txt", ".log", ".pdf", ".mkv", ".mp4", ".avi", ".mov", ".mjpeg", ".mjpg", ".bag", ".bin", ".html", ".csv"}:
            raise CalibrationError("不支持此文件类型")
        return path

    def catalog(self):
        videos, results, flashes = [], [], []
        for root in (self.directory, *((self.legacy,) if self.legacy else ())):
            for path in sorted(root.glob("data/*/recording.mkv"), reverse=True)[:100]:
                metadata = self._json_file(path.parent / "capture.json")
                videos.append({"id": self.file_id(path), "name": path.parent.name, "bytes": path.stat().st_size, "status": metadata.get("status", "unknown"), "resolution": (metadata.get("video_info", {}).get("streams") or [{}])[0].get("width")})
            if root == self.directory:
                for path in sorted(root.glob("imports/*/video.*"), reverse=True)[:100]:
                    if path.is_file():
                        videos.append({"id": self.file_id(path), "name": "导入 · " + path.parent.name, "bytes": path.stat().st_size, "status": "已导入", "resolution": None})
            manifests = set(root.glob("results/*/calibration/processing.json"))
            manifests.update(p.parent / "processing.json" for p in root.glob("results/*/calibration/camera-camchain.yaml"))
            for path in sorted(manifests, reverse=True)[:100]:
                manifest = self._json_file(path)
                results.append({"id": self.file_id(path), "name": path.parent.parent.name, "status": manifest.get("status", "待复核"), "model": manifest.get("camera_model", ""), "yaml_id": self.file_id(path.parent / "camera-camchain.yaml") if (path.parent / "camera-camchain.yaml").exists() else "", "files": [{"name": p.name, "id": self.file_id(p)} for p in sorted(path.parent.glob("*")) if p.is_file() and p.suffix in (".yaml", ".json", ".pdf", ".txt", ".log", ".html", ".csv")]})
            for path in sorted(root.glob("flash_backups/*/result.json"), reverse=True)[:100]:
                result = self._json_file(path)
                files = result.get("files") or [{"name": p.name, "id": self.file_id(p)} for p in sorted(path.parent.glob("*")) if p.is_file() and p.suffix in (".yaml", ".json", ".bin", ".log", ".jsonl")]
                flashes.append({"id": self.file_id(path), "name": path.parent.name, "status": result.get("status", result.get("error", "已保存")), "files": files})
        return {"videos": videos, "results": results, "flashes": flashes}

    @staticmethod
    def _json_file(path):
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def backend(self, script, *args):
        if script == "calibration_flash.py":
            if getattr(sys, "frozen", False):
                return [sys.executable, "--flash-backend", *map(str,args)]
            return [sys.executable, str(self.project / script), *map(str,args)]
        return ["bash", str(self.project / "run.sh"), "python", str(self.project / script), *map(str, args)]

    def run(self, command, log_path, stop_event, *, cancellable=True):
        with self.lock:
            self.log_path = log_path
        with log_path.open("wb") as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, env=child_environment(), start_new_session=True)
            stopped_at = None
            while process.poll() is None:
                if cancellable and stop_event.is_set() and stopped_at is None:
                    stopped_at = time.monotonic()
                    os.killpg(process.pid, signal.SIGTERM)
                if stopped_at and time.monotonic() - stopped_at > 5:
                    os.killpg(process.pid, signal.SIGKILL)
                time.sleep(.1)
            code = process.wait()
        if stopped_at is not None:
            raise CalibrationError("已停止 单目任务，视频、日志和已有结果已保留")
        if code:
            raise CalibrationError(f"单目后端退出码 {code}；请查看下方完整失败日志")

    def close_preview(self):
        if self.stream:
            self.stream.close()
            self.stream = None

    def invalidate_review(self):
        self.review, self.review_token, self.review_device = None, "", ""

    def capture(self, device, settings, *, recording=False):
        if not device or device.kind not in ("dex-mono", "uvc"):
            raise CalibrationError("请选择单目 UVC 相机；不会把 Ego 双目当作单目")
        if platform.system() != "Linux":
            raise CalibrationError("采集后端使用 Linux V4L2")
        from ego_calibration.backends import video_pipeline as module
        directory = self.new_directory("data" if recording else "preview")
        self.stream = MonoStream(module, device, settings, directory, recording=recording)
        self.log_path = self.stream.log_path
        return self.stream

    def solve(self, data, uploads, stop_event):
        settings = MonoSettings.from_dict(data.get("settings", {}))
        if data.get("upload"):
            video = self.import_video(data["upload"], uploads)
        else:
            video = self.file(data.get("video_id") or self.video_id)
        if video.suffix.lower() not in (".mkv", ".avi", ".mp4", ".mov", ".mjpeg", ".mjpg"):
            raise CalibrationError("请选择单目原始视频")
        self.video_id = self.file_id(video)
        directory = self.new_directory("results")
        target = directory / "target.yaml"
        target.write_text(settings.target_yaml(), encoding="utf-8")
        (directory / "request.json").write_text(json.dumps({"settings": asdict(settings), "source_video": self.video_id}, ensure_ascii=False, indent=2), encoding="utf-8")
        self.result = {"status": "running", "directory": self.file_id(target).rsplit("/", 1)[0], "files": []}
        self.invalidate_review()
        output = directory / "calibration"
        command = self.backend("video_pipeline.py", "--video", video, "--target", target, "--output", output, "--model", settings.model, "--sample-hz", settings.sample_hz)
        try:
            if settings.engine == "opencv" or data.get("verify"):
                from ego_calibration.mono_inspection import analyze_video
                yaml_text = data.get("yaml_text")
                if data.get("verify"):
                    if data.get("yaml_id"):
                        yaml_text = self.file(data["yaml_id"]).read_text(encoding="utf-8")
                    if not yaml_text:
                        raise CalibrationError("复测需要导入或选择已有单目 YAML")
                else:
                    yaml_text = None
                self.log_path = directory / "processing.log"
                with self.log_path.open("w",encoding="utf-8") as log:
                    def progress(message):
                        log.write(message+"\n"); log.flush()
                    report = analyze_video(video, settings, output, stop_event, yaml_text=yaml_text, progress=progress)
                self.result["verification"] = {k:v for k,v in report.items() if k!="samples"}
            else:
                self.run(command, directory / "processing.log", stop_event)
            manifest = self._json_file(output / "processing.json")
            if manifest.get("status") != "complete":
                raise CalibrationError("Kalibr 未生成完整结果")
            checked = self._inspect_files(output / "camera-camchain.yaml", None, stop_event)
            self.result.update(camera=checked["camera"], yaml_text=checked["yaml_text"], yaml_id=self.file_id(output / "camera-camchain.yaml"), status="complete")
            self.log_path = directory / "processing.log"
        except Exception as exc:
            output.mkdir(parents=True, exist_ok=True)
            (output / "processing.json").write_text(json.dumps({"status": "cancelled" if stop_event.is_set() else "failed", "error": str(exc), "camera_model": settings.model}, ensure_ascii=False), encoding="utf-8")
            self.result.update(status="cancelled" if stop_event.is_set() else "failed", error=str(exc))
            raise
        finally:
            self.result["files"] = [{"name": p.name, "id": self.file_id(p)} for p in sorted(output.glob("*")) if p.is_file() and p.suffix in (".yaml", ".json", ".pdf", ".txt", ".log", ".bag", ".html", ".csv")]
            self.result["files"].append({"name": "processing.log", "id": self.file_id(directory / "processing.log")})
            (directory / "result.json").write_text(json.dumps(self.result, ensure_ascii=False, indent=2), encoding="utf-8")

    def import_video(self, name, uploads):
        if not isinstance(name, str) or Path(name).name != name:
            raise CalibrationError("上传标识无效")
        video = uploads / name
        if not video.is_file() or video.suffix.lower() not in (".mkv", ".mp4", ".avi", ".mov", ".mjpeg", ".mjpg"):
            raise CalibrationError("视频不存在或类型无效")
        imported = self.new_directory("imports") / ("video" + video.suffix)
        shutil.move(str(video), imported)
        self.video_id = self.file_id(imported)
        return imported

    def _inspect_files(self, yaml_path, profile_path, stop_event):
        from ego_calibration.backends.calibration_flash import inspect_inputs
        return inspect_inputs(yaml_path, profile_path)

    def inspect_flash(self, data, device, stop_event):
        self.invalidate_review()
        directory = self.new_directory("imports")
        yaml_path = None
        if data.get("yaml_text"):
            if not isinstance(data["yaml_text"], str) or len(data["yaml_text"].encode()) > 1024 * 1024:
                raise CalibrationError("YAML 文件大小无效")
            yaml_path = directory / "camera-camchain.yaml"
            yaml_path.write_text(data["yaml_text"], encoding="utf-8")
        elif data.get("yaml_id"):
            source = self.file(data["yaml_id"])
            yaml_path = directory / "camera-camchain.yaml"
            shutil.copyfile(source, yaml_path)
        profile_path = directory / "reservation.json"
        if data.get("profile_text"):
            profile_path.write_text(data["profile_text"], encoding="utf-8")
        else:
            shutil.copyfile(self.project / "flash_tools/reservation.sector126.json", profile_path)
        result = self._inspect_files(yaml_path, profile_path, stop_event)
        if yaml_path:
            result["yaml_id"] = self.file_id(yaml_path)
        with self.lock:
            self.review = result
            self.review_device = device.identifier if device else ""
            self.review_token = secrets.token_urlsafe(24)
            self.yaml_path, self.profile_path = yaml_path, profile_path
        return result

    def flash(self, action, data, device, stop_event):
        if not device or device.kind != "dex-mono":
            raise CalibrationError("Flash 操作需要选择已连接的 Dex 相机")
        if not self.environment()["flash_sdk"]:
            raise CalibrationError("Dex Flash 桥接库不存在，请按部署说明构建 backends/flash_tools/build.sh")
        if action in ("read", "write"):
            if action == "read" and (not self.review or self.review_device != device.identifier):
                self.inspect_flash({}, device, stop_event)
            if not self.review or self.review_device != device.identifier or not self.review.get("ready_to_" + action):
                raise CalibrationError("请先校验 YAML、Flash 区域和当前设备")
        if action == "write" and (data.get("confirmed") is not True or not secrets.compare_digest(str(data.get("review_token", "")), self.review_token)):
            raise CalibrationError("写入前必须核对当前设备、YAML 摘要与扇区，并明确确认本次写入")
        directory = self.new_directory("flash_backups")
        self.flash_result = None
        output = directory / "operation"
        args = [action, "--device", device.identifier, "--output", output]
        if action in ("read", "write"):
            args.extend(("--reservation", self.profile_path, "--profile-sha256", self.review["profile_sha256"]))
        if action == "write":
            args.extend(("--yaml", self.yaml_path, "--yaml-sha256", self.review["yaml_sha256"]))
            self.review_token = ""
        try:
            self.run(self.backend("calibration_flash.py", *args), directory / "operation.log", stop_event, cancellable=action != "write")
            result = self._json_file(output / "result.json")
            if result.get("error") or not result:
                raise CalibrationError(result.get("error", "后端未生成结果，请查看日志"))
            self.flash_result = result
        finally:
            if action == "write":
                self.invalidate_review()
            files = [{"name": p.name, "id": self.file_id(p)} for p in sorted(output.glob("*")) if p.is_file()]
            files.append({"name": "operation.log", "id": self.file_id(directory / "operation.log")})
            if not self.flash_result:
                self.flash_result = {"status": "未验证，请查看日志与备份"}
            self.flash_result["files"] = files
            (directory / "result.json").write_text(json.dumps(self.flash_result, ensure_ascii=False, indent=2), encoding="utf-8")

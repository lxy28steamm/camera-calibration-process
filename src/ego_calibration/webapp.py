"""Local browser application; camera access stays on the connected computer."""
from __future__ import annotations

import argparse
import errno
import ipaddress
import os
import json
import mimetypes
import secrets
import signal
import shutil
import threading
import time
import uuid
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from urllib.parse import parse_qs, quote, unquote, urlparse

from ego_calibration.inspection_service import InspectionService
from ego_calibration import __version__
from ego_calibration.models import CalibrationError
from ego_calibration.network import device_ip


class CameraWebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, service):
        self.service = service
        self.stopping = threading.Event()
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, CameraRequestHandler)
        host, port = self.server_address[:2]
        self.allowed_hosts = {f"{host}:{port}", f"localhost:{port}"}

    def server_close(self):
        self.stopping.set()
        super().server_close()


class CameraRequestHandler(BaseHTTPRequestHandler):
    server: CameraWebServer

    def log_message(self, _format, *_args):
        pass

    def _allowed(self, *, mutation=False):
        if self.headers.get("Host") not in self.server.allowed_hosts:
            self.send_error(HTTPStatus.FORBIDDEN, "Host not allowed")
            return False
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc not in self.server.allowed_hosts:
            self.send_error(HTTPStatus.FORBIDDEN, "Origin not allowed")
            return False
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            self.send_error(HTTPStatus.FORBIDDEN, "Cross-site request blocked")
            return False
        if mutation and not secrets.compare_digest(self.headers.get("X-Camera-Token", ""), self.server.token):
            self.send_error(HTTPStatus.FORBIDDEN, "Invalid request token")
            return False
        return True

    def _send(self, body: bytes, mime: str, status=200, *, filename=None):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, value, status=200):
        self._send(json.dumps(value, ensure_ascii=False, allow_nan=False).encode(), "application/json; charset=utf-8", status)

    def _stream_frames(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        self.connection.settimeout(3)
        previous, idle_since = None, time.monotonic()
        try:
            while not self.server.stopping.is_set():
                frame = self.server.service.frame(live_only=True)
                if frame and frame is not previous:
                    header = f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode()
                    self.wfile.write(header + frame + b"\r\n")
                    self.wfile.flush()
                    previous, idle_since = frame, time.monotonic()
                elif time.monotonic() - idle_since > 3:
                    break
                # No per-client frame queue: a slow connection gets the latest
                # available preview when its previous write completes.
                self.server.stopping.wait(0.005)
        except OSError:
            pass

    def do_GET(self):
        if not self._allowed():
            return
        try:
            path = unquote(urlparse(self.path).path)
            if path == "/api/health":
                self._json({"app": "ego-camera-inspection", "version": __version__})
            elif path == "/api/state":
                self._json(self.server.service.state())
            elif path == "/api/records":
                self._json(self.server.service.records())
            elif path == "/api/dex/catalog":
                self._json(self.server.service.dex.catalog())
            elif path in ('/api/lite/catalog', '/api/std/catalog'):
                self._json(getattr(self.server.service, path.split('/')[2]).catalog())
            elif path == '/api/lite/frame.jpg':
                frame = self.server.service.lite.preview
                self._send(frame, 'image/jpeg', 200 if frame else 204)
            elif path == "/api/dex/frame.jpg":
                stream = self.server.service.dex.stream
                frame = stream.preview if stream else b""
                self._send(frame, "image/jpeg", 200 if frame else 204)
            elif path.startswith(("/dex-files/", "/lite-files/", "/std-files/")):
                prefix, identifier = path.lstrip('/').split('/', 1)
                target = getattr(self.server.service, prefix.removesuffix('-files')).file(identifier)
                self.send_response(200)
                self.send_header("Content-Type", mimetypes.guess_type(target.name)[0] or "application/octet-stream")
                self.send_header("Content-Length", str(target.stat().st_size))
                self.send_header("X-Content-Type-Options", "nosniff")
                if target.suffix not in (".pdf", ".txt", ".log"):
                    self.send_header("Content-Disposition", "attachment; filename*=UTF-8''" + quote(target.name))
                self.end_headers()
                try:
                    with target.open("rb") as stream:
                        shutil.copyfileobj(stream, self.wfile, length=1024*1024)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif path == "/api/frame.jpg":
                frame = self.server.service.frame()
                self._send(frame, "image/jpeg", 200 if frame else 204)
            elif path == "/api/sample.jpg":
                frame = self.server.service.sample_frame()
                self._send(frame, "image/jpeg", 200 if frame else 204)
            elif path == "/api/stream.mjpg":
                self._stream_frames()
            elif path.startswith("/reports/"):
                relative = Path(path.removeprefix("/reports/"))
                root = self.server.service.directory
                target = (root / relative).resolve()
                if not target.is_relative_to(root) or not target.is_file() or target.suffix not in (".html", ".json", ".csv", ".jpg", ".png", ".jsonl") or target.relative_to(root).parts[0] == "uploads":
                    self.send_error(404)
                    return
                download = target.name if target.suffix in (".json", ".csv", ".jsonl") else None
                self._send(target.read_bytes(), mimetypes.guess_type(target.name)[0] or "application/octet-stream", filename=download)
            elif path in ("/", "/app.js", "/preview.js", "/dex.js", "/workflows.js", "/style.css", "/dex.css", "/favicon.svg"):
                name = "index.html" if path == "/" else path[1:]
                body = files("ego_calibration").joinpath("web", name).read_bytes()
                if name == "index.html":
                    body = body.replace(b"__CAMERA_TOKEN__", self.server.token.encode())
                self._send(body, (mimetypes.guess_type(name)[0] or "text/plain") + "; charset=utf-8")
            else:
                self.send_error(404)
        except (OSError, ValueError, KeyError, CalibrationError) as exc:
            self._json({"error": str(exc)}, 400)

    def do_POST(self):
        if not self._allowed(mutation=True):
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            route = urlparse(self.path)
            if route.path == "/api/upload":
                if self.server.service.state()["busy"]:
                    raise CalibrationError("当前任务尚未结束")
                purpose = parse_qs(route.query).get("purpose")
                limit = 8 if purpose in (["dex"], ["std"]) else 2
                if not 0 < length <= limit * 1024**3:
                    raise CalibrationError(f"视频大小须为 1 字节至 {limit} GB")
                name = parse_qs(route.query).get("name", ["video.avi"])[0]
                suffix = Path(name).suffix.lower()
                if suffix not in ((".h264", ".264") if purpose == ["std"] else (".avi", ".mp4", ".mkv", ".mov", ".mjpeg", ".mjpg")):
                    raise CalibrationError("不支持的视频扩展名")
                directory = self.server.service.directory / "uploads"
                directory.mkdir(exist_ok=True)
                target = directory / (uuid.uuid4().hex + suffix)
                try:
                    with target.open("xb") as stream:
                        remaining = length
                        self.connection.settimeout(30)
                        while remaining:
                            block = self.rfile.read(min(1024*1024, remaining))
                            if not block:
                                raise CalibrationError("上传中断")
                            stream.write(block)
                            remaining -= len(block)
                except Exception:
                    target.unlink(missing_ok=True)
                    raise
                self._json({"upload": target.name})
            elif route.path == "/api/action":
                if not 0 < length <= 2 * 1024**2:
                    raise CalibrationError("请求大小无效")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict) or not isinstance(data.get("data", {}), dict) or not isinstance(data.get("action"), str):
                    raise CalibrationError("请求格式无效")
                self.server.service.dispatch(data.get("action", ""), data.get("data", {}))
                self._json({"accepted": True}, 202)
            else:
                self.send_error(404)
        except (OSError, ValueError, CalibrationError) as exc:
            self._json({"error": str(exc)}, 400)


def existing_workbench(url: str) -> bool:
    from urllib.request import ProxyHandler, Request, build_opener
    request = Request(url + "/api/health")
    try:
        # A local LAN service must not be checked through the user's HTTP proxy.
        with build_opener(ProxyHandler({})).open(request, timeout=2) as response:
            return json.load(response).get("app") == "ego-camera-inspection"
    except (OSError, ValueError):
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="相机检测网页服务（前端页面 + Python 后端）")
    parser.add_argument("--host", default=os.environ.get("CAMERA_WEB_HOST", "auto"), help="默认 auto，自动选择运行设备的局域网 IPv4；可指定固定 IP 或 127.0.0.1")
    parser.add_argument("--doctor", action="store_true", help="检查部署依赖并输出 JSON")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--foreground", action="store_true", help="本终端独立运行服务；端口占用时报错，按 Ctrl+C 退出")
    parser.add_argument("--data-dir", type=Path, default=Path(os.environ.get("CAMERA_DATA_DIR", str(Path.home() / "Documents" / "CameraWorkbench"))))
    parser.add_argument("--dex-project", type=Path, help="可选：只读列出旧项目数据，不用于加载执行代码")
    args = parser.parse_args()
    if args.doctor:
        from ego_calibration.diagnostics import diagnose
        print(json.dumps(diagnose(args.data_dir), ensure_ascii=False, indent=2))
        return 0
    if args.host == "auto":
        args.host = device_ip()
        if args.host == "127.0.0.1":
            print("未检测到可用的局域网 IPv4，暂使用本机地址；连接网络后重启可自动更新。", flush=True)
    try:
        address = ipaddress.IPv4Address(args.host)
    except ValueError:
        parser.error("--host 请填写实际 IPv4 地址")
    if address.is_unspecified:
        parser.error("请指定实际局域网 IP，例如 --host 192.168.1.20")
    service = InspectionService(args.data_dir, args.dex_project)
    url = f"http://{args.host}:{args.port}"
    server, browser_timer, previous_sigint = None, None, None
    try:
        try:
            server = CameraWebServer((args.host, args.port), service)
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise CalibrationError(f"无法监听 {url}：{exc}") from None
            if args.foreground:
                print(f"启动失败：{url} 已被占用。此终端没有启动新服务；请先停止旧服务，再重新运行。", flush=True)
                return 2
            if existing_workbench(url):
                print(f"相机检测工作台已运行：{url}", flush=True)
                if not args.no_browser:
                    webbrowser.open(url)
                return 0
            raise CalibrationError(f"地址 {url} 已被占用；请检查正在运行的实例或指定其他 --port") from None
        url = f"http://{args.host}:{server.server_port}"
        print(f"相机检测网页：{url}\n数据目录：{service.directory}\n按 Ctrl+C 停止服务并释放相机。", flush=True)
        if not args.no_browser:
            browser_timer = threading.Timer(0.5, lambda: webbrowser.open(url))
            browser_timer.start()
        server.serve_forever()
    except KeyboardInterrupt:
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("正在停止任务并释放相机；如正在写入标定，将等待写入完成。", flush=True)
    finally:
        try:
            if browser_timer is not None:
                browser_timer.cancel()
            try:
                if server is not None:
                    server.server_close()
            finally:
                service.close()
        finally:
            if previous_sigint is not None:
                signal.signal(signal.SIGINT, previous_sigint)
    print("网页服务已停止。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

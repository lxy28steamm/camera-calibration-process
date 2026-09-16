"""Timed capture checks; reports host arrival timing without claiming hardware sync."""
from __future__ import annotations
import json
import time
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
from ego_calibration.capture import _open_video_capture, _is_video_frame
from ego_calibration.inspection import image_quality
from ego_calibration.models import CalibrationError


def check_camera(device, settings, directory: Path, stop, progress):
    directory.mkdir(parents=True, exist_ok=True)
    capture = _open_video_capture(cv2, device.identifier, ((settings.width, settings.height),))
    capture.set(cv2.CAP_PROP_FPS, settings.fps)
    started = time.monotonic()
    arrivals, dimensions, qualities, errors = [], set(), [], 0
    report = {'device': asdict(device), 'requested': asdict(settings), 'status': 'running', 'timing_basis': '主机接收解码帧；时间间隙不等同于硬件丢帧数'}
    try:
        while time.monotonic() - started < settings.duration_s and not stop.is_set():
            try:
                ok, frame = capture.read()
            except cv2.error:
                ok, frame = False, None
            if not ok or not _is_video_frame(frame):
                errors += 1
                if errors >= 30:
                    raise CalibrationError('读帧错误达到 30 次，检查设备连接和视频模式')
                stop.wait(.02)
                continue
            now = time.monotonic()
            arrivals.append(now)
            dimensions.add((frame.shape[1], frame.shape[0]))
            if len(arrivals) == 1 or now - qualities[-1]['time_s'] - started >= 1:
                qualities.append(dict(time_s=now-started, **image_quality(frame)))
                if len(qualities) == 1:
                    cv2.imwrite(str(directory / 'sample.jpg'), frame)
            report.update(frames=len(arrivals), elapsed_s=now-started, resolution=list(dimensions)[-1])
            if len(arrivals)==1 or now-arrivals[max(0,len(arrivals)-2)]>=.2 or len(arrivals)%6==0:
                progress(dict(report), frame)
        report['status'] = 'cancelled' if stop.is_set() else 'complete'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        capture.release()
        gaps = np.diff(arrivals)
        fps = (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0
        report.update(frames=len(arrivals), read_errors=errors, actual_resolutions=sorted(dimensions), measured_fps=fps, gap_p95_ms=float(np.percentile(gaps,95)*1000) if len(gaps) else None, max_gap_ms=float(max(gaps)*1000) if len(gaps) else None, quality_samples=qualities,
                      checks=[{'name':'实际尺寸匹配请求', 'passed':dimensions=={(settings.width,settings.height)}}, {'name':'收到可解码帧', 'passed':len(arrivals)>1}, {'name':'未持续严重过暗或过曝', 'passed':bool(qualities) and not all(max(q['dark_fraction'],q['bright_fraction'])>.95 for q in qualities)}, {'name':'读帧无报错', 'passed':errors==0}, {'name':'平均帧率达到请求的 90%', 'passed':fps>=settings.fps*.9}],
                      scope='画面、尺寸、主机接收帧率、读帧错误、亮度和清晰度；不替代几何精度、硬件同步、坏点或厂家验收')
        (directory/'health.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        progress(report, None)
    return report

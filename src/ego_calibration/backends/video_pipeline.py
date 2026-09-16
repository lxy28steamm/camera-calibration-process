#!/usr/bin/env python3
"""Sample a video by presentation timestamps, write a ROS1 bag, and run Kalibr."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

MODELS = ('pinhole-radtan', 'pinhole-equi', 'omni-radtan', 'ds-none')


def ffmpeg_command(device, width, height, fps, preview_height, video=None, seconds=120):
    """Shared by the desktop and web UIs; recordings keep native MJPEG frames."""
    args = ['-hide_banner', '-loglevel', 'warning', '-nostats', '-n',
            '-thread_queue_size', '64', '-f', 'v4l2', '-input_format', 'mjpeg',
            '-framerate', str(fps), '-video_size', '%dx%d' % (width, height), '-i', device]
    if video is not None:
        args += ['-map', '0:v:0', '-c:v', 'copy', '-t', str(seconds), str(video)]
    args += ['-map', '0:v:0', '-vf', 'fps=8,scale=960:%d' % preview_height,
             '-pix_fmt', 'rgb24', '-c:v', 'rawvideo', '-threads:v', '2']
    if video is not None:
        args += ['-t', str(seconds)]
    return args + ['-f', 'rawvideo', 'pipe:1']


def probe_video(path):
    result = subprocess.run([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height:frame=best_effort_timestamp_time,width,height',
        '-show_frames', '-of', 'json', str(path),
    ], capture_output=True, text=True, check=True)
    info = json.loads(result.stdout)
    streams = info.get('streams', [])
    frames = info.get('frames', [])
    if not streams or not frames:
        raise ValueError('视频没有可解码的图像帧')
    size = (streams[0]['width'], streams[0]['height'])
    timestamps = []
    for frame in frames:
        timestamp = float(frame.get('best_effort_timestamp_time', 'nan'))
        if not math.isfinite(timestamp):
            raise ValueError('视频缺少有效的逐帧时间戳，请使用本工具重新录制')
        if timestamps and timestamp < timestamps[-1]:
            raise ValueError('视频时间戳倒退，不能用于此采样流程')
        if (frame['width'], frame['height']) != size:
            raise ValueError('视频中途改变分辨率，请拆成独立片段标定')
        timestamps.append(timestamp)
    return streams[0], timestamps


def sample_indices(timestamps, sample_hz):
    if not math.isfinite(sample_hz) or sample_hz <= 0:
        raise ValueError('抽帧频率必须为正数')
    selected = []
    deadline = 0.0
    for index, timestamp in enumerate(timestamps):
        elapsed = timestamp - timestamps[0]
        if elapsed + 1e-9 >= deadline:
            selected.append(index)
            deadline = (math.floor((elapsed + 1e-9) * sample_hz) + 1) / sample_hz
    return selected


def validate_target(path):
    import yaml
    target = yaml.safe_load(path.read_text())
    if not isinstance(target, dict):
        raise ValueError('标定板配置必须是 YAML 字典')
    kind = target.get('target_type')
    if kind == 'aprilgrid':
        counts, lengths = ('tagCols', 'tagRows'), ('tagSize',)
        spacing = float(target.get('tagSpacing', -1))
        if not math.isfinite(spacing) or spacing < 0:
            raise ValueError('tagSpacing 必须为非负的间隙/标签边长比例')
    elif kind == 'checkerboard':
        counts, lengths = ('targetCols', 'targetRows'), ('rowSpacingMeters', 'colSpacingMeters')
    else:
        raise ValueError('当前视频流程支持 aprilgrid 和 checkerboard 配置')
    for key in counts:
        value = target.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise ValueError('%s 必须为至少 2 的整数' % key)
    for key in lengths:
        value = float(target.get(key, 0))
        if not math.isfinite(value) or value <= 0:
            raise ValueError('%s 必须为正数（单位米）' % key)
    return target


def video_to_bag(video, output_bag, stream, timestamps, indices):
    import cv2
    import rosbag
    import rospy
    from sensor_msgs.msg import Image

    cap = cv2.VideoCapture(str(video), cv2.CAP_FFMPEG)
    if not cap.isOpened():
        raise ValueError('OpenCV 无法读取视频')
    pending = set(indices)
    written = 0
    expected_size = (stream['width'], stream['height'])
    try:
        with rosbag.Bag(str(output_bag), 'w') as bag:
            for index, timestamp in enumerate(timestamps):
                ok, frame = cap.read()
                if not ok:
                    raise ValueError('视频解码提前结束：帧 %d / %d' % (index, len(timestamps)))
                if (frame.shape[1], frame.shape[0]) != expected_size:
                    raise ValueError('实际解码分辨率与视频信息不一致')
                if index not in pending:
                    continue
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Camera-only calibration needs ordering, not an IMU clock.
                # Preserve actual video PTS intervals with an arbitrary 1 s origin.
                ns = 1_000_000_000 + round((timestamp - timestamps[0]) * 1_000_000_000)
                stamp = rospy.Time(ns // 1_000_000_000, ns % 1_000_000_000)
                message = Image()
                message.header.seq = written
                message.header.stamp = stamp
                message.header.frame_id = 'cam0'
                message.height, message.width = gray.shape
                message.encoding = 'mono8'
                message.step = message.width
                message.data = gray.tobytes()
                bag.write('/cam0/image_raw', message, stamp)
                written += 1
                if written % 10 == 0 or written == len(indices):
                    print('视频抽帧：%d / %d，分辨率 %d×%d' % (written, len(indices), *expected_size), flush=True)
            if cap.read()[0]:
                raise ValueError('解码帧数与 ffprobe 不一致，停止以避免时间戳错配')
    finally:
        cap.release()
    return written


def run_kalibr(command, log_path):
    """Stop at failed initialization before Kalibr propagates NaNs into results."""
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, text=True, errors='replace', bufsize=1)
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
                if ('initialization of focal length failed' in line.lower()
                        or 'initialization of focal length for cam with topic' in line.lower()):
                    raise ValueError(
                        '焦距初始化失败：请录入整块标定板清晰可见的画面，确认板型/规格，'
                        '并在不同倾角停稳后重试。本次未得到有效内参。详细日志：' + str(log_path))
            code = process.wait()
            if code:
                raise ValueError('Kalibr 求解失败（退出码 %d），详细日志：%s' % (code, log_path))
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--target', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='A new directory; existing directories are rejected')
    parser.add_argument('--model', choices=MODELS, default='pinhole-equi')
    parser.add_argument('--sample-hz', type=float, default=2.0)
    parser.add_argument('--prepare-only', action='store_true', help='Only write the bag, without calibration')
    args = parser.parse_args()
    args.video = args.video.resolve(strict=True)
    args.target = args.target.resolve(strict=True)
    args.output = args.output.resolve()
    validate_target(args.target)
    if args.output.exists():
        raise ValueError('输出目录已存在，请使用新目录，避免覆盖结果')
    print('读取视频时间戳…', flush=True)
    stream, timestamps = probe_video(args.video)
    indices = sample_indices(timestamps, args.sample_hz)
    if len(indices) < (1 if args.prepare_only else 12):
        raise ValueError('抽取图像不足 12 张，请增加录制时长或抽帧频率')
    args.output.mkdir(parents=True)
    target = args.output / 'target.yaml'
    shutil.copyfile(args.target, target)
    bag = args.output / 'camera.bag'
    manifest = {
        'status': 'preparing', 'source_video': str(args.video), 'stream': stream,
        'video_frames': len(timestamps), 'sample_hz': args.sample_hz,
        'selected_indices': indices, 'selected_pts_seconds': [timestamps[i] for i in indices],
        'bag_time_origin': '1 second + video PTS relative to first video frame; no IMU synchronization',
        'camera_model': args.model,
    }
    manifest_file = args.output / 'processing.json'
    try:
        manifest_file.write_text(json.dumps(manifest, indent=2) + '\n')
        manifest['bag_images'] = video_to_bag(args.video, bag, stream, timestamps, indices)
        manifest['status'] = 'prepared'
        if not args.prepare_only:
            command = [sys.executable, str(Path(__file__).with_name('kalibr_runner.py')), '--bag', str(bag),
                       '--target', str(target), '--models', args.model, '--topics', '/cam0/image_raw',
                       '--dont-show-report']
            print('开始 Kalibr 单目内参标定…', flush=True)
            run_kalibr(command, args.output / 'kalibr.log')
            import yaml
            result = yaml.safe_load((args.output / 'camera-camchain.yaml').read_text())
            camera = result['cam0']
            values = camera['intrinsics'] + camera['distortion_coeffs']
            if not all(math.isfinite(float(v)) for v in values):
                raise ValueError('Kalibr 输出包含非有限参数')
            if camera['resolution'] != [stream['width'], stream['height']]:
                raise ValueError('Kalibr 输出分辨率与视频不一致')
            for name in ('camera-results-cam.txt', 'camera-report-cam.pdf'):
                if not (args.output / name).is_file():
                    raise ValueError('Kalibr 未生成预期文件：' + name)
            manifest['status'] = 'complete'
            print('标定完成：' + str(args.output / 'camera-camchain.yaml'), flush=True)
    except BaseException as exc:
        manifest['status'] = 'failed'
        manifest['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        manifest_file.write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print('处理失败：' + (str(exc) or type(exc).__name__), file=sys.stderr, flush=True)
        sys.exit(1)

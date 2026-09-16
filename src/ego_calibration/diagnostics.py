"""Read-only deployment checks, also available from the portable launcher."""
from __future__ import annotations
import ctypes
import importlib.util
import os
import platform
import shutil
import sys
from pathlib import Path
from ego_calibration import __version__


def diagnose(directory):
    import cv2
    import yaml
    from ego_calibration.devices import scan_uvc_devices
    backend=Path(__file__).with_name('backends')
    sdk=backend/'flash_tools/libcamera_flash.so'
    try:
        ctypes.CDLL(str(sdk))
        sdk_status='ready'
    except OSError as exc:
        sdk_status=str(exc)
    parent=directory.resolve()
    while not parent.exists():
        parent=parent.parent
    return {'version':__version__,'platform':platform.platform(),'architecture':platform.machine(),'glibc':platform.libc_ver(),
            'python':sys.version.split()[0], 'opencv':cv2.__version__, 'yaml':yaml.__version__,
            'ffmpeg':shutil.which('ffmpeg'), 'ffprobe':shutil.which('ffprobe'), 'v4l2_ctl':shutil.which('v4l2-ctl'),
            'data_directory':str(directory.resolve()),'data_writable':os.access(parent,os.W_OK),'free_gib':round(shutil.disk_usage(parent).free/1024**3,2),
            'bundled_backend':all((backend/p).is_file() for p in ('run.sh','video_pipeline.py','kalibr_runner.py','calibration_flash.py')),
            'depthai_available':importlib.util.find_spec('depthai') is not None,'sunplus_sdk':sdk_status,
            'kalibr_setup':os.environ.get('CAMERA_KALIBR_SETUP','自动查找用户 Conda 和 ~/kalibr_ws；仅 Kalibr 求解需要'),
            'video_devices':[{'path':d.identifier,'model':d.model,'accessible':d.accessible} for d in scan_uvc_devices()],
            'workflow':'扫描 → 选择布局 → 基础检查 → 采集 → 求解或导入 → 独立数据复测 → 导出',
            'limits':['工业相机专有协议需厂商适配器','单目标定内置 OpenCV；omni/ds 需要可用的 Kalibr 环境','本机未接入的设备型号仍需实机验收']}

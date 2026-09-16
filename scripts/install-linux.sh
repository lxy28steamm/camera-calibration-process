#!/usr/bin/env bash
set -euo pipefail
workbench_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv "$workbench_root/.venv"
"$workbench_root/.venv/bin/python" -m pip install --upgrade pip
"$workbench_root/.venv/bin/python" -m pip install "$workbench_root"
echo '源码环境已安装。启动：bash start-workbench.sh；环境检查：bash start-workbench.sh --doctor'
echo '视频录制需要系统 ffmpeg；设备信息检查可安装 v4l-utils。'

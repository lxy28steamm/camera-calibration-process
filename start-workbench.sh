#!/usr/bin/env bash
set -euo pipefail
workbench_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$workbench_root/config/local.env" ]]; then
  set -a
  source "$workbench_root/config/local.env"
  set +a
fi
if [[ -f "$workbench_root/config/std-write.secret" ]]; then
  export YCTC_XU_UNLOCK_SECRET_FILE="${YCTC_XU_UNLOCK_SECRET_FILE:-$workbench_root/config/std-write.secret}"
fi
export CAMERA_DATA_DIR="${CAMERA_DATA_DIR:-$workbench_root/data}"
export CAMERA_WEB_HOST="${CAMERA_WEB_HOST:-auto}"
for workbench_binary in "$workbench_root/camera-workbench-linux-x86_64.bin" "$workbench_root/dist/ego-calibration-linux-x86_64.bin"; do
  if [[ -x "$workbench_binary" ]]; then exec "$workbench_binary" "$@"; fi
done
if [[ -x "$workbench_root/.venv/bin/python" ]]; then
  export PYTHONPATH="$workbench_root/src${PYTHONPATH:+:$PYTHONPATH}"
  exec "$workbench_root/.venv/bin/python" -m ego_calibration "$@"
fi
echo '未找到可执行文件或源码环境。请运行 bash scripts/install-linux.sh。' >&2
exit 2

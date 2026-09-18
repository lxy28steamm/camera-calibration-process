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
workbench_python="${CAMERA_PYTHON:-$workbench_root/.venv/bin/python}"
if ! command -v "$workbench_python" >/dev/null 2>&1; then
  echo '请先运行 bash scripts/install-linux.sh，或通过 CAMERA_PYTHON 指定已有 Python 环境。' >&2
  exit 2
fi
export PYTHONPATH="$workbench_root/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export CAMERA_DATA_DIR="${CAMERA_DATA_DIR:-$workbench_root/data}"
export CAMERA_WEB_HOST="${CAMERA_WEB_HOST:-auto}"
exec "$workbench_python" -m ego_calibration.webapp --foreground --no-browser "$@"

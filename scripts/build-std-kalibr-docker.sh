#!/usr/bin/env bash
# Optional: build the original customer Kalibr image from the bundled source.
set -euo pipefail
workbench_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workbench_backend="$workbench_root/src/ego_calibration/backends"
workbench_context="$(mktemp -d -t camera-std-docker.XXXXXXXX)"
trap 'rm -rf -- "$workbench_context"' EXIT
command -v docker >/dev/null
PYTHONPATH="$workbench_backend" python3 - "$workbench_context" <<'PY'
import sys
from std_runner import prepare_source
prepare_source(sys.argv[1])
PY
cp -R "$workbench_backend/ego_std_delivery/docker" "$workbench_context/docker"
cp "$workbench_backend/ego_std_delivery/.dockerignore" "$workbench_context/.dockerignore"
docker build --build-arg "CATKIN_JOBS=${CATKIN_JOBS:-1}" \
  -t kalibr-h264-imu-demo:20260805 -f "$workbench_context/docker/Dockerfile" "$workbench_context"

#!/usr/bin/env bash
# Source this file to expose the Docker-backed Kalibr entry points.
set -e

docker_bin="${EGO_KALIBR_DOCKER_BIN:-$(command -v docker 2>/dev/null || true)}"
kalibr_image="${EGO_KALIBR_DOCKER_IMAGE:-ego-kalibr:ros1-noetic}"
kalibr_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${docker_bin}" ]; then
  echo "未找到 Docker，请先安装 Docker Engine/Desktop。" >&2
  return 1 2>/dev/null || exit 1
fi
if ! "${docker_bin}" image inspect "${kalibr_image}" >/dev/null 2>&1; then
  echo "未找到 Kalibr 镜像：${kalibr_image}" >&2
  echo "请先运行 scripts/install-kalibr-docker.sh。" >&2
  return 1 2>/dev/null || exit 1
fi

export EGO_KALIBR_DOCKER_BIN="${docker_bin}"
export EGO_KALIBR_DOCKER_IMAGE="${kalibr_image}"
# PyInstaller data files may be extracted without the executable bit.
chmod +x "${kalibr_script_dir}/docker/kalibr_bagcreater" \
  "${kalibr_script_dir}/docker/kalibr_calibrate_imu_camera" \
  "${kalibr_script_dir}/kalibr-docker-command.sh" 2>/dev/null || true
export PATH="${kalibr_script_dir}/docker:${PATH}"
export EGO_KALIBR_SETUP="${BASH_SOURCE[0]}"

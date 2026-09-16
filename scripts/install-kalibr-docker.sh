#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
docker_bin="${EGO_KALIBR_DOCKER_BIN:-$(command -v docker 2>/dev/null || true)}"
image="${EGO_KALIBR_DOCKER_IMAGE:-ego-kalibr:ros1-noetic}"

if [ -z "${docker_bin}" ]; then
  echo "未找到 Docker。Ubuntu 24.04 请先安装 Docker Engine 或 Docker Desktop。" >&2
  exit 2
fi
echo "使用官方 ROS Noetic/Ubuntu 20.04 基础镜像构建：${image}"
"${docker_bin}" build --tag "${image}" \
  --file "${project_root}/scripts/Dockerfile.kalibr" "${project_root}"
echo
echo "Docker Kalibr 安装完成。验证："
echo "  source ${project_root}/scripts/kalibr-env-docker.sh"
echo "  command -v kalibr_bagcreater"
echo "  command -v kalibr_calibrate_imu_camera"

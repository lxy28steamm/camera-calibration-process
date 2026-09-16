#!/usr/bin/env bash
# Internal runner used by the two Docker-backed Kalibr entry points.
set -euo pipefail

command_name="${1:?missing Kalibr command name}"
shift
docker_bin="${EGO_KALIBR_DOCKER_BIN:-$(command -v docker 2>/dev/null || true)}"
kalibr_image="${EGO_KALIBR_DOCKER_IMAGE:-ego-kalibr:ros1-noetic}"
if [ -z "${docker_bin}" ]; then
  echo "未找到 Docker。" >&2
  exit 127
fi
if ! "${docker_bin}" image inspect "${kalibr_image}" >/dev/null 2>&1; then
  echo "未找到 Kalibr 镜像：${kalibr_image}；请运行 scripts/install-kalibr-docker.sh。" >&2
  exit 127
fi

# Keep host absolute paths unchanged inside the container. This lets Kalibr
# write the bag/YAML next to the captured images without copying the dataset.
declare -A mounted_dirs=()
volumes=()
for argument in "$@"; do
  case "${argument}" in
    /*)
      host_path="${argument%/}"
      [ -n "${host_path}" ] || host_path="/"
      mount_dir="$(dirname "${host_path}")"
      if [ -d "${host_path}" ]; then
        mount_dir="${host_path}"
      fi
      if [ -d "${mount_dir}" ] && [ -z "${mounted_dirs[${mount_dir}]+x}" ]; then
        mounted_dirs["${mount_dir}"]=1
        volumes+=(-v "${mount_dir}:${mount_dir}:rw")
      fi
      ;;
  esac
done

# Pass arguments as positional parameters to bash rather than interpolating
# them into a shell string; paths containing spaces remain safe.
"${docker_bin}" run --rm "${volumes[@]}" \
  --user "$(id -u):$(id -g)" \
  -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  "${kalibr_image}" bash -lc \
  'source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && exec "$@"' \
  -- "${command_name}" "$@"

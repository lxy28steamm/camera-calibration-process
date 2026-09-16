#!/usr/bin/env bash
# Kalibr runs in a ROS environment; the web application remains headless.
set -e
if [[ -n "${CAMERA_KALIBR_SETUP:-}" ]]; then
  source "$CAMERA_KALIBR_SETUP"
else
  calibration_conda="${CONDA_BASE:-}"
  if [[ -z "$calibration_conda" ]]; then
    for candidate in "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3"; do
      if [[ -f "$candidate/etc/profile.d/conda.sh" ]]; then calibration_conda="$candidate"; break; fi
    done
  fi
  if [[ -f "$calibration_conda/etc/profile.d/conda.sh" ]]; then
    source "$calibration_conda/etc/profile.d/conda.sh"
    conda activate "${EGO_KALIBR_CONDA_ENV:-ego-kalibr}"
    [[ ! -f "${CONDA_PREFIX}/setup.bash" ]] || source "${CONDA_PREFIX}/setup.bash"
  fi
  calibration_workspace="${EGO_KALIBR_WS:-$HOME/kalibr_ws}"
  for setup in "$calibration_workspace/devel/setup.bash" "$calibration_workspace/install/setup.bash"; do
    if [[ -f "$setup" ]]; then source "$setup"; break; fi
  done
fi
export OPENBLAS_NUM_THREADS="${CALIBRATION_THREADS:-2}"
export OMP_NUM_THREADS="${CALIBRATION_THREADS:-2}"
export MPLBACKEND=Agg
if ! python -c 'import rosbag, rospkg, cv2, yaml, igraph; rospkg.RosPack().get_path("kalibr")' >/dev/null 2>&1; then
  echo 'Kalibr / ROS 环境未就绪。可先使用内置 OpenCV；Kalibr 部署见 docs/LINUX.md，或设置 CAMERA_KALIBR_SETUP。' >&2
  exit 2
fi
exec "$@"

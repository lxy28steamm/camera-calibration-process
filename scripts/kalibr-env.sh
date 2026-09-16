#!/usr/bin/env bash
# Load the user-local ROS/Kalibr environment used by the calibration GUI.
# This file is intentionally source-able: it must not start a new shell.
set -e

kalibr_conda_base="${CONDA_BASE:-${HOME}/anaconda3}"
kalibr_env_name="${EGO_KALIBR_CONDA_ENV:-ego-kalibr}"
kalibr_workspace="${EGO_KALIBR_WS:-${HOME}/kalibr_ws}"

if [ -f "${kalibr_conda_base}/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1091
  source "${kalibr_conda_base}/etc/profile.d/conda.sh"
  conda activate "${kalibr_env_name}"
elif [ -x "${kalibr_conda_base}/bin/conda" ]; then
  eval "$("${kalibr_conda_base}/bin/conda" shell.bash hook)"
  conda activate "${kalibr_env_name}"
else
  echo "未找到 Conda：${kalibr_conda_base}" >&2
  return 1 2>/dev/null || exit 1
fi

# Robostack exports ROS variables from the Conda environment setup script.
# Loading it explicitly also makes the helper work from a plain GUI process
# where no interactive shell activation hooks have run yet.
if [ -f "${CONDA_PREFIX:-}/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${CONDA_PREFIX}/setup.bash"
fi

if [ -f "${kalibr_workspace}/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${kalibr_workspace}/devel/setup.bash"
elif [ -f "${kalibr_workspace}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${kalibr_workspace}/install/setup.bash"
fi

# catkin_install_python places Kalibr entry points below the package's devel
# directory; that directory is not on PATH in every ROS/Conda combination.
for kalibr_bin_dir in \
  "${kalibr_workspace}/devel/lib/kalibr" \
  "${kalibr_workspace}/install/lib/kalibr"; do
  if [ -d "${kalibr_bin_dir}" ]; then
    export PATH="${kalibr_bin_dir}:${PATH}"
  fi
done

export EGO_KALIBR_SETUP="${BASH_SOURCE[0]}"

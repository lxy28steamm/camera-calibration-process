#!/usr/bin/env bash
set -euo pipefail

# User-local ROS Noetic + Kalibr setup for Ubuntu systems where native ROS apt
# packages are unavailable (for example Ubuntu 24.04). It does not require sudo.
project_root="$(cd "$(dirname "$0")/.." && pwd)"
conda_bin="${CONDA_BIN:-${HOME}/anaconda3/bin/conda}"
env_name="${EGO_KALIBR_CONDA_ENV:-ego-kalibr}"
workspace="${EGO_KALIBR_WS:-${HOME}/kalibr_ws}"
kalibr_source="${workspace}/src/kalibr"

if [ ! -x "${conda_bin}" ]; then
  echo "未找到 Conda：${conda_bin}" >&2
  echo "请先安装 Miniconda/Anaconda，或设置 CONDA_BIN 指向 conda 可执行文件。" >&2
  exit 2
fi

echo "[1/4] 创建/更新 Conda ROS Noetic 环境：${env_name}"
"${conda_bin}" create -n "${env_name}" -y \
  --override-channels -c robostack -c conda-forge \
  python=3.9 ros-noetic-ros-base ros-noetic-cv-bridge catkin_tools

echo "[2/4] 安装 Kalibr 编译和运行依赖"
"${conda_bin}" install -n "${env_name}" -y \
  --override-channels -c conda-forge \
  eigen suitesparse tbb opencv scipy matplotlib ipython pyx wxpython igraph doxygen

echo "[3/4] 获取 Kalibr 官方源码"
mkdir -p "${workspace}/src"
if [ ! -d "${kalibr_source}/.git" ]; then
  git clone --depth 1 https://github.com/ethz-asl/kalibr.git "${kalibr_source}"
fi
if grep -q 'list(APPEND BOOST_COMPONENTS python38)' \
    "${kalibr_source}/Schweizer-Messer/python_module/cmake/add_python_export_library.cmake" \
    || grep -q 'boost/detail/endian.hpp' \
    "${kalibr_source}/Schweizer-Messer/sm_boost/include/boost/portable_binary_archive.hpp" \
    || ! grep -q 'NUMPY_INCLUDE_DIR' \
    "${kalibr_source}/Schweizer-Messer/numpy_eigen/CMakeLists.txt" \
    || grep -q 'self.camera = cv.PinholeCameraGeometry(proj)' \
    "${kalibr_source}/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py" \
    || grep -q 'cb = pl.colorbar(SM)$' \
    "${kalibr_source}/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccPlots.py" \
    || grep -q 'cb = pl.colorbar(SM)$' \
    "${kalibr_source}/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py"; then
  patch --directory="${kalibr_source}" --forward --batch --strip=1 \
    < "${project_root}/scripts/kalibr-compat.patch" || true
fi

echo "[4/4] 编译 Kalibr（资源较少时可设置 CATKIN_JOBS=1）"
source "${conda_bin%/bin}/etc/profile.d/conda.sh"
conda activate "${env_name}"
python -m pip install igraph==0.11.9 texttable==1.7.0
cd "${workspace}"
if [ ! -f .catkin_tools/profiles/default/config.yaml ]; then
  catkin init
fi
catkin config --extend "${CONDA_PREFIX}"
catkin config --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DEIGEN3_INCLUDE_DIRS="${CONDA_PREFIX}/include/eigen3" \
  -DEIGEN3_INCLUDE_DIR="${CONDA_PREFIX}/include/eigen3" \
  -DCMAKE_INCLUDE_PATH="${CONDA_PREFIX}/include" \
  -DCMAKE_CXX_FLAGS="-isystem ${CONDA_PREFIX}/include" \
  -DCATKIN_ENABLE_TESTING=OFF
catkin build -j"${CATKIN_JOBS:-2}"

echo
echo "Kalibr 安装完成。GUI 环境脚本：${project_root}/scripts/kalibr-env.sh"
echo "验证命令："
echo "  source ${project_root}/scripts/kalibr-env.sh"
echo "  command -v kalibr_bagcreater"
echo "  command -v kalibr_calibrate_imu_camera"

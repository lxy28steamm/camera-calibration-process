#!/usr/bin/env bash
set -euo pipefail
flash_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
: "${SUNPLUS_SDK_DIR:?请设置 SUNPLUS_SDK_DIR 指向厂家 SDK 根目录（含 include 和 lib）}"
flash_arch="$(uname -m)"
cp "$SUNPLUS_SDK_DIR/lib/libSPV4L2/GNU/$flash_arch/libSPV4L2.so" "$flash_dir/libSPV4L2.so"
# Keep the bridge's device-path override; vendor SDK must be dynamically linked.
g++ -std=c++17 -Wall -Wextra -Werror -O2 -fPIC -shared \
  "$flash_dir/sdk_bridge.cpp" -I "$SUNPLUS_SDK_DIR/include" \
  -L "$flash_dir" -lSPV4L2 -Wl,-rpath,'$ORIGIN' -o "$flash_dir/libcamera_flash.so"

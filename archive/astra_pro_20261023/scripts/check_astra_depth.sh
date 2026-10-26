#!/usr/bin/env bash
# 独立构建/运行诊断，固定 SDK 搜索顺序，不修改全局库配置或相机持久参数。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="${ASTRA_OPENNI2_ROOT:-$project_dir/../vio_std/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63}"
sdk_lib="$sdk_dir/Samples/Bin"
if [[ ! -f "$sdk_dir/Include/OpenNI.h" || ! -f "$sdk_lib/libOpenNI2.so" || ! -f "$sdk_lib/OpenNI2/Drivers/liborbbec.so" ]]; then
  echo 'OpenNI SDK unavailable; set ASTRA_OPENNI2_ROOT to the OpenNI 2.3.0.63 directory.' >&2
  exit 1
fi
mkdir -p "$project_dir/build"
c++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I "$sdk_dir/Include" "$project_dir/tools/astra_depth_check.cpp" \
  -L "$sdk_lib" "-Wl,-rpath,$sdk_lib" -lOpenNI2 \
  -o "$project_dir/build/astra_depth_check"
# SDK 库必须在系统 OpenNI 之前，系统 libusb 必须在海康 SDK 的同名库之前。
export LD_LIBRARY_PATH="$sdk_lib:/usr/local/lib:/lib/x86_64-linux-gnu"
exec timeout --signal=TERM --kill-after=2s 20s "$project_dir/build/astra_depth_check" "$@"

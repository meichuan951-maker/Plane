#!/usr/bin/env bash
# Keep configuration, binaries and CTest results in the documented build directory.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build"

usage() {
  cat <<'EOF'
用法：scripts/build.sh [--test] [--] [-D变量=值 ...] [-U缓存项 ...]
  默认：配置并构建 build；--test 随后运行无硬件 CTest。
  已有构建保留缓存；首次构建使用 Release，并关闭四个可选后端。
  可通过 -DAIM_WITH_ASTRA=ON 等参数启用功能；ROS 2 需先 source 环境。
  JOBS 控制并行编译数，默认 2。固定源码和输出目录，不接受 -S/-B/-G。

示例：
  ./scripts/build.sh --test
  ./scripts/build.sh --test -- -DAIM_WITH_ASTRA=ON
EOF
}

run_tests=false
cmake_args=()
while (( $# )); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --test) run_tests=true; shift ;;
    --) shift ;;
    -D|-U)
      if (( $# < 2 )) || [[ "$2" == -* ]]; then
        echo "选项 $1 缺少参数" >&2; exit 2
      fi
      cmake_args+=("$1" "$2"); shift 2 ;;
    -D?*|-U?*) cmake_args+=("$1"); shift ;;
    *) echo "不支持的参数：$1；请运行 --help" >&2; exit 2 ;;
  esac
done
jobs="${JOBS:-2}"
if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'JOBS 必须是正整数。' >&2; exit 2
fi

defaults=()
if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
  defaults=(-DCMAKE_BUILD_TYPE=Release -DAIM_WITH_REALSENSE=OFF
    -DAIM_WITH_ROS2=OFF -DAIM_WITH_MAVLINK=OFF -DAIM_WITH_ASTRA=OFF)
fi
test_args=()
if "$run_tests"; then
  # Put this last: --test must not silently succeed with BUILD_TESTING=OFF.
  test_args=(-DBUILD_TESTING=ON)
fi
cmake -S "$project_dir" -B "$build_dir" "${defaults[@]}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "${cmake_args[@]}" "${test_args[@]}"
cmake --build "$build_dir" --parallel "$jobs"
if "$run_tests"; then
  # Working-directory form also works with the project's CMake 3.16 minimum.
  (cd "$build_dir" && ctest --output-on-failure)
fi

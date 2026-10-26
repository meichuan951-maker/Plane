#!/bin/bash
# Run on the board through: rk-ros2 bash /workspace/no_demo/deploy/rk3566/build_workspace.sh
set -euo pipefail

usage() {
    cat <<'HELP'
用法：rk-ros2 bash /workspace/no_demo/deploy/rk3566/build_workspace.sh [--vision-only]
  默认构建 ROS 2 包和视觉程序；--vision-only 仅更新 YLR4K/UVC 视觉程序。
  视觉输出固定在 no_demo/build/rk3566-humble，单任务编译，不启动硬件。
HELP
}

vision_only=false
while (($#)); do
    case "$1" in
        --vision-only) vision_only=true ;;
        --help|-h) usage; exit 0 ;;
        *) printf '未知参数：%s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

[[ "$(uname -m)" == aarch64 ]] || { echo 'ARM64 board required' >&2; exit 1; }
[[ "${ROS_DISTRO:-}" == humble ]] || { echo 'Enter rk-ros2 first' >&2; exit 1; }
cd /workspace
export CMAKE_BUILD_PARALLEL_LEVEL=1
# colcon-cmake otherwise passes its own -j<cpu_count> to GNU Make.
export MAKEFLAGS=-j1
if ! "$vision_only"; then
    colcon build --base-paths src --executor sequential \
        --packages-select quadrotor_msgs traj_utils plan_env path_searching bspline_opt \
            ego_planner rosmsg_tcp_bridge volans_msgs px4_control \
        --event-handlers console_direct+ \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DEGO_DEBUG_SYMBOLS=OFF
fi
if [[ -f /workspace/install/local_setup.bash ]]; then
    set +u
    source /workspace/install/local_setup.bash
    set -u
fi
cmake -S /workspace/no_demo -B /workspace/no_demo/build/rk3566-humble \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DAIM_WITH_ROS2=ON \
    -DAIM_WITH_REALSENSE=OFF -DAIM_WITH_MAVLINK=OFF
cmake --build /workspace/no_demo/build/rk3566-humble \
    --target demo_aim_cpp armor_auto camera_light_test vision_video_worker --parallel 1

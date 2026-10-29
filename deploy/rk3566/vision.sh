#!/usr/bin/env bash
# Run inside rk-ros2; use native ARM64 binaries and the project's MaixCAM Lite UVC config.
set -euo pipefail

usage() {
    cat <<'HELP'
用法：rk-ros2 bash /workspace/no_demo/deploy/rk3566/vision.sh COMMAND [参数...]
  list        列出相机设备，不打开视频流
  camera      UVC 灯条预览，沿用 armor_auto.yaml 的相机参数，可用参数覆盖
  armor       自动装甲识别，使用 config/armor_auto.yaml
  calibrate   标定当前 MaixCAM Lite；须提供棋盘格参数

示例：
  .../vision.sh list
  .../vision.sh camera --device /dev/video0 --no-show --frames 120
  .../vision.sh armor --no-show --seconds 90
  .../vision.sh calibrate --board-cols 8 --board-rows 11 --square-mm 23.1
HELP
}

if (($# == 0)) || [[ "$1" == --help || "$1" == -h ]]; then
    usage
    exit 0
fi
command_name="$1"
shift
case "$command_name" in
    list|camera|armor|calibrate) ;;
    *) printf '未知命令：%s\n' "$command_name" >&2; usage >&2; exit 2 ;;
esac
[[ "$(uname -m)" == aarch64 ]] || { echo '请在 ARM64 RK3566 板端运行。' >&2; exit 1; }
[[ "${ROS_DISTRO:-}" == humble ]] || { echo '请先通过 rk-ros2 进入 Humble 环境。' >&2; exit 1; }

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$project_dir/build/rk3566-humble"
case "$command_name" in
    list|camera) binary="$build_dir/camera_light_test" ;;
    armor|calibrate) binary="$build_dir/armor_auto" ;;
esac
[[ -x "$binary" ]] || {
    echo '缺少板端视觉程序，请先运行 build_workspace.sh --vision-only。' >&2
    exit 1
}
cd -- "$project_dir"
case "$command_name" in
    list) exec "$binary" --list "$@" ;;
    camera)
        exec python3 - "$binary" "$@" <<'PY'
import os
from pathlib import Path
import sys
import yaml

config_path = Path('config/armor_auto.yaml')
config = yaml.safe_load(config_path.read_text())
camera = config['camera']
args = [sys.argv[1], '--no-controls', '--config', str(config_path.parent / config['detector_config'])]
for field in ('device', 'width', 'height', 'fps', 'format'):
    args.extend(['--' + field, str(camera[field])])
os.execv(sys.argv[1], args + sys.argv[2:])
PY
        ;;
    armor) exec "$binary" --config config/armor_auto.yaml "$@" ;;
    calibrate) exec "$binary" --config config/armor_auto.yaml --calibrate "$@" ;;
esac

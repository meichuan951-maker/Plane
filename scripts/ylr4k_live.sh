#!/usr/bin/env bash
# 本机 YLR4K 已验证配置：1080p MJPG / 30 FPS；商品标称 60 FPS 的启用条件仍待确认。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$project_dir/build/camera_light_test" ]]; then
  echo "请先构建：$project_dir/scripts/build.sh" >&2
  exit 1
fi
# 桌面 IDE 可能继承 Snap 的 schemas 和厂商库路径，使用系统 GUI 环境。
export LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu
if [[ "${GSETTINGS_SCHEMA_DIR:-}" == */snap/* && -f /usr/share/glib-2.0/schemas/gschemas.compiled ]]; then
  export GSETTINGS_SCHEMA_DIR=/usr/share/glib-2.0/schemas
fi
cd "$project_dir"
# 使用 by-id，避免插拔后 /dev/videoN 改变或打开笔记本内置相机。
# 默认只开主预览；需要颜色掩膜时追加 --mask，其他 CLI 参数也可覆盖。
exec "$project_dir/build/camera_light_test" \
  --device /dev/v4l/by-id/usb-Lombo_Technology_YLR4Kusb_camera_000000000000000-video-index0 \
  --width 1920 --height 1080 --fps 30 --format MJPG \
  --config config/light_bars.yaml "$@"

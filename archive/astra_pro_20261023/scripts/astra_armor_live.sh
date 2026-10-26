#!/usr/bin/env bash
# Astra 双窗口：彩色自动装甲候选 + 中心准星/手动深度测量。
# 用户定义的局部世界原点为固定深度相机光心；相机移动后必须重新建立坐标系。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  cat <<'HELP'
Astra Pro live RGB detection + center/manual depth / fixed local world.
Usage: bash ./scripts/astra_armor_live.sh [depth viewer options]
Aim the DEPTH center white box inside armor to display surface distance.
Press c for center probe; drag or press r for manual depth ROI.
RGB detections are NOT automatically registered to depth / ROI.
Camera XYZ: X right, Y down, Z forward (depth optical center, meters).
Local world XYZ: X forward, Y left, Z up, same fixed origin.
Keep the camera stationary. ROI is not tracked; reselect if target moves.
Depth options: --manual-depth, --roi x y width height. q/Esc exits either window.
RGB hardware controls: config/astra_rgb_controls.yaml (verified at startup).
HELP
  exit 0
fi
cd "$project_dir"
run_dir="runs/astra_armor_live_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_dir"
if ! python3 ./tools/astra_rgb_controls.py --apply >"$run_dir/controls.log" 2>&1; then
  cat "$run_dir/controls.log" >&2
  exit 1
fi
cat "$run_dir/controls.log"
rgb_pid=''
depth_pid=''
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$rgb_pid" ]] || kill -INT "$rgb_pid" 2>/dev/null || true
  [[ -z "$depth_pid" ]] || kill -INT "$depth_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
./scripts/armor_auto.sh --output "$run_dir/armor.jsonl" >"$run_dir/rgb.log" 2>&1 &
rgb_pid=$!
./scripts/astra_live.sh --local-world-fixed --center-depth --output "$run_dir/depth_world.jsonl" \
  --snapshot "$run_dir/depth_preview.png" "$@" >"$run_dir/depth.log" 2>&1 &
depth_pid=$!
printf 'Live camera logs: %s/%s\n' "$project_dir" "$run_dir"
printf 'Aim DEPTH center white box inside armor. World assumes a stationary camera.\n'
wait -n "$rgb_pid" "$depth_pid"

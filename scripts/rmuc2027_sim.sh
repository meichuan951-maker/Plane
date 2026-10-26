#!/usr/bin/env bash
# One active micro aircraft; legacy X500 launchers are archived.
set -euo pipefail

usage() {
  cat <<'EOF'
用法：scripts/rmuc2027_sim.sh [sim|keyboard|route|check|contact|contact-preview] [参数...]
  sim              统一选择菜单，默认入口；支持 --mode keyboard|route
  keyboard         163 g / 130 mm 微型机，键盘手动起降与移动
  route            同一微型机自动100 mm起飞、绕行接触、安全返航和原位降落
  check            检查两方微型机和航线，不启动Gazebo
  contact          底层接触实验入口，默认只生成文件，--simulate 才运行
  contact-preview  只生成去返路线预览

键盘：R/F升降，WASD移动，Q/E转向；1/2/3速度档，松键减速，C瞬时复位。
route支持 --team blue|red|both、--headless、--case contact-timeout。
两种模式支持 --check、--start-paused、--renderer auto|nvidia|default|software。
默认auto检测硬件渲染；切换前先关闭当前窗口。
地图默认位于 ../rmuc_gazebo_map，可用 RMUC_MAP_ROOT 指定。
旧 inspect / px4 / navigation 已归档，现用入口只加载微型机。
EOF
}

COMMAND="${1:-sim}"
case "${COMMAND}" in
  -h|--help|help) usage; exit 0 ;;
  sim|keyboard|route|check|contact|contact-preview)
    if [[ $# -gt 0 ]]; then shift; fi ;;
  inspect|px4|navigation)
    echo "旧 ${COMMAND} 入口已归档。请使用 keyboard 或 route；归档见地图工程 archive/legacy_aircraft_20261019/README.md。" >&2
    exit 2 ;;
  *) echo "未知命令：${COMMAND}" >&2; usage >&2; exit 2 ;;
esac
if [[ "${COMMAND}" == keyboard || "${COMMAND}" == route ]]; then
  for ARGUMENT in "$@"; do
    if [[ "${ARGUMENT}" == --mode || "${ARGUMENT}" == --mode=* ]]; then
      echo "${COMMAND} 已指定模式；选择其他模式请使用 sim --mode。" >&2
      exit 2
    fi
  done
fi
if [[ "${COMMAND}" == contact-preview ]]; then
  for ARGUMENT in "$@"; do
    if [[ "${ARGUMENT}" == --s* ]]; then
      echo "contact-preview 只生成预览；运行请使用 contact --simulate。" >&2
      exit 2
    fi
  done
fi
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_ROOT="${RMUC_MAP_ROOT:-${PROJECT_ROOT}/../rmuc_gazebo_map}"
if [[ ! -d "${MAP_ROOT}" ]]; then
  echo "地图工程不存在：${MAP_ROOT}；请设置 RMUC_MAP_ROOT。" >&2
  exit 1
fi
MAP_ROOT="$(cd "${MAP_ROOT}" && pwd)"
for REQUIRED_FILE in config/arena.yaml config/inspection_aircraft.yaml tools/start_simulation.py tools/outpost_contact.py; do
  if [[ ! -f "${MAP_ROOT}/${REQUIRED_FILE}" ]]; then
    echo "地图工程缺少文件：${MAP_ROOT}/${REQUIRED_FILE}" >&2
    exit 1
  fi
done
python3 - "${MAP_ROOT}/config/arena.yaml" <<'PYTHON'
from pathlib import Path
import sys
import yaml
config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding='utf-8'))
source = config.get('metadata', {}).get('source_stl')
if source != 'RMUC2027_V1.0.0.STL' or config.get('cad', {}).get('navigation_source_mesh') != source:
    raise SystemExit('该入口要求完整 RMUC2027_V1.0.0.STL 配置；请检查 RMUC_MAP_ROOT。')
PYTHON
echo "使用地图工程：${MAP_ROOT}"
cd "${MAP_ROOT}"
case "${COMMAND}" in
  sim) exec python3 tools/start_simulation.py "$@" ;;
  keyboard|route) exec python3 tools/start_simulation.py --mode "${COMMAND}" "$@" ;;
  check)
    if [[ $# -ne 0 ]]; then echo "check 不接受额外参数。" >&2; exit 2; fi
    python3 tools/start_simulation.py --mode keyboard --team blue --check
    python3 tools/start_simulation.py --mode keyboard --team red --check
    exec python3 tools/start_simulation.py --mode route --team both --check
    ;;
  contact|contact-preview) exec python3 tools/outpost_contact.py "$@" ;;
esac

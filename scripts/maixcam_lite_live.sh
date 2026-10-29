#!/usr/bin/env bash
# 通过 USB 归属发现 MaixCAM Lite，再启动主机端预览和传统灯条检测。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 -B "$project_dir/tools/maixcam/run_light_test.py" "$@"

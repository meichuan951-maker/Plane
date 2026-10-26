#!/usr/bin/env bash
# 运行已构建的实时测量程序；厂商 OpenNI 与系统 libusb 优先，避免同名库冲突。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="${ASTRA_OPENNI2_ROOT:-$project_dir/../vio_std/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63}"
if [[ ! -x "$project_dir/build/astra_live_measure" ]]; then
  echo "请先构建：$project_dir/scripts/build.sh -- -DAIM_WITH_ASTRA=ON" >&2
  exit 1
fi
export LD_LIBRARY_PATH="$sdk_dir/Samples/Bin:/usr/local/lib:/lib/x86_64-linux-gnu"
# Snap IDE 的 schema 与系统 GTK 不匹配；仅修正本进程的搜索路径。
if [[ "${GSETTINGS_SCHEMA_DIR:-}" == */snap/* && -f /usr/share/glib-2.0/schemas/gschemas.compiled ]]; then
  export GSETTINGS_SCHEMA_DIR=/usr/share/glib-2.0/schemas
fi
exec "$project_dir/build/astra_live_measure" "$@"

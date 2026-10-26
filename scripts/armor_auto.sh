#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$project_dir/build/armor_auto" ]]; then
  echo "请先构建：$project_dir/scripts/build.sh" >&2
  exit 1
fi
export LD_LIBRARY_PATH=/usr/local/lib:/lib/x86_64-linux-gnu
if [[ "${GSETTINGS_SCHEMA_DIR:-}" == */snap/* && -f /usr/share/glib-2.0/schemas/gschemas.compiled ]]; then
  export GSETTINGS_SCHEMA_DIR=/usr/share/glib-2.0/schemas
fi
cd "$project_dir"
exec "$project_dir/build/armor_auto" "$@"

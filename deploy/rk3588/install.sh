#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'HELP'
在 RK3588 / Ubuntu 22.04 上安装开机服务；只 enable，不立即启动。
用法：sudo bash deploy/rk3588/install.sh \
  --project-dir /absolute/path/to/no_demo \
  --config /absolute/path/to/yaw_follow.json \
  [--user user] [--python /absolute/path/to/python3]

路径仅接受 /、英文字母、数字、下划线、点、加号和连字符。
配置检查以目标用户执行，不连接相机或飞控。
HELP
}

fail() {
  printf '安装失败：%s\n' "$*" >&2
  exit 1
}

safe_absolute_path() {
  [[ "$1" =~ ^/[A-Za-z0-9_./+-]+$ ]] ||
    fail "路径必须为绝对路径，且不能含空格、%、\$、换行等特殊字符：$1"
}

project_dir=''
config_path=''
run_user='user'
python_path='/usr/bin/python3'
while (($#)); do
  case "$1" in
    --project-dir|--config|--user|--python)
      (($# >= 2)) || fail "$1 缺少参数"
      case "$1" in
        --project-dir) project_dir="$2" ;;
        --config) config_path="$2" ;;
        --user) run_user="$2" ;;
        --python) python_path="$2" ;;
      esac
      shift 2
      ;;
    --help|-h) usage; exit 0 ;;
    *) fail "未知参数：$1" ;;
  esac
done

[[ -n "$project_dir" && -n "$config_path" ]] || { usage >&2; exit 1; }
safe_absolute_path "$project_dir"
safe_absolute_path "$config_path"
safe_absolute_path "$python_path"
[[ "$run_user" =~ ^[a-z_][a-z0-9_-]*$ ]] || fail '用户名含不支持的字符'
[[ "$EUID" -eq 0 ]] || fail '请在目标板上使用 sudo 执行安装'
[[ -d /run/systemd/system ]] || fail '当前系统不是正在运行的 systemd 环境'

for command_name in realpath dirname basename getent id runuser install mktemp systemctl; do
  command -v "$command_name" >/dev/null || fail "缺少命令：$command_name"
done
id "$run_user" >/dev/null 2>&1 || fail "用户不存在：$run_user"
for group_name in video dialout; do
  getent group "$group_name" >/dev/null || fail "系统组不存在：$group_name"
done
[[ -d "$project_dir" ]] || fail "项目目录不存在：$project_dir"
[[ -f "$config_path" ]] || fail "配置文件不存在：$config_path"
[[ -x "$python_path" && ! -d "$python_path" ]] || fail "Python 不可执行：$python_path"
[[ -x /usr/bin/python3 ]] || fail '缺少 /usr/bin/python3（用于渲染服务模板）'

project_dir="$(realpath -e -- "$project_dir")"
config_path="$(realpath -e -- "$config_path")"
# 保留 python 最后一层符号链接，避免将虚拟环境解释器解析成系统解释器。
python_dir="$(realpath -e -- "$(dirname -- "$python_path")")"
python_path="$python_dir/$(basename -- "$python_path")"
safe_absolute_path "$project_dir"
safe_absolute_path "$config_path"
safe_absolute_path "$python_path"
[[ -f "$project_dir/px4_go/yaw_follow.py" ]] || fail '项目中没有 px4_go/yaw_follow.py'

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
template_path="$script_dir/plane-yaw-follow.service.template"
[[ -f "$template_path" ]] || fail "服务模板不存在：$template_path"

printf '以用户 %s 检查配置（不连接硬件）……\n' "$run_user"
(
  cd -- "$project_dir"
  runuser -u "$run_user" -- /usr/bin/env -u PYTHONPATH -u PYTHONHOME \
    "$python_path" - "$config_path" "$project_dir" <<'PY'
import json
from pathlib import Path
import sys

try:
    with Path(sys.argv[1]).open(encoding="utf-8") as stream:
        data = json.load(stream)
    declared = data.get("project_dir") if isinstance(data, dict) else None
    if not isinstance(declared, str) or not Path(declared).is_absolute():
        raise ValueError("JSON project_dir 必须为绝对路径")
    if Path(declared).resolve(strict=True) != Path(sys.argv[2]).resolve(strict=True):
        raise ValueError("--project-dir 与 JSON project_dir 规范化后不一致")
except (OSError, ValueError) as error:
    raise SystemExit(f"安装失败：{error}") from error
PY
  runuser -u "$run_user" -- /usr/bin/env -u PYTHONPATH -u PYTHONHOME \
    "$python_path" -u -m px4_go.yaw_follow --config "$config_path" --check-config
)

rendered_path="$(mktemp /tmp/plane-yaw-follow.XXXXXX)"
trap 'rm -f -- "$rendered_path"' EXIT
/usr/bin/python3 - "$template_path" "$rendered_path" "$project_dir" \
  "$config_path" "$run_user" "$python_path" <<'PY'
from pathlib import Path
import sys

template, output, project, config, user, python = sys.argv[1:]
text = Path(template).read_text(encoding="utf-8")
for marker, value in {
    "@PROJECT_DIR@": project,
    "@CONFIG_PATH@": config,
    "@RUN_USER@": user,
    "@PYTHON_PATH@": python,
}.items():
    if marker not in text:
        raise SystemExit(f"服务模板缺少占位符：{marker}")
    text = text.replace(marker, value)
if "@" in text:
    raise SystemExit("服务模板仍含未替换占位符")
Path(output).write_text(text, encoding="utf-8")
PY

unit_path='/etc/systemd/system/plane-yaw-follow.service'
install -o root -g root -m 0644 -- "$rendered_path" "$unit_path"
systemctl daemon-reload
systemctl enable plane-yaw-follow.service
printf '\n已安装并设为开机启动，本脚本没有启动或重启服务。\n'
printf '手动执行任务：sudo systemctl start plane-yaw-follow.service\n'
printf '或在目标板下一次重启时执行一次任务。\n'
printf '查看日志：journalctl -u plane-yaw-follow.service -f\n'

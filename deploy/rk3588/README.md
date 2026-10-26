# RK3588 上电自动启动视觉与 PX4 偏航跟随

适用目标：RK3588、Ubuntu 22.04、用户 `user`，无显示器、无需登录桌面。`systemd` 启动 `python -m px4_go.yaw_follow`，该进程管理无窗口 `armor_auto` 和唯一的 MAVLink 串口连接。

运行器沿用 `gogogo.py` 的预发送 setpoint、OFFBOARD、自动解锁握手方式。原稿保留，本服务运行时不要再运行 `gogogo.py` 或另起 `px4_localization_cpp` 占用同一串口。这里的 PX4 是独立飞控，板端无需启动 PX4 固件进程。

本任务在就绪后自动解锁起飞，保持起飞时的本地 XY 和配置高度，仅根据目标在图像中的水平误差调整偏航；不向目标平移、不执行接触动作。没有相机内参时使用灯条/装甲候选角点的像素水平误差，不报告目标距离或视线角度。`armor_auto` 的米制 `valid=false` 与像素跟踪可用性需要分别判断，运行器使用检测、跟踪及帧时效字段。

自身位置来自 PX4 `LOCAL_POSITION_NED`，并用 `ESTIMATOR_STATUS` 门控有效性。本入口不实现 TIMESYNC、外部定位融合或场地世界坐标对齐；PX4 的自身定位必须已经正常工作。飞控需要提供 `HEARTBEAT`、20 Hz 的 `LOCAL_POSITION_NED`、10 Hz 的 `ESTIMATOR_STATUS` 和 2 Hz 的 `EXTENDED_SYS_STATE`。运行器会请求这些消息率，仍须核验实际输出。相机、串口等在 `ready_timeout_s` 内未就绪时，本次进程退出，服务不会自动重试。

## 先填写目标板配置

起飞高度、失目标策略/等待时间和任务时长由用户在部署时自行填写；相机安装朝向、镜像与偏航方向也通过配置核验。示例保留路径占位符、`null` 和未确认标志，未填完整或安装方向未确认时，`--check-config` 会拒绝，运行器禁止起飞。

在目标板的 `no_demo` 目录复制配置：

```bash
cp deploy/rk3588/yaw_follow.example.json deploy/rk3588/yaw_follow.json
cp deploy/rk3588/armor_ylr4k.example.yaml deploy/rk3588/armor_ylr4k.yaml
```

编辑 JSON：

- `project_dir`：目标板项目绝对路径，替换 `/home/user/REPLACE_ME/no_demo`。
- `vision_config`：改为 `deploy/rk3588/armor_ylr4k.yaml`。
- `camera_device`、`connection`：填写目标板核验过的相机和飞控稳定路径，例如各自的 `/dev/v4l/by-id/…`、`/dev/serial/by-id/…`；串口波特率与飞控保持一致。
- `flight_height_m`：填写相对本次起飞位置的高度，单位米。
- `target_loss_action`：自行选择 `"land"` 或 `"hold"`。`land` 在目标丢失时立即停止偏航、保持 XY 和高度，等待配置的超时后降落；`hold` 立即将偏航角速度归零并保持 XY 和高度，等待目标恢复后继续跟随，仍受任务总时长限制。
- `target_loss_timeout_s`：`land` 策略下目标持续丢失到降落的等待秒数。`hold` 不按此时限降落，但该字段仍需填入配置允许的数值。
- `mission_duration_s`：填写从确认 ARM 成功开始计时的最长持续时间，包含爬升阶段，单位秒；到时请求 LAND。
- `camera_forward_unmirrored_confirmed`：实测相机朝前、画面未镜像且目标左右方向正确后，改为 `true`。这项核验用于确定图像误差与偏航方向的关系。

`auto_arm` 已按自动起飞要求设为 `true`。`max_yaw_rate_rad_s` 限制偏航角速度，`yaw_gain_rad_s` 为水平归一化误差的增益，`deadband_normalized` 为居中死区；`yaw_direction` 控制误差符号。初始数值是配置起点，需要板端验证。

编辑相机 YAML，使 `camera.device`、`width`、`height` 与 JSON 一致。示例为 640×480、30 FPS、MJPG；先核验 YLR4K 实际支持的模式，必要时改为 YUYV。颜色 `1` 是蓝色，`2` 是红色。灯条阈值使用 `../../config/light_bars.yaml`，也需按这台相机的曝光与真实画面调整。

`rgb_calibration` 指向有意未提供的 `ylr4k_rgb_calibration.yaml`，`dimensions_confirmed` 保持 `false`，用于明确本次没有米制 PnP。不要借用其他相机的标定或将示例装甲尺寸视为现场实测值。

## 构建和离线检查

在 RK3588 上构建 ARM64 可执行文件，不能复制开发电脑上的 x86_64 程序：

```bash
./scripts/build.sh -- -DAIM_WITH_REALSENSE=OFF -DAIM_WITH_ROS2=OFF -DAIM_WITH_MAVLINK=OFF
/usr/bin/python3 -c 'import pymavlink, yaml'
/usr/bin/python3 -m px4_go.yaw_follow --config /实际绝对路径/no_demo/deploy/rk3588/yaw_follow.json --check-config
```

Python 环境需要 `pymavlink` 和 `PyYAML`。`--check-config` 只读取并检查 JSON、相机 YAML 及两者一致性，不打开相机、不连接飞控、不运行视觉；通过不代表设备、定位或飞行已经验收。运行器直接启动 `build/armor_auto`，不使用含 x86_64 库路径的旧视觉启动脚本。

运行不访问设备的自动化检查：

```bash
python3 -m unittest discover -s test/io -p 'test_yaw_*.py' -v
bash -n deploy/rk3588/install.sh
```

## 安装开机服务

仅在目标板执行，先将下列路径替换为实际路径：

```bash
sudo bash deploy/rk3588/install.sh \
  --project-dir /实际绝对路径/no_demo \
  --config /实际绝对路径/no_demo/deploy/rk3588/yaw_follow.json \
  --user user
```

安装器仅接受由 `/`、英文字母、数字、下划线、点、加号及连字符组成的绝对路径，避免 shell 和 systemd 特殊字符改变命令含义。`--project-dir` 必须与 JSON 内的 `project_dir` 在解析符号链接和规范化后指向同一目录。它先以目标用户检查目录一致性及运行配置，再安装 `/etc/systemd/system/plane-yaw-follow.service`、执行 `daemon-reload` 和 `enable`，不会立即启动或重启服务。

推荐在目标板准备虚拟环境，并在检查与安装时选择同一个解释器；不依赖 `.bashrc`、`activate` 或桌面终端环境。将下面路径替换为实际目录：

```bash
/usr/bin/python3 -m venv /home/user/plane/venv
/home/user/plane/venv/bin/python3 -m pip install pymavlink PyYAML
/home/user/plane/venv/bin/python3 -m px4_go.yaw_follow \
  --config /home/user/plane/no_demo/deploy/rk3588/yaw_follow.json --check-config
```

安装时指定同一解释器：

```bash
sudo bash deploy/rk3588/install.sh \
  --project-dir /home/user/plane/no_demo \
  --config /home/user/plane/no_demo/deploy/rk3588/yaw_follow.json \
  --user user \
  --python /home/user/plane/venv/bin/python3
```

系统必须存在 `video`、`dialout` 组。服务通过 `SupplementaryGroups` 授予相机与串口访问组；目标用户还需要读取项目、配置及依赖的权限。

## 执行、停止与日志

安装后，选择在目标板手动开始本次任务，或在下一次上电启动：

```bash
sudo systemctl start plane-yaw-follow.service
systemctl status plane-yaw-follow.service
journalctl -u plane-yaw-follow.service -f
```

每次开机只执行一次任务。服务配置 `Restart=no`，任务完成或异常退出后不会自动重新起飞；再次手动 `start` 或下一次系统启动会创建新任务。

```bash
sudo systemctl stop plane-yaw-follow.service
sudo systemctl disable plane-yaw-follow.service
```

`stop` 向主进程发送 SIGTERM，仍由本任务控制且已请求解锁时，运行器请求降落，并保留视觉子进程到退出清理阶段；若遥控器或 PX4 已切出 OFFBOARD，则保留接管后的模式。`KillMode=mixed` 避免停止服务时先把视觉子进程一起终止；60 秒停止期限到达后 systemd 会清理剩余进程。`landing_timeout_s` 最多为 50 秒，低于服务停止期限，默认 45 秒。`disable` 只取消后续开机启动，要同时停止当前任务需先执行 `stop`。

标准输出与错误写入 journal，不向无界增长的 JSONL 文件持续追加。如果 journald 已持久化并保留跨启动日志，可用 `journalctl -u plane-yaw-follow.service -b -1` 查看上次启动记录。

服务选项依据 [systemd 249 的 systemd.service 手册](https://www.freedesktop.org/software/systemd/man/249/systemd.service.html)及 [systemd.kill 手册](https://www.freedesktop.org/software/systemd/man/249/systemd.kill.html)。OFFBOARD 的预发送与持续 setpoint 要求见 [PX4 官方 Offboard 模式说明](https://docs.px4.io/main/en/flight_modes/offboard)，需按飞控实际固件版本核对。

# RMUC 2027 微型机与 no_demo 的仿真入口

当前唯一现用模型为同级地图工程的 `rmuc_micro_drone`：163 g、整体130 mm外包络。旧X500尺寸检查机及其仿真专用入口已归档，见[归档说明](../../rmuc_gazebo_map/archive/legacy_aircraft_20261019/README.md)。本次整理不修改真实飞控的串口或台架通信功能。

## 启动

在 `no_demo` 根目录运行：

```bash
./scripts/rmuc2027_sim.sh
./scripts/rmuc2027_sim.sh keyboard --team blue
./scripts/rmuc2027_sim.sh route --team both
./scripts/rmuc2027_sim.sh check
```

默认菜单选择1为键盘、2为航线。两种模式使用同一模型，先关闭当前窗口再切换；统一入口的文件锁防止重复打开完整地图。
键盘按R/F起降、WASD移动、Q/E转向，先垂直升高100 mm后横移。松键减速，空格/X急停，1/2/3为0.2/0.6/1.0 m/s档位，C瞬时复位。
航线自动执行100 mm起飞、基地侧退让、中央机关侧绕、顶部接触、安全退出、实测位置返程重规划和原位物理降落。正常或接触失败后均返航；10 N持续3秒是外力仿真参数。

```bash
./scripts/rmuc2027_sim.sh route --team both --headless --case contact-timeout
./scripts/rmuc2027_sim.sh keyboard --team red --check
./scripts/rmuc2027_sim.sh sim --mode route --team blue --start-paused
```

`check`不启动Gazebo或绑定端口。默认地图路径为 `../rmuc_gazebo_map`，可通过 `RMUC_MAP_ROOT`覆盖。

## 模型、地图和输出

地图保留完整2027原CAD碰撞。红蓝方保留原机库XY与朝向，停车面z=1.901 m、机体参考原点z=1.966 m；起飞100 mm后低空巡航z=2.066 m，仅在前哨附近调整高度。
微型机通过合并显示网格把208个显示对象减少为12个，仍保留208个碰撞体、原质量惯量及独立七颗阵营灯。两种模式加载各自独立控制器，不混用控制来源。

地图工程的 `config/inspection_aircraft.yaml`定义模型，`config/outpost_contact.yaml`定义接触返航参数。修改模型后运行 `python3 tools/generate_micro_aircraft.py`重新生成网格和SDF。
常规输出位于地图工程 `generated/simulation_modes/`；键盘包含mode.json、mode_session.json和日志，航线包含双方任务、轨迹、接触测力、动态返程及落地报告。

## 接口范围

本入口不加载旧X500仿真，不启动PX4/ROS，不连接真实飞控，也不把no_demo的视觉观测自动变成导航目标。
键盘使用仿真速度控制，航线平移使用运动学回放，接触使用物理求解与外力；这些结果不构成真机闭环或实际旋翼接触能力验收。
历史算法与几何回归继续保留，归档资源不进入现用模型搜索路径或安装清单。
更多构建和测试指令见[地图工程README](../../rmuc_gazebo_map/README_zh-CN.md)，路线、阶段和测力含义见[接触实验说明](../../rmuc_gazebo_map/docs/OUTPOST_CONTACT.md)。

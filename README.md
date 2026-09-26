# 无人机红蓝装甲板识别 Demo

项目目录：`/home/user/nohuman_plane_std/no_demo`。机载平台为 **Orange Pi 5 8GB + Ubuntu 22.04**，开发语言为 C++17。

## 当前方案与实现状态

最新技术路线见 [CURRENT_PLAN.md](CURRENT_PLAN.md)。阅读顺序：当前方案 → 已有代码说明 → 数据训练指南；旧方案文档保留作参考。

本轮已确认改为双板分工：

- **PX4 飞控板**：负责姿态控制、电机输出、遥控接管和飞控侧失效保护；具体板型、固件版本待定。
- **Orange Pi 5**：通过普通 USB 彩色相机采集图像，运行 YOLOv8 红蓝识别，输出目标观测并接收 PX4 状态。
- **板间通信**：MAVLink 串口，C++ 使用 MAVLink 头文件并自行管理串口；计划波特率 921600，设备路径与参数配置化。
- **本轮范围**：观测接口与 PX4 状态接收；不发送解锁、起飞、速度、姿态、电机或其他飞行控制指令。
- **距离**：先保留距离估计接口；没有实现并验证测距前，不能输出伪造的目标距离或三维位置。

以上是重构目标，**普通 USB 相机采集和 MAVLink 收发尚未实现**。现有代码仍是 D435i RGB-D、合成输入和 PNG 回放框架。D435i 保留为既有设备与旧后端，不再是新方案的必需相机。

MicoVIO 仍是候选自身定位模块，未接入；它不代替前向目标识别或目标距离估计。自定义 JSON 观测封装消息的具体方言与接收方式仍待确定，标准 PX4 固件不会自动解析自定义消息。

## 文档与目录入口

- [当前双板方案与重构路线](CURRENT_PLAN.md)：当前决策、待确认项、实施顺序和边界。
- [已有 C++ 框架使用说明](CPP_FRAMEWORK.md)：现有 D435i、回放和合成模式的真实用法；其中旧硬件路线以当前方案为准。
- [红蓝双类别训练指南](YOLO_RED_BLUE_TRAINING.md)：固定 `0=target_red`、`1=target_blue`；末尾 D435i 启动示例属于现有后端。
- [设计资料导航](drone_contact_demo/README.md)：预算、坐标、任务设计与旧验收资料。
- `src/`、`include/`、`test/`：C++ 实现、公共接口与回归测试。
- `config/`：现有视觉阈值与外参；尚无已实现的串口配置文件。
- `picture/`：目标参考图片；`test_1/`：MicoVIO 产品参考资料。
- `models/`：模型；`examples/`：回放示例；`runs/`：运行输出。

## 当前可以运行的验证命令

以下命令验证现有框架，不启动普通 USB 相机或飞控通信：

```bash
cd /home/user/nohuman_plane_std/no_demo
cmake -S . -B build_cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build_cpp -j2
ctest --test-dir build_cpp --output-on-failure
./build_cpp/demo_aim_cpp --source synthetic --frames 60 --output runs/cpp_01.jsonl
```

合成输入使用注入检测框，不运行 YOLO。日志重名会自动编号。现有 `valid` 表示通过检查的三维观测，不能在单目重构中静默改成“有检测框”；二维有效性和三维有效性需要分别定义。

## 部署和项目边界

所有机载识别在香橙派运行，地面电脑用于训练、编译、回放与日志分析。USB 传输不会产生“开发板坐标系”。当前推理后端为 OpenCV CPU，未接入 RKNN/NPU；板端相机兼容性、供电、散热及延迟尚待实测。

飞控保留独立控制能力不代表视觉失效后一定悬停；具体行为取决于估计器状态、飞控配置和可用传感器，必须验证。

`../rmuc_gazebo_map` 是独立地图工程，本轮不修改；项目也尚未实现自动接近、接触和返航。长期目标与预算参考 [设计资料](drone_contact_demo/README.md)。

## 早期尺寸要求记录

早期硬件要求：整机重量 150～249 g，最大伸展尺寸与重量关系为 `y = −0.001225x + 0.3225`，其中 y 为 kg、x 为 mm，长宽高各不超过 x。首版原型允许暂时放宽该重量/尺寸约束；不能因此宣称已满足比赛版要求。原始规则 PDF 保留在项目根目录。
双板架构的必做修改项见 [README_REFACTOR_REQUIRED.md](README_REFACTOR_REQUIRED.md)。

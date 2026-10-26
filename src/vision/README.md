# vision：视觉算法与观测数据

本目录实现图像检测、目标关联、标定和测距算法，供应用入口与离线测试工具复用。
这里处理“看到了什么、测量是否有效”，相机采集由 [`../input/`](../input/README.md) 负责，
窗口、命令行和结果落盘由 [`../app/`](../app/README.md) 负责；本目录不发送飞控指令。

## 主要文件

- [`light_bars.cpp`](light_bars.cpp)：从 BGR 图像提取红蓝灯条；
  [`light_bar_config.cpp`](light_bar_config.cpp) 读取并校验阈值配置，
  [`light_bar_overlay.cpp`](light_bar_overlay.cpp) 绘制检测结果。
- [`armor_geometry.cpp`](armor_geometry.cpp)：根据灯条几何关系配对装甲候选，处理候选间的灯条冲突。
  [`armor_pose.cpp`](armor_pose.cpp) 使用装甲角点、相机内参和实际尺寸进行 PnP，检查重投影质量及姿态歧义。
- [`armor_tracker.cpp`](armor_tracker.cpp)：按颜色选择候选、连续确认、保持临时目标关联，处理丢失与重新获取；
  [`armor_auto_config.cpp`](armor_auto_config.cpp) 管理自动装甲流程的配置。
- [`camera_calibration.cpp`](camera_calibration.cpp)：根据采集到的棋盘格角点求解 RGB 内参，
  检查画面覆盖、视角变化、标定误差和留出视图误差。角点采集和文件写入由应用层负责。
- [`light_bar_observation.cpp`](light_bar_observation.cpp)：`detectArmorObservations` 复用灯条与装甲配对，
  将真实灯条/角点、颜色类别和几何评分转换为公共 `Detection`，供 RealSense 观测入口复用。
- [`core.cpp`](core.cpp)：既有 `Pipeline`，接收图像与检测框，完成时效检查、目标关联、连续确认、
  视线计算和中心 ROI 深度质量检查，生成 `Solution`。
- [`observation.cpp`](observation.cpp)：公共观测类型的内参校验、反投影、时效判断及 schema=2 JSON 编码。
- [`depth_plane.cpp`](depth_plane.cpp)：从米制 XYZ 图的 ROI 拟合局部平面，计算中心、法线、距离和角度；
  算法独立于具体深度相机 SDK。

## 两条视觉链路

当前普通 USB 相机流程是：

```text
src/input 的 UVC 采集
  → src/app/camera 或 src/app/armor_auto
  → 灯条检测 → 装甲配对 → 跟踪 / 可选 PnP
  → 应用预览、截图和测量 JSON
```

`camera_light_test` 用于画面和阈值验证；`armor_auto` 增加目标选择、连续跟踪与测量输出，
其标定入口调用本目录的棋盘格算法。PnP 数值需要当前相机的有效标定和装甲实际尺寸。

既有 `demo_aim_cpp` 流程是：

```text
合成输入提供图像与注入检测框，或 RealSense 图像经 detectArmorObservations 转换为 Detection
  → core.cpp 的 Pipeline → observation.cpp 的 Solution / JSON
```

这两条链路复用传统检测算法，由不同应用编排；普通 UVC 采集尚未直接接入 `demo_aim_cpp`。
`demo_aim_cpp --detector-config` 默认使用 `config/light_bars.yaml`，不经过 `ArmorTracker` 的应用层测量输出。
`Pipeline` 的三维位置来自对齐深度 ROI，传统装甲链路的三维位置来自 PnP，不能混用两者的有效性条件。

## 与其他目录的联系

- [`../../include/demo_aim/`](../../include/demo_aim/)：对应公共接口，包括 `light_bars.hpp`、`light_bar_observation.hpp`、
  `armor_auto.hpp`、`camera_calibration.hpp`、`core.hpp`、`observation.hpp` 和 `depth_plane.hpp`。
- [`../../config/`](../../config/)：`light_bars.yaml`、`armor_auto.yaml` 服务于传统检测和自动装甲流程；
  `aim.yaml` 配置既有观测流水线。标定路径由配置指定，仓库没有可直接代替当前相机实测的标定结果。
- [`../io/`](../io/README.md) 与 [`../navigation/`](../navigation/README.md)：遥测和定位在各自模块维护；
  导航通过公共观测、几何与定位接口组合视觉结果。真实应用尚未把普通 UVC 观测、定位与任务执行连成闭环。
- [`../../tools/`](../../tools/README.md)：`vision_memory_probe.cpp` 和 `vision_video_worker.cpp`
  复用传统检测与跟踪实现测量资源开销；静态图可按配置启用 PnP，未知内参的录像禁用 PnP。
- [`../../test/vision/`](../../test/vision/)：灯条、装甲跟踪、标定/PnP、深度平面等回归；
  [`../../test/core/test_aim.cpp`](../../test/core/test_aim.cpp) 覆盖观测流水线与合成输入。

库的拆分和依赖以 [`../../CMakeLists.txt`](../../CMakeLists.txt) 为准：
`aim_light_bars`、`aim_armor_auto`、`aim_core`、`aim_observation` 和 `aim_depth_plane`
分别承载上述职责。坐标、时间与字段约定见 [`../../docs/interfaces.md`](../../docs/interfaces.md)，
运行入口见[项目 README](../../README.md)。

# Astra Pro 专用程序归档

归档日期：2026-10-23（Asia/Shanghai）。当前相机改用 YLR4K，本目录保存退出当前构建的 Astra Pro 专用采集、深度交互、诊断工具、启动脚本及配置。

专用文件按项目原路径移动；构建集成文件、切换前的默认配置及旧安装配置另存复制快照。`manifest.json` 的 `action` 区分 `moved` 与 `snapshot`，并记录原路径、归档路径、SHA-256、大小、权限和修改时间；归档副本均已逐项核对，内容和权限保持原样。

`CMakeLists.txt`、`test/CMakeLists.txt`、`scripts/build.sh` 和 `cmake/ArchiveDisabledTargets.cmake` 保存切换前的构建接线，`config/armor_auto.yaml` 保存旧默认配置；`installed_snapshot/` 保存退出主线的旧安装配置。历史启动说明和实验记录见 [USAGE_legacy.md](USAGE_legacy.md)。

这些文件不属于当前构建入口，`COLCON_IGNORE` 阻止 colcon 发现归档中的旧构建文件。此目录不能独立构建，恢复时需结合构建快照和主工程的共享代码。归档脚本基于原目录布局定位工程、SDK 和可执行文件，不能直接作为当前启动入口使用。

原构建目录中的旧可执行文件保留在 `build/disabled/<SHA256>/`，不会由当前目标重新生成。

通用无硬件功能仍保留在活动源码中：`src/app/depth/output.cpp`、`include/demo_aim/detail/app/depth/output.hpp`、`src/vision/depth_plane.cpp`、`include/demo_aim/depth_plane.hpp` 及相关离线测试。通用 UVC、装甲识别和相机标定流程也继续保留。工作区外的 `../vio_std` SDK 未移动或修改。

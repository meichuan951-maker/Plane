# MaixCAM 实验代码归档

归档日期：2026-10-25（Asia/Shanghai）。按要求从当前开发入口移除 MaixCAM，保留代码、文档和测试以便查阅或恢复。

文件按原项目相对路径保存：

- `tools/maixcam/preview_maixvision.py`：MaixVision 预览、FPS、手动曝光、传统灯条与装甲候选检测。
- `tools/maixcam/enable_uvc.py`、`stream_uvc.py`：设备端 UVC 配置与推流。
- `tools/maixcam/run_light_test.py`：电脑端 UVC 查找和 C++ 识别启动器。
- [实验用法与排查记录](tools/maixcam/README.md)：参数、历史实测、设备端和电脑端运行方式。
- `test/input/test_maixcam_*.py`：三组无硬件回归。
- `integration/test_CMakeLists.txt`：移除 MaixCAM 测试登记前的构建接线快照，仅供恢复参考。
- `runs/maixcam_check_20261025_wut2lil3/`：清理主工程 `runs/` 时一并移入的原图、叠加图和采集日志；这些本机生成记录仍受 `runs/` 忽略规则约束，不随 Git 提交。

`manifest.json` 记录迁移前与归档后文件校验值。脚本算法及测试内容保留；仅调整电脑端启动器的工程根目录定位，以及说明文档中的运行路径。Python 缓存不归档。

主工程不加载此目录，不再登记 MaixCAM 专用 CTest。`COLCON_IGNORE` 阻止 colcon 扫描归档。普通 USB/UVC 的 C++ 入口仍由主工程提供；归档启动器依赖主工程的 `build/camera_light_test` 和 `config/light_bars.yaml`，本目录不是独立 C++ 工程，也不创建额外 build 文件夹。

从主工程根目录手动检查归档代码：

```bash
python3 -B -m unittest discover -s archive/maixcam_20261025/test/input -p 'test_maixcam_*.py'
```

恢复开发时，将 `tools/maixcam/` 和三份测试移回原路径，把 `run_light_test.py` 的 `ROOT` 改回 `Path(__file__).resolve().parents[2]`，再参考快照恢复对应三项测试登记；不要用整个旧快照覆盖当前 `test/CMakeLists.txt`。

归档时，上一版全画面识别实测约 9 FPS。后续中央 ROI、曝光和小灯条调整仅完成离线回归，真实识别效果和设备帧率尚未验收。此操作只整理本地工程，没有卸载 MaixVision 或修改相机固件、USB 配置。

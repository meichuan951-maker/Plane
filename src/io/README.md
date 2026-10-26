# io：飞控遥测、MAVLink 与串口

本目录负责接收与校验飞控数据，向上层提供带时效信息的遥测快照和自身定位。当前定位链路为 MicoVIO → PX4 融合 → 本机；这里不直接解码 MicoVIO 私有协议，也不向 PX4 回灌融合后的位姿。

## 主要文件

- [flight_state.cpp](flight_state.cpp)：实现 `FlightStateCache`，按指定 system/component 缓存心跳、姿态和本地位置，拒绝重复或乱序更新，分别处理心跳和状态超时，并生成 JSON 快照。接口见 [flight_state.hpp](../../include/demo_aim/flight_state.hpp)。它属于默认构建的 `aim_flight_state`，不依赖 MAVLink 生成头文件。
- [mavlink_link.cpp](mavlink_link.cpp)：实现 `MavlinkLink`，解析分片或连续 MAVLink 报文，检查来源、PX4 心跳、估计器状态、时钟同步、位姿协方差和坐标约定，将有效融合 `ODOMETRY` 转为 `PoseSample`。同步跳变、定位复位和参考系变化会使旧会话或地图对齐失效。接口见 [mavlink_link.hpp](../../include/demo_aim/mavlink_link.hpp)。
- [serial_port.cpp](serial_port.cpp)：Linux UART 打开、raw 模式配置、非阻塞读写、等待超时和断连处理，支持 115200、460800、921600 波特率，单次数据量有界。接口见 [serial_port.hpp](../../include/demo_aim/serial_port.hpp)。

`mavlink_link.cpp` 与 `serial_port.cpp` 仅在 `AIM_WITH_MAVLINK=ON` 时编译为 `aim_mavlink`；构建需要外部提供包含 `common/mavlink.h` 的官方 MAVLink v2 生成头文件目录。具体依赖见 [根 CMakeLists.txt](../../CMakeLists.txt)。

## 与其他目录的联系

- [src/app/localization_main.cpp](../app/localization_main.cpp) 是 `px4_localization_cpp` 入口：读取 YAML、打开串口、周期发送 TIMESYNC、将收到的字节交给 `MavlinkLink`，输出链路诊断、遥测快照、当前定位和历史大小。
- [config/px4_link.yaml](../../config/px4_link.yaml) 提供运行链路参数；[config/px4_usb_bench.yaml](../../config/px4_usb_bench.yaml) 是现有 USB 台架诊断配置。两者默认都未确认 `px4_odometry_contract_confirmed`，因此收到报文不代表能输出有效导航位姿。配置加载属于应用层，不在本目录内部完成。
- [src/navigation](../navigation/README.md) 提供几何变换和 `PoseSample`。MAVLink 层把支持的 PX4 父坐标和机体 FRD 转成工程使用的右手坐标表达，随后由导航层维护历史、插值和地图对齐；串口连接不会自动建立场地原点。
- [src/vision](../vision/README.md) 与本目录共享 [observation.hpp](../../include/demo_aim/observation.hpp) 的基础类型和 JSON 定义。构建关系中 `aim_core` 保留对 `aim_flight_state` 的公共依赖，但当前 `demo_aim_cpp` 视觉入口没有自动打开 PX4 串口或完成视觉与真实定位的联合任务。
- [test/core/test_aim.cpp](../../test/core/test_aim.cpp) 验证遥测缓存；[test/io/test_mavlink.cpp](../../test/io/test_mavlink.cpp) 使用编码报文和伪终端验证同步、定位拒绝条件、复位及串口行为，后者随 MAVLink 功能开关启用。测试登记见 [test/CMakeLists.txt](../../test/CMakeLists.txt)。
- [px4_go](../../px4_go/README.md) 是独立 Python 控制与离线联测模块，并不调用本目录的 C++ 接收器。其视觉偏航入口自行管理视觉进程和 MAVLink；这里的定位诊断结果尚未成为该控制入口的共享输入。

## 当前边界与阅读顺序

C++ 链路唯一生成并发送的消息是 `TIMESYNC` 探针，不发送解锁、飞行目标、参数或消息率命令。心跳、`ATTITUDE_QUATERNION`、`LOCAL_POSITION_NED` 用于状态诊断，合格的融合 `ODOMETRY` 才进入定位输出；报文可解析、链路在线和定位有效是不同条件。重启、重连或 `reset_required` 后不能沿用旧地图对齐。

建议先读 [flight_state.hpp](../../include/demo_aim/flight_state.hpp) 和 [mavlink_link.hpp](../../include/demo_aim/mavlink_link.hpp)，再看本目录实现、[localization_main.cpp](../app/localization_main.cpp) 及通信回归。构建与台架运行方式见 [根 README](../../README.md)，完整坐标和时间约定见 [docs/interfaces.md](../../docs/interfaces.md)。

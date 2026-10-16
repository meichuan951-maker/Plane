# 构建入口：生成可执行程序、分层静态库和 CTest 测试；运行命令见 README.md。
cmake_minimum_required(VERSION 3.16)
project(no_demo_aim_cpp VERSION 0.2.0 LANGUAGES CXX)
# 统一要求 C++17，使用 optional/filesystem 等标准库功能。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 相机 SDK 可选；缺失时仍能构建合成与回放模式。
option(AIM_WITH_REALSENSE "Build D435i live input when SDK is available" ON)
# ROS 发布默认关闭；开启前需要加载对应 ROS 环境。
option(AIM_WITH_ROS2 "Build ROS 2 diagnostic JSON publisher" OFF)
# 查找构建所需库；实际链接依赖由各 target 分别声明。
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio dnn calib3d)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

# 数据契约和遥测缓存独立构建，导航不再依赖视觉流水线的实现。
add_library(aim_observation src/vision/observation.cpp)
target_include_directories(aim_observation PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aim_observation PUBLIC opencv_core nlohmann_json::nlohmann_json)
add_library(aim_flight_state src/io/flight_state.cpp)
target_link_libraries(aim_flight_state PUBLIC aim_observation)

# 保留 aim_core 的公共依赖，兼容现有视觉调用者。
add_library(aim_core src/vision/core.cpp)
target_link_libraries(aim_core PUBLIC aim_observation aim_flight_state
                               PRIVATE opencv_imgproc yaml-cpp)

# 检测库封装 DNN 与模型文件哈希，向应用输出公共 Detection 类型。
add_library(aim_detector src/vision/yolo.cpp)
target_link_libraries(aim_detector PUBLIC aim_core opencv_dnn
                                   PRIVATE opencv_imgproc OpenSSL::Crypto)

# 输入库把合成、回放、相机拆开实现，通过共同 Frame/Sample 接口汇合。
add_library(aim_input
  src/input/synthetic.cpp
  src/input/replay.cpp
  src/input/realsense.cpp)
target_link_libraries(aim_input PUBLIC aim_observation PRIVATE opencv_imgcodecs)
if(AIM_WITH_REALSENSE)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    target_link_libraries(aim_input PRIVATE realsense2::realsense2)
    # 此宏同时让应用帮助页知道相机功能已编译。
    target_compile_definitions(aim_input PUBLIC AIM_HAVE_REALSENSE=1)
  else()
    message(STATUS "RealSense SDK unavailable: synthetic/replay builds remain enabled")
  endif()
endif()

# App-only headers live next to their implementation, outside the public API.
# 应用库组织模块、参数、日志与 ROS；私有头文件不放进公共 include 目录。
add_library(aim_app STATIC
  src/app/application.cpp
  src/app/options.cpp
  src/app/json_log.cpp
  src/app/ros_output.cpp)
target_link_libraries(aim_app PRIVATE aim_core aim_detector aim_input
                                     opencv_highgui opencv_imgcodecs)
if(AIM_WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  # ROS 依赖仅接入应用层；PUBLIC 关键字与其他 CMake 链接调用保持一致。
  ament_target_dependencies(aim_app PUBLIC rclcpp std_msgs)
  target_compile_definitions(aim_app PRIVATE AIM_HAVE_ROS2=1)
endif()

# 最终 CLI 入口只调用应用层 run()，保留现有二进制名称。
add_executable(demo_aim_cpp src/app/main.cpp)
target_link_libraries(demo_aim_cpp PRIVATE aim_app)

# 进程监管/共享帧只构建一次；RGB 和 Astra 保留各自采集 worker。
add_library(aim_camera_stream STATIC
  src/input/uvc_stream.cpp
  src/input/uvc_controls.cpp)
target_include_directories(aim_camera_stream PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aim_camera_stream PUBLIC opencv_core Threads::Threads)
add_library(aim_uvc_worker STATIC src/input/uvc_worker.cpp)
target_link_libraries(aim_uvc_worker PUBLIC aim_camera_stream PRIVATE opencv_videoio)

# 无模型、无相机厂商 SDK 的独立 UVC/传统灯条验证入口。
add_library(aim_light_bars
  src/vision/light_bars.cpp
  src/vision/light_bar_config.cpp
  src/vision/light_bar_overlay.cpp
  src/vision/armor_geometry.cpp
  src/vision/armor_pose.cpp)
target_include_directories(aim_light_bars PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aim_light_bars PUBLIC opencv_core
  PRIVATE opencv_imgproc opencv_calib3d yaml-cpp)
add_executable(camera_light_test
  src/app/camera_light_test.cpp
  src/app/camera/application.cpp
  src/app/camera/options.cpp
  src/app/camera/preview.cpp
  src/app/camera/snapshot.cpp)
target_link_libraries(camera_light_test PRIVATE aim_light_bars aim_uvc_worker
  opencv_highgui opencv_imgcodecs opencv_imgproc Threads::Threads)

add_library(aim_armor_auto
  src/vision/armor_auto_config.cpp
  src/vision/armor_tracker.cpp
  src/vision/camera_calibration.cpp)
target_link_libraries(aim_armor_auto PUBLIC aim_light_bars PRIVATE yaml-cpp opencv_calib3d)
add_library(aim_armor_output STATIC src/app/armor_auto/output.cpp)
target_link_libraries(aim_armor_output PUBLIC aim_armor_auto nlohmann_json::nlohmann_json)
add_executable(armor_auto src/app/armor_auto_main.cpp src/app/armor_calibrate.cpp
  src/app/armor_auto/application.cpp src/app/armor_auto/options.cpp
  src/app/armor_auto/preview.cpp)
target_link_libraries(armor_auto PRIVATE aim_armor_output aim_uvc_worker opencv_highgui
  opencv_imgproc opencv_imgcodecs opencv_calib3d yaml-cpp nlohmann_json::nlohmann_json Threads::Threads)

# 深度区域平面测量独立于厂商 SDK，便于离线验证。
add_library(aim_depth_plane src/vision/depth_plane.cpp)
target_include_directories(aim_depth_plane PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aim_depth_plane PUBLIC opencv_core)
# 输出有效性可在没有 Astra SDK/硬件的环境下回归。
add_library(aim_depth_output STATIC src/app/depth/output.cpp)
target_link_libraries(aim_depth_output PUBLIC aim_depth_plane nlohmann_json::nlohmann_json)
option(AIM_WITH_ASTRA "Build Astra Pro manual depth live measurement" OFF)
set(AIM_ASTRA_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../vio_std/OpenNI_2.3.0.63/Linux/OpenNI-Linux-x64-2.3.0.63" CACHE PATH "Vendor OpenNI 2.3.0.63 root")
if(AIM_WITH_ASTRA)
  if(NOT EXISTS "${AIM_ASTRA_SDK_ROOT}/Include/OpenNI.h" OR
     NOT EXISTS "${AIM_ASTRA_SDK_ROOT}/Samples/Bin/libOpenNI2.so" OR
     NOT EXISTS "${AIM_ASTRA_SDK_ROOT}/Samples/Bin/OpenNI2/Drivers/liborbbec.so")
    message(FATAL_ERROR "Set AIM_ASTRA_SDK_ROOT to a complete vendor OpenNI 2.3.0.63 SDK (Include/OpenNI.h, Samples/Bin/libOpenNI2.so and Samples/Bin/OpenNI2/Drivers/liborbbec.so)")
  endif()
  add_executable(astra_live_measure src/app/astra_live_measure.cpp
    src/app/depth/application.cpp src/app/depth/options.cpp
    src/app/depth/preview.cpp src/input/astra_depth_worker.cpp)
  target_include_directories(astra_live_measure SYSTEM PRIVATE "${AIM_ASTRA_SDK_ROOT}/Include")
  target_link_libraries(astra_live_measure PRIVATE aim_depth_output aim_camera_stream
    "${AIM_ASTRA_SDK_ROOT}/Samples/Bin/libOpenNI2.so"
    opencv_highgui opencv_imgproc opencv_imgcodecs Threads::Threads nlohmann_json::nlohmann_json)
  install(TARGETS astra_live_measure RUNTIME DESTINATION bin)
endif()

# 定位和任务决策只依赖核心几何；离线验证不需要 MAVLink/飞控。
add_library(aim_navigation
  src/navigation/geometry.cpp
  src/navigation/localization.cpp
  src/navigation/mission.cpp)
target_link_libraries(aim_navigation PUBLIC aim_observation)
add_executable(demo_mission_cpp src/app/mission_demo.cpp)
target_link_libraries(demo_mission_cpp PRIVATE aim_navigation)

# 使用官方生成头文件，不自动联网下载，也不改变用户 PX4 工程。
option(AIM_WITH_MAVLINK "Build PX4 serial localization receiver" OFF)
set(AIM_MAVLINK_INCLUDE_DIR "" CACHE PATH "Generated MAVLink v2 directory containing common/mavlink.h")
if(AIM_WITH_MAVLINK)
  if(NOT EXISTS "${AIM_MAVLINK_INCLUDE_DIR}/common/mavlink.h")
    message(FATAL_ERROR "Set AIM_MAVLINK_INCLUDE_DIR to generated MAVLink v2 headers (common/mavlink.h)")
  endif()
  add_library(aim_mavlink src/io/mavlink_link.cpp src/io/serial_port.cpp)
  target_include_directories(aim_mavlink SYSTEM PRIVATE "${AIM_MAVLINK_INCLUDE_DIR}")
  target_link_libraries(aim_mavlink PUBLIC aim_navigation aim_flight_state)
  add_executable(px4_localization_cpp src/app/localization_main.cpp)
  target_link_libraries(px4_localization_cpp PRIVATE aim_mavlink yaml-cpp)
  install(TARGETS px4_localization_cpp RUNTIME DESTINATION bin)
  install(FILES config/px4_link.yaml DESTINATION share/no_demo_aim)
endif()

# BUILD_TESTING 默认开启；登记视觉、定位、任务与可选通信回归。
include(CTest)
if(BUILD_TESTING)
  add_executable(aim_app_output_tests test/test_app_output.cpp)
  target_link_libraries(aim_app_output_tests PRIVATE aim_armor_output aim_depth_output)
  add_test(NAME aim_app_output_regression COMMAND aim_app_output_tests)
  add_executable(aim_armor_snapshot_tests test/test_armor_snapshot.cpp
    src/app/armor_auto/preview.cpp)
  target_link_libraries(aim_armor_snapshot_tests PRIVATE aim_armor_output
    opencv_imgproc opencv_imgcodecs)
  add_test(NAME aim_armor_snapshot_regression COMMAND aim_armor_snapshot_tests)
  add_executable(aim_armor_auto_tests test/test_armor_auto.cpp)
  target_link_libraries(aim_armor_auto_tests PRIVATE aim_armor_auto yaml-cpp)
  add_test(NAME aim_armor_auto_regression COMMAND aim_armor_auto_tests ${CMAKE_CURRENT_SOURCE_DIR}/config/armor_auto.yaml)
  add_test(NAME aim_armor_auto_help COMMAND armor_auto --help)
  add_executable(aim_depth_plane_tests test/test_depth_plane.cpp)
  target_link_libraries(aim_depth_plane_tests PRIVATE aim_depth_plane)
  add_test(NAME aim_depth_plane_regression COMMAND aim_depth_plane_tests)
  if(AIM_WITH_ASTRA)
    add_test(NAME aim_astra_live_help COMMAND astra_live_measure --help)
  endif()
  add_executable(aim_light_bar_tests test/test_light_bars.cpp)
  target_link_libraries(aim_light_bar_tests PRIVATE aim_light_bars opencv_imgproc opencv_calib3d)
  add_test(NAME aim_light_bar_regression COMMAND aim_light_bar_tests ${CMAKE_CURRENT_SOURCE_DIR}/config/light_bars.yaml)
  add_test(NAME aim_camera_light_help COMMAND camera_light_test --help)
  add_executable(aim_uvc_control_tests test/test_uvc_controls.cpp)
  target_link_libraries(aim_uvc_control_tests PRIVATE aim_camera_stream)
  target_link_options(aim_uvc_control_tests PRIVATE "-Wl,--wrap=ioctl")
  add_test(NAME aim_uvc_control_regression COMMAND aim_uvc_control_tests)
  add_executable(aim_camera_stream_tests test/test_camera_stream.cpp src/app/camera/preview.cpp)
  target_link_libraries(aim_camera_stream_tests PRIVATE aim_camera_stream opencv_imgproc)
  add_test(NAME aim_camera_stream_regression COMMAND aim_camera_stream_tests)
  set_tests_properties(aim_camera_stream_regression PROPERTIES TIMEOUT 20)
  add_executable(aim_navigation_tests test/test_navigation.cpp)
  target_link_libraries(aim_navigation_tests PRIVATE aim_navigation)
  add_test(NAME aim_navigation_regression COMMAND aim_navigation_tests)
  add_test(NAME aim_mission_demo COMMAND demo_mission_cpp)
  if(AIM_WITH_MAVLINK)
    add_executable(aim_mavlink_tests test/test_mavlink.cpp)
    target_include_directories(aim_mavlink_tests SYSTEM PRIVATE "${AIM_MAVLINK_INCLUDE_DIR}")
    target_link_libraries(aim_mavlink_tests PRIVATE aim_mavlink util)
    add_test(NAME aim_mavlink_regression COMMAND aim_mavlink_tests)
    add_test(NAME aim_localization_cli_help COMMAND px4_localization_cpp --help)
  endif()
  add_executable(aim_core_tests test/test_aim.cpp)
  target_link_libraries(aim_core_tests PRIVATE aim_core aim_detector aim_input opencv_imgcodecs)
  add_test(NAME aim_core_regression COMMAND aim_core_tests ${CMAKE_CURRENT_SOURCE_DIR}/config/aim.yaml)
  add_test(NAME aim_cli_help COMMAND demo_aim_cpp --help)
endif()

# 统一设置本项目目标的警告；可选程序或关闭测试时跳过不存在的目标。
foreach(target
    aim_observation aim_flight_state aim_core aim_detector aim_input aim_app
    aim_camera_stream aim_uvc_worker aim_light_bars aim_armor_auto aim_armor_output
    aim_depth_plane aim_depth_output aim_navigation aim_mavlink
    demo_aim_cpp camera_light_test armor_auto astra_live_measure
    demo_mission_cpp px4_localization_cpp
    aim_app_output_tests aim_armor_snapshot_tests aim_armor_auto_tests aim_depth_plane_tests
    aim_light_bar_tests aim_uvc_control_tests aim_camera_stream_tests
    aim_navigation_tests aim_mavlink_tests aim_core_tests)
  if(TARGET ${target})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()
# 可选安装规则：复制程序和默认配置，不自动安装模型权重。
install(TARGETS demo_aim_cpp demo_mission_cpp camera_light_test armor_auto RUNTIME DESTINATION bin)
install(FILES config/aim.yaml config/light_bars.yaml config/armor_auto.yaml config/astra_light_bars.yaml DESTINATION share/no_demo_aim)

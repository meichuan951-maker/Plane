# Test-only helpers and private application headers stay out of production APIs.
function(aim_add_regression target name source)
  cmake_parse_arguments(TEST "" "LABEL" "LIBRARIES;SOURCES;ARGS" ${ARGN})
  add_executable(${target} ${source} ${TEST_SOURCES})
  target_include_directories(${target} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(${target} PRIVATE ${TEST_LIBRARIES})
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  # Keep ./build/aim_*_tests and disabled-target archiving in the one build directory.
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  add_test(NAME ${name} COMMAND ${target} ${TEST_ARGS})
  set_tests_properties(${name} PROPERTIES LABELS "${TEST_LABEL}")
endfunction()

set(config_dir "${PROJECT_SOURCE_DIR}/config")

aim_add_regression(aim_core_tests aim_core_regression core/test_aim.cpp
  LABEL core LIBRARIES aim_core aim_input
  ARGS "${config_dir}/aim.yaml")

aim_add_regression(aim_light_bar_tests aim_light_bar_regression vision/test_light_bars.cpp
  LABEL vision LIBRARIES aim_light_bars aim_core opencv_imgproc opencv_calib3d
  ARGS "${config_dir}/light_bars.yaml")
aim_add_regression(aim_armor_auto_tests aim_armor_auto_regression vision/test_armor_auto.cpp
  LABEL vision LIBRARIES aim_armor_auto yaml-cpp
  ARGS "${config_dir}/armor_auto.yaml")
aim_add_regression(aim_depth_plane_tests aim_depth_plane_regression vision/test_depth_plane.cpp
  LABEL vision LIBRARIES aim_depth_plane)

aim_add_regression(aim_camera_stream_tests aim_camera_stream_regression input/test_camera_stream.cpp
  LABEL input LIBRARIES aim_camera_stream opencv_imgproc
  SOURCES "${PROJECT_SOURCE_DIR}/src/app/camera/preview.cpp")
set_tests_properties(aim_camera_stream_regression PROPERTIES TIMEOUT 20)
aim_add_regression(aim_uvc_control_tests aim_uvc_control_regression input/test_uvc_controls.cpp
  LABEL input LIBRARIES aim_camera_stream)
target_link_options(aim_uvc_control_tests PRIVATE "-Wl,--wrap=ioctl")

aim_add_regression(aim_app_output_tests aim_app_output_regression app/test_app_output.cpp
  LABEL app LIBRARIES aim_armor_output aim_depth_output aim_app_common)
aim_add_regression(aim_json_log_tests aim_json_log_regression app/test_json_log.cpp
  LABEL app LIBRARIES nlohmann_json::nlohmann_json
  SOURCES "${PROJECT_SOURCE_DIR}/src/app/json_log.cpp")
target_include_directories(aim_json_log_tests PRIVATE "${PROJECT_SOURCE_DIR}/include")
set_tests_properties(aim_json_log_regression PROPERTIES LABELS "app;io" TIMEOUT 10)
aim_add_regression(aim_armor_snapshot_tests aim_armor_snapshot_regression app/test_armor_snapshot.cpp
  LABEL app LIBRARIES aim_armor_output opencv_imgproc opencv_imgcodecs
  SOURCES "${PROJECT_SOURCE_DIR}/src/app/armor_auto/preview.cpp")

aim_add_regression(aim_navigation_tests aim_navigation_regression navigation/test_navigation.cpp
  LABEL navigation LIBRARIES aim_navigation)
if(AIM_WITH_MAVLINK)
  aim_add_regression(aim_mavlink_tests aim_mavlink_regression io/test_mavlink.cpp
    LABEL io LIBRARIES aim_mavlink util)
  target_include_directories(aim_mavlink_tests SYSTEM PRIVATE "${AIM_MAVLINK_INCLUDE_DIR}")
  aim_add_regression(aim_localization_cli_tests aim_localization_cli_regression app/test_localization_cli.cpp
    LABEL io LIBRARIES aim_observation util Threads::Threads
    ARGS "$<TARGET_FILE:px4_localization_cpp>")
  target_include_directories(aim_localization_cli_tests SYSTEM PRIVATE "${AIM_MAVLINK_INCLUDE_DIR}")
  add_dependencies(aim_localization_cli_tests px4_localization_cpp)
  set_tests_properties(aim_localization_cli_regression PROPERTIES LABELS "cli;io" TIMEOUT 60)
endif()

# CLI/demo checks exercise existing application targets without opening devices.
function(aim_add_smoke name target label)
  add_test(NAME ${name} COMMAND ${target} ${ARGN})
  set_tests_properties(${name} PROPERTIES LABELS "cli;${label}")
endfunction()
aim_add_smoke(aim_cli_help demo_aim_cpp core --help)
add_test(NAME aim_removed_replay_cli_regression
  COMMAND "${CMAKE_COMMAND}" "-DDEMO_AIM_BINARY=$<TARGET_FILE:demo_aim_cpp>"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/app/test_removed_replay_cli.cmake")
set_tests_properties(aim_removed_replay_cli_regression PROPERTIES LABELS "cli;core" TIMEOUT 20)
add_test(NAME aim_removed_yolo_cli_regression
  COMMAND "${CMAKE_COMMAND}" "-DDEMO_AIM_BINARY=$<TARGET_FILE:demo_aim_cpp>"
    "-DVIDEO_WORKER_BINARY=$<TARGET_FILE:vision_video_worker>"
    "-DMEMORY_PROBE_BINARY=$<TARGET_FILE:vision_memory_probe>"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/app/test_removed_yolo_cli.cmake")
set_tests_properties(aim_removed_yolo_cli_regression PROPERTIES LABELS "cli;core;benchmark" TIMEOUT 30)
aim_add_smoke(aim_camera_light_help camera_light_test input --help)
aim_add_smoke(aim_armor_auto_help armor_auto vision --help)
aim_add_smoke(aim_mission_demo demo_mission_cpp navigation)
if(AIM_WITH_MAVLINK)
  aim_add_smoke(aim_localization_cli_help px4_localization_cpp io --help)
endif()

add_test(NAME aim_build_tools_regression
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/build/test_build_tools.sh" "${PROJECT_SOURCE_DIR}")
set_tests_properties(aim_build_tools_regression PROPERTIES LABELS build)

# Offline Python checks cover control policies and benchmark tooling.
# Optional fixture dependencies are not required by the C++ regressions.
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  foreach(yaw_test IN ITEMS policy mission)
    add_test(NAME aim_yaw_${yaw_test}_regression
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/io/test_yaw_${yaw_test}.py")
    set_tests_properties(aim_yaw_${yaw_test}_regression PROPERTIES LABELS "io;control" TIMEOUT 30)
  endforeach()
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import pymavlink, yaml"
    RESULT_VARIABLE aim_have_yaw_runtime_dependencies OUTPUT_QUIET ERROR_QUIET)
  if(aim_have_yaw_runtime_dependencies EQUAL 0)
    add_test(NAME aim_yaw_follow_regression
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/io/test_yaw_follow.py")
    set_tests_properties(aim_yaw_follow_regression PROPERTIES LABELS "io;control" TIMEOUT 30)
  endif()
  add_test(NAME aim_resource_sampling_regression
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/build/test_resource_sampling.py")
  add_test(NAME aim_vision_control_benchmark_regression
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/build/test_vision_control_benchmark.py")
  set_tests_properties(aim_resource_sampling_regression aim_vision_control_benchmark_regression
    PROPERTIES LABELS "build;benchmark" TIMEOUT 30)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import pymavlink"
    RESULT_VARIABLE aim_have_pymavlink OUTPUT_QUIET ERROR_QUIET)
  if(aim_have_pymavlink EQUAL 0)
    add_test(NAME aim_px4_go_offline_regression
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/io/test_px4_go_control.py")
    set_tests_properties(aim_px4_go_offline_regression
      PROPERTIES LABELS "io;benchmark" TIMEOUT 30)
  endif()
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import cv2, numpy"
    RESULT_VARIABLE aim_have_python_cv OUTPUT_QUIET ERROR_QUIET)
  if(aim_have_python_cv EQUAL 0)
    add_test(NAME aim_video_worker_regression
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/vision/test_video_worker.py"
        --worker "$<TARGET_FILE:vision_video_worker>" --root "${PROJECT_SOURCE_DIR}")
    set_tests_properties(aim_video_worker_regression
      PROPERTIES LABELS "vision;benchmark" TIMEOUT 30)
  endif()
endif()

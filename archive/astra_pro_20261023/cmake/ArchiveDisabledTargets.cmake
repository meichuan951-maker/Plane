# CMake does not remove old binaries when an optional target disappears.
# Archive only known generated files; keep independent diagnostics and user files.
function(no_demo_archive_disabled_target target filename)
  set(artifact "${CMAKE_CURRENT_BINARY_DIR}/${filename}")
  if(NOT TARGET ${target} AND EXISTS "${artifact}" AND NOT IS_DIRECTORY "${artifact}")
    file(SHA256 "${artifact}" digest)
    set(archive "${CMAKE_CURRENT_BINARY_DIR}/disabled/${digest}")
    file(MAKE_DIRECTORY "${archive}")
    file(RENAME "${artifact}" "${archive}/${filename}")
    message(STATUS "Disabled target ${target}: archived ${filename} in ${archive}")
  endif()
endfunction()

foreach(target
    astra_live_measure px4_localization_cpp
    aim_app_output_tests aim_armor_snapshot_tests aim_armor_auto_tests
    aim_depth_plane_tests aim_light_bar_tests aim_uvc_control_tests
    aim_camera_stream_tests aim_navigation_tests aim_mavlink_tests aim_core_tests
    aim_localization_cli_tests)
  no_demo_archive_disabled_target(${target} "${target}${CMAKE_EXECUTABLE_SUFFIX}")
endforeach()
no_demo_archive_disabled_target(aim_mavlink
  "${CMAKE_STATIC_LIBRARY_PREFIX}aim_mavlink${CMAKE_STATIC_LIBRARY_SUFFIX}")

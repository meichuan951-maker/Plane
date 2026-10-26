# Keep all build trees under one physical build/ directory. colcon places each
# package in build/<package>, while the standalone CMake entry uses build/ itself.
# Resolve the actual paths, but not the expected root, to reject symlink escapes.
get_filename_component(_aim_source_dir "${CMAKE_SOURCE_DIR}" REALPATH)
get_filename_component(_aim_binary_dir "${CMAKE_BINARY_DIR}" REALPATH)
set(_aim_build_dir "${_aim_source_dir}/build")
# The trailing separator distinguishes build/ from siblings such as build_cpp/.
string(FIND "${_aim_binary_dir}/" "${_aim_build_dir}/" _aim_build_prefix)
if(NOT _aim_build_prefix EQUAL 0)
  message(FATAL_ERROR
    "Build output must be ${_aim_build_dir} or a subdirectory within it. "
    "Use ./scripts/build.sh, cmake -S . -B build, or colcon build; "
    "in-source builds and output outside build/ are not supported.")
endif()
unset(_aim_source_dir)
unset(_aim_binary_dir)
unset(_aim_build_dir)
unset(_aim_build_prefix)

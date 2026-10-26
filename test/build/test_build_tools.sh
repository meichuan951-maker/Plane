#!/usr/bin/env bash
# No SDK or hardware needed: test one build directory and disabled-target archiving.
set -euo pipefail
source_dir="$1"
fixture="$(mktemp -d "${TMPDIR:-/tmp}/no-demo-build-tools.XXXXXX")"
trap 'rm -rf -- "$fixture"' EXIT
mkdir -p "$fixture/project/cmake" "$fixture/project/scripts" "$fixture/bin"
cp "$source_dir/cmake/ArchiveDisabledTargets.cmake" "$fixture/project/cmake/"
cp "$source_dir/cmake/RequireBuildDirectory.cmake" "$fixture/project/cmake/"
cp "$source_dir/scripts/build.sh" "$fixture/project/scripts/"
cat > "$fixture/project/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/RequireBuildDirectory.cmake")
project(build_tools_fixture NONE)
option(ENABLE_OPTIONAL "Enable fixture targets" ON)
if(ENABLE_OPTIONAL)
  add_custom_target(px4_localization_cpp)
  add_custom_target(aim_mavlink_tests)
endif()
include(cmake/ArchiveDisabledTargets.cmake)
EOF
cmake -S "$fixture/project" -B "$fixture/project/build"
printf 'old optional program\n' > "$fixture/project/build/px4_localization_cpp"
printf 'old test program\n' > "$fixture/project/build/aim_mavlink_tests"
printf 'independent diagnostic\n' > "$fixture/project/build/generic_diagnostic"
printf 'user content\n' > "$fixture/project/build/notes.txt"
declare -A retired_hashes
for target in astra_live_measure astra_depth_check; do
  printf 'retired camera program: %s\n' "$target" > "$fixture/project/build/$target"
  retired_hashes[$target]="$(sha256sum "$fixture/project/build/$target" | cut -d ' ' -f 1)"
done
cmake -S "$fixture/project" -B "$fixture/project/build"
test -f "$fixture/project/build/px4_localization_cpp"
for target in astra_live_measure astra_depth_check; do
  test ! -e "$fixture/project/build/$target"
  test -f "$fixture/project/build/disabled/${retired_hashes[$target]}/$target"
done
program_hash="$(sha256sum "$fixture/project/build/px4_localization_cpp" | cut -d ' ' -f 1)"
test_hash="$(sha256sum "$fixture/project/build/aim_mavlink_tests" | cut -d ' ' -f 1)"
cmake -S "$fixture/project" -B "$fixture/project/build" -DENABLE_OPTIONAL=OFF
test ! -e "$fixture/project/build/px4_localization_cpp"
test ! -e "$fixture/project/build/aim_mavlink_tests"
test -f "$fixture/project/build/disabled/$program_hash/px4_localization_cpp"
test -f "$fixture/project/build/disabled/$test_hash/aim_mavlink_tests"
test -f "$fixture/project/build/generic_diagnostic"
test -f "$fixture/project/build/notes.txt"
cmake -S "$fixture/project" -B "$fixture/project/build" -DENABLE_OPTIONAL=OFF

# colcon isolates each package below the same top-level build/ directory.
cmake -S "$fixture/project" -B "$fixture/project/build/build_tools_fixture" -DENABLE_OPTIONAL=OFF
test -f "$fixture/project/build/build_tools_fixture/CMakeCache.txt"

# Exercise the actual CMake guard in temporary projects, never another workspace build.
expect_cmake_rejection() {
  local label="$1"
  shift
  if cmake "$@" > "$fixture/rejected-$label.log" 2>&1; then
    echo "CMake accepted an unsupported build directory: $label" >&2; exit 1
  fi
  if ! grep -Fq 'Build output must be' "$fixture/rejected-$label.log"; then
    cat "$fixture/rejected-$label.log" >&2
    echo "CMake failed before reaching the build-directory guard: $label" >&2; exit 1
  fi
}
expect_cmake_rejection legacy -S "$fixture/project" -B "$fixture/project/build_cpp"
expect_cmake_rejection verification -S "$fixture/project" -B "$fixture/project/build_verify"
expect_cmake_rejection prefix-sibling -S "$fixture/project" -B "$fixture/project/build-extra/package"
expect_cmake_rejection external -S "$fixture/project" -B "$fixture/external"
expect_cmake_rejection parent-escape -S "$fixture/project" -B "$fixture/project/build/../outside"
ln -s "$fixture/elsewhere-child" "$fixture/project/build/escaped-package"
mkdir -p "$fixture/elsewhere-child"
expect_cmake_rejection child-symlink-escape -S "$fixture/project" -B "$fixture/project/build/escaped-package"

mkdir -p "$fixture/in-source/cmake" "$fixture/escaped/cmake" "$fixture/elsewhere"
for project in in-source escaped; do
  cp "$fixture/project/CMakeLists.txt" "$fixture/$project/"
  cp "$fixture/project/cmake/"*.cmake "$fixture/$project/cmake/"
done
expect_cmake_rejection in-source -S "$fixture/in-source" -B "$fixture/in-source"
ln -s "$fixture/elsewhere" "$fixture/escaped/build"
expect_cmake_rejection symlink-escape -S "$fixture/escaped" -B "$fixture/escaped/build"

# A source alias still reaches the same physical project/build directory.
ln -s "$fixture/project" "$fixture/project-alias"
cmake -S "$fixture/project-alias" -B "$fixture/project-alias/build" -DENABLE_OPTIONAL=OFF

# Record each CLI argument separately, including an include path with spaces.
cat > "$fixture/bin/cmake" <<'EOF'
#!/usr/bin/env bash
printf '<cmake>\n' >> "$BUILD_TEST_LOG"
printf '<%s>\n' "$@" >> "$BUILD_TEST_LOG"
EOF
cat > "$fixture/bin/ctest" <<'EOF'
#!/usr/bin/env bash
printf '<ctest:%s>\n' "$PWD" >> "$BUILD_TEST_LOG"
EOF
chmod +x "$fixture/bin/cmake" "$fixture/bin/ctest"
export BUILD_TEST_LOG="$fixture/commands.log"
export PATH="$fixture/bin:$PATH"
(cd / && bash "$fixture/project/scripts/build.sh" --test -- \
  '-DAIM_MAVLINK_INCLUDE_DIR=/tmp/a sdk' -DBUILD_TESTING=OFF)
grep -Fx -- "<$fixture/project/build>" "$BUILD_TEST_LOG"
grep -Fx -- '<-DAIM_MAVLINK_INCLUDE_DIR=/tmp/a sdk>' "$BUILD_TEST_LOG"
grep -Fx -- "<ctest:$fixture/project/build>" "$BUILD_TEST_LOG"
mapfile -t testing_args < <(grep -F -- '<-DBUILD_TESTING=' "$BUILD_TEST_LOG")
test "${#testing_args[@]}" -eq 2
test "${testing_args[0]}" = '<-DBUILD_TESTING=OFF>'
test "${testing_args[1]}" = '<-DBUILD_TESTING=ON>'
if grep -Fq -- '<-DAIM_WITH_REALSENSE=OFF>' "$BUILD_TEST_LOG"; then
  echo 'Existing cache settings must be preserved' >&2; exit 1
fi
rm "$fixture/project/build/CMakeCache.txt"
: > "$BUILD_TEST_LOG"
bash "$fixture/project/scripts/build.sh"
for backend in REALSENSE ROS2 MAVLINK; do
  grep -Fx -- "<-DAIM_WITH_${backend}=OFF>" "$BUILD_TEST_LOG"
done
if grep -Fq -- 'AIM_WITH_ASTRA' "$BUILD_TEST_LOG" ||
   grep -Fq -- 'AIM_ASTRA_SDK' "$BUILD_TEST_LOG"; then
  echo 'Fresh builds must not configure the retired Astra backend' >&2; exit 1
fi
: > "$BUILD_TEST_LOG"
if bash "$fixture/project/scripts/build.sh" -B /tmp/undesired-build; then
  echo 'Alternate output directory should be rejected' >&2; exit 1
fi
if JOBS=0 bash "$fixture/project/scripts/build.sh"; then
  echo 'Invalid JOBS should be rejected' >&2; exit 1
fi
test ! -s "$BUILD_TEST_LOG"
echo 'Build tooling regression passed'

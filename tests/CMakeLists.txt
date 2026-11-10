# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Host tests for vi_core and vi_session.
#
# Configured only in host mode (see the root CMakeLists.txt): these run on the
# machine that builds them, and an iOS-arm64 test binary is not runnable there.
# They cover vi_core, vi_session and the bundled configuration -- everything
# past them needs a VkDevice, a live task port or an ARFrame, none of which
# exist off the device.
#
# googletest, matching the sibling libraries rather than inventing a harness for
# one repo. Fetched only here, so an iOS configure never sees it.
include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.18.0 # matches volumetric_kit_gfx's pin
  GIT_SHALLOW TRUE)

# Nothing here is installed, and the gmock half is unused. Off rather than left
# to default so the host build stays the small thing it is meant to be.
set(INSTALL_GTEST
    OFF
    CACHE BOOL "" FORCE)
set(BUILD_GMOCK
    OFF
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(
  vi_core_tests
  allocation_stop_test.cpp
  camera_open_test.cpp
  capture_report_test.cpp
  frame_fanout_test.cpp
  freshness_test.cpp
  growth_policy_test.cpp
  keyframe_binding_test.cpp
  link_state_test.cpp
  map_steps_test.cpp
  memory_budget_test.cpp
  orbit_camera_test.cpp
  picture_slot_test.cpp
  rate_gate_test.cpp
  reader_line_test.cpp
  scanner_mode_test.cpp
  self_test_test.cpp
  stat_tone_test.cpp
  stream_rate_test.cpp
  tracking_line_test.cpp
  view_orientation_test.cpp
  view_picture_test.cpp)

find_package(Threads REQUIRED)
target_link_libraries(vi_core_tests PRIVATE vi_core GTest::gtest_main
                                            Threads::Threads)

# Registers each TEST() with ctest individually, so a failure names the case
# rather than the binary; the time limit makes a case that hangs fail by name
# too, rather than at the job's limit.
include(GoogleTest)
gtest_discover_tests(
  vi_core_tests
  PROPERTIES
  TIMEOUT 60)

# vi_session's writer, on a fake sensor's frames, read back through grid's
# reader and sensor's wire form; with vi_core's map-step rule and rate gate in
# the iPad's order. Its live link on 127.0.0.1, read through grid's client on
# the core's listener, as the lab reads it; and the on-screen entry.
add_executable(vi_session_tests dial_entry_test.cpp rgbd_stream_link_test.cpp
                                rgbd_stream_writer_test.cpp)
target_link_libraries(
  vi_session_tests
  PRIVATE vi_session
          vi_core
          volumetric_kit::sensor_wire
          volumetric_kit::grid_stream
          volumetric_kit::grid_networking
          volumetric_kit::core_net
          GTest::gtest_main)
gtest_discover_tests(vi_session_tests)

# vi_stream's sender on this Mac's GPU and VideoToolbox: sensor's ArkitSensor
# with its test pattern (no camera) through the sender into a session, read back
# with grid's reader, each picture decoded on VideoToolbox and its index read on
# the GPU (sensor_prep's PatternReader); the live link on 127.0.0.1. Skips
# without a device or an encoder unless the run promises them.
if(APPLE)
  add_executable(vi_stream_tests rgbd_stream_sender_test.cpp)
  target_link_libraries(
    vi_stream_tests
    PRIVATE vi_stream
            vi_session
            vi_core
            volumetric_kit::sensor_arkit
            volumetric_kit::sensor_prep
            volumetric_kit::sensor_video
            volumetric_kit::sensor_wire
            volumetric_kit::grid_stream
            volumetric_kit::grid_networking
            volumetric_kit::core_net
            volumetric_kit::core_test_support
            GTest::gtest_main)
  gtest_discover_tests(vi_stream_tests)
endif()

add_executable(vi_config_tests scanner_config_test.cpp)
target_link_libraries(vi_config_tests PRIVATE volumetric_kit::core_config
                                              GTest::gtest_main)
target_compile_definitions(
  vi_config_tests
  PRIVATE
    VI_SCANNER_CONFIG="${PROJECT_SOURCE_DIR}/apps/scanner/config/scanner.json")
gtest_discover_tests(vi_config_tests)

# Allocation injection stays in its own process; these exercise the exact
# platform-free policies used by capture and worker entry points.
add_executable(vi_failure_tests failure_handling_test.cpp
                                allocation_failure.cpp)
target_link_libraries(vi_failure_tests PRIVATE vi_core GTest::gtest_main
                                               Threads::Threads)
gtest_discover_tests(vi_failure_tests)

# The scanner's entitlements as cmake/ScannerEntitlements.cmake writes them for
# the app, with VI_EXTENDED_VIRTUAL_ADDRESSING off and on. A script, not a
# binary: nothing to build, and plutil does the reading.
add_test(
  NAME scanner_entitlements
  COMMAND
    "${CMAKE_COMMAND}" "-DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/entitlements" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/scanner_entitlements_test.cmake")
set_tests_properties(scanner_entitlements PROPERTIES SKIP_REGULAR_EXPRESSION
                                                     "SKIP: plutil not found")

# The scanner's build stamp as cmake/ScannerVersion.cmake writes it into the
# app's Info.plist: the first-parent count, the hash and its -dirty and +local,
# a worktree, a shallow clone and every fallback, on throwaway repositories; and
# the plist from the configure and from the command each build runs. A script,
# like the entitlements: git and plutil do the work.
add_test(
  NAME scanner_version
  COMMAND "${CMAKE_COMMAND}" "-DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/version" -P
          "${CMAKE_CURRENT_SOURCE_DIR}/scanner_version_test.cmake")
set_tests_properties(
  scanner_version PROPERTIES SKIP_REGULAR_EXPRESSION
                             "SKIP: git or plutil not found" TIMEOUT 60)

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Host tests for vi_core.
#
# Configured only in host mode (see the root CMakeLists.txt): these run on the
# machine that builds them, and an iOS-arm64 test binary is not runnable there.
# That is also why they cover vi_core and nothing else -- everything past it
# needs a VkDevice, a live task port or an ARFrame, none of which exist off the
# device.
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

add_executable(vi_core_tests memory_budget_test.cpp orbit_camera_test.cpp
                             stat_tone_test.cpp view_orientation_test.cpp)

target_link_libraries(vi_core_tests PRIVATE vi_core GTest::gtest_main)

# Registers each TEST() with ctest individually, so a failure names the case
# rather than the binary.
include(GoogleTest)
gtest_discover_tests(vi_core_tests)

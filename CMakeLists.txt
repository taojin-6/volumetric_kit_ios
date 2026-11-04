# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

cmake_minimum_required(VERSION 3.21)

project(
  volumetric_kit_ios
  VERSION 0.0.1
  DESCRIPTION
    "iOS application shell for the volumetric_kit family: ARKit capture -> volumetric_kit_recon fusion -> volumetric_kit_gfx rendering, on MoltenVK"
  HOMEPAGE_URL "https://github.com/taojin-6/volumetric_kit_ios"
  # CXX only here; the app languages are enabled below, once it is known whether
  # this is an iOS configure. CMake's Swift support requires the Ninja or Xcode
  # generator, so declaring it unconditionally would make a host configure fail
  # under the default Makefiles generator -- on a target that has no Swift in
  # it. See the enable_language calls.
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The app targets are iOS-only. `vi_core` is not, and that is the whole point of
# it: it is the scanner's pure logic. Together with the session writer
# (`vi_session`) and the shared configuration parser it can be tested on the
# host. The renderer is device-only -- MoltenVK ships no simulator slice and
# ARKit scene depth needs LiDAR -- so without this mode CI can assert that the
# app compiles and nothing more.
#
# Opt-in rather than inferred from the absence of the toolchain file, because
# the FATAL_ERROR below is load-bearing: configuring without the toolchain is
# the ordinary mistake, and it used to fail here with one actionable line
# instead of somewhere deep in recon. Making a bare `cmake -S . -B build`
# silently succeed at building a fraction of the repo would trade that away.
option(VI_HOST_TESTS
       "Build vi_core, vi_session and their tests for the host (not iOS)" OFF)

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  set(VI_IOS TRUE)
else()
  set(VI_IOS FALSE)
endif()

if(NOT VI_IOS AND NOT VI_HOST_TESTS)
  message(
    FATAL_ERROR
      "This project targets iOS only. Configure with:\n"
      "  cmake -S . -B build-ios -G Xcode "
      "-DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake\n"
      "To build and run the host tests for vi_core and vi_session instead:\n"
      "  cmake -S . -B build-host -DVI_HOST_TESTS=ON")
endif()

if(VI_IOS AND VI_HOST_TESTS)
  message(
    FATAL_ERROR
      "VI_HOST_TESTS=ON is for a host configure; this one targets iOS. The "
      "tests run on the machine that builds them, and an iOS-arm64 test binary "
      "is not runnable here.")
endif()

# Swift for the app shell and UI, Objective-C++ for the seam onto the C++
# libraries. ARKit and UIKit are Swift-first, and the UI grows from here;
# Objective-C++ stays the bridge because an .mm is the only translation unit
# where an ARFrame and a C++ move-only type are both first-class.
#
# Enabled here rather than in project() so the host build needs no Swift:
# vi_core and vi_session are plain C++, and requiring a Swift-capable generator
# to run their tests would be a dependency on a language none of them touch.
if(VI_IOS)
  enable_language(OBJCXX)
  enable_language(Swift)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
# The Apple Development team the app bundle is signed with. Required to install
# on hardware; find yours with `security find-identity -v -p codesigning` (the
# parenthesised ID) or in Xcode > Settings > Accounts.
set(VI_DEVELOPMENT_TEAM
    ""
    CACHE STRING "Apple Development Team ID used to sign the app bundle")

set(VI_BUNDLE_PREFIX
    "io.taojin.volumetrickit"
    CACHE STRING "Reverse-DNS prefix for app bundle identifiers")

# --- The sibling libraries ---------------------------------------------------
# Fetched, not vendored: sensor, grid, recon and gfx are independent siblings
# that release on their own, and this repo consumes them exactly as any other
# downstream would. None needs an iOS-specific change -- the core's iOS module,
# on the MoltenVK the toolchain file names, is the whole of the platform port --
# so these are ordinary pinned checkouts.
#
# For local development against a working tree, point CMake at it instead of the
# remote:
#
# ~~~
#   -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=/path/to/volumetric_kit_recon
# ~~~
include(FetchContent)

# The common scenario parser is CPU-only; host tests exercise the exact file
# bundled into the scanner. The Vulkan tier is on in both modes, as sensor's
# interface reaches the Vulkan headers through its frame tier: a host build
# takes the Vulkan SDK from Homebrew, an iOS one the core's iOS module (its
# cmake/vkc_ios.cmake), which links MoltenVK's static library from MoltenVK_ROOT
# and answers every find_package(glm) -- the siblings' and this file's -- with
# the host's glm headers as a header-only target. The net tier (core_net),
# always built, checks the on-screen address and dials the live link's reader;
# 0.44.0 brings its port and host:port parser.
set(VKC_WITH_CONFIG ON)
set(VKC_WITH_VULKAN
    ON
    CACHE BOOL "" FORCE)
# The host tests of the stream sender run on the Mac's GPU, through the core's
# GoogleTest fixtures (core_test_support).
if(VI_HOST_TESTS)
  set(VKC_BUILD_TEST_SUPPORT
      ON
      CACHE BOOL "" FORCE)
endif()
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG aa0b4b2e36e4e550e8f765b4caa596c74083d830) # core #90 (0.44.0)
FetchContent_MakeAvailable(volumetric_kit_core)
vkc_require_core(0.44.0)

# glm, found here so that the imported glm::glm is visible to vi_core and to the
# siblings below: an imported target belongs to the directory that found it and
# those beneath it. A host build finds Homebrew's; an iOS build, the core's
# answer.
find_package(glm CONFIG REQUIRED)

# The siblings are pinned by commit, and adopted deliberately: a PR bumps a pin,
# and CI builds exactly that set. Tracking `main` let an upstream change break
# this build with no commit here (volumetric_kit_core's DECISIONS.md, "Consumers
# pin"). A commit hash rules out GIT_SHALLOW, which takes only a branch or tag.
#
# All of them build on volumetric_kit_core, the family's shared foundation, so
# it is declared first, and sensor and grid before recon, which fetches both:
# FetchContent keeps the first declaration of a name, which makes these pins the
# ones every sibling builds against (the core's README). The core's vulkan tier
# is on, as gfx requires, and the Device, Allocator and SharedDevice the bridge
# shares live there. Each floor below is its pin's version, so an older
# FETCHCONTENT_SOURCE_DIR_<NAME> override is refused, and vkc_require_sibling
# also refuses a sibling without a target the apps link. Each sibling's tests
# and examples default to "am I top level?" -- OFF for a fetched copy -- so only
# its library tiers build.

# sensor: the camera itself (sensor_arkit's ArkitSensor, an IRgbdSensor that
# owns the ARSession and imports each picture where ARKit left it) and the GPU
# pass that turns its frames into what fusion reads (sensor_prep's
# GpuFramePrep); the frame contract they share (sensor_interface); the HEVC
# encoder (sensor_video) with its rules (sensor_video_testing), which the
# scanner's `-self-test hevc` runs; and the frames' wire form (sensor_wire),
# which vi_session writes.
FetchContent_Declare(
  volumetric_kit_sensor
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_sensor.git
  GIT_TAG f0d920822749c2378de89edcdce693d8b8f2ad0a) # sensor #52 (0.44.0)
FetchContent_MakeAvailable(volumetric_kit_sensor)
vkc_require_sibling(
  volumetric_kit::sensor
  0.44.0
  TARGETS
  volumetric_kit::sensor_interface
  volumetric_kit::sensor_apple
  volumetric_kit::sensor_arkit
  volumetric_kit::sensor_prep
  volumetric_kit::sensor_video
  volumetric_kit::sensor_video_testing
  volumetric_kit::sensor_wire)

# grid: the sparse voxel block grid and Marching Cubes, whose DeviceMesh gfx
# draws in place, and the .vrst container (grid_stream) vi_session writes and
# serves to the reader it dials (grid_networking).
FetchContent_Declare(
  volumetric_kit_grid
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_grid.git
  GIT_TAG 17d48f30b3058a7ac9e2548ce2f11b95f659deab) # grid #18 (0.12.0)
FetchContent_MakeAvailable(volumetric_kit_grid)
vkc_require_sibling(
  volumetric_kit::grid
  0.12.0
  TARGETS
  volumetric_kit::grid_volume
  volumetric_kit::grid_mesh
  volumetric_kit::grid_stream
  volumetric_kit::grid_networking)

# --- Host mode: pure scanner logic, the session writer and sender, config ----
# vi_core is plain C++ over glm; vi_session is host code on sensor's and grid's
# public types; vi_stream, the sender, codes pictures on VideoToolbox, and its
# tests run on the Mac's GPU (sensor's ArkitSensor pattern, its decoder and
# PatternReader). None needs recon, gfx or a MoltenVK release, so the host
# configure stops here. It builds only what the tests link: the siblings'
# directories are left out of `all`, so their other tiers are configured and not
# compiled.
if(NOT VI_IOS)
  foreach(_vi_sibling volumetric_kit_core volumetric_kit_sensor
                      volumetric_kit_grid)
    set_property(DIRECTORY "${${_vi_sibling}_SOURCE_DIR}"
                 PROPERTY EXCLUDE_FROM_ALL ON)
  endforeach()
  enable_testing()
  add_subdirectory(apps/scanner/Core)
  add_subdirectory(apps/scanner/Session)
  add_subdirectory(tests)
  return()
endif()

# recon: TSDF fusion and the projective texturer.
FetchContent_Declare(
  volumetric_kit_recon
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_recon.git
  GIT_TAG 83dd202d0c08fb5f4fa95da54f6611bb3c7edfc4) # recon #337 (0.1.0)
FetchContent_MakeAvailable(volumetric_kit_recon)
vkc_require_sibling(volumetric_kit::recon 0.1.0 TARGETS
                    volumetric_kit::recon_tsdf volumetric_kit::recon_texture)

# gfx: the renderer. VG_WITH_GLFW gates examples only; the library tiers are
# GLFW-free by design, which is exactly why gfx ports to iOS untouched: its
# windowing tier takes a consumer-supplied surface rather than owning a window
# system. gfx sets no package version, so it has no floor here; its own
# vkc_require_core checks the core above.
FetchContent_Declare(
  volumetric_kit_gfx
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_gfx.git
  GIT_TAG ffc1a17a3d14aeb546e5484067dbf0160e76c60c) # gfx #173
set(VG_WITH_GLFW
    OFF
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

# Added from here rather than from apps/scanner/CMakeLists.txt, which is where
# they sit: the host build needs vi_core and vi_session *without* the scanner,
# and the scanner cannot be configured at all without the siblings and MoltenVK
# above. vi_session builds for iOS too, ahead of the scanner linking it.
add_subdirectory(apps/scanner/Core)
add_subdirectory(apps/scanner/Session)

add_subdirectory(apps/compute_smoke)
add_subdirectory(apps/scanner)

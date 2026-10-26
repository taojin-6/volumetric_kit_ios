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
# it: it is the scanner's pure logic. Together with the shared configuration
# parser it can be tested on the host. The renderer is device-only -- MoltenVK
# ships no simulator slice and ARKit scene depth needs LiDAR -- so without this
# mode CI can assert that the app compiles and nothing more.
#
# Opt-in rather than inferred from the absence of the toolchain file, because
# the FATAL_ERROR below is load-bearing: configuring without the toolchain is
# the ordinary mistake, and it used to fail here with one actionable line
# instead of somewhere deep in recon. Making a bare `cmake -S . -B build`
# silently succeed at building a fraction of the repo would trade that away.
option(VI_HOST_TESTS "Build vi_core and its tests for the host (not iOS)" OFF)

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
      "To build and run the host tests for vi_core instead:\n"
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
# Enabled here rather than in project() so the host build needs neither: vi_core
# is plain C++, and requiring a Swift-capable generator to run its tests would
# be a dependency on a language none of them touch.
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
# Fetched, not vendored: recon and gfx are independent siblings that release on
# their own, and this repo consumes them exactly as any other downstream would.
# Neither needs an iOS-specific change -- the core's iOS module, on the MoltenVK
# the toolchain file names, is the whole of the platform port -- so these are
# ordinary pinned checkouts.
#
# For local development against a working tree, point CMake at it instead of the
# remote:
#
# ~~~
#   -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=/path/to/volumetric_kit_recon
# ~~~
include(FetchContent)

# The common scenario parser is CPU-only; host tests exercise the exact file
# bundled into the scanner. Only iOS builds enable the Vulkan tier, and with it
# the core's iOS module (its cmake/vkc_ios.cmake), which links MoltenVK's static
# library from MoltenVK_ROOT and answers every find_package(glm) -- recon's,
# gfx's and this file's -- with the host's glm headers as a header-only target.
set(VKC_WITH_CONFIG ON)
set(VKC_WITH_VULKAN
    ${VI_IOS}
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG fa4638c2bcd4aee4a0a02819dc106ecd640441d9) # core #57 (0.22.0)
FetchContent_MakeAvailable(volumetric_kit_core)
vkc_require_core(0.22.0)

# --- Host mode: pure scanner logic and configuration tests -------------------
# Everything below this block needs recon, gfx and MoltenVK, none of which a
# host build wants or can use. So the host configure stops here.
#
# vi_core is plain C++ over glm, so this mode fetches glm 1.0.3, header-only
# (GLM_BUILD_LIBRARY=OFF), the version recon and gfx build against on desktop.
if(NOT VI_IOS)
  set(GLM_BUILD_LIBRARY
      OFF
      CACHE BOOL "" FORCE)
  set(GLM_BUILD_TESTS
      OFF
      CACHE BOOL "" FORCE)
  set(GLM_BUILD_INSTALL
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.3
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glm)
  enable_testing()
  add_subdirectory(apps/scanner/Core)
  add_subdirectory(tests)
  return()
endif()

# The core's answer, found here so that the imported glm::glm is visible to
# vi_core and to both siblings below: an imported target belongs to the
# directory that found it and those beneath it.
find_package(glm CONFIG REQUIRED)

# recon and gfx are pinned by commit, and adopted deliberately: a PR bumps a
# pin, and CI builds exactly that pair. Tracking `main` let an upstream change
# break this build with no commit here (volumetric_kit_core's DECISIONS.md,
# "Consumers pin"). A commit hash rules out GIT_SHALLOW, which takes only a
# branch or tag.
#
# Both build on volumetric_kit_core, the family's shared foundation, so it is
# declared first: FetchContent keeps the first declaration of a name, which
# makes this pin the one recon and gfx both build against (the core's README).
# Its vulkan tier is on, as gfx requires -- gfx refuses to configure against a
# core without it, or older than its own pin -- and recon's Device, Allocator
# and SharedDevice live there. Bump it with, or at or after, recon's and gfx's
# core pins: recon pins core #57 (0.22.0), this pin, and gfx core #50 (0.20.0).

FetchContent_Declare(
  volumetric_kit_recon
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_recon.git
  GIT_TAG 978f6d4fef4dd2355c590e51100c0b9a1ffa9708) # recon #293

# recon's VR_BUILD_EXAMPLES / VR_BUILD_TESTS default to PROJECT_IS_TOP_LEVEL, so
# a fetched copy builds the library tiers only -- no gtest, no example
# dataloader deps, nothing that would need a host-runnable binary. Nothing to
# force off here; that default is doing the work.
FetchContent_MakeAvailable(volumetric_kit_recon)

# gfx: the renderer. Its examples need GLFW and its tests need googletest, and
# both default to "am I top level?" -- OFF for a fetched copy -- so only the
# library tiers build. VG_WITH_GLFW gates examples only; the library tiers are
# GLFW-free by design, which is exactly why gfx ports to iOS untouched: its
# windowing tier takes a consumer-supplied surface rather than owning a window
# system.
FetchContent_Declare(
  volumetric_kit_gfx
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_gfx.git
  GIT_TAG 0106609fdbc395e3f733648382e4e86101c770b2) # gfx #158
set(VG_WITH_GLFW
    OFF
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

# Added from here rather than from apps/scanner/CMakeLists.txt, which is where
# it sits: the host build needs vi_core *without* the scanner, and the scanner
# cannot be configured at all without recon, gfx and MoltenVK above.
add_subdirectory(apps/scanner/Core)

add_subdirectory(apps/compute_smoke)
add_subdirectory(apps/scanner)

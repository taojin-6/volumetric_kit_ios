# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# iOS (arm64, device) cross-compilation toolchain for the volumetric_kit family.
#
# The whole point of this file: iOS needs exactly ONE thing a desktop build does
# not -- a Vulkan implementation. iOS ships no ICD loader and no libvulkan, so
# MoltenVK's static library *is* the Vulkan implementation and is linked
# directly (no loader indirection, and therefore no validation layers on
# device).
#
# volumetric_kit_core's iOS module (its cmake/vkc_ios.cmake, run as the core is
# added) links it: from MoltenVK_ROOT it sets Vulkan_LIBRARY /
# Vulkan_INCLUDE_DIR, so every find_package(Vulkan) in volumetric_kit_recon and
# volumetric_kit_gfx makes Vulkan::Vulkan from MoltenVK's iOS xcframework, and
# it links MoltenVK's system frameworks into the core's vulkan tier. This file
# names the platform, the MoltenVK the module takes, and the host tools.
#
# Usage:
#
# ~~~
#   tools/fetch_moltenvk.sh                       # once, populates third_party/
#   cmake -S . -B build-ios -G Xcode \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake
# ~~~
#
# Override VI_MOLTENVK_DIR to point at a MoltenVK from the LunarG SDK instead of
# the fetched copy.

set(CMAKE_SYSTEM_NAME iOS)

# Device only. MoltenVK's iOS release ships an ios-arm64 slice and no simulator
# slice -- and ARKit scene depth needs LiDAR hardware anyway, so the simulator
# was never a target.
set(CMAKE_OSX_ARCHITECTURES
    arm64
    CACHE STRING "iOS architectures")

# 26.0: every target device runs it or newer, and CI's macos-26 image (Xcode 26)
# builds for it.
set(CMAKE_OSX_DEPLOYMENT_TARGET
    26.0
    CACHE STRING "Minimum iOS version")

# --- MoltenVK: the Vulkan implementation ------------------------------------
# Default to the copy tools/fetch_moltenvk.sh unpacks into third_party/.
if(NOT VI_MOLTENVK_DIR)
  set(VI_MOLTENVK_DIR
      "${CMAKE_CURRENT_LIST_DIR}/../third_party/MoltenVK/MoltenVK"
      CACHE PATH "MoltenVK iOS distribution root (contains include/ + static/)")
endif()

# CMake re-runs this toolchain inside the throwaway projects it builds for
# compiler ABI detection, and those do NOT inherit the outer cache -- so a
# -DVI_MOLTENVK_DIR pointing outside the default location would be lost there
# and the existence check below would fail the configure before the real project
# was ever reached. Forwarding it keeps an explicit override working.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES VI_MOLTENVK_DIR)

set(_vi_mvk_lib
    "${VI_MOLTENVK_DIR}/static/MoltenVK.xcframework/ios-arm64/libMoltenVK.a")

if(NOT EXISTS "${_vi_mvk_lib}")
  message(
    FATAL_ERROR
      "MoltenVK for iOS not found at:\n  ${_vi_mvk_lib}\n"
      "Run tools/fetch_moltenvk.sh, or set -DVI_MOLTENVK_DIR=<MoltenVK root> "
      "(the directory containing include/ and static/).")
endif()

# The MoltenVK the core's iOS module links, which also checks its version.
set(MoltenVK_ROOT
    "${VI_MOLTENVK_DIR}"
    CACHE PATH "The MoltenVK/ directory of MoltenVK's iOS release")

# --- Host tooling ------------------------------------------------------------
# glslc compiles GLSL -> SPIR-V at build time and must be a *host* binary; under
# a cross toolchain a bare find_program would search the iOS sysroot. Both
# libraries' shader modules read Vulkan_GLSLC_EXECUTABLE, so resolve it here.
if(NOT Vulkan_GLSLC_EXECUTABLE)
  find_program(
    Vulkan_GLSLC_EXECUTABLE
    NAMES glslc
    HINTS /opt/homebrew/bin /usr/local/bin
    NO_CMAKE_FIND_ROOT_PATH)
  if(NOT Vulkan_GLSLC_EXECUTABLE)
    message(FATAL_ERROR "glslc not found on the host (macOS: `brew install "
                        "shaderc`). It compiles the compute shaders to SPIR-V.")
  endif()
endif()

# Host CMake package configs stay visible, which the iOS default (search the
# sysroot only) would hide; glm is the core's iOS module's answer, from the
# host's headers, before any search. Programs likewise resolve on the host --
# see glslc above.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
# Libraries and headers, by contrast, must come from the iOS sysroot only --
# linking a Homebrew macOS .dylib into an iOS binary is exactly the mistake this
# guards. MoltenVK is exempt: the core's module names it by absolute path.
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /usr/local)

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
# CMake's FindVulkan accepts a pre-seeded Vulkan_LIBRARY / Vulkan_INCLUDE_DIR
# and builds Vulkan::Vulkan from them, so seeding those with MoltenVK's iOS
# xcframework satisfies every find_package(Vulkan) in volumetric_kit_recon and
# volumetric_kit_gfx **without either repo changing a line**. That keeps the
# "independent siblings" rule intact: iOS support lives here, not in the
# libraries.
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

# 16.0: the floor that carries ARKit sceneDepth on every LiDAR device we target.
set(CMAKE_OSX_DEPLOYMENT_TARGET
    16.0
    CACHE STRING "Minimum iOS version")

# --- MoltenVK: the Vulkan implementation ------------------------------------
# Default to the copy tools/fetch_moltenvk.sh unpacks into third_party/.
if(NOT VI_MOLTENVK_DIR)
  set(VI_MOLTENVK_DIR
      "${CMAKE_CURRENT_LIST_DIR}/../third_party/MoltenVK/MoltenVK"
      CACHE PATH "MoltenVK iOS distribution root (contains include/ + static/)")
endif()

set(_vi_mvk_lib
    "${VI_MOLTENVK_DIR}/static/MoltenVK.xcframework/ios-arm64/libMoltenVK.a")
set(_vi_mvk_inc "${VI_MOLTENVK_DIR}/include")

if(NOT EXISTS "${_vi_mvk_lib}")
  message(
    FATAL_ERROR
      "MoltenVK for iOS not found at:\n  ${_vi_mvk_lib}\n"
      "Run tools/fetch_moltenvk.sh, or set -DVI_MOLTENVK_DIR=<MoltenVK root> "
      "(the directory containing include/ and static/).")
endif()

# Seed FindVulkan. It validates these, parses the version out of vulkan_core.h,
# and creates Vulkan::Vulkan -- so recon/gfx see an ordinary found Vulkan.
set(Vulkan_LIBRARY
    "${_vi_mvk_lib}"
    CACHE FILEPATH "MoltenVK static library (the iOS Vulkan implementation)")
set(Vulkan_INCLUDE_DIR
    "${_vi_mvk_inc}"
    CACHE PATH "Vulkan headers shipped with MoltenVK")

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

# Host CMake package configs must stay visible: recon and gfx both
# find_package(glm CONFIG), and glm is header-only, so the host copy is exactly
# right for an iOS target. The iOS default (search the sysroot only) would hide
# it. Programs likewise resolve on the host -- see glslc above.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
# Libraries and headers, by contrast, must come from the iOS sysroot only --
# linking a Homebrew macOS .dylib into an iOS binary is exactly the mistake this
# guards. MoltenVK is exempt because it is named by absolute path above.
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /usr/local)

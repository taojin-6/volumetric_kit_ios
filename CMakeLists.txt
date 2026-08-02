# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

cmake_minimum_required(VERSION 3.21)

project(
  volumetric_kit_ios
  VERSION 0.0.1
  DESCRIPTION
    "iOS application shell for the volumetric_kit family: ARKit capture -> volumetric_kit_recon fusion -> volumetric_kit_gfx rendering, on MoltenVK"
  HOMEPAGE_URL "https://github.com/taojin-6/volumetric_kit_ios"
  # Swift for the app shell and UI, Objective-C++ for the seam onto the C++
  # libraries. ARKit and UIKit are Swift-first, and the UI grows from here;
  # Objective-C++ stays the bridge because an .mm is the only translation unit
  # where an ARFrame and a C++ move-only type are both first-class.
  LANGUAGES CXX OBJCXX Swift)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "iOS")
  message(
    FATAL_ERROR
      "This project targets iOS only. Configure with:\n"
      "  cmake -S . -B build-ios -G Xcode "
      "-DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake")
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
# Neither needs an iOS-specific change -- the toolchain file's MoltenVK seeding
# is the whole of the platform port -- so these are ordinary pinned checkouts.
#
# For local development against a working tree, point CMake at it instead of the
# remote:
#
# ~~~
#   -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=/path/to/volumetric_kit_recon
# ~~~
include(FetchContent)

# --- glm, header-only ---------------------------------------------------------
# recon links the canonical `glm::glm` target rather than `glm::glm-header-only`
# -- deliberately, so older packagings that lack the header-only name (Ubuntu's
# libglm-dev) still resolve. On macOS that is fine; but Homebrew's glm 1.0.3
# defines `glm::glm` as a SHARED IMPORTED *macOS* dylib, which the iOS linker
# rightly refuses ("building for 'iOS', but linking in dylib built for
# 'macOS'").
#
# glm is header-only in this family's use, so the fix is to supply a glm whose
# `glm::glm` is an INTERFACE target: vendor it with GLM_BUILD_LIBRARY=OFF and
# let OVERRIDE_FIND_PACKAGE redirect recon's find_package(glm CONFIG REQUIRED)
# here instead of to the Homebrew copy. This belongs in the iOS consumer, not in
# recon -- recon's target choice is correct for its own portability.
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
  GIT_TAG 1.0.3 # matches the version recon and gfx build against on desktop
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)

FetchContent_Declare(
  volumetric_kit_recon
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_recon.git
  GIT_TAG main
  GIT_SHALLOW TRUE)

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
  GIT_TAG main
  GIT_SHALLOW TRUE)
set(VG_WITH_GLFW
    OFF
    CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

add_subdirectory(apps/compute_smoke)
add_subdirectory(apps/scanner)

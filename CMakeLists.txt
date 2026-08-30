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
# it: it is the scanner's pure logic, and it is the only thing in this repo a
# host can build or a test can run. Nothing else here is runnable off-device --
# MoltenVK ships no simulator slice and ARKit scene depth needs LiDAR -- so
# without this mode CI can assert that the app compiles and nothing more.
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

# --- Host mode: vi_core and its tests, and nothing else ----------------------
# Everything below this block needs recon, gfx and MoltenVK, none of which a
# host build wants or can use. So the host configure stops here.
#
# glm is made available explicitly because nothing in this mode asks for it:
# OVERRIDE_FIND_PACKAGE populates on a `find_package(glm)`, and the call that
# normally triggers it is recon's, further down.
if(NOT VI_IOS)
  FetchContent_MakeAvailable(glm)
  enable_testing()
  add_subdirectory(apps/scanner/Core)
  add_subdirectory(tests)
  return()
endif()

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

# Added from here rather than from apps/scanner/CMakeLists.txt, which is where
# it sits: the host build needs vi_core *without* the scanner, and the scanner
# cannot be configured at all without recon, gfx and MoltenVK above.
add_subdirectory(apps/scanner/Core)

add_subdirectory(apps/compute_smoke)
add_subdirectory(apps/scanner)

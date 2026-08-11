# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# scanner: the live reconstruction app.
#
# Two targets, deliberately. Swift and C++ in a *single* target under the Xcode
# generator is the fragile configuration; splitting the Objective-C++/C++ engine
# bridge into a static library and letting the Swift app link it through a
# bridging header is both more robust and how Apple lays these projects out
# anyway.
#
# ~~~
#   scanner_bridge  Objective-C++ + C++  -- owns Vulkan, recon, gfx
#   scanner         Swift                -- owns the app, view, lifecycle
# ~~~
#
# This step brings up gfx on a CAMetalLayer and draws a procedural triangle.
# ARKit capture and recon fusion land on top of the same seam.

set(VI_APP_NAME scanner)
set(VI_BUNDLE_ID "${VI_BUNDLE_PREFIX}.scanner")

# --- scanner_bridge: the Objective-C++ seam ----------------------------------
add_library(
  scanner_bridge STATIC
  Bridge/VolumetricRenderer.mm Bridge/ARKitCapture.mm Bridge/SharedDevice.mm
  Bridge/Fusion.mm Bridge/OrbitCamera.cpp Bridge/MemoryBudget.cpp)

# ARC, selected by language rather than by a second copy of the source list.
#
# The set_source_files_properties form this replaces named four of the six
# sources explicitly, seven lines below the list it had to agree with and with
# nothing deriving one from the other. Omitting a new Bridge/*.mm from it is
# silent: a translation unit compiled without -fobjc-arc is not an error and not
# a warning, it is manual retain/release, so every ARFrame, CVPixelBuffer and
# CAMetalDrawable that file touches simply leaks -- in the app whose documented
# worst failure is a jetsam SIGKILL with nothing in the log.
#
# COMPILE_LANGUAGE:OBJCXX is the discriminator the exclusion was really about:
# .cpp sources compile as CXX and never see the flag, so OrbitCamera.cpp and
# MemoryBudget.cpp stay out of it by construction rather than by being
# remembered. (Neither is excluded for reaching the OS through C headers -- an
# ARC translation unit may include <mach/mach.h> and <sys/sysctl.h> and call
# into them perfectly well. It is the file extension that decides.)
target_compile_options(scanner_bridge
                       PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)

# VK_USE_PLATFORM_METAL_EXT makes <vulkan/vulkan.h> pull in vulkan_metal.h, so
# VkMetalSurfaceCreateInfoEXT is declared. gfx's core/vulkan.hpp umbrella
# deliberately includes no platform defines -- naming the platform is the
# consumer's job, which is exactly this target.
target_compile_definitions(scanner_bridge PRIVATE VK_USE_PLATFORM_METAL_EXT)

# glm targets OpenGL's [-1, 1] clip depth; Vulkan's is [0, 1]. This switches
# glm::perspective over, and it must be a *target* definition rather than a
# #define in one file: glm is header-only, so the convention is fixed per
# translation unit at instantiation, and a TU that missed it would silently
# build a projection the depth buffer disagrees with.
#
# Not inherited from anywhere. gfx sets it PUBLIC on volumetric_kit::gfx_camera,
# which this target does not link -- gfx_pipelines pulls in gfx_core +
# gfx_assets, and gfx_app pulls in gfx_core + gfx_windowing. Without it the near
# plane lands at 2fn/(f+n) instead of n: ~10 cm rather than the 5 cm kNearClip
# promises, and the turntable's closest zoom sits inside the clip volume.
target_compile_definitions(scanner_bridge PRIVATE GLM_FORCE_DEPTH_ZERO_TO_ONE)

# The triangle's SPIR-V is embedded as a constexpr byte array, matching how the
# library tiers ship their kernels: no runtime file, nothing to resolve in the
# bundle. Reuses recon's embed module (gfx has an identical vg_embed_shaders).
list(APPEND CMAKE_MODULE_PATH "${volumetric_kit_recon_SOURCE_DIR}/cmake")
include(vr_embed)
vr_embed_shaders(
  scanner_bridge SYMBOL_PREFIX "vi_" SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.vert"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.frag")

target_include_directories(scanner_bridge
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Bridge")

# recon_sensor carries the capture contract and the camera-convention
# conversions. Since the camera parameter blocks moved into core it links
# recon_core and nothing else, so the ARKit driver pulls in no fusion tier just
# to implement ICameraCapture; recon_tsdf and recon_texture below are what the
# fusion spine consumes, and recon_mesh/recon_volume arrive transitively. Build
# the scanner as a measurement binary: one mesh slot, sharing off, spans
# tracked, and NO geometry published, so recon's incremental extract can be
# timed at the device's own dirty fraction. Compile-time rather than runtime
# because iOS resumes a process instead of cold-starting it, so an environment
# variable survives exactly one launch and then lies. See
# FusionConfig::incremental_benchmark.
option(VI_INCREMENTAL_BENCHMARK
       "Scanner measures incremental extraction and draws no mesh" OFF)
if(VI_INCREMENTAL_BENCHMARK)
  # On the bridge, which is where VolumetricRenderer.mm compiles.
  target_compile_definitions(scanner_bridge PRIVATE VI_INCREMENTAL_BENCHMARK)
endif()

target_link_libraries(
  scanner_bridge
  PUBLIC volumetric_kit::gfx_app volumetric_kit::gfx_core
         volumetric_kit::recon_sensor volumetric_kit::recon_texture
         volumetric_kit::recon_tsdf volumetric_kit::gfx_pipelines)

# --- scanner: the Swift app --------------------------------------------------
add_executable(
  ${VI_APP_NAME} MACOSX_BUNDLE
  Swift/AppDelegate.swift Swift/ARSessionController.swift
  Swift/CameraGestureController.swift Swift/Dashboard.swift
  Swift/MetalView.swift Swift/ScannerViewController.swift)

target_link_libraries(${VI_APP_NAME} PRIVATE scanner_bridge)

# MoltenVK is a static archive, so its platform dependencies resolve at the app
# link, not the library's. Metal is the translation target; QuartzCore provides
# CAMetalLayer; IOSurface and CoreVideo back the CVPixelBuffer path the ARKit
# slice will need.
target_link_libraries(
  ${VI_APP_NAME}
  PRIVATE "-framework Metal"
          "-framework Foundation"
          "-framework QuartzCore"
          "-framework IOSurface"
          "-framework CoreGraphics"
          "-framework CoreVideo"
          # ARKit supplies the posed depth/colour frames; Accelerate's vImage
          # does the YCbCr -> RGB conversion, which at 1920x1440 would dominate
          # the capture path as a scalar loop.
          "-framework ARKit"
          "-framework Accelerate"
          "-framework UIKit")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
               "${CMAKE_CURRENT_BINARY_DIR}/Info.plist" @ONLY)

set_target_properties(
  ${VI_APP_NAME}
  PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_BINARY_DIR}/Info.plist"
             XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${VI_BUNDLE_ID}"
             # ~~~
             # Universal (1 = iPhone, 2 = iPad). This must be the *build
             # setting*, not a UIDeviceFamily key in Info.plist.in: Xcode
             # rewrites that key from TARGETED_DEVICE_FAMILY while processing
             # the plist, so a value set there is silently overwritten. Left at
             # the iPhone-only default, an iPad runs the app in compatibility
             # mode and hands the CAMetalLayer a 750x1334 letterboxed drawable
             # instead of the real display.
             # ~~~
             XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
             XCODE_ATTRIBUTE_SWIFT_VERSION "5.0"
             # The header that makes VolumetricRenderer visible to Swift.
             XCODE_ATTRIBUTE_SWIFT_OBJC_BRIDGING_HEADER
             "${CMAKE_CURRENT_SOURCE_DIR}/Bridge/Scanner-Bridging-Header.h"
             # A Swift target linking C++ needs the C++ standard library named
             # explicitly; the Swift driver does not infer it from the linked
             # archive.
             XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++"
             XCODE_ATTRIBUTE_CODE_SIGN_STYLE "Automatic"
             XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${VI_DEVELOPMENT_TEAM}"
             XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "Apple Development"
             # ~~~
             # increased-memory-limit. A live 1 cm reconstruction holds
             # gigabytes: the run that motivated this held 3 GB of mesh arena
             # beside a 0.8 GB voxel grid and was SIGKILLed by jetsam, which
             # reports nothing -- the process just disappears. See
             # scanner.entitlements, which also records why
             # extended-virtual-addressing is *not* here (a personal development
             # team cannot provision it, and its presence fails the build rather
             # than being dropped).
             #
             # Capability entitlements fail signing loudly when the App ID lacks
             # them, which is the good outcome -- a silently dropped entitlement
             # would look like a fix that quietly is not one. So a build that
             # runs at all is already carrying this one; to confirm it on the
             # binary rather than infer it from a running app:
             #
             # codesign -d --entitlements - <path to scanner.app>
             #
             # That command lives here rather than in scanner.entitlements
             # because a doubled hyphen cannot appear inside an XML comment --
             # see the note at the top of that file.
             # ~~~
             XCODE_ATTRIBUTE_CODE_SIGN_ENTITLEMENTS
             "${CMAKE_CURRENT_SOURCE_DIR}/scanner.entitlements")

if(NOT VI_DEVELOPMENT_TEAM)
  message(
    WARNING
      "VI_DEVELOPMENT_TEAM is unset: ${VI_APP_NAME} will build but cannot be "
      "signed for or installed on hardware. Pass -DVI_DEVELOPMENT_TEAM=<team id>."
  )
endif()

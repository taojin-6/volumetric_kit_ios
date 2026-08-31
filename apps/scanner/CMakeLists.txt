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
  Bridge/VolumetricRenderer.mm
  Bridge/AllocationStopDisplay.mm
  Bridge/ARKitCapture.mm
  Bridge/AtlasRing.cpp
  Bridge/BridgeStrings.mm
  Bridge/FrameTrace.cpp
  Bridge/Fusion.mm
  Bridge/MemoryQuery.cpp
  Bridge/RendererErrors.mm
  Bridge/SharedDevice.mm)

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
# Both lines, because the two generators take different routes and the failure
# mode is the same silence the source list had.
#
# CMake's Xcode generator DROPS a COMPILE_LANGUAGE genex in
# target_compile_options -- it has no per-language flag setting to put it in --
# so the genex alone reached no file in this target at all. Nothing said so:
# every .mm here had been compiling as manual retain/release, with no dealloc
# anywhere in Bridge/ and the one that exists written for ARC's ordering. The
# Xcode build setting is what actually arrives, and Xcode applies it only to the
# ObjC dialects, so MemoryQuery.cpp and FrameTrace.cpp still stay out by
# construction rather than by being remembered -- as does every source in
# vi_core. (They are not excluded for reaching the OS through C headers -- an
# ARC translation unit may include <mach/mach.h> and call into it perfectly
# well. It is the file extension that decides.)
#
# The genex stays for the generators that honour it: this repo configures with
# -G Xcode, but a Ninja or Makefile configure that silently lost ARC again is
# exactly the failure being fixed, and it costs one line to not have it.
set_target_properties(scanner_bridge
                      PROPERTIES XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES)
target_compile_options(scanner_bridge
                       PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)

# The warning set the sibling libraries build under, which this target had none
# of -- not -Wall, not -Wextra, not -Werror.
#
# -Werror=switch is the load-bearing one. Bridge/AllocationStopDisplay.hpp and
# Bridge/RendererErrors.hpp both promise that a value added upstream stops the
# compile, and both siblings track main rather than a pin. As a bare warning
# that promise is false: a new gfx or recon Status::Code links cleanly and
# reports itself to Swift as VolumetricRendererErrorUnknown, and a new
# AllocationStop cause reports a scan that has stopped taking geometry as a
# healthy one on every rendering at once. Core/AllocationStop.cpp carries the
# other half of the same guarantee and sets the same flags.
target_compile_options(scanner_bridge PRIVATE -Wall -Wextra -Werror=switch)

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
# Now also inherited from vi_core, which sets it PUBLIC for the same reason, and
# kept here regardless: this target is where `glm::perspective` is actually
# called, so it is where the convention has to hold whatever it links. It is not
# inherited from the libraries -- gfx sets it PUBLIC on
# volumetric_kit::gfx_camera, which this target does not link (gfx_pipelines
# pulls in gfx_core + gfx_assets, and gfx_app pulls in gfx_core +
# gfx_windowing). Without it the near plane lands at 2fn/(f+n) instead of n: ~10
# cm rather than the 5 cm kNearClip promises, and the turntable's closest zoom
# sits inside the clip volume.
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
# the scanner as a measurement binary: one mesh slot, spans tracked, and NO
# geometry published, so recon's incremental extract can be timed at the
# device's own dirty fraction. Vertex sharing stays ON -- unsharing is what the
# first version did and it cost 3x the vertex arena, which is a settled revert,
# not a knob. Compile-time rather than runtime because iOS resumes a process
# instead of cold-starting it, so an environment variable survives exactly one
# launch and then lies. See FusionConfig::incremental_benchmark.
#
# A define cannot drift across a *resume*, which is the failure the environment
# variable had. It can still drift across a CONFIGURE: option() writes a CACHE
# entry, so -DVI_INCREMENTAL_BENCHMARK=ON latches into the build tree and every
# later cmake in that directory keeps it -- including the plain configure line
# in README.md, which names no such flag and so cannot clear it. That is a worse
# failure than the one it replaced if it goes unnoticed, because it turns an
# ordinary app build into one that publishes no geometry: a scanner that renders
# an empty room and looks broken.
#
# So it announces itself. Every configure of a tree carrying it prints this, and
# the message names the way out, because the natural repair -- re-run the
# configure line from the README -- is exactly the one that does not work.
option(VI_INCREMENTAL_BENCHMARK
       "Scanner measures incremental extraction and draws no mesh" OFF)
if(VI_INCREMENTAL_BENCHMARK)
  message(
    WARNING
      "VI_INCREMENTAL_BENCHMARK is ON: this scanner build MEASURES incremental "
      "extraction and publishes NO geometry, so it renders an empty scene by "
      "design. It is cached in this build tree and survives a plain re-configure"
      " -- clear it with -DVI_INCREMENTAL_BENCHMARK=OFF.")
  # On the bridge, which is where VolumetricRenderer.mm compiles.
  target_compile_definitions(scanner_bridge PRIVATE VI_INCREMENTAL_BENCHMARK)
endif()

# vi_core is PUBLIC because the bridge's own headers expose it: the renderer
# holds an OrbitCamera by value and hands MemoryBudget across the seam.
target_link_libraries(
  scanner_bridge
  PUBLIC vi_core
         volumetric_kit::gfx_app
         volumetric_kit::gfx_core
         volumetric_kit::recon_sensor
         volumetric_kit::recon_texture
         volumetric_kit::recon_tsdf
         volumetric_kit::gfx_pipelines)

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

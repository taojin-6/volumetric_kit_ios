# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# scanner: the live reconstruction app.
#
# The app uses two targets. Swift and C++ in a *single* target under the Xcode
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

# The bridge's shaders, in a target of their own so the generated symbols and
# build rules have one owner under Xcode as well as Make/Ninja.
add_library(vi_scanner_shaders INTERFACE)
vkc_embed_shaders(
  vi_scanner_shaders SYMBOL_PREFIX "vi_" SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.vert"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.frag")

# --- scanner_bridge: the Objective-C++ seam ----------------------------------
function(vi_add_scanner_bridge bridge_target)
  add_library(
    ${bridge_target} STATIC
    ${ARGN}
    Bridge/VolumetricRenderer.mm
    Bridge/AllocationStopDisplay.mm
    Bridge/ARKitCapture.mm
    Bridge/AtlasRing.cpp
    Bridge/BridgeStrings.mm
    Bridge/FrameTrace.cpp
    Bridge/Fusion.mm
    Bridge/MemoryQuery.cpp
    Bridge/Readout.mm
    Bridge/RendererErrors.mm)

  # ARC, selected by language rather than by a second copy of the source list.
  #
  # The set_source_files_properties form this replaces named four of the six
  # sources explicitly, seven lines below the list it had to agree with and with
  # nothing deriving one from the other. Omitting a new Bridge/*.mm from it is
  # silent: a translation unit compiled without -fobjc-arc is not an error and
  # not a warning, it is manual retain/release, so every ARFrame, CVPixelBuffer
  # and CAMetalDrawable that file touches simply leaks -- in the app whose
  # documented worst failure is a jetsam SIGKILL with nothing in the log.
  #
  # Both lines, because the two generators take different routes and the failure
  # mode is the same silence the source list had.
  #
  # CMake's Xcode generator DROPS a COMPILE_LANGUAGE genex in
  # target_compile_options -- it has no per-language flag setting to put it in
  # -- so the genex alone reached no file in this target at all. Nothing said
  # so: every .mm here had been compiling as manual retain/release, with no
  # dealloc anywhere in Bridge/ and the one that exists written for ARC's
  # ordering. The Xcode build setting is what actually arrives, and Xcode
  # applies it only to the ObjC dialects, so MemoryQuery.cpp and FrameTrace.cpp
  # still stay out by construction rather than by being remembered -- as does
  # every source in vi_core. (They are not excluded for reaching the OS through
  # C headers -- an ARC translation unit may include <mach/mach.h> and call into
  # it perfectly well. It is the file extension that decides.)
  #
  # The genex stays for the generators that honour it: this repo configures with
  # -G Xcode, but a Ninja or Makefile configure that silently lost ARC again is
  # exactly the failure being fixed, and it costs one line to not have it.
  set_target_properties(${bridge_target}
                        PROPERTIES XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES)
  target_compile_options(${bridge_target}
                         PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)

  # The warning set the sibling libraries build under, which this target had
  # none of -- not -Wall, not -Wextra, not -Werror.
  #
  # -Werror=switch is the load-bearing one. Bridge/AllocationStopDisplay.hpp and
  # Bridge/RendererErrors.hpp both promise that a value added upstream stops the
  # compile -- the pin bump that brings it in, rather than a build that drifts.
  # As a bare warning that promise is false: a new core Status::Code links
  # cleanly and reports itself to Swift as VolumetricRendererErrorUnknown, and a
  # new AllocationStop cause reports a scan that has stopped taking geometry as
  # a healthy one on every rendering at once. Core/AllocationStop.cpp carries
  # the other half of the same guarantee and sets the same flags.
  target_compile_options(${bridge_target} PRIVATE -Wall -Wextra -Werror=switch)

  # VK_USE_PLATFORM_METAL_EXT makes <vulkan/vulkan.h> pull in vulkan_metal.h, so
  # VkMetalSurfaceCreateInfoEXT is declared. The core's
  # volumetric_kit/core/vulkan/vulkan.hpp umbrella deliberately includes no
  # platform defines -- naming the platform is the consumer's job, which is
  # exactly this target.
  target_compile_definitions(${bridge_target} PRIVATE VK_USE_PLATFORM_METAL_EXT)

  # glm targets OpenGL's [-1, 1] clip depth; Vulkan's is [0, 1]. This switches
  # glm::perspective over, and it must be a *target* definition rather than a
  # #define in one file: glm is header-only, so the convention is fixed per
  # translation unit at instantiation, and a TU that missed it would silently
  # build a projection the depth buffer disagrees with.
  #
  # Now also inherited from vi_core, which sets it PUBLIC for the same reason,
  # and kept here regardless: this target is where `glm::perspective` is
  # actually called, so it is where the convention has to hold whatever it
  # links. It is not inherited from the libraries -- gfx sets it PUBLIC on
  # volumetric_kit::gfx_camera, which this target does not link (gfx_pipelines
  # pulls in gfx_core + gfx_assets, and gfx_app pulls in gfx_core +
  # gfx_windowing). Without it the near plane lands at 2fn/(f+n) instead of n:
  # ~10 cm rather than the 5 cm kNearClip promises, and the turntable's closest
  # zoom sits inside the clip volume.
  target_compile_definitions(${bridge_target}
                             PRIVATE GLM_FORCE_DEPTH_ZERO_TO_ONE)

  target_include_directories(${bridge_target}
                             PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Bridge")

  # recon_sensor carries the camera and colour conversions ARKitCapture applies.
  # recon_tsdf and recon_texture supply fusion; recon_mesh and recon_volume
  # arrive transitively.
  #
  # vi_core is PUBLIC because the bridge's own headers expose it: the renderer
  # holds an OrbitCamera by value and hands MemoryBudget across the seam. The
  # core's vulkan tier is named too, not left to recon's and gfx's links: the
  # bridge builds the SharedDevice, adopts recon's Device and makes its
  # Allocator itself.
  target_link_libraries(
    ${bridge_target}
    PUBLIC vi_core
           volumetric_kit::core_vulkan
           volumetric_kit::gfx_app
           volumetric_kit::gfx_core
           volumetric_kit::recon_sensor
           volumetric_kit::recon_texture
           volumetric_kit::recon_tsdf
           volumetric_kit::gfx_pipelines
    PRIVATE volumetric_kit::core_config vi_scanner_shaders)
endfunction()

vi_add_scanner_bridge(scanner_bridge)

# --- scanner: the Swift app --------------------------------------------------
add_executable(
  ${VI_APP_NAME} MACOSX_BUNDLE
  Swift/AppDelegate.swift
  Swift/ARSessionController.swift
  Swift/CameraGestureController.swift
  Swift/Dashboard.swift
  Swift/MetalView.swift
  Swift/ScannerViewController.swift
  Swift/SceneDelegate.swift)

target_link_libraries(${VI_APP_NAME} PRIVATE scanner_bridge)

set_source_files_properties(config/scanner.json
                            PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
target_sources(${VI_APP_NAME} PRIVATE config/scanner.json)

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

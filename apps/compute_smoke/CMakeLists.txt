# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# compute_smoke: the on-device de-risk gate.
#
# Runs volumetric_kit_recon's compute path against MoltenVK on a real iOS GPU
# and shows the result. This is the family's standing "prove the path before
# building on it" rule applied to iOS -- the app that has to pass before an
# ARKit capture tier is worth writing.

set(VI_APP_NAME compute_smoke)
set(VI_BUNDLE_ID "${VI_BUNDLE_PREFIX}.computesmoke")

add_executable(${VI_APP_NAME} MACOSX_BUNDLE main.mm smoke_report.cpp)

# Objective-C++ for main.mm: it is the UIKit shell, and the same C++/Obj-C seam
# the ARKit capture source will use.
set_source_files_properties(main.mm PROPERTIES COMPILE_FLAGS "-fobjc-arc")

# The shader is baked into the binary as a constexpr byte array (recon's
# vr_embed_shaders, reused here) rather than shipped as a bundle resource: an
# app that carries its own SPIR-V has no runtime file to find, no bundle path to
# resolve, and no way to be installed half-configured.
list(APPEND CMAKE_MODULE_PATH "${volumetric_kit_recon_SOURCE_DIR}/cmake")
include(vr_embed)
vr_embed_shaders(${VI_APP_NAME} SYMBOL_PREFIX "vi_" SHADERS
                 "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fill.comp")

target_include_directories(${VI_APP_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

# Every recon tier the smoke exercises. recon_core comes in transitively, but
# the stages name volume/tsdf/mesh directly, so link them directly.
target_link_libraries(
  ${VI_APP_NAME} PRIVATE volumetric_kit::recon_volume volumetric_kit::recon_tsdf
                         volumetric_kit::recon_mesh)

# MoltenVK is a static archive, so its own platform dependencies do not come
# along automatically the way a dylib's would -- the app is where they resolve.
# Metal is the translation target; IOSurface and QuartzCore back the
# CAMetalLayer/CVPixelBuffer paths the ARKit slice will need; Foundation and
# UIKit are the shell.
target_link_libraries(
  ${VI_APP_NAME}
  PRIVATE "-framework Metal" "-framework Foundation" "-framework QuartzCore"
          "-framework IOSurface" "-framework CoreGraphics" "-framework UIKit")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
               "${CMAKE_CURRENT_BINARY_DIR}/Info.plist" @ONLY)

set_target_properties(
  ${VI_APP_NAME}
  PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_BINARY_DIR}/Info.plist"
             XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${VI_BUNDLE_ID}"
             # Automatic signing: Xcode provisions the app against the team
             # below on first build, so no checked-in provisioning profile.
             XCODE_ATTRIBUTE_CODE_SIGN_STYLE "Automatic"
             XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${VI_DEVELOPMENT_TEAM}"
             XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "Apple Development")

if(NOT VI_DEVELOPMENT_TEAM)
  message(
    WARNING
      "VI_DEVELOPMENT_TEAM is unset: ${VI_APP_NAME} will build but cannot be "
      "signed for or installed on hardware. Pass -DVI_DEVELOPMENT_TEAM=<team id>."
  )
endif()

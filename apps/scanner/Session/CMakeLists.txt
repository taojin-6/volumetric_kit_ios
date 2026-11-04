# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vi_session: a sensor's frames as a session, written to a .vrst file and served
# live to a reader the writer dials; and the on-screen entry that names the
# reader.
#
# Host code, typed only on the siblings' public types: sensor's SensorInfo and
# RgbdFrame and its wire form (sensor_wire), grid's .vrst container
# (grid_stream) and server (grid_networking), the core's addresses and dial
# (core_net). It takes the caller's coded picture and decodes nothing, so it
# needs no encoder, no ARKit and no device, and its tests run on the host beside
# vi_core's, the link's on 127.0.0.1. Not in vi_core, which links no sibling
# library: sensor's interface reaches the Vulkan headers through its frame tier.
add_library(vi_session STATIC dial_entry.cpp rgbd_stream_writer.cpp)

# The warning set vi_core builds under.
target_compile_options(vi_session PRIVATE -Wall -Wextra -Wpedantic -Werror)

target_include_directories(vi_session PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Threads REQUIRED)

# PUBLIC: the headers take sensor's frame and colour codes, return the core's
# Result, and name grid's default port. Behind them: the file it writes, the
# core's dial and address checks, and the dial thread.
target_link_libraries(
  vi_session
  PUBLIC volumetric_kit::sensor_interface volumetric_kit::sensor_video_syntax
         volumetric_kit::grid_networking
  PRIVATE volumetric_kit::sensor_wire volumetric_kit::grid_stream
          volumetric_kit::core_net Threads::Threads)

# vi_stream: a camera's frames sent as the session above (rgbd_stream_sender):
# the rate gate and map steps (vi_core), each picture coded on the hardware as
# the camera left it (sensor_video's HevcEncoder through send_pixel_buffer,
# sensor_apple's pixel_buffer_of and color_codes_of), then the frame written.
# Apple only, as VideoToolbox is; built on the host too, where its tests run on
# the Mac's GPU and encoder.
if(APPLE)
  add_library(vi_stream STATIC rgbd_stream_sender.cpp)
  target_compile_options(vi_stream PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_include_directories(vi_stream PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  # PUBLIC: the header takes the writer's options and sensor's encoder fault.
  target_link_libraries(
    vi_stream
    PUBLIC vi_session volumetric_kit::sensor_video
    PRIVATE vi_core volumetric_kit::sensor_apple Threads::Threads
            "-framework CoreVideo")
endif()

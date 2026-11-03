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

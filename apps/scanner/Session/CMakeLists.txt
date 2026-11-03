# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vi_session: a sensor's frames as a session, written to a .vrst file.
#
# Host code, typed only on the siblings' public types: sensor's SensorInfo and
# RgbdFrame and its wire form (sensor_wire), grid's .vrst container
# (grid_stream). It takes the caller's coded picture and decodes nothing, so it
# needs no encoder, no ARKit and no device, and its tests run on the host beside
# vi_core's. Not in vi_core, which links no sibling library: sensor's interface
# reaches the Vulkan headers through its frame tier.
add_library(vi_session STATIC rgbd_stream_writer.cpp)

# The warning set vi_core builds under.
target_compile_options(vi_session PRIVATE -Wall -Wextra -Wpedantic -Werror)

target_include_directories(vi_session PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# PUBLIC: the header takes sensor's frame and colour codes and returns the
# core's Result. grid_stream is the file it writes, behind the header.
target_link_libraries(
  vi_session
  PUBLIC volumetric_kit::sensor_interface volumetric_kit::sensor_video_syntax
  PRIVATE volumetric_kit::sensor_wire volumetric_kit::grid_stream)

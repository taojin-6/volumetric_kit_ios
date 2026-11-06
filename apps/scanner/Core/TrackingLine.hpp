// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file TrackingLine.hpp
/// @brief What the tracking chip says, and whether the stream chip says paused.
///
/// The camera hands out only the frames ARKit tracked normally, so the stream
/// stops whenever ARKit does not, or has the camera stopped. The picture may
/// go on meanwhile, so the panel has to say why nothing is going out.

#include <string>
#include <string_view>

#include "StatTone.hpp"

namespace volumetric_kit::ios_app {

/// @brief The chip's words, how they read, and the stream's state.
struct TrackingLine {
  std::string text;
  StatTone tone = StatTone::Neutral;
  /// A stream is open and the camera gives it nothing now.
  bool stream_paused = false;
};

/// @param tracking     ARKit's tracking at the last frame, as the console lines
///                     name it (`to_string(ArkitTracking)`).
/// @param interrupted  ARKit has stopped the camera for a while, whatever the
///                     last frame's tracking was.
/// @param stream_open  The stream's sender opened.
/// @return "normal" at Good, frames going out; otherwise short words at Warn,
///         a name it does not know as it came.
TrackingLine tracking_line(std::string_view tracking, bool interrupted,
                           bool stream_open);

}  // namespace volumetric_kit::ios_app

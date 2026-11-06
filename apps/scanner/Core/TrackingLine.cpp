// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "TrackingLine.hpp"

namespace volumetric_kit::ios_app {
namespace {

std::string lost_words(std::string_view tracking, bool interrupted) {
  if (interrupted) return "Camera interrupted";
  if (tracking == "relocalizing") return "Relocalizing";
  if (tracking == "initializing") return "Initializing";
  if (tracking == "excessive motion") return "Moving too fast";
  if (tracking == "insufficient features") return "Too little in view";
  if (tracking == "not available") return "No tracking";
  return std::string(tracking);
}

}  // namespace

TrackingLine tracking_line(std::string_view tracking, bool interrupted,
                           bool stream_open) {
  if (!interrupted && tracking == "normal") {
    return {"normal", StatTone::Good, false};
  }
  return {lost_words(tracking, interrupted), StatTone::Warn, stream_open};
}

}  // namespace volumetric_kit::ios_app

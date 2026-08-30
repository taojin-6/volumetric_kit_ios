// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "ViewOrientation.hpp"

#include <glm/gtc/constants.hpp>

namespace volumetric_kit::ios_app {
namespace {

// `viewport_turn` subtracts two of these raw values and reads the difference as
// a count of quarter turns, which means something only while they are
// consecutive and in this order. Pinned rather than assumed: reordering them is
// an ordinary-looking edit that would otherwise turn every scan by a silent
// multiple of 90 degrees.
//
// The Objective-C mirror is pinned to these in VolumetricRenderer.mm, so the
// two enums cannot drift apart either.
static_assert(static_cast<int>(ViewOrientation::LandscapeLeft) == 0,
              "ViewOrientation raw values are quarter turns; "
              "landscape-left must be 0");
static_assert(static_cast<int>(ViewOrientation::Portrait) == 1,
              "ViewOrientation raw values are quarter turns; "
              "portrait must be 1");
static_assert(static_cast<int>(ViewOrientation::LandscapeRight) == 2,
              "ViewOrientation raw values are quarter turns; "
              "landscape-right must be 2");
static_assert(static_cast<int>(ViewOrientation::PortraitUpsideDown) == 3,
              "ViewOrientation raw values are quarter turns; "
              "upside-down must be 3");

}  // namespace

float viewport_turn(ViewOrientation orientation) noexcept {
  const auto quarter_turns =
      static_cast<float>(static_cast<int>(orientation) -
                         static_cast<int>(kSensorBasisOrientation));
  return -quarter_turns * glm::half_pi<float>();
}

}  // namespace volumetric_kit::ios_app

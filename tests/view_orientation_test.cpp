// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file view_orientation_test.cpp
/// @brief Pins the sensor-basis-to-viewport mapping.
///
/// This mapping has been wrong twice, in two different directions, and was
/// settled only by a measurement on an iPad Pro M5 on 2026-08-10 -- see
/// ViewOrientation.hpp, which records what was measured and why it took two
/// orientations to do it. These tests are that measurement written down where a
/// change has to get past it.
///
/// They deliberately assert the *numbers*, not the formula. Re-deriving the
/// expected value from `kSensorBasisOrientation` would pass for any zero and
/// any sign, which is exactly the class of mistake that has been made here.

#include "ViewOrientation.hpp"

#include <cmath>

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>

namespace {

namespace app = volumetric_kit::ios_app;
using app::ViewOrientation;
using app::viewport_turn;

constexpr float kQuarter = glm::half_pi<float>();

/// **Landscape-right pins the zero.** It is the orientation
/// `kSensorBasisOrientation` names, so the turn applied there is 0 -- and the
/// device rendering upright there is the statement that the sensor basis and
/// the viewport genuinely coincide. The build before the measured one turned
/// 180 degrees here and was upside down.
TEST(ViewOrientation, LandscapeRightIsTheZero) {
  EXPECT_FLOAT_EQ(viewport_turn(ViewOrientation::LandscapeRight), 0.0f);
}

/// **Portrait pins the step**, and it is the only orientation that can: the
/// turn is +-90 x (raw - 2), so at both landscapes it lands on 0 or 180, each
/// its own negation. The two landscapes therefore look identical whichever way
/// the step runs, and a sign error there is invisible by construction. Portrait
/// is where the two differ, and on device it came up upright at +90.
TEST(ViewOrientation, PortraitIsAQuarterTurnPositive) {
  EXPECT_FLOAT_EQ(viewport_turn(ViewOrientation::Portrait), kQuarter);
}

/// Follows from the zero and the step rather than resting on the prose, but
/// asserted because it is what the single expression actually produces.
TEST(ViewOrientation, LandscapeLeftIsHalfATurn) {
  EXPECT_FLOAT_EQ(viewport_turn(ViewOrientation::LandscapeLeft),
                  glm::pi<float>());
}

/// Unreachable in the app -- Info.plist declares Portrait, LandscapeLeft and
/// LandscapeRight only -- so this is the one row the device could not test.
/// Asserted here because a host can reach it and the expression covers it.
TEST(ViewOrientation, UpsideDownIsAQuarterTurnNegative) {
  EXPECT_FLOAT_EQ(viewport_turn(ViewOrientation::PortraitUpsideDown),
                  -kQuarter);
}

/// The step must run the *same* way across the whole enum: consecutive
/// orientations differ by exactly one quarter turn, in one direction. A mapping
/// that got the zero right and the step backwards would pass the landscape
/// tests above and fail here.
TEST(ViewOrientation, StepIsOneQuarterTurnThroughout) {
  const ViewOrientation order[] = {
      ViewOrientation::LandscapeLeft,
      ViewOrientation::Portrait,
      ViewOrientation::LandscapeRight,
      ViewOrientation::PortraitUpsideDown,
  };
  for (int i = 1; i < 4; ++i) {
    EXPECT_FLOAT_EQ(viewport_turn(order[i]) - viewport_turn(order[i - 1]),
                    -kQuarter)
        << "step from index " << (i - 1) << " to " << i;
  }
}

/// The raw values are load-bearing: `viewport_turn` subtracts two of them and
/// reads the difference as a count of quarter turns. ViewOrientation.cpp
/// static_asserts this at compile time; asserted again here so the reason is
/// visible to someone reading the tests rather than only to someone who breaks
/// the build.
TEST(ViewOrientation, RawValuesAreQuarterTurns) {
  EXPECT_EQ(static_cast<int>(ViewOrientation::LandscapeLeft), 0);
  EXPECT_EQ(static_cast<int>(ViewOrientation::Portrait), 1);
  EXPECT_EQ(static_cast<int>(ViewOrientation::LandscapeRight), 2);
  EXPECT_EQ(static_cast<int>(ViewOrientation::PortraitUpsideDown), 3);
}

}  // namespace

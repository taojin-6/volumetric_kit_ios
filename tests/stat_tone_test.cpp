// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file stat_tone_test.cpp
/// @brief The one rule every meter in the app shares about when a number stops
///        being routine.

#include "StatTone.hpp"

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::StatTone;
using app::tone_for;

TEST(StatTone, BelowWarnIsNeutral) {
  EXPECT_EQ(tone_for(0.0, 0.75, 0.9), StatTone::Neutral);
  EXPECT_EQ(tone_for(0.5, 0.75, 0.9), StatTone::Neutral);
  EXPECT_EQ(tone_for(0.7499, 0.75, 0.9), StatTone::Neutral);
}

/// Inclusive on the way up: a gauge sitting exactly on its threshold is the
/// case the threshold exists to catch, not one to round past.
TEST(StatTone, ThresholdsAreInclusive) {
  EXPECT_EQ(tone_for(0.75, 0.75, 0.9), StatTone::Warn);
  EXPECT_EQ(tone_for(0.9, 0.75, 0.9), StatTone::Critical);
}

TEST(StatTone, BetweenThresholdsIsWarn) {
  EXPECT_EQ(tone_for(0.8, 0.75, 0.9), StatTone::Warn);
}

/// Critical is checked first, so it wins wherever the two overlap -- including
/// a caller that passes them the wrong way round, which must not downgrade an
/// alarm to a warning.
TEST(StatTone, CriticalWinsWhenThresholdsOverlap) {
  EXPECT_EQ(tone_for(0.95, 0.9, 0.9), StatTone::Critical);
  EXPECT_EQ(tone_for(0.95, 0.95, 0.9), StatTone::Critical);
}

/// Not clamped. A fraction above 1.0 is a real reading on the counters this
/// serves -- a footprint measured against a limit that has since moved -- and
/// it must report Critical rather than being treated as invalid input.
TEST(StatTone, OverfullIsCritical) {
  EXPECT_EQ(tone_for(1.0, 0.75, 0.9), StatTone::Critical);
  EXPECT_EQ(tone_for(12.5, 0.75, 0.9), StatTone::Critical);
}

/// Usable in a constant expression, which is what lets a caller fold a
/// threshold decision into a static table without a second implementation.
TEST(StatTone, IsConstexpr) {
  static_assert(tone_for(0.5, 0.75, 0.9) == StatTone::Neutral);
  static_assert(tone_for(0.8, 0.75, 0.9) == StatTone::Warn);
  static_assert(tone_for(0.99, 0.75, 0.9) == StatTone::Critical);
  SUCCEED();
}

}  // namespace

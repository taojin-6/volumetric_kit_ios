// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file StatTone.hpp
/// @brief When a number on the read-out stops being routine.
///
/// Pure, so the thresholds every meter in the app shares can be tested on the
/// host. The Objective-C `VolumetricStatTone` mirrors @ref StatTone value for
/// value and `VolumetricRenderer.mm` static_asserts the two agree.

namespace volumetric_kit::ios_app {

/// @brief How a stat should read: plain, healthy, or wanting attention.
enum class StatTone {
  Neutral = 0,
  Good,
  Warn,
  Critical,
};

/// @brief A fraction's tone against two thresholds, so every meter in the app
///        agrees about when a number stops being routine.
///
/// Boundaries are inclusive on the way up — a fraction *equal* to @p crit is
/// already critical — because these gate alarms: a gauge sitting exactly on its
/// limit is the case the threshold exists to catch, not one to round past.
///
/// @param fraction  The measured fraction. Not clamped: a value above 1.0 is a
///                  real reading on the counters this serves (a footprint past
///                  a stale limit), and it reports Critical rather than being
///                  treated as invalid.
/// @param warn      Fraction at or above which the value reads as Warn.
/// @param crit      Fraction at or above which the value reads as Critical.
///                  Checked first, so it wins where the two overlap.
constexpr StatTone tone_for(double fraction, double warn,
                            double crit) noexcept {
  if (fraction >= crit) return StatTone::Critical;
  if (fraction >= warn) return StatTone::Warn;
  return StatTone::Neutral;
}

}  // namespace volumetric_kit::ios_app

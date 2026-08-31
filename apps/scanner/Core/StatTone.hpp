// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file StatTone.hpp
/// @brief When a number on the read-out stops being routine.
///
/// Pure, so the thresholds every meter in the app shares can be tested on the
/// host. The Objective-C `VolumetricStatTone` mirrors @ref StatTone value for
/// value and `Readout.mm` static_asserts the two agree.

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

/// @brief The two tiers a gauge or a row is read against.
struct ToneThresholds {
  double warn = 0.0;
  double critical = 0.0;
};

/// @overload
constexpr StatTone tone_for(double fraction, ToneThresholds t) noexcept {
  return tone_for(fraction, t.warn, t.critical);
}

/// @name The panel's gauge thresholds
///
/// Named here because **two renderings draw the same figure and each used to
/// carry its own copy of the pair.** `Readout.mm` tones a row with them and the
/// SwiftUI panel draws a bar against them, and the bar is not a second view of
/// the row -- the row is suppressed where a bar exists (@ref
/// VolumetricStatRow.drawnAsGauge), so on a gauge these numbers are the *only*
/// thing deciding what colour a reader sees.
///
/// That has already drifted once. The arena gauge was built with a single 0.9
/// tier while its row kept 0.9/0.98, so at 0.99 the row rendered red directly
/// beneath a merely-orange bar drawn from the same measurement. That was
/// repaired by giving the bar two tiers -- which fixed the *shape* and left
/// both sides still holding their own values, four literal pairs on the C++
/// side and four more written out again in Swift. These are what the bridge
/// publishes so the Swift side has nothing left to restate; see
/// `VolumetricGaugeThresholds`.
///
/// @{

/// Block-table occupancy. Not the allocate guard: that one is a hard refusal at
/// @ref kRefuseAllocateAtOccupancy and lives with the policy it enforces. This
/// is when to *tell* someone, which is deliberately the same number so the bar
/// turns red as allocation stops rather than after it.
inline constexpr ToneThresholds kOccupancyThresholds{0.7, 0.85};

/// Mesh arena fill. Tighter than the rest because the consequence is abrupt: an
/// arena that fills does not degrade, it drops triangles.
inline constexpr ToneThresholds kArenaFillThresholds{0.9, 0.98};

/// Footprint against the jetsam ceiling, and against the GPU working set. One
/// pair for both: they are two ceilings on the same held bytes, and a reader
/// comparing the two bars is served by them being on one scale.
inline constexpr ToneThresholds kMemoryThresholds{0.7, 0.85};

/// @}

}  // namespace volumetric_kit::ios_app

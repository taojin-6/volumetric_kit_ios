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
///
/// Both tiers are required, and that is the whole of why this has a constructor
/// rather than being an aggregate with defaults. Defaulted members made
/// `ToneThresholds{0.9}` -- a pair written as though it were the one-tier gauge
/// this file exists to have removed -- compile silently into a `critical` of
/// 0.0, under which every fraction from 0.0 up reads Critical and a 1%-full
/// arena paints solid red on every frame. `-Wmissing-field-initializers` does
/// not fire on a member that has a default initialiser, so nothing caught it.
/// The three-argument `tone_for` this type replaced could not express a
/// one-tier pair at all: it took both numbers or it did not compile. Neither
/// can this.
struct ToneThresholds {
  constexpr ToneThresholds(double warn_at, double critical_at) noexcept
      : warn(warn_at), critical(critical_at) {}

  double warn;
  double critical;
};

/// @overload
constexpr StatTone tone_for(double fraction, ToneThresholds t) noexcept {
  return tone_for(fraction, t.warn, t.critical);
}

/// @name The panel's gauge thresholds
///
/// Named here because **two renderings draw the same figure and each used to
/// carry its own copy of the pair.** `Readout.mm` tones a row with them and the
/// SwiftUI panel draws a bar against them -- and for the figures below that is
/// both at once, on screen together. @ref VolumetricStatRow.drawnAsGauge is set
/// on the Memory rows only, so the `arena` row renders directly beneath the
/// `arena` bar and the `occupied` row beneath the headline meter, each pair
/// drawn from one measurement. A reader sees the disagreement side by side.
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
/// Block-table occupancy is deliberately **not** here. Neither of its tiers is
/// this file's to state -- the critical one is the allocate guard and the warn
/// one is recon's grow threshold -- so it is derived from both by @ref
/// occupancy_thresholds, in the header that owns the first and takes the second
/// as a parameter.
///
/// @{

/// Mesh arena fill. Tighter than the rest because the consequence is abrupt: an
/// arena that fills does not degrade, it drops triangles.
inline constexpr ToneThresholds kArenaFillThresholds{0.9, 0.98};

/// Footprint against the jetsam ceiling, and against the GPU working set. One
/// pair for both: they are two ceilings on the same held bytes, and a reader
/// comparing the two bars is served by them being on one scale.
inline constexpr ToneThresholds kMemoryThresholds{0.7, 0.85};

/// @}

}  // namespace volumetric_kit::ios_app

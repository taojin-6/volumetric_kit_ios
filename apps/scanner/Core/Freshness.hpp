// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file Freshness.hpp
/// @brief Whether a published figure still describes the present.
///
/// The frame-counted half of the app's staleness rules. `Fusion` publishes the
/// extract's breakdown and the dirty survey's sample only on their own fully
/// successful paths, so both go on being read long after they stop being
/// written -- and the read-out gates whole cards on a latch that never lowers.
/// These say when a number has fallen far enough behind its own cadence that it
/// should stop being presented as current.
///
/// Pure, and here rather than in `Fusion` because the producer and the read-out
/// have to agree about it: the margin below was written out as `x + x / 2` in
/// both, under two different names, each correctly derived from @ref
/// kSurveyEveryFrames and each free to be re-derived differently.
///
/// The *millisecond* half of the same idea -- whether the fuse loop is running
/// at all -- is `AllocationStop.hpp`'s @ref kFuseStaleAfterMs and @ref
/// fuse_loop_running. It lives there because what it qualifies is a cause: a
/// stopped scan is a present-tense claim, and that file is what stops it being
/// made about a scan that is merely paused.

#include <cstdint>

// @ref kFuseStaleAfterMs, which @ref stages_stale measures against. The
// millisecond half of these rules lives there because what it qualifies is a
// *cause*; this file borrows the span rather than restating it.
#include "AllocationStop.hpp"

namespace volumetric_kit::ios_app {

/// @brief How many *fused* frames between dirty-block surveys.
///
/// Fused rather than captured, which is the unit the window is reported in and
/// the one `FusionConfig::fuse_every` does not distort: keying the survey off
/// the capture counter made the real period 60/gcd(60, fuse_every) fused
/// frames, so every value sharing a factor with 60 shortened the window
/// silently and `fuse_every == 60` collapsed it to *every* fused frame -- a
/// full compaction, fence and readback per fuse, on the knob someone reaches
/// for precisely to buy frame budget back.
///
/// Shared, because the read-out needs it to tell a survey that has not happened
/// yet from one that is failing. Both leave `FusionStats::survey_active_blocks`
/// at 0 and the gate on it is a one-way latch, so the *only* thing separating
/// them is whether enough fused frames have gone by for a sample to have been
/// due -- and a panel that hardcoded its own 60 to find that out would be the
/// second copy of this number, drifting the first time it moved.
inline constexpr std::uint64_t kSurveyEveryFrames = 60;

/// @brief How far past its own cadence a survey reading may fall before it
///        stops being presented as current.
///
/// A window and a half: wide enough that the ordinary cadence never trips it
/// (the sample publishes exactly every @ref kSurveyEveryFrames frames when it
/// is working), narrow enough that a survey which has stopped is named within
/// another half-window rather than at some unbounded later point.
///
/// The same margin answers both of the questions asked about a survey, which is
/// why it is one constant. `Fusion` uses it to stop presenting a *published*
/// sample as current; the read-out uses it to decide whether a survey that has
/// never published is still merely early -- the first sample is due at @ref
/// kSurveyEveryFrames, and naming a fault the moment that frame goes by would
/// name one on the frame the sample is being taken.
inline constexpr std::uint64_t kSurveyStaleAfter =
    kSurveyEveryFrames + kSurveyEveryFrames / 2;

/// @brief Whether a survey sample has fallen behind its own cadence.
///
/// @param measured      Whether any survey has ever published. A scan that has
///                      not reached its first window is not stale; it is early,
///                      and the read-out says so differently.
/// @param frames_since  Fused frames since the last published sample.
constexpr bool survey_stale(bool measured,
                            std::uint64_t frames_since) noexcept {
  return measured && frames_since > kSurveyStaleAfter;
}

/// @brief Whether the extract's published breakdown has fallen behind the
///        remesh cadence.
///
/// Two cadences of slack rather than none: a remesh that skips because the
/// renderer has not collected the last mesh is the ordinary steady state, and a
/// marker that flickered on it would be noise rather than signal.
///
/// @param measured      Whether any extract has ever published.
/// @param frames_since  Fused frames since the last successful extract.
/// @param remesh_every  `FusionConfig::remesh_every`. Clamped to at least 1, so
///                      a zero cadence cannot make every frame read as stale.
constexpr bool extract_stale(bool measured, std::uint64_t frames_since,
                             std::uint32_t remesh_every) noexcept {
  const std::uint64_t cadence = remesh_every < 1u ? 1u : remesh_every;
  return measured && frames_since > 2ull * cadence;
}

/// @brief Whether the stage timings are older than the frame they sit beside.
///
/// Measured against the *fuse* age rather than against the clock, which is the
/// whole point: a scan that has been paused for a minute has stage rows a
/// minute old and there is nothing wrong with them. What this catches is stage
/// rows that stopped advancing while fusing carried on -- `measure_stages`
/// publishes them only on frames it seeds, so they can fall behind a running
/// loop.
///
/// @param ms_since_stages  Milliseconds since the stage rows were published.
/// @param ms_since_fuse    Milliseconds since the last fused frame.
constexpr bool stages_stale(float ms_since_stages,
                            float ms_since_fuse) noexcept {
  return ms_since_stages > ms_since_fuse + kFuseStaleAfterMs;
}

}  // namespace volumetric_kit::ios_app

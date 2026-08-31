// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file AllocationStop.hpp
/// @brief Why a scan stopped taking new geometry, and every way that is said.
///
/// Pure, so the renderings can be tested on the host. `Fusion` names the cause
/// and this decides what each medium says about it -- which is a decision, and
/// the rule for this directory is that a decision made without the device
/// belongs here with a test. The Objective-C rendering, which is the one that
/// genuinely needs the bridge, is the only piece left in
/// `Bridge/AllocationStopDisplay.hpp`.
///
/// What the renderings must not differ about is *which* cause, so they are all
/// exhaustive switches over the same enum and they sit together. A cause added
/// below stops the compile in exactly one file, with every rendering in front
/// of whoever is fixing it -- and that is a guarantee rather than a hope only
/// because `-Werror=switch` is set on the targets that compile them.

#include <cstdint>
#include <string>

namespace volumetric_kit::ios_app {

/// @brief Why a fused frame took no new geometry in, when it took none.
///
/// A cause rather than a bool so the read-out can name one without knowing
/// `Fusion`'s thresholds. See @ref FusionStats::allocation_stop.
enum class AllocationStop : std::uint8_t {
  /// The frame allocated normally.
  None = 0,
  /// Occupancy is past the refuse-to-allocate guard: the documented trade
  /// working, and the one cause a user can act on (coarser voxels, or a higher
  /// `FusionConfig::max_buckets` if the map is not already at it).
  VolumeFull,
  /// `load_factor` could not be read, so the guard refused on a fabricated
  /// occupancy. Not a full volume; the fault is upstream and is in @ref
  /// FusionStats::last_error. Kept distinct because the actionable advice for a
  /// full volume is actively wrong here.
  OccupancyUnknown,
  /// The allocate reported a capacity limit and the blocks were dropped -- at
  /// the bucket ceiling, or with the frame's grow budget spent. Reaches this
  /// through `AllocFailures::capacity_limited`, which includes bucket-local
  /// chain exhaustion, so it can fire with occupancy far below the guard.
  BlocksDropped,
};

/// @brief A cause in the words a person reads.
struct AllocationStopText {
  /// The cause in two or three words. The panel's headline, and the phrase
  /// every shorter rendering names the cause with -- written once here so the
  /// log and the panel cannot end up calling one cause two things.
  const char* headline;
  /// What to do about it. This is the part that made a per-cause table
  /// necessary: it was one hardcoded string -- coarsen the voxels, or raise
  /// `max_buckets` -- which is right for `VolumeFull` and actively wrong for
  /// the other two. `OccupancyUnknown` is a failed `load_factor` read on a
  /// volume with room left, and `BlocksDropped` fires with occupancy far below
  /// the guard. Both sent the reader after a limit they had not reached.
  const char* advice;
  /// Whether the fault itself is upstream and named on the read-out's errors
  /// row, so a rendering with no room for `advice` can point at it instead.
  bool on_errors_row;
};

/// @brief The headline and advice for @p stop.
AllocationStopText allocation_stop_text(AllocationStop stop) noexcept;

/// @brief The read-out's suffix for the `table` row, empty for @ref
///        AllocationStop::None.
///
/// Composed from @ref AllocationStopText rather than written out again: the
/// three cause phrases used to appear once here and once as a headline, thirty
/// lines apart, so rewording one for clarity left the log and the panel naming
/// the same cause differently -- read side by side exactly when a collected
/// device log is compared against a screenshot.
///
/// A lookup rather than a literal at each site, because the read-out's job here
/// is to report a cause, not to guess one. The `table` row used to append a
/// hard-coded "(volume full)" to a flag that meant only "not allocating this
/// frame", so a failed `load_factor` -- which fabricates a full table to fail
/// safe -- printed a full volume directly beneath the banner naming the real
/// upstream fault, and told the user to coarsen their voxels over it. Fusion
/// deliberately withholds its own "volume full" string on that path; this is
/// what stops the panel undoing that.
std::string allocation_stop_note(AllocationStop stop);

/// @brief One word for the frame trace's `alloc=` column.
///
/// Its own vocabulary rather than @ref AllocationStopText::headline: the trace
/// is one line per frame in a dump read as a table, so this side wants a token
/// that stays short, while the headline is a phrase a person reads once.
const char* allocation_stop_tag(AllocationStop stop) noexcept;

/// @brief How stale a fuse-loop publication makes its per-frame values.
///
/// A second is generous against a 60 Hz capture decimated to whatever
/// `fuse_every` is, and deliberately so: this should fire on an interruption,
/// not on a slow frame.
inline constexpr float kFuseStaleAfterMs = 1000.0f;

/// @brief Whether the fuse loop published recently enough for its latched
///        per-frame values to still describe the present.
constexpr bool fuse_loop_running(float ms_since_fuse) noexcept {
  return ms_since_fuse <= kFuseStaleAfterMs;
}

/// @brief The cause a read-out may state in the present tense, given how long
///        it has been since the fuse loop published.
///
/// A stopped scan is a present-tense claim, so it has to stop being made when
/// the fuse loop stops running. @ref FusionStats::allocation_stop is a latch
/// `Fusion` never clears, and an ARKit interruption -- a call, Control Centre,
/// the app switcher -- stops fuse frames without stopping the display link. So
/// the banner, the panel's `state` row and the log's `table` row went on
/// announcing a full volume for the length of a phone call, about a scan that
/// was not allocating because it was not scanning, and told the user to abandon
/// it and restart at a coarser voxel size.
///
/// Every rendering that makes the claim runs through here, rather than the one
/// that happened to be fixed first. A forensic dump is the exception and does
/// not use this: it records the latched cause beside the age that qualifies it,
/// because discarding what the cause *was* is the wrong trade in the one
/// artifact a device loss leaves behind.
constexpr AllocationStop reportable_allocation_stop(
    AllocationStop latched, float ms_since_fuse) noexcept {
  return fuse_loop_running(ms_since_fuse) ? latched : AllocationStop::None;
}

}  // namespace volumetric_kit::ios_app

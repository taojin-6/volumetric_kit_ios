// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file AllocationStopDisplay.hpp
/// @brief Every rendering of an @ref app::AllocationStop, in one place.
///
/// Four of them, and each medium genuinely differs: the log gets a suffix on a
/// fixed-width line, the frame trace gets one word in a column, the bridge gets
/// an enum, and the panel gets a phrase a person reads once and acts on.
///
/// What they must not differ about is *which* cause. That is why they are all
/// exhaustive switches on the same enum and why they sit together -- a
/// statement the old anonymous namespace could only make by adjacency, and this
/// file makes by name. A cause added to `AllocationStop` stops the compile in
/// exactly one file, with all four renderings in front of whoever is fixing it.

#import "VolumetricRenderer.h"

#import "Fusion.hpp"

namespace volumetric_kit::ios_app {

/// @brief The read-out's word for an @ref AllocationStop, and the only place
///        one is turned into English.
///
/// A lookup rather than a literal at each site, because the read-out's job here
/// is to report a cause, not to guess one. The `table` row used to append a
/// hard-coded "(volume full)" to a flag that meant only "not allocating this
/// frame", so a failed `load_factor` -- which fabricates a full table to fail
/// safe -- printed a full volume directly beneath the banner naming the real
/// upstream fault, and told the user to coarsen their voxels over it. Fusion
/// deliberately withholds its own "volume full" string on that path; this is
/// what stops the panel undoing that.
const char* allocation_stop_note(AllocationStop stop) noexcept;

/// @brief One word for the frame trace's fixed-width column.
const char* allocation_stop_tag(AllocationStop stop) noexcept;

/// @brief The same cause, as a dashboard row and as the banner's own sentence.
///
/// `advice` is the part that made this necessary. It was one hardcoded string
/// -- coarsen the voxels, or raise `max_buckets` -- which is the right answer
/// for `VolumeFull` and actively wrong for the other two: `OccupancyUnknown` is
/// a failed `load_factor` read on a volume with room left, and `BlocksDropped`
/// fires with occupancy far below the guard. Both sent the reader after a limit
/// they had not reached.
struct AllocationStopText {
  const char* headline;
  const char* advice;
};

/// @brief The headline and advice for @p stop.
AllocationStopText allocation_stop_text(AllocationStop stop) noexcept;

/// @brief The same cause, as the Swift-facing enum.
///
/// Switched rather than cast, though the two enumerations are declared in the
/// same order: a cast makes that order load-bearing across two files that no
/// build step compares, and the failure is a sample reporting the
/// *neighbouring* cause -- a wrong answer that looks exactly like a right one.
/// This way a value added on one side stops the compile.
VolumetricAllocationStop allocation_stop_value(AllocationStop stop) noexcept;

}  // namespace volumetric_kit::ios_app

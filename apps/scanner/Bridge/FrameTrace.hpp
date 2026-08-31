// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file FrameTrace.hpp
/// @brief What the last few frames drew, kept so a device loss can say so.

#include <cstddef>
#include <cstdint>

#include "AllocationStop.hpp"

namespace volumetric_kit::ios_app {

/// @brief A ring of the last few frames' render state, dumped on a GPU fault.
///
/// A device loss is reported by the *next* vkWaitForFences, so by the time the
/// error surfaces the frame that faulted is already gone and nothing on the
/// stack says what it did. This keeps the last few frames' worth of the state
/// that could plausibly cause a GPU fault and dumps it when the loss is
/// detected -- at whichever call notices, since `end_frame` reports one just as
/// readily as `begin_frame` does.
///
/// A ring rather than per-frame logging: at 60 Hz an os_log per frame is both
/// noise and a perturbation, and only the frames immediately before the fault
/// matter.
///
/// Plain C++, deliberately. This is a POD ring and some wrap arithmetic; it was
/// Objective-C-only because of one import it did not use, which cost every
/// includer 13.5 MB of preprocessed text and put the wrap arithmetic out of
/// reach of the host tests.
///
/// @warning **Render thread only.** Written and read from the render thread, so
///          nothing here locks.
struct FrameTrace {
  struct Entry {
    std::uint64_t frame = 0;
    std::uint64_t generation = 0;  // recon generation this frame drew
    std::size_t mesh_slot = 0;
    std::uint64_t released_through = 0;  // what we told recon it may reuse
    std::uint32_t triangles = 0;
    std::uint32_t triangle_capacity = 0;
    std::uint64_t arena_bytes = 0;  // grew? compare against the previous entry
    std::uint32_t active_blocks = 0;
    float extract_ms = 0.0f;
    // The table as the *map* reported it, beside `active_blocks` as the last
    // successful extract reported it. Both, because the gap between them is
    // often the fault: this ring is dumped on a device-lost, which is what the
    // occupancy guard exists to prevent, and in the regime that fires the
    // guard `active_blocks` is exactly the frozen number the guard stopped
    // trusting. `stop` says whether the guard was engaged at the time.
    float occupancy = 0.0f;
    /// Whether @ref occupancy is a reading rather than a fallback.
    ///
    /// Carried, because a failed `load_factor` makes Fusion publish a
    /// deliberately fabricated 1.0 so the allocate guard fails safe -- and
    /// this ring printed that as `occ=100.0%`, inventing a measurement in the
    /// one artifact a device loss leaves behind. Both the live read-out and
    /// `FusionStats::occupancy` say in as many words that the figure must not
    /// be shown as a reading; the dump was the last place still doing it.
    bool occupancy_known = true;
    AllocationStop stop = AllocationStop::None;
    /// How long since the fuse loop published, at this frame.
    ///
    /// The qualifier `stop` needs and nothing else here carries. Fusion never
    /// clears the latch, so an ARKit interruption leaves a stale cause sitting
    /// on frames that allocated nothing -- and unlike the live renderings,
    /// which drop the claim (see @ref reportable_allocation_stop), a forensic
    /// dump wants both numbers: what the cause was, and how old it is.
    float ms_since_fuse = 0.0f;
    bool drew_mesh = false;
  };

  static constexpr std::size_t kCapacity = 24;
  Entry entries[kCapacity];
  std::uint64_t next = 0;

  Entry& begin_frame_entry() {
    Entry& e = entries[next % kCapacity];
    e = Entry{};
    e.frame = next;
    ++next;
    return e;
  }

  /// @brief Dump the ring, oldest-first, naming @p why.
  ///
  /// Oldest-first, so the last line is the frame closest to the fault.
  ///
  /// @p why should name the `VkResult`, not just the call that returned it --
  /// `describe` in RendererErrors.hpp is what builds that line. gfx puts the
  /// stringified call in `Status::message()` and the result in
  /// `Status::code()`, so a banner built from the message alone cannot tell a
  /// lost device from a slow one.
  ///
  /// Both channels on purpose: os_log is what survives a run with no debugger
  /// attached (readable afterwards via `log collect`), and stderr is what
  /// reaches `devicectl process launch --console` live. os_log alone goes
  /// nowhere near the console, which is the mistake worth not repeating.
  void dump(const char* why) const;
};

}  // namespace volumetric_kit::ios_app

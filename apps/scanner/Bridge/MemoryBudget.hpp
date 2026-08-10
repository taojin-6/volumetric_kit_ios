// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file MemoryBudget.hpp
/// @brief What the OS will let this process hold, read from the OS rather than
///        assumed.

#include <cstdint>

namespace volumetric_kit::ios_app {

/// @brief One reading of this process's memory position against the ceiling
///        jetsam enforces.
///
/// A live 1 cm reconstruction is a genuinely large-memory app whose worst
/// failure mode returns no `Status`: jetsam sends `SIGKILL`, so the process
/// disappears with nothing in the log, no allocator return to check, and no
/// opportunity to shrink first. Every knob that decides whether a scan fits --
/// `FusionConfig::voxel_size`, `max_buckets`, `mesh_slots` -- was nonetheless
/// tuned against a figure read off one device once and written into a comment
/// in `scanner.entitlements`. This is that figure, read at runtime, on whatever
/// device is actually running.
///
/// The ceiling is **not** the device's RAM, and not a constant: it depends on
/// the hardware, the OS version, and on whether the app carries
/// `com.apple.developer.kernel.increased-memory-limit` (this one does -- see
/// `scanner.entitlements`). Comparing @ref limit_bytes against @ref
/// device_ram_bytes is also the only in-app evidence that the entitlement took
/// effect: a capability the App ID does not grant is dropped at signing, and is
/// otherwise invisible right up until the process vanishes.
///
/// @note This is the **jetsam** ceiling, which is one of two limits and the one
///       that kills silently. The other is virtual address space, lifted by
///       `com.apple.developer.kernel.extended-virtual-addressing`, which this
///       app deliberately does not carry (a personal development team cannot
///       provision it). Exhausting *that* one fails an allocation rather than
///       the process, so it surfaces as a `vr::Status` out of recon -- which is
///       what distinguishes the two after the fact: one reports, one does not.
struct MemoryBudget {
  /// Bytes this process is charged for: `phys_footprint`, the ledger jetsam
  /// compares against the limit.
  ///
  /// Not resident size. The two differ by hundreds of megabytes here, because
  /// the footprint charges compressed and IOKit-mapped pages that RSS does not
  /// -- and the mesh arenas are host-visible VMA allocations over a
  /// unified-memory heap, which is precisely where they diverge. It is also
  /// what Xcode's memory gauge shows, so the two agree.
  std::uint64_t footprint_bytes = 0;
  /// Bytes still allocatable before the limit (`os_proc_available_memory`).
  std::uint64_t available_bytes = 0;
  /// The ceiling: @ref footprint_bytes + @ref available_bytes.
  ///
  /// Derived rather than queried, because iOS exposes no call for it -- the API
  /// answers "how much is left", so recovering the total needs what is already
  /// spent.
  std::uint64_t limit_bytes = 0;
  /// Physical RAM installed (`hw.memsize`), for scale only. Never the budget:
  /// the per-process ceiling is a fraction of it.
  std::uint64_t device_ram_bytes = 0;
  /// `true` when the kernel answered; the byte fields are meaningless if not.
  ///
  /// Gated on `task_info` alone, since that is the one call here that reports a
  /// real error code. `os_proc_available_memory` returns 0 both when the
  /// process is at its limit and when the OS does not treat it as an app at all
  /// (a test host, a command-line tool), and the two are indistinguishable --
  /// so a spurious 0 is reported as @ref limit_bytes equal to @ref
  /// footprint_bytes, i.e. "no headroom". That is the safe direction to be
  /// wrong in for a number read to decide whether to keep allocating, and it
  /// cannot arise in this app, which is always an app.
  bool valid = false;
};

/// @brief Read this process's memory position from the kernel.
///
/// Two syscalls. Cheap enough for a read-out polled at a few hertz, and not
/// intended for a per-frame path.
///
/// @return The reading, or a default-constructed value (@ref
///         MemoryBudget::valid `== false`) when the kernel declined to answer.
MemoryBudget query_memory_budget() noexcept;

}  // namespace volumetric_kit::ios_app

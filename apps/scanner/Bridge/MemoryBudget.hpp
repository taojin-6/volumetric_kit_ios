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
/// `scanner.entitlements`).
///
/// @note @ref device_ram_bytes is context, not evidence. An earlier version of
///       this comment offered the limit/RAM ratio as the one in-app sign that
///       the entitlement took effect; that is wrong twice over. A capability
///       the App ID does not grant fails **signing**, loudly, which
///       `apps/scanner/CMakeLists.txt` records as the deliberate outcome -- so
///       a build that runs at all already carries it, and `codesign -d
///       --entitlements -` answers the question on the binary without launching
///       a scan. The ratio is also not a test: iOS reports `hw.memsize` net of
///       firmware and secure-enclave carve-outs, so on a 16 GB device the
///       entitled ceiling sits *near* the reported RAM whether or not the
///       entitlement is in force, and on a phone build it sits below it either
///       way.
///
/// @note This is the **jetsam** ceiling, and it is not the only limit that
///       binds. Two others sit beside it and neither is measured here:
///       - The **GPU working set** (`MTLDevice.recommendedMaxWorkingSetSize`,
///         two thirds of physical RAM on this hardware) is *lower* than the
///         jetsam ceiling, and the voxel grid and mesh arenas are Metal buffers
///         charged against it. `VolumetricRenderer` reads that one separately
///         and prints it directly beneath this reading; see
///         `scanner.entitlements`.
///       - **Virtual address space**, lifted by
///         `com.apple.developer.kernel.extended-virtual-addressing`, which this
///         app deliberately does not carry (a personal development team cannot
///         provision it). Exhausting *that* one fails an allocation rather than
///         the process, so it surfaces as a `vr::Status` out of recon -- which
///         is what distinguishes the two after the fact: one reports, one does
///         not.
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
  /// High-water mark of @ref footprint_bytes over the process's life
  /// (`ledger_phys_footprint_peak`), or 0 when the kernel did not supply it.
  ///
  /// **The field that makes this read-out able to see the event it exists
  /// for.** @ref footprint_bytes is a sample, and this is polled at whatever
  /// rate the Swift view happens to tick -- roughly 2 Hz. The allocation that
  /// gets a scan killed is not a plateau: `VoxelBlockGrid::resize` builds the
  /// grown buffers beside the old ones and commits once, so a doubling toward
  /// `FusionConfig::max_buckets` spikes for well under one polling interval and
  /// is gone before the next sample. Sampling alone therefore shows the
  /// pre-spike steady state and reads as comfortable, which is worse than
  /// showing nothing. The kernel keeps the maximum for us, so the spike
  /// survives the gap between polls at the cost of one field read -- it comes
  /// out of the same `task_info` call as the footprint.
  std::uint64_t peak_footprint_bytes = 0;
  /// Bytes still allocatable before the limit
  /// (`task_vm_info::limit_bytes_remaining`).
  ///
  /// The only directly measured quantity here; @ref limit_bytes is derived from
  /// it. The kernel clamps it to 0 once the process is at or past the limit,
  /// which is reported as @ref at_limit rather than folded into a ceiling --
  /// see @ref limit_known.
  std::uint64_t available_bytes = 0;
  /// The ceiling: @ref footprint_bytes + @ref available_bytes. Meaningful only
  /// when @ref limit_known.
  ///
  /// Derived rather than queried, because iOS exposes no call for it -- the
  /// kernel answers "how much is left", so recovering the total needs what is
  /// already spent. Both halves come out of a **single** `task_info` call, and
  /// that matters: `os_proc_available_memory()` is documented as equivalent to
  /// `limit_bytes_remaining` but is a separate sample, so pairing it with a
  /// footprint read at another instant yields `LIMIT - (footprint(t1) -
  /// footprint(t0))`. The fuse thread allocates concurrently and unsynchronised
  /// with this call, so a `resize` landing in that window moves the reported
  /// ceiling by hundreds of megabytes -- during exactly the doubling the
  /// reading is meant to inform. One call makes the two halves coherent and
  /// costs one syscall less.
  std::uint64_t limit_bytes = 0;
  /// Physical RAM installed (`hw.memsize`), for scale only. Never the budget:
  /// the per-process ceiling is a fraction of it, and iOS reports this net of
  /// firmware carve-outs (~915 MB on a 16 GB device), so it is not the nominal
  /// capacity either.
  std::uint64_t device_ram_bytes = 0;
  /// The `kern_return_t` from `task_info`, kept so a failed reading can say
  /// *why* rather than printing an unactionable placeholder. `KERN_SUCCESS`
  /// (0) when @ref valid.
  int task_info_status = 0;
  /// `true` when @ref limit_bytes is a real ceiling rather than a coincidence.
  ///
  /// False in two cases, both of which would otherwise print a fabricated
  /// number: the kernel returned a struct too short to carry
  /// `limit_bytes_remaining` (pre-REV4), or the process is at or past its limit
  /// and the kernel clamped the remainder to 0 (@ref at_limit). The second is
  /// the dangerous one -- with a clamped 0 the derived ceiling collapses onto
  /// @ref footprint_bytes, so the read-out would show the *limit rising to meet
  /// the footprint* and report a tidy 100%, in precisely the pre-jetsam window
  /// this instrument exists to catch.
  bool limit_known = false;
  /// `true` when the kernel reports no headroom left. Read this as the alarm it
  /// is: it is the last state before jetsam.
  bool at_limit = false;
  /// `true` when the kernel answered; the byte fields are meaningless if not.
  ///
  /// Gated on `task_info`, the one call here that reports a real error code.
  /// A failure to read `hw.memsize` is reported as @ref device_ram_bytes `== 0`
  /// and does not clear this, since the footprint and the ceiling are the
  /// load-bearing halves and the RAM figure is context.
  bool valid = false;
};

/// @brief Read this process's memory position from the kernel.
///
/// One `task_info` call, plus a `sysctlbyname` on the first call only
/// (`hw.memsize` is immutable for the process's lifetime, so it is cached).
/// Cheap enough for a read-out polled at a few hertz, and not intended for a
/// per-frame path.
///
/// @return The reading, or a default-constructed value (@ref
///         MemoryBudget::valid `== false`, with @ref
///         MemoryBudget::task_info_status carrying the kernel's code) when the
///         kernel declined to answer.
MemoryBudget query_memory_budget() noexcept;

}  // namespace volumetric_kit::ios_app

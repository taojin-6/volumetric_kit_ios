// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file GrowthPolicy.hpp
/// @brief When the block table grows, and when it stops taking new geometry.
///
/// The two decisions that bound how much scene a scan can hold, and the
/// arithmetic they rest on. Both were reached inside `Fusion::fuse` -- pure
/// branches over an occupancy, a bucket count and a memory reading, embedded in
/// a function that also submits GPU work -- so neither could be exercised
/// without a device, a LiDAR frame and a table driven into its pathological
/// regime on purpose.
///
/// They are here for the reason stated in this directory's CMakeLists: they
/// decide something, and they decide it without the device. What stays on the
/// other side of this header is everything that *acts* on the decision -- the
/// `resize`, the `allocate_from_depth`, the stage rows, the error strings that
/// quote these figures back to the reader.
///
/// @note **Nothing here restates recon's grow threshold.** `VoxelHashMap`
///       documents `kGrowThreshold` as a property of that table, and says
///       outright that an embedder refusing at its own threshold ends up
///       disagreeing with the library it is guarding. So @ref
///       GrowthInputs::grow_threshold is a field the caller fills from recon,
///       never a constant on this side. @ref kRefuseAllocateAtOccupancy *is*
///       this app's own number, and is the one thing in this file recon has no
///       opinion about.

#include <cstdint>

#include "AllocationStop.hpp"
#include "MemoryBudget.hpp"

namespace volumetric_kit::ios_app {

/// @brief Blocks per bucket -- `VoxelGridParams::bucket_size`.
///
/// Named once rather than written as an `8` wherever the block-table capacity
/// is derived: the occupancy guards divide by this, and a bucket size changed
/// at the grid params with the guards left restating the old one would silently
/// mis-scale the very thresholds that keep the allocate kernel out of its
/// pathological regime.
inline constexpr std::int32_t kBlocksPerBucket = 8;

/// @brief How many times one frame may grow the map and retry its allocation.
///
/// One doubling is not enough when the user pans onto a whole new room section.
/// Bounded rather than unbounded so a frame cannot spend the whole budget
/// resizing -- and bounded well below the number of doublings that span the
/// table's whole range, which is the part that has to be re-checked whenever
/// `FusionConfig::max_buckets` moves.
///
/// It was 5 against a 16384-bucket ceiling reached in four doublings from the
/// 1024-bucket start, so the cap and the range were not the same number and one
/// frame could not walk from one end to the other. Doubling the ceiling made
/// them equal: five doublings is exactly 1024 -> 32768, so a single `fuse` call
/// could commit the full ~1.5 GiB grid with a ~2.3 GiB transient beside it, on
/// the fuse thread, in one frame -- while `capacity_limited()` is true for
/// bucket-local `chain` exhaustion, which fires with the global load factor
/// well under the grow threshold. Two keeps this a backstop for a frame that
/// outruns one doubling, which is what it is documented to be, and leaves the
/// rest of the range to the preemptive path that checks the memory budget.
inline constexpr int kMaxGrowAttempts = 2;

/// @brief The occupancy past which no new blocks are allocated at all.
///
/// **Deliberately not recon's `kGrowThreshold`.** That one says "start
/// growing"; this says "stop feeding the overflow scan", and the band between
/// them is the room a doubling needs to land in. Collapsing them would refuse
/// allocation at the moment growth begins, on a table with plenty of room.
///
/// The growth this guards only helps while there is somewhere to grow. At the
/// `max_buckets` ceiling occupancy climbs unchecked and the overflow scan is
/// O(num_buckets * bucket_size) per insert -- so a *larger* ceiling makes the
/// pathological case worse, not better, and no ceiling is high enough to be a
/// fix on its own. Measured: 31480 of 32768 blocks (96%) at the old
/// 4096-bucket ceiling hung the GPU outright.
///
/// Past that point, stop feeding it. Skipping allocation costs *new* geometry
/// only: integrate still fuses every block already in the table, so the
/// existing surface keeps refining and the app keeps running. That is the trade
/// `max_buckets` was always documented to make -- "a scan that is missing far
/// geometry, still running, and saying so" -- which until this guard existed
/// lived only in a comment, and whose unenforced version was a GPU hang.
///
/// @note **This threshold's justification is older than the kernel it guards,
///       and 0.85 has not been re-measured since.** The behaviour above
///       describes `allocate_in_overflow` before recon d282bbd and e36f6ad
///       (both 2026-08-08), which together stopped the exhaustive sweep paying
///       a contended atomicCompSwap per slot: a candidate's pointer is read
///       unlocked first, so only free-looking slots pay the atomic (~25x fewer
///       at 96% occupancy), and an empty heap short-circuits on a single atomic
///       load. CMakeLists.txt tracks recon's `main`, so this app already builds
///       that kernel.
///
///       Kept in force regardless, because a cliff that costs coverage is the
///       safe side of an unmeasured guess and nothing has run on device since
///       the fix. But it is now capping scans against a pathology repaired
///       upstream, so this is the number to re-measure -- not the ceiling it is
///       protecting.
inline constexpr float kRefuseAllocateAtOccupancy = 0.85f;

/// @brief The resident bytes `VoxelBlockGrid` holds for @p buckets buckets.
///
/// `buckets * kBlocksPerBucket` blocks, 512 voxels each, and 12 B of attributes
/// per voxel -- tsdf + weight + color, the three specs `Fusion::start`
/// registers. At 16384 buckets that is 805 MB, which is the figure
/// `scanner.entitlements` records for the grid, so the arithmetic here and the
/// measurement there agree.
///
/// The grid only. The mesh arena ring is the larger term and is not derived
/// from this; see `FusionConfig::max_buckets` for what that means for the
/// headroom check that calls this.
constexpr std::uint64_t grid_bytes_for(std::int32_t buckets) noexcept {
  constexpr std::uint64_t kVoxelsPerBlock = 512;
  constexpr std::uint64_t kAttributeBytesPerVoxel = 12;
  return static_cast<std::uint64_t>(buckets) *
         static_cast<std::uint64_t>(kBlocksPerBucket) * kVoxelsPerBlock *
         kAttributeBytesPerVoxel;
}

/// @brief The blocks a table of @p buckets buckets holds.
///
/// The denominator every occupancy sentence quotes. Derived here rather than
/// multiplied out at each site, because the sites that need it are the ones
/// reporting a stopped scan, and a block count computed from a bucket figure
/// the doubling had already advanced is how one of them came to print a total
/// that never existed.
constexpr std::int64_t table_blocks_for(std::int32_t buckets) noexcept {
  return static_cast<std::int64_t>(buckets) *
         static_cast<std::int64_t>(kBlocksPerBucket);
}

/// @brief The size a doubling targets, never past the ceiling.
constexpr std::int32_t grow_target(std::int32_t buckets,
                                   std::int32_t max_buckets) noexcept {
  const std::int32_t doubled = buckets * 2;
  return doubled < max_buckets ? doubled : max_buckets;
}

/// @brief Everything the fuse loop knows when it decides whether to grow.
struct GrowthInputs {
  /// The map's load factor this frame.
  float occupancy = 0.0f;
  /// `false` when `load_factor` could not be read, in which case @ref occupancy
  /// is a fabricated 1.0 that fails safe. Growth is gated on this as well as on
  /// the threshold: a fabricated full table would otherwise read as a standing
  /// instruction to double.
  bool occupancy_known = false;
  /// recon's `VoxelHashMap::kGrowThreshold`, passed in rather than restated --
  /// see this file's note.
  float grow_threshold = 0.0f;
  /// The table's size now.
  std::int32_t num_buckets = 0;
  /// The ceiling `FusionConfig::max_buckets` sets.
  std::int32_t max_buckets = 0;
  /// A size whose resize has already been refused, or 0.
  ///
  /// `num_buckets` advances only on a *successful* resize, so without this a
  /// failing grow satisfies the same condition again on the very next fused
  /// frame -- asking the allocator for the largest block this app requests, at
  /// capture rate.
  std::int32_t declined_at = 0;
  /// The kernel's memory reading, for the headroom check.
  MemoryBudget budget{};
};

/// @brief What @ref plan_growth decided to do.
enum class GrowthAction : std::uint8_t {
  /// Occupancy is below the threshold, the table is at its ceiling, or this
  /// size has already been refused. Nothing to do.
  None = 0,
  /// Grow to @ref GrowthPlan::grow_to.
  Resize,
  /// A doubling is due, and the headroom will not cover it. Not a fault in the
  /// scan: the existing surface keeps fusing, more slowly, into a denser table.
  DeclinedForMemory,
};

/// @brief The decision, and the figures a caller needs to report it.
struct GrowthPlan {
  GrowthAction action = GrowthAction::None;
  /// The target size. Meaningful for @ref GrowthAction::Resize and @ref
  /// GrowthAction::DeclinedForMemory -- the declining message names it.
  std::int32_t grow_to = 0;
  /// What that target would commit, from @ref grid_bytes_for. Meaningful under
  /// the same two actions.
  std::uint64_t needed_bytes = 0;
};

/// @brief Whether a doubling is due, ignoring whether it can be afforded.
///
/// The half of the decision that costs nothing, split out so a caller can ask
/// it *before* paying for @ref GrowthInputs::budget. `query_memory_budget` is a
/// `task_info` trap and `MemoryBudget` documents itself as not for a per-frame
/// path -- which the fuse loop is. Asking this first keeps the syscall on the
/// handful of frames where a grow is actually in question, rather than on every
/// fused frame at capture rate.
///
/// Reads no budget field, so the caller may leave it default-constructed.
constexpr bool growth_due(const GrowthInputs& in) noexcept {
  return in.occupancy_known && in.occupancy > in.grow_threshold &&
         in.num_buckets < in.max_buckets && in.num_buckets != in.declined_at;
}

/// @brief Decide whether to grow ahead of density.
///
/// Returns @ref GrowthAction::None whenever @ref growth_due is false, so a
/// caller that skipped the budget read and called this anyway gets the same
/// answer -- the split is an optimisation, not a precondition.
///
/// Reactive growth is not enough, and the reason is in the kernel: when a
/// coord's own bucket and chain are full, `allocate_in_overflow` falls back to
/// scanning every table entry. So the cost of one insert climbs with occupancy,
/// and past ~90% nearly every new coord takes that path.
///
/// Measured on an M5 iPad Pro at 1 cm voxels: at 7809 of 8192 blocks the
/// allocate dispatch went 1.2 ms -> 3.1 ms -> 4.2 ms over ~65 frames, dropped
/// the capture to 45 fps, and then hung the GPU outright --
/// kIOGPUCommandBufferCallbackErrorHang on recon's queue, which took gfx's
/// queue down with it and surfaced as VK_ERROR_DEVICE_LOST from the next
/// vkWaitForFences.
///
/// Waiting for a *failure* to grow means waiting until the table is already in
/// that regime: the failure is only reported after the scan that cannot be
/// afforded. So this grows on occupancy instead, well before the fallback
/// dominates, and the reactive retry loop stays a backstop for a frame that
/// fills the table faster than one doubling absorbs.
///
/// The headroom check asks the kernel before asking the allocator. `resize`
/// builds the grown attribute arrays beside the live ones, so the transient is
/// the whole new size on top of what is already resident -- ~1.5 GiB at the
/// 32768-bucket ceiling, beside a mesh arena ring that is the larger term still
/// (3089 MB of arenas against an 805 MB grid, measured). This is the allocation
/// that gets a scan SIGKILLed rather than failed, and jetsam returns no
/// `Status`.
///
/// A reading that is invalid, or whose ceiling is unknown, does **not** decline
/// the grow: an unreadable budget is not evidence of a full one, and refusing
/// on it would stop every scan on a device whose kernel answered short.
GrowthPlan plan_growth(const GrowthInputs& in) noexcept;

/// @brief Whether to allocate this frame's blocks, and why not when not.
struct AllocationGuard {
  /// `false` when the table is past @ref kRefuseAllocateAtOccupancy.
  bool allocate = true;
  /// The cause to publish. @ref AllocationStop::None while allocating.
  ///
  /// The guard's two causes are separated here rather than at the read-out: a
  /// fabricated occupancy trips it exactly as a genuinely full table does, and
  /// only the scope holding @p occupancy_known knows which happened. The
  /// read-out's advice for the two is different, and the full-volume advice is
  /// actively wrong for the other.
  AllocationStop stop = AllocationStop::None;
};

/// @brief Whether a table at @p occupancy may still take new blocks.
constexpr AllocationGuard guard_allocation(float occupancy,
                                           bool occupancy_known) noexcept {
  if (occupancy <= kRefuseAllocateAtOccupancy) {
    return {true, AllocationStop::None};
  }
  return {false, occupancy_known ? AllocationStop::VolumeFull
                                 : AllocationStop::OccupancyUnknown};
}

}  // namespace volumetric_kit::ios_app

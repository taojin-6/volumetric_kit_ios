// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "Fusion.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#import "MemoryBudget.hpp"

namespace volumetric_kit::ios_app {
namespace {

using Clock = std::chrono::steady_clock;

float ms_since(Clock::time_point t0) {
  return std::chrono::duration<float, std::milli>(Clock::now() - t0).count();
}

/// How many times one frame may grow the map and retry its allocation.
///
/// One doubling is not enough when the user pans onto a whole new room section.
/// Bounded rather than unbounded so a frame cannot spend the whole budget
/// resizing -- and bounded well below the number of doublings that now span the
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
constexpr int kMaxGrowAttempts = 2;

/// Blocks per bucket, which is `VoxelGridParams::bucket_size` below.
///
/// Named once rather than written as an `8` wherever the block-table capacity
/// is derived: the occupancy guards divide by this, and a bucket size changed
/// at the grid params with the guards left restating the old one would silently
/// mis-scale the very thresholds that keep the allocate kernel out of its
/// pathological regime.
constexpr std::int32_t kBlocksPerBucket = 8;

/// The resident bytes `VoxelBlockGrid` holds for a table of `buckets` buckets.
///
/// `num_buckets * kBlocksPerBucket` blocks, 512 voxels each, and 12 B of
/// attributes per voxel -- tsdf + weight + color, the three specs `start`
/// registers. At 16384 buckets that is 805 MB, which is the figure
/// `scanner.entitlements` records for the grid, so the arithmetic here and the
/// measurement there agree.
///
/// The grid only. The mesh arena ring is the larger term and is not derived
/// from this; see FusionConfig::max_buckets for what that means for the
/// headroom check that calls this.
std::uint64_t grid_bytes_for(std::int32_t buckets) {
  constexpr std::uint64_t kVoxelsPerBlock = 512;
  constexpr std::uint64_t kAttributeBytesPerVoxel = 12;
  return static_cast<std::uint64_t>(buckets) *
         static_cast<std::uint64_t>(kBlocksPerBucket) * kVoxelsPerBlock *
         kAttributeBytesPerVoxel;
}

/// The floor on `FusionConfig::mesh_slots`; see that field.
constexpr std::uint32_t kMinMeshSlots = 2;

/// How many *fused* frames between dirty-block surveys.
///
/// Fused rather than captured, which is the unit the window is reported in and
/// the one `FusionConfig::fuse_every` does not distort: keying the survey off
/// the capture counter made the real period 60/gcd(60, fuse_every) fused
/// frames, so every value sharing a factor with 60 shortened the window
/// silently and `fuse_every == 60` collapsed it to *every* fused frame -- a
/// full compaction, fence and readback per fuse, on the knob someone reaches
/// for precisely to buy frame budget back.
constexpr std::uint64_t kSurveyEveryFrames = 60;

/// How far past its own cadence a survey reading may fall before the read-out
/// stops presenting it as current.
///
/// A window and a half: wide enough that the ordinary cadence never trips it
/// (the sample publishes exactly every kSurveyEveryFrames frames when it is
/// working), narrow enough that a survey which has stopped is named within
/// another half-window rather than at some unbounded later point.
constexpr std::uint64_t kSurveyStaleAfter =
    kSurveyEveryFrames + kSurveyEveryFrames / 2;

}  // namespace

vr::Status Fusion::start(vr::Device& device, vr::Allocator& allocator,
                         const FusionConfig& config) {
  // Refused, not clamped, and refused here rather than trusted from the caller.
  // At one slot recon reuses a single arena in place and `release_through`
  // changes no behaviour -- so every DeviceMesh this class publishes would be
  // valid only until the next extract, while the renderer draws it. The failure
  // is a destroyed VkBuffer under a live draw, which raises no Status and no
  // validation message on iOS; a start() that refuses is the only place it can
  // still be said out loud. See FusionConfig::mesh_slots.
  if (config.mesh_slots < kMinMeshSlots) {
    return vr::Status::invalid_argument(
        "FusionConfig::mesh_slots is " + std::to_string(config.mesh_slots) +
        ", which switches recon's slot-release contract off; Published::mesh "
        "borrows the extractor's buffers, so it needs at least " +
        std::to_string(kMinMeshSlots) +
        " (and the consumer's frames in flight "
        "plus one to actually pipeline).");
  }
  config_ = config;
  // Cleared, not carried. Everything in here is about the scan that just ended,
  // and several fields are one-way latches -- `allocation_stop` and `occupancy`
  // most of all, which would otherwise have a new scan open still announcing
  // the full volume that ended the last one. The per-scan bookkeeping outside
  // `stats_` goes with them, for the same reason.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_ = FusionStats{};
    stats_.mesh_slots = config.mesh_slots;
  }
  last_fuse_ns_.store(0, std::memory_order_relaxed);
  active_blocks_at_frame_ = 0;
  active_blocks_measured_ = false;
  preemptive_grow_failed_at_ = 0;

  vr::volume::VoxelGridParams grid{};
  grid.voxel_size = config.voxel_size;
  grid.block_size = 8;
  grid.voxels_per_block = 512;  // 8^3
  grid.trunc_dist = config.trunc_dist;
  grid.bucket_size = kBlocksPerBucket;
  grid.num_buckets = config.num_buckets;
  grid.num_blocks = config.num_buckets * kBlocksPerBucket;
  grid.max_chain = 128;

  // tsdf + weight are what the integrator writes; color is what makes the
  // reconstruction look like the room rather than a grey shell, and marching
  // cubes interpolates it into Vertex::color for gfx's hybrid path.
  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)},
                                             {"color", sizeof(std::uint32_t)}};
  vr::Result<vr::volume::VoxelBlockGrid> made =
      vr::volume::VoxelBlockGrid::create(device, allocator, grid, attrs, 3);
  if (!made) {
    return made.status();
  }
  grid_.emplace(std::move(made).value());

  // Dirty tracking is opt-in: it costs a num_blocks*4 array and a store per
  // voxel, which a consumer that reads no flag should not pay. This one reads
  // it -- the survey in `fuse` is the whole reason the counters exist.
  //
  // Taken from the config rather than pinned on here, and that is the point of
  // the field: recon allocates the flag array inside `integrate` and rebuilds
  // it on every map grow, so a failure to get it fails the *frame*, not just
  // the diagnostic. See FusionConfig::track_dirty_blocks.
  vr::tsdf::TsdfIntegratorConfig integ_config;
  integ_config.track_dirty_blocks = config.track_dirty_blocks;
  vr::Result<vr::tsdf::TsdfIntegrator> integrator =
      vr::tsdf::TsdfIntegrator::create(device, allocator, integ_config);
  if (!integrator) {
    return integrator.status();
  }
  integrator_.emplace(std::move(integrator).value());

  vr::Result<vr::mesh::MarchingCubes> mc =
      vr::mesh::MarchingCubes::create(device, allocator, [&] {
        vr::mesh::MarchingCubesConfig mc_config;
        // The renderer binds these buffers as geometry rather than being handed
        // a host copy, so it needs its own usage bits on the same allocation --
        // only it knows which, which is why this tier takes them.
        mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
        // In-block vertex sharing: one vertex where the cells meeting on an
        // edge would each have emitted their own, instead of three private ones
        // per triangle. Selects recon's second sparse kernel at create().
        //
        // Safe HERE specifically because FusionConfig::texture is off.
        // ProjectiveTexturer decides visibility per *triangle* and writes uv0
        // per *vertex*, which is well-defined only while a vertex belongs to
        // one triangle -- and recon's texturer refuses a shared mesh outright
        // rather than letting the write order decide. Flipping `texture` back
        // on means turning this off, until the per-primitive camera id lands.
        //
        // Roughly a 3x vertex reduction for the same surface; the exact figure
        // is a property of the scan, so trust this device's own read-out
        // (`arena_bytes` against the triangle count) over any number quoted
        // from a desktop fixture. Memory is why it is on here: an iPad is where
        // the arena ceiling is real.
        mc_config.share_vertices = true;
        mc_config.slot_count = config.mesh_slots;
        // Held by value there, so copied rather than pointed at -- the config
        // outlives this lambda's temporaries.
        for (std::uint32_t i = 0; i < config.queue_family_count; ++i) {
          mc_config.queue_families[i] = config.queue_families[i];
        }
        mc_config.queue_family_count = config.queue_family_count;
        return mc_config;
      }());
  if (!mc) {
    return mc.status();
  }
  marching_cubes_.emplace(std::move(mc).value());

  vr::Result<vr::texture::ProjectiveTexturer> texturer =
      vr::texture::ProjectiveTexturer::create(device, allocator);
  if (!texturer) {
    return texturer.status();
  }
  texturer_.emplace(std::move(texturer).value());

  return vr::Status();
}

void Fusion::fuse(const vr::sensor::CapturedFrame& frame) {
  if (!valid()) {
    return;
  }
  // Liveness, stamped before any path can leave. What the read-out does with it
  // is refuse to assert a *present-tense* condition about a loop that has
  // stopped running: `allocation_stop` and `occupancy` are per-frame values
  // latched into a persistent snapshot, and an ARKit interruption stops frames
  // without stopping the display link, so the panel went on announcing a
  // stopped scan for the length of a phone call. See
  // FusionStats::ms_since_fuse.
  last_fuse_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                          Clock::now().time_since_epoch())
                          .count(),
                      std::memory_order_relaxed);
  // Decimate before doing any work: ARKit runs at 60 Hz and the surface barely
  // moves between consecutive frames, so fusing every one mostly re-averages
  // the same voxels to the same value.
  if (++captured_ % config_.fuse_every != 0) {
    return;
  }

  // Anything this frame got wrong, carried to the single publish below rather
  // than written straight into stats_. The success path used to clear
  // last_error unconditionally, which meant a frame fused with missing blocks
  // left no trace anywhere -- the read-out showed a rising vertex count and an
  // empty error line while a whole region of the scan quietly never filled.
  // One reporting window per fused frame. The tiers open their own host scopes
  // and publish their device spans into it, so the hand-rolled Clock::now()
  // spans this function used to keep are gone -- and each row now carries the
  // GPU half those never could.
  vr::StageMetrics stages;

  std::string frame_error;
  // Whether that error is a *stage failure* rather than a report of an expected
  // outcome. Only the former reaches `stats_.errors` -- see that field: dropped
  // blocks and a volume at its ceiling are the common case on a healthy scan,
  // and counting them made the one genuine failure impossible to see.
  bool frame_stage_failed = false;

  // --- How full is the table, right now? -----------------------------------
  //
  // `VoxelHashMap::load_factor` -- a 4-byte read of the mapped heap counter,
  // which recon added as the constant-time reading a per-frame caller can
  // afford. Both guards below used `stats_.extract.active_blocks` instead,
  // which only a *successful extract* refreshes, so a persistent extract
  // failure froze the numerator while the real table went on filling and the
  // guards waved through exactly the regime they exist to prevent.
  //
  // The whole staleness apparatus that stood here -- a fused-frame window, a
  // measured-yet flag, a stale-reading branch on both guards and a separate
  // error message for it -- existed only to cope with that input. A reading
  // that cannot go stale needs none of it: this one is taken this frame, from
  // the map itself, and does not depend on meshing having succeeded.
  float occupancy = 0.0f;
  bool occupancy_known = true;
  if (vr::Result<float> lf = grid_->map().load_factor()) {
    occupancy = lf.value();
  } else {
    // A moved-from map is the only failure, and this class owns it -- so this
    // is unreachable rather than tolerated. Report and treat the table as full:
    // refusing to allocate on an unknown occupancy is the safe direction, and
    // it is the direction the old stale branch took for the same reason.
    //
    // Safe for the *allocate* guard only, which is why the flag travels beside
    // it. A fabricated 1.0 is also above kGrowThreshold, so a reader that takes
    // it at face value doubles the map -- and since the documented failure is
    // permanent, doubles it again every frame until max_buckets, answering "I
    // cannot tell how full the table is" with the largest allocation this app
    // makes, five times. The deleted staleness apparatus ANDed `!stale` into
    // the grow guard for exactly this reason; `occupancy_known` is that term.
    //
    // Not counted here. `stats_` is published under `mutex_`, which is not held
    // until well below this point, and an unsynchronised read-modify-write on
    // `errors` races `Fusion::stats()` on the main thread -- a data race by the
    // memory model, and a torn or lost count on the field the read-out treats
    // as durable. `frame_stage_failed` carries it to the one locked publish,
    // which is what the adjacent resize failure already does.
    frame_error = "load_factor: " + lf.status().message();
    frame_stage_failed = true;
    occupancy = 1.0f;
    occupancy_known = false;
  }

  // --- Grow *ahead* of density, before allocating into it -------------------
  //
  // Reactive growth is not enough, and the reason is in the kernel: when a
  // coord's own bucket and chain are full, allocate_in_overflow falls back to
  // scanning every table entry, taking a contended atomicCompSwap per slot to
  // test it. So the cost of one insert climbs with occupancy, and past ~90%
  // nearly every new coord takes that path -- num_buckets * bucket_size atomics
  // each, thousands of invocations per depth frame.
  //
  // Measured on an M5 iPad Pro at 1 cm voxels: at 7809 of 8192 blocks the
  // allocate dispatch went 1.2 ms -> 3.1 ms -> 4.2 ms over ~65 frames, dropped
  // the capture to 45 fps, and then hung the GPU outright --
  // kIOGPUCommandBufferCallbackErrorHang on recon's queue, which took gfx's
  // queue down with it as kIOGPUCommandBufferCallbackErrorInnocentVictim and
  // surfaced as VK_ERROR_DEVICE_LOST from the next vkWaitForFences.
  //
  // Waiting for a *failure* to grow means waiting until the table is already in
  // that regime: the failure is only reported after the scan that cannot be
  // afforded. So grow on occupancy instead, well before the fallback dominates.
  // The reactive path below stays as a backstop for a frame that fills the
  // table faster than one doubling absorbs.
  // recon's own constant, not a third copy of the number: the map's header
  // gives this figure as the occupancy a caller should grow at rather than run
  // past, and a UI drawing its own ceiling or an embedder refusing at its own
  // threshold otherwise ends up disagreeing with the library it is guarding.
  //
  // Gated on `occupancy_known` as well as on the threshold: see the fabricated
  // 1.0 above, which would otherwise read as a standing instruction to double.
  // And skipped at a size that has already been refused --
  // `config_.num_buckets` advances only on success, so a failing resize
  // otherwise satisfies this condition again on the very next fused frame, and
  // keeps asking the allocator for the largest block this app requests at
  // capture rate.
  if (occupancy_known && occupancy > vr::volume::VoxelHashMap::kGrowThreshold &&
      config_.num_buckets < config_.max_buckets &&
      config_.num_buckets != preemptive_grow_failed_at_) {
    const std::int32_t grown_to =
        std::min(config_.num_buckets * 2, config_.max_buckets);
    // Ask the kernel before asking the allocator. `resize` builds the grown
    // attribute arrays beside the live ones, so the transient is the whole new
    // size on top of what is already resident -- ~1.5 GiB at the 32768-bucket
    // ceiling, beside a mesh arena ring that is the larger term still (3089 MB
    // of arenas against an 805 MB grid, measured; see FusionConfig::
    // max_buckets). This is the allocation that gets a scan SIGKILLed rather
    // than failed, and jetsam does not return a Status.
    //
    // Affordable despite MemoryBudget's own "not intended for a per-frame path"
    // note, because this is not a per-frame path: it is gated on a doubling,
    // which happens a handful of times in a scan's life. One `task_info` call
    // against a 1.5 GiB commit is not the cost worth saving.
    const MemoryBudget budget = query_memory_budget();
    const std::uint64_t needed = grid_bytes_for(grown_to);
    if (budget.valid && budget.limit_known && budget.available_bytes < needed) {
      frame_error = "preemptive resize declined: doubling to " +
                    std::to_string(grown_to) + " buckets needs " +
                    std::to_string(needed / (1024 * 1024)) + " MB and " +
                    std::to_string(budget.available_bytes / (1024 * 1024)) +
                    " MB is left before the process limit; not growing "
                    "(existing surface still fusing)";
      frame_stage_failed = true;
      preemptive_grow_failed_at_ = config_.num_buckets;
    } else if (const vr::Status grown = grid_->resize(grown_to)) {
      config_.num_buckets = grown_to;
      preemptive_grow_failed_at_ = 0;
      // Re-read, because the guard below acts on this number and the doubling
      // just halved it. Sampling once above and testing there is what made the
      // frame that grew the map also the frame that refused to allocate into
      // it: one pan took occupancy across both thresholds at once, the resize
      // took the true figure from 0.88 to 0.44, and the stale 0.88 then
      // skipped the allocate outright and announced a stopped scan on a table
      // that had just been given room. Another 4-byte read of the heap counter.
      if (vr::Result<float> after = grid_->map().load_factor()) {
        occupancy = after.value();
      }
    } else {
      // Not fatal: the table is merely denser than preferred, and the
      // allocate below still works -- more slowly. Reported rather than
      // returned so the frame still fuses.
      frame_error = "preemptive resize: " + grown.message();
      frame_stage_failed = true;
      preemptive_grow_failed_at_ = config_.num_buckets;
    }
  }

  // --- Refuse to allocate into a table with no room left --------------------
  //
  // The growth above only helps while there is somewhere to grow. At the
  // max_buckets ceiling occupancy climbs unchecked, and the overflow scan is
  // O(num_buckets * bucket_size) per insert -- so a *larger* ceiling makes the
  // pathological case worse, not better, and no ceiling is high enough to be a
  // fix on its own. Measured: 31480 of 32768 blocks (96%) at the old
  // 4096-bucket ceiling hung the GPU.
  //
  // So past the point where the fallback dominates, stop feeding it. Skipping
  // allocation costs *new* geometry only: integrate still fuses every block
  // already in the table, so the existing surface keeps refining and the app
  // keeps running. That is the trade max_buckets was always documented to make
  // ("a scan that is missing far geometry, still running, and saying so") -- it
  // just was not actually enforced anywhere, and the unenforced version was a
  // GPU hang.
  // Deliberately NOT kGrowThreshold. That one says "start growing"; this one
  // says "stop feeding the overflow scan", and the band between them is the
  // room a doubling needs to land in. Collapsing them would refuse allocation
  // at the moment growth begins, on a table with plenty of room.
  //
  // @note **This threshold's justification is older than the kernel it guards,
  //       and 0.85 has not been re-measured since.** Everything above describes
  //       `allocate_in_overflow` as it behaved before recon d282bbd and e36f6ad
  //       (both 2026-08-08), which together stopped the exhaustive sweep from
  //       paying a contended atomicCompSwap per slot: a candidate's pointer is
  //       read unlocked first, so only free-looking slots pay the atomic (~25x
  //       fewer at 96% occupancy), and an empty heap short-circuits on a single
  //       atomic load -- named there as "the state the iPad was in, sweeping
  //       the whole table to discover nothing". CMakeLists.txt tracks recon's
  //       `main`, so this app already builds that kernel, and this very change
  //       took `kGrowThreshold` from the same changeset.
  //
  //       Kept in force regardless, because a cliff that costs coverage is the
  //       safe side of an unmeasured guess and nothing has run on device since
  //       the fix. But it is now capping scans against a pathology that has
  //       been repaired upstream, so the number to re-measure is this one --
  //       not the ceiling it is protecting.
  constexpr float kRefuseAllocateAtOccupancy = 0.85f;
  const bool table_exhausted = occupancy > kRefuseAllocateAtOccupancy;

  // --- Allocate the blocks this frame's depth touches ----------------------
  const auto t_alloc = Clock::now();
  vr::volume::AllocFailures failures;
  vr::Result<std::uint32_t> overflow =
      table_exhausted
          ? vr::Result<std::uint32_t>(0u)
          : grid_->map().allocate_from_depth(frame.depth, frame.depth_camera,
                                             &failures, &stages);
  // Whether this frame took no new geometry in, by either route. The guard
  // above is one of them; the reactive loop below hitting the ceiling, or
  // running out of per-frame grow budget with blocks still dropped, is the
  // other. Published once at the end rather than here, so it can carry both --
  // setting it from the guard alone left the second route as silent as it was
  // before the field existed, and that route fires while the table reads 60%.
  //
  // The guard's own two causes are separated here rather than at the read-out:
  // a fabricated occupancy trips it just as a genuinely full table does, and
  // only this scope knows which happened.
  AllocationStop allocation_stop = !table_exhausted ? AllocationStop::None
                                   : occupancy_known
                                       ? AllocationStop::VolumeFull
                                       : AllocationStop::OccupancyUnknown;
  if (table_exhausted && frame_error.empty()) {
    // Both halves of the ratio taken after the grow above, not across it. The
    // first cut divided a numerator sampled before the doubling by
    // `config_.num_buckets`, which the doubling had already advanced, and then
    // asserted the ceiling unconditionally -- so a successful grow to 2048 of a
    // 32768 ceiling printed "88% of 16384 blocks at the 32768-bucket ceiling"
    // and prescribed raising a ceiling four doublings away. Multiplying the
    // sentence's own two numbers gave a block count that never existed.
    const std::int64_t blocks =
        static_cast<std::int64_t>(config_.num_buckets) * kBlocksPerBucket;
    const bool at_ceiling = config_.num_buckets >= config_.max_buckets;
    frame_error =
        "volume full: " + std::to_string(static_cast<int>(occupancy * 100.0f)) +
        "% of " + std::to_string(blocks) + " blocks" +
        (at_ceiling ? " at the " + std::to_string(config_.max_buckets) +
                          "-bucket ceiling"
                    : " (short of the " + std::to_string(config_.max_buckets) +
                          "-bucket ceiling, which the map has not reached)") +
        "; not allocating new blocks (existing surface still fusing). " +
        (at_ceiling ? "Raise max_buckets or use a coarser voxel_size."
                    : "A doubling is due and did not happen -- see any resize "
                      "error above.");
  }
  if (!overflow) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "allocate: " + overflow.status().message();
    // Published on the early return too: this path took nothing in either, and
    // a snapshot that keeps last frame's figures here is the frozen read-out
    // the occupancy field exists to end.
    stats_.occupancy = occupancy;
    stats_.occupancy_known = occupancy_known;
    stats_.table_blocks =
        static_cast<std::uint32_t>(config_.num_buckets * kBlocksPerBucket);
    stats_.allocation_stop = allocation_stop != AllocationStop::None
                                 ? allocation_stop
                                 : AllocationStop::BlocksDropped;
    return;
  }
  // The table filled. Grow and retry *while* it keeps filling: resize preserves
  // block indices, so everything already fused keeps its tsdf/weight/color at
  // the same offset, and another doubling is cheap next to the frame it would
  // otherwise fuse with holes.
  //
  // Gated on capacity_limited(), not on the count alone. Depth is the most
  // contended entry point in the map -- adjacent LiDAR pixels dilate into the
  // same block, and the kernel's bucket spin-lock gives up after a bounded
  // number of retries -- so a residue of pure lock failures is the expected
  // outcome on a table with room to spare. Growing on that is the worst trade
  // this app can make: resize builds the grown attribute arrays beside the old
  // ones, so the doubling that reaches the 32768-bucket ceiling commits ~1.5
  // GiB with ~2.3 GiB transient, and that is the jetsam range max_buckets is
  // chosen to stay out of.
  //
  // The figure in this comment was "around 288 MiB transient" against a
  // 4096-bucket ceiling, two ceiling raises out of date -- an order of
  // magnitude under the cost, in the sentence that justifies the gate. It is
  // restated here rather than pointed at because whoever is deciding whether to
  // loosen the gate is reading exactly this paragraph.
  //
  // The per-frame budget is what bounds it; see kMaxGrowAttempts for why that
  // number has to move whenever max_buckets does. Note also that
  // `capacity_limited()` is true for bucket-local `chain` exhaustion, which a
  // clustered region can reach with the global load factor far below the grow
  // threshold -- so these attempts are not all backed by global pressure.
  int grow_attempts = 0;
  while (overflow.value() > 0 && failures.capacity_limited() &&
         grow_attempts < kMaxGrowAttempts) {
    if (config_.num_buckets >= config_.max_buckets) {
      frame_error = "allocate: map at its " +
                    std::to_string(config_.max_buckets) + "-bucket ceiling, " +
                    std::to_string(overflow.value()) +
                    " blocks dropped; far geometry will be missing";
      break;
    }
    const std::int32_t grown_to =
        std::min(config_.num_buckets * 2, config_.max_buckets);
    const vr::Status grown = grid_->resize(grown_to);
    if (!grown) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error = "resize: " + grown.message();
      return;
    }
    config_.num_buckets = grown_to;
    ++grow_attempts;
    // Checked, not discarded. This is the same call as above and its return is
    // the only thing that says whether the grown map actually absorbed the
    // frame; dropping it fuses against a grid still missing those blocks and
    // reports success.
    // Accumulates into the same "allocate" row: a frame that resizes and
    // re-allocates reports what it actually cost, not just the last attempt.
    overflow = grid_->map().allocate_from_depth(frame.depth, frame.depth_camera,
                                                &failures, &stages);
    if (!overflow) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error =
          "allocate (after resize): " + overflow.status().message();
      return;
    }
  }
  if (grow_attempts > 0) {
    // The loop's doublings moved the table under the reading taken above, and
    // that reading is about to be published as this frame's occupancy beside a
    // capacity from after them. Refreshed rather than left alone for the same
    // reason the preemptive path refreshes it: a ratio whose halves come from
    // opposite sides of a resize is wrong by exactly the factor it doubled.
    if (vr::Result<float> lf = grid_->map().load_factor()) {
      occupancy = lf.value();
      occupancy_known = true;
    }
  }
  // Blocks dropped to a *capacity* limit means this frame took in less than the
  // depth map offered, and at the ceiling or with the grow budget spent that is
  // not a transient race -- it is the scan no longer acquiring. Raised here so
  // the read-out says so on the frame it happens, whatever occupancy reads:
  // `capacity_limited()` covers bucket-local chain exhaustion, which a
  // clustered region reaches at 60%. Without this the panel showed a
  // comfortable percentage behind an `errors 0` banner while acquisition had
  // permanently stopped -- the very failure `allocation_stop` was added for,
  // left open on the half of the failure space the guard does not see.
  if (overflow.value() > 0 && failures.capacity_limited() &&
      allocation_stop == AllocationStop::None) {
    allocation_stop = AllocationStop::BlocksDropped;
  }
  if (overflow.value() > 0 && frame_error.empty()) {
    // Named by reason, because the two call for opposite responses from whoever
    // reads the overlay: a capacity limit means the scan has outgrown the map,
    // while contention means this frame lost a race and the next one will not
    // (the blocks that did land are present now, so fewer threads collide).
    if (failures.capacity_limited()) {
      frame_error = "allocate: " + std::to_string(overflow.value()) +
                    " blocks dropped after " + std::to_string(grow_attempts) +
                    " grow(s) (" + std::to_string(failures.chain) + " chain, " +
                    std::to_string(failures.heap) + " heap, " +
                    std::to_string(failures.table) +
                    " table); some geometry will be missing";
    } else {
      frame_error = "allocate: " + std::to_string(overflow.value()) +
                    " allocations lost bucket-lock races (no capacity limit); "
                    "not grown -- the next frame re-dilates the same blocks";
    }
  }
  // Integrated anyway when blocks were dropped: what *did* allocate still
  // fuses, and a partial frame beats none. The error above is what keeps it
  // from reading as a clean one.
  const float allocate_ms = ms_since(t_alloc);

  // --- Fuse depth, and colour when the frame carries it --------------------
  const auto t_integrate = Clock::now();
  // The encoding is carried across, not left to default. `ColorFrame::encoding`
  // defaults to *canonical*, so omitting it does not say "unspecified" -- it
  // declares canonical on the frame's behalf, and a non-canonical frame would
  // then be fused through the wrong transfer curve instead of refused. Passing
  // it is what arms recon's rejection at the one seam that can arm it: this is
  // the only place ARKit's declaration reaches the integrator. ARKitCapture
  // normalises or refuses today, so this is latent rather than live -- and it
  // stays latent only until someone adds a fast path there.
  vr::tsdf::ColorFrame color{};
  color.pixels = frame.color;
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  const vr::Status fused = integrator_->integrate(
      *grid_, frame.depth, frame.depth_camera, /*max_weight=*/5.0f,
      vr::tsdf::IntegrationMode::Classic, frame.has_color() ? &color : nullptr,
      &stages);
  if (!fused) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "integrate: " + fused.message();
    if (config_.track_dirty_blocks) {
      // Named here because dirty tracking put an allocation on this path that
      // was not on it before: recon sizes the per-block flag array inside
      // integrate and rebuilds it beside the old one on every map grow, so the
      // frame after a resize can fail here for want of a *diagnostic* buffer
      // while the fuse itself was affordable. Nothing falls back, so a reader
      // who cannot otherwise account for this failure needs the switch named.
      stats_.last_error +=
          " -- note: FusionConfig::track_dirty_blocks is on, which allocates a "
          "num_blocks*4 flag array inside integrate on every map grow; turning "
          "it off takes the dirty survey with it but removes that allocation "
          "from this path";
    }
    return;
  }
  const float integrate_ms = ms_since(t_integrate);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.frames_fused;
    last_pose_ = frame.depth_camera.cam_to_world;
    stats_.allocate_ms = allocate_ms;
    stats_.integrate_ms = integrate_ms;
    // The table's own state, all four taken from after every resize this frame
    // performed, so the ratio the read-out prints has both halves at one
    // instant. `table_blocks` rather than `table_capacity` for the denominator:
    // see those two declarations for why they are separate fields and why
    // pairing this numerator with that one was the bug.
    stats_.occupancy = occupancy;
    stats_.occupancy_known = occupancy_known;
    stats_.table_blocks =
        static_cast<std::uint32_t>(config_.num_buckets * kBlocksPerBucket);
    stats_.allocation_stop = allocation_stop;
    // Copied whole rather than picked apart: every field recon adds to a row
    // arrives without a lockstep edit here, and the read-out below reads the
    // same rows the bridge publishes -- one source, not two that can disagree.
    stats_.stage_count = 0;
    for (const vr::StageRow& row : stages.rows()) {
      if (stats_.stage_count >= FusionStats::kMaxStages) break;
      stats_.stages[stats_.stage_count++] = row;
    }
    // How far behind this frame the published extract breakdown now is.
    // Everything in `stats_.extract` -- the phases, the block count, the arena,
    // `dispatches` -- is written only by a fully-successful remesh, so without
    // this the read-out reprints an older extract's numbers as though they were
    // this frame's, with nothing on screen saying otherwise.
    //
    // `table_capacity` deliberately does *not* refresh here. It is the
    // denominator for `extract.active_blocks`, and it is stamped beside that
    // numerator in remesh; refreshing it every frame is what made occupancy
    // halve after a grow whose remesh then skipped. See its declaration.
    stats_.frames_since_extract =
        active_blocks_measured_ ? stats_.frames_fused - active_blocks_at_frame_
                                : 0;
    // Two cadences of slack rather than none: a remesh that skips because the
    // renderer has not collected the last mesh is the ordinary steady state,
    // and a marker that flickered on it would be noise rather than signal.
    const std::uint64_t fresh_within =
        2ull * std::max<std::uint32_t>(config_.remesh_every, 1u);
    stats_.extract_stale =
        active_blocks_measured_ && stats_.frames_since_extract > fresh_within;
    // The same pair for the survey, which needs it more: the read-out gates the
    // dirty rows on `survey_active_blocks > 0` and never lowers that gate, so
    // without this a survey that has stopped publishing keeps its last sample
    // on screen indefinitely, presented as this frame's.
    stats_.frames_since_survey =
        survey_measured_ ? stats_.frames_fused - survey_at_frame_ : 0;
    stats_.survey_stale =
        survey_measured_ && stats_.frames_since_survey > kSurveyStaleAfter;
    // Assigned, not cleared: a frame that fused with dropped blocks says so.
    //
    // This assignment is also what makes `errors` load-bearing. It runs every
    // fused frame and overwrites whatever the previous frame's *later* stages
    // raised -- remesh's extract and texture failures, and anything the fuse
    // thread's exception guard noted -- because those happen after this point
    // in the frame and this is the next thing to touch `last_error`. The
    // counter is the part that does not get overwritten.
    //
    // Gated on `frame_stage_failed`, not on the message being non-empty. Most
    // of what lands in `frame_error` is a report of an expected outcome --
    // blocks lost to bucket-lock contention, which happens on nearly every
    // frame, and a volume at its ceiling, which republishes every frame it
    // keeps fusing. Counting those ran the total to four figures within a
    // minute of a clean scan and buried the failures the counter is for.
    if (frame_stage_failed) {
      ++stats_.errors;
    }
    stats_.last_error = frame_error;
  }

  if (stats_.frames_fused % config_.remesh_every == 0) {
    remesh(frame);
  }

  // --- Dirty-block survey ---------------------------------------------------
  //
  // Throttled hard: one compaction -- a dispatch, a fence wait and a readback
  // of the whole active set -- plus an O(active) dilation walk and an
  // O(num_blocks) host scan of the flag array. Once per kSurveyEveryFrames is
  // enough to characterise a scan; `survey_ms` is what it actually cost, rather
  // than this comment's word for it. See FusionStats::survey_active_blocks for
  // what the number does and does not prove.
  //
  // Keyed on frames_fused, matching the remesh gate above and the unit the
  // window is reported in -- see kSurveyEveryFrames for what keying it off the
  // capture counter did to `fuse_every`.
  if (stats_.frames_fused % kSurveyEveryFrames == 0) {
    const auto t_survey = Clock::now();
    // Sampled into locals first, published in one short critical section below.
    // dirty_block_count in particular walks the whole num_blocks flag array on
    // the host -- 262144 entries at the max_buckets ceiling -- and mutex_ is
    // the lock the *main* thread takes four times per rendered frame. This file
    // already declined to malloc under it (see FusionTraceStats); a 262k-entry
    // scan is not a smaller ask than that one.
    //
    // That figure doubled with the ceiling and this sentence did not follow it,
    // which matters because it is the sentence justifying the
    // sample-outside-the-mutex exception: the scan this paragraph prices is now
    // twice what it says. kSurveyEveryFrames was set against the old size and
    // has not been revisited; at 60 fused frames the survey is ~1 s apart, so
    // the doubled walk is still affordable, but that is the check, not an
    // assumption -- `survey_ms` is what it actually costs.
    std::uint32_t active = 0;
    std::uint32_t changed = 0;
    std::uint32_t to_remesh = 0;
    bool sampled = false;
    std::string survey_error;
    vr::Result<std::vector<vr::volume::BlockIndex>> all =
        grid_->map().compact_active_blocks();
    if (!all) {
      survey_error = "dirty survey (compact): " + all.status().message();
    } else {
      // The caller's already-compacted set is passed in rather than letting the
      // integrator compact a second time -- a dispatch, a fence wait and a full
      // readback, on a call that is already O(active blocks).
      //
      // `remesh_set`, not `remesh`: the member function of that name is called
      // earlier in this same function body, and a local shadowing it resolves
      // to the member only because the declaration has not been reached yet.
      // Moving this block above that call -- which is what hoisting the survey
      // over the error early-returns would mean -- turns it into a "called
      // object type is not a function" error on a line nobody edited.
      vr::Result<std::vector<vr::Vec3i>> remesh_set =
          integrator_->dirty_remesh_blocks(*grid_, all.value().data(),
                                           all.value().size());
      if (!remesh_set) {
        survey_error =
            "dirty survey (remesh set): " + remesh_set.status().message();
      } else {
        active = static_cast<std::uint32_t>(all.value().size());
        to_remesh = static_cast<std::uint32_t>(remesh_set.value().size());
        changed = integrator_->dirty_block_count();
        sampled = true;
      }
    }
    const float survey_ms = ms_since(t_survey);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Published either way: what the survey cost is worth knowing most on the
      // frame where it failed, since a failure still paid for the compaction.
      stats_.survey_ms = survey_ms;
      if (sampled) {
        stats_.survey_active_blocks = active;
        stats_.survey_changed_blocks = changed;
        stats_.survey_remesh_blocks = to_remesh;
        // Measured from the last re-arm, not assumed to be the cadence: see
        // FusionStats::survey_window_frames.
        stats_.survey_window_frames = stats_.frames_fused - dirty_window_start_;
        stats_.survey_first_window = dirty_window_start_ == 0;
        survey_at_frame_ = stats_.frames_fused;
        survey_measured_ = true;
        stats_.frames_since_survey = 0;
        stats_.survey_stale = false;
      } else {
        // Counted and named, like every other stage in this function. These two
        // calls were the only fallible ones here that reported nothing at all
        // -- and the read-out's gate on the survey is a one-way latch, so a
        // survey that fails from here on leaves its last good sample on screen
        // forever. See FusionStats::frames_since_survey.
        ++stats_.errors;
        stats_.last_error = survey_error;
      }
    }
    // Re-armed every window, so each sample is ONE window's changes rather than
    // everything since the scan began.
    //
    // Unconditional, including on the failure path above, because this call is
    // also the *recovery*: dirty_remesh_blocks refuses outright once blocks
    // have been removed from the grid -- a slot-keyed flag means nothing across
    // a remove, the heap being LIFO -- and reset_dirty is what re-arms it.
    // Skipping it when the sample failed would latch that refusal for the rest
    // of the scan, which is the one failure mode here that cannot self-heal.
    integrator_->reset_dirty();
    dirty_window_start_ = stats_.frames_fused;
  }
}

void Fusion::remesh(const vr::sensor::CapturedFrame& frame) {
  // --- Hand the consumer's release to recon, on this thread -----------------
  //
  // `MarchingCubes::release_through` is not atomic, and its header makes
  // serializing it against the extracting thread a caller obligation: calling
  // it concurrently with an `extract_device` on the same object is a data race,
  // and a stale read of the mark makes `claim_output_slot` refuse with "every
  // output slot is still outstanding" -- which then freezes the occupancy guard
  // in `fuse` and is self-sustaining. So `Fusion::release_through` only records
  // the mark, and it is applied here, on the fuse thread. Nothing else in this
  // class touches the extractor, so recon sees exactly one thread.
  //
  // *Before* the extract, not after. This release is what makes room for the
  // extract below; running it afterwards kept the ring permanently one slot
  // shallower than its depth suggests, and on a failing extract the early
  // return skipped it altogether -- stranding the slot whose absence caused the
  // failure, which is how the ring stalled for good.
  std::uint64_t consumer_released = 0;
  bool uncollected = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    consumer_released = consumer_released_;
    uncollected = !published_taken_ && published_generation_ != 0;
  }
  if (consumer_released != 0) {
    marching_cubes_->release_through(consumer_released);
  }

  // --- Do not publish over a mesh nobody collected --------------------------
  //
  // It still holds its slot, and freeing that slot is not something this side
  // can do: `release_through` is a monotonic high-water mark, so releasing the
  // uncollected generation retires every *older* one with it -- including the
  // generations the renderer's in-flight frames are drawing out of. recon
  // reclaims one, a grow frees its buffers outright (`vmaDestroyBuffer`, no
  // fence wait), and the live `vkCmdDrawIndexedIndirect` reads a destroyed
  // VkBuffer: VK_ERROR_DEVICE_LOST, which is the fault this seam was built to
  // remove. recon does have a single-slot primitive for this and keeps it
  // private (`free_slot_of`), precisely because the high-water mark is the
  // consumer's to move and not the producer's.
  //
  // So the uncollected mesh is left where it is and this extract simply does
  // not run. Its result would have been thrown away regardless, so the skip
  // costs nothing and saves the dispatch -- and it is what lets the ring be the
  // consumer's frames in flight plus one instead of plus two. The renderer
  // collects on every frame it draws, so in the steady state this skips at most
  // one remesh.
  if (uncollected) {
    return;
  }

  const auto t_extract = Clock::now();
  vr::mesh::ExtractTimings extract_timings{};
  vr::Result<vr::mesh::DeviceMesh> device_mesh =
      marching_cubes_->extract_device(*grid_, 0.0f, &extract_timings);
  if (!device_mesh) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "extract: " + device_mesh.status().message();
    return;
  }
  const float extract_ms = ms_since(t_extract);

  // Texture in place, on the device buffers marching cubes just wrote -- no
  // host round trip between the two GPU tiers. The current frame would be the
  // atlas, i.e. the live single-camera path (recon's 2026-07-07 decision).
  //
  // Off by default, and the reason is in FusionConfig::texture: nothing carries
  // frame.color across this seam yet, so the renderer binds a 1x1 white atlas
  // and every uv0 this pass writes selects white. Kept wired up rather than
  // deleted because the missing half is the atlas upload, not this call.
  float texture_ms = 0.0f;
  if (config_.texture) {
    const auto t_texture = Clock::now();
    const vr::Status textured = texturer_->texture(
        device_mesh.value(), frame.depth, frame.depth_camera);
    if (!textured) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error = "texture: " + textured.message();
    }
    texture_ms = ms_since(t_texture);
  }

  // No host copy. The renderer draws these very buffers -- interop seam B --
  // so the ~53 MB round trip per remesh that seam A cost is simply not here.
  // What replaces it is the release contract handled at the top of this
  // function: the slot this mesh lives in is not extracted into again until the
  // consumer says it has finished with it.
  std::lock_guard<std::mutex> lock(mutex_);

  mesh_ = device_mesh.value();
  published_generation_ = device_mesh.value().generation;
  published_taken_ = false;
  ++mesh_version_;
  ++stats_.remeshes;
  stats_.vertices = mesh_.vertex_count;
  stats_.triangles = mesh_.triangle_count;
  stats_.mesh_version = mesh_version_;
  stats_.extract_ms = extract_ms;
  // recon's struct, whole -- not a field-by-field transcription. `extract_ms`
  // above is this call measured from here; this is what it decomposes into.
  // See FusionStats::extract for why the copy is one assignment.
  stats_.extract = extract_timings;
  // Stamped beside the block count it is the denominator for. Occupancy is
  // `extract.active_blocks` over this, and a ratio whose two halves are
  // published on different cadences is wrong in whichever direction they
  // differ -- see FusionStats::table_capacity.
  stats_.table_capacity =
      static_cast<std::uint32_t>(config_.num_buckets * kBlocksPerBucket);
  // Stamped with the reading, because the reading is what `fuse`'s anti-hang
  // guards run on and a successful extract is the only thing that refreshes it.
  // Without the stamp there is no way to tell a live occupancy figure from one
  // frozen by an extract that has been failing for a minute.
  active_blocks_at_frame_ = stats_.frames_fused;
  active_blocks_measured_ = true;
  // This extract *is* the current one, so the read-out's staleness marker
  // clears here and nowhere else. `fuse` recomputes it every frame from
  // active_blocks_at_frame_ above.
  stats_.frames_since_extract = 0;
  stats_.extract_stale = false;
  stats_.texture_ms = texture_ms;
}

std::optional<Fusion::Published> Fusion::take_mesh(
    std::uint32_t known_version) {
  std::lock_guard<std::mutex> lock(mutex_);
  // The validity check does double duty: nothing has been meshed yet, or this
  // version has already been taken.
  if (mesh_version_ == known_version || !mesh_.valid()) {
    return std::nullopt;
  }
  // Copied, not moved -- it is a handful of handles and counts now, and the
  // buffers it names stay the extractor's. The consumer must call
  // release_through once the frames drawing them retire; marking it taken here
  // is what stops the next publish releasing a slot the renderer is using.
  published_taken_ = true;

  Published out;
  out.mesh = mesh_;
  out.version = mesh_version_;
  return out;
}

void Fusion::release_through(std::uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Recorded, not forwarded. recon's extractor is touched by the fuse thread
  // and only the fuse thread; `remesh` applies this at the top of its next run.
  // Forwarding from here would race `extract_device` -- recon's header names
  // that a caller obligation rather than making the member atomic -- and the
  // repair that would keep the call here, holding this mutex across the
  // extract, blocks the caller for a whole extract. The caller is the render
  // thread, which on this app is the main thread.
  //
  // Monotonic, matching the contract it stands in for: a generation already
  // released stays released, and a value older than the newest reported is
  // ignored rather than un-releasing anything.
  consumer_released_ = std::max(consumer_released_, generation);
}

void Fusion::note_error(const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.errors;
  stats_.last_error = message;
}

vr::Mat4f Fusion::last_pose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_pose_;
}

FusionStats Fusion::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  FusionStats out = stats_;
  // Computed at read time rather than published by the fuse thread, because the
  // question is how long ago that thread last ran -- which it cannot answer
  // about a frame it has not had. A zero stamp means no frame ever fused, where
  // "0 ms ago" would read as a scan that is running.
  const std::int64_t stamped = last_fuse_ns_.load(std::memory_order_relaxed);
  out.ms_since_fuse =
      stamped == 0 ? 0.0f
                   : std::chrono::duration<float, std::milli>(
                         std::chrono::steady_clock::now().time_since_epoch() -
                         std::chrono::nanoseconds(stamped))
                         .count();
  return out;
}

FusionTraceStats Fusion::trace_stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // Field by field rather than returning stats_, because the point is to leave
  // the string behind: copying FusionStats here would malloc inside this lock
  // on nearly every frame of a healthy scan. See FusionTraceStats.
  FusionTraceStats out;
  out.triangles = stats_.triangles;
  out.triangle_capacity = stats_.extract.triangle_capacity;
  out.arena_bytes = stats_.extract.arena_bytes;
  out.active_blocks = stats_.extract.active_blocks;
  out.extract_ms = stats_.extract_ms;
  // The table's live state, which the ring did not carry. `active_blocks` above
  // is stamped by a successful extract, so a dump taken in the
  // persistent-extract-failure regime -- which is the regime a device-lost dump
  // is read in -- reported a frozen count and could not say whether the
  // occupancy guard was engaged. See FusionTraceStats.
  out.occupancy = stats_.occupancy;
  out.allocation_stop = stats_.allocation_stop;
  return out;
}

}  // namespace volumetric_kit::ios_app

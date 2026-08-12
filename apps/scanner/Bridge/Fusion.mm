// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "Fusion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
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

/// The stage labels this file seeds, restated from the tiers that report them.
///
/// Restated rather than pointed at because recon has no constants to point at:
/// each tier passes its own literal, and `StageMetrics` matches rows by string
/// **content** precisely so a label named from two places still lands in one
/// row. That is what lets this file seed a row whose timing happens inside the
/// library -- and it is also the coupling: a label that drifts upstream splits
/// into two rows rather than failing to compile. Cheap to notice, since the
/// seeded row then reads 0.00 forever beside a duplicate that does not.
///
/// @ref kResizeStage is the exception -- it is this file's own row, for the
/// grow between allocate retries that `allocate_from_depth`'s own row does not
/// cover (`voxel_hash_map.hpp` says as much where it documents that row).
constexpr const char* kAllocateStage = "allocate";
constexpr const char* kResizeStage = "resize";
constexpr const char* kIntegrateStage = "integrate";
constexpr const char* kTextureStage = "texture";
/// The active-set compaction inside `integrate`, which recon names as a
/// breakdown of the row above it -- `StageMetrics::kBreakdownPrefix` plus the
/// stage's own name. Seeded in that position so the sub-row sits directly under
/// the row it decomposes.
constexpr const char* kActiveSetStage = "  ..active set";

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

/// Whether this configuration hands a consumer buffers the extractor owns.
///
/// The invariant behind kMinMeshSlots, named once so the floor and the publish
/// below cannot drift apart. The floor is not about a mode -- it is about the
/// borrow: `Published::mesh` is a *borrowed* DeviceMesh, so a ring is what
/// keeps the next extract off the buffers a live draw is reading. A
/// configuration that publishes nothing has nothing to corrupt.
///
/// Written as a predicate rather than inlined as `!incremental_benchmark` at
/// each site because the two sites are a safety check and the thing it protects
/// against. Keying the check on the mode name let them be changed
/// independently; keying both on this makes a future non-publishing mode safe
/// by construction and a publishing one refused the moment it takes one slot.
constexpr bool publishes_mesh(const FusionConfig& config) {
  return !config.incremental_benchmark;
}

/// The slot count the EXTRACTOR is built with, which is not always the request.
///
/// recon refuses a ring for `extract_device_incremental` outright: a re-meshed
/// block writes into the arena the last extract filled, and a ring hands this
/// one a different slot. So the measurement mode runs at one and the read-out
/// reports this rather than `FusionConfig::mesh_slots`.
constexpr std::uint32_t effective_mesh_slots(const FusionConfig& config) {
  return config.incremental_benchmark ? 1u : config.mesh_slots;
}

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
  // ... except where nothing borrows those buffers, which is what the floor is
  // really about. Keyed on `publishes_mesh` and not on the mode name, so the
  // exemption belongs to the property rather than to one flag that happens to
  // have it -- and so a configuration that publishes while asking for one slot
  // is still refused, whatever it calls itself. The refusal also cannot be
  // talked around by setting mesh_slots to 1.
  if (publishes_mesh(config) && config.mesh_slots < kMinMeshSlots) {
    return vr::Status::invalid_argument(
        "FusionConfig::mesh_slots is " + std::to_string(config.mesh_slots) +
        ", which switches recon's slot-release contract off; Published::mesh "
        "borrows the extractor's buffers, so it needs at least " +
        std::to_string(kMinMeshSlots) +
        " (and the consumer's frames in flight "
        "plus one to actually pipeline).");
  }
  // Refused here, beside the guard above and *before* anything is written.
  // Through the setter rather than stored directly, so a config carrying a
  // nonsense value is refused instead of reaching the kernel -- and the setter
  // stores nothing on the path that returns false, so a refusal leaves this
  // object exactly as it found it.
  //
  // That ordering is the whole point of the move. This check used to sit forty
  // lines below, after `config_ = config` and after the locked block had wiped
  // `stats_`, the frame history and the published keyframe: a refused config
  // was already installed and the previous scan's read-out was already gone, so
  // a Fusion that answered `valid()` from an earlier successful start went on
  // describing a configuration this method had just rejected. Reachable by any
  // caller that *computes* the tolerance rather than typing it -- a NaN out of
  // a division is the ordinary way in. Both guards now refuse the same way: no
  // state touched, nothing to unwind.
  if (!set_occlusion_threshold(config.occlusion_threshold)) {
    return vr::Status::invalid_argument(
        "FusionConfig::occlusion_threshold must be finite and >= 0");
  }
  config_ = config;
  // Normalized to what this scan will ACTUALLY run with, not to what was asked
  // for. The measurement mode implies dirty tracking -- the flags its extract
  // dilates on-device are exactly those -- and storing the raw request left
  // `config_.track_dirty_blocks` false while the integrator had it on. Every
  // later reader of the member then described a configuration that was not
  // running, the costly one being the note in `fuse` that names this allocation
  // when an integrate fails for want of it: it is gated on this flag, so the
  // mode that caused the failure was the one configuration that suppressed the
  // sentence explaining it.
  config_.track_dirty_blocks =
      config.track_dirty_blocks || config.incremental_benchmark;
  // Cleared, not carried. Everything in here is about the scan that just ended,
  // and several fields are one-way latches -- `allocation_stop` and `occupancy`
  // most of all, which would otherwise have a new scan open still announcing
  // the full volume that ended the last one. The per-scan bookkeeping outside
  // `stats_` goes with them, for the same reason.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_ = FusionStats{};
    // What the EXTRACTOR got, not what the caller asked for -- through the same
    // helper that feeds MarchingCubesConfig, so the two cannot disagree.
    // Reporting the request instead had the read-out saying "3 slots" while one
    // was in use: the one line that would have shown the mode was on, saying it
    // was off.
    stats_.mesh_slots = effective_mesh_slots(config);
    // The two figures the mode makes non-comparable, and the flag that explains
    // an empty view. See FusionStats::spans_tracked and
    // ::incremental_benchmark.
    stats_.spans_tracked = config.incremental_benchmark;
    stats_.incremental_benchmark = config.incremental_benchmark;
    // Seeded rather than left at the struct default, so the gap before the
    // first remesh reads as "on, nothing measured yet" instead of "off". Those
    // are the two states a reader most needs told apart while a scan is opening
    // and no keyframe has appeared: one is waiting, the other is misconfigured.
    stats_.texture_state =
        config.texture ? TextureState::Pending : TextureState::Off;
    stats_.occlusion_threshold = config.occlusion_threshold;
    // The frame history goes with them, and it is the one that reads worst if
    // it does not: `frames_fused` restarts at zero just above, so a ring left
    // full would hand a chart the last scan's 240 samples carrying *higher*
    // frame indices than the live ones arriving behind them -- a series that
    // runs backwards in the middle, with a timestamp gap the length of however
    // long the app sat between scans. Only the counter needs clearing; the
    // entries themselves become unreachable once it is zero.
    history_next_ = 0;
    // With the same reasoning: a new scan must not hand out the previous one's
    // keyframe. The ring's storage is deliberately kept -- it is the capacity
    // that makes the per-remesh copy a memcpy rather than a fresh 11 MB
    // mapping -- but nothing may be published out of it until a remesh fills it
    // again.
    published_atlas_ = nullptr;
    atlas_width_ = 0;
    atlas_height_ = 0;
    // ...and the mesh half with it, which is the same argument and the more
    // dangerous omission. Clearing only the keyframe left the pair internally
    // asymmetric: `mesh_` still named the *previous* scan's DeviceMesh, whose
    // buffers belong to a MarchingCubes that `marching_cubes_.emplace()` below
    // is about to replace. A renderer whose `uploaded_version` was stale would
    // take that mesh on its next frame -- a destroyed VkBuffer under a live
    // vkCmdDrawIndexedIndirect, which on this configuration is a device loss
    // with no validation message.
    //
    // `consumer_released_` goes too, and for its own reason: it is a monotonic
    // high-water mark over the *old* extractor's generations, and carrying it
    // into a new one would hand recon a release covering slots the fresh
    // extractor has not written yet on its very first remesh.
    //
    // `published_taken_ = true` rather than false -- it is the member's own
    // default and it means "nothing outstanding", which is exactly the state a
    // scan starts in. False would claim a mesh is waiting to be collected.
    mesh_ = vr::mesh::DeviceMesh{};
    published_generation_ = 0;
    published_taken_ = true;
    consumer_released_ = 0;
    // `mesh_version_` deliberately NOT reset, and it is the one member here
    // where resetting is worse than keeping. It is a change-detection token,
    // not a per-scan count: `take_mesh` returns nothing when it equals the
    // version the caller already has. Restart it at zero and the new scan's
    // first publish can land on a number the renderer is still holding from the
    // old one -- take_mesh then says "nothing new", and the renderer goes on
    // drawing the previous scan's DeviceMesh, which is precisely the
    // use-after-free the rest of this block exists to prevent. Monotonic across
    // scans, it cannot collide.
    //
    // Note this block can only clear what *this* object owns. A renderer that
    // has already taken a mesh keeps naming those buffers until its next
    // successful take, so a genuine mid-session restart also needs the consumer
    // to drop what it holds. Nothing wires that today because `start` is called
    // exactly once, from -initWithLayer: -- which is a property of the caller,
    // not of this method, and is why the rest of this is worth doing now.
  }
  last_fuse_ns_.store(0, std::memory_order_relaxed);
  // With the same reasoning, and `gpu_timing_seen_` most of all: it is a
  // one-way latch, so a scan that carried it in would report the *next* scan's
  // ordinary host-only first frames as a retired timer.
  last_stages_ns_.store(0, std::memory_order_relaxed);
  gpu_timing_seen_ = false;
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
  // it -- the survey in `fuse`, or the incremental extract in `remesh`, is the
  // whole reason the counters exist. (Never both; see the survey's own gate.)
  //
  // Taken from the config rather than pinned on here, and that is the point of
  // the field: recon allocates the flag array inside `integrate` and rebuilds
  // it on every map grow, so a failure to get it fails the *frame*, not just
  // the diagnostic. See FusionConfig::track_dirty_blocks.
  vr::tsdf::TsdfIntegratorConfig integ_config;
  // From the NORMALIZED member, not the raw argument. The mode implies the
  // tracking -- the flags its extract dilates on-device are these -- and
  // `config_` was already folded above, so reading it here is what keeps the
  // stored configuration and the running one the same thing.
  integ_config.track_dirty_blocks = config_.track_dirty_blocks;
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
        // On together with FusionConfig::texture, which is a change from what
        // this comment used to say and is deliberate. The old pairing was:
        // ProjectiveTexturer decides visibility per *triangle* and writes uv0
        // per *vertex*, so a shared vertex has several triangles racing to
        // write it and recon refuses the mesh outright rather than letting the
        // write order decide. A per-vertex dispatch settles that at the source
        // -- each thread owns one vertex and its own uv0 -- and lifts the
        // refusal without anything changing here.
        //
        // **Do not "fix" a texturing failure by turning this off.** That is the
        // instruction the previous comment gave and it is the wrong trade twice
        // over. It does not repair anything a per-vertex recon does not already
        // repair; and unsharing roughly triples the vertex count for the same
        // surface, in the extractor arena, on the device the paragraph below
        // calls the place the arena ceiling is real -- against a saving nothing
        // has yet measured on hardware. If the texture pass is failing, read
        // the reason: a recon that still refuses shared meshes reports exactly
        // that in FusionStats::last_error with TextureState::Failed beside it,
        // and the repair is the sibling revision, not this line.
        //
        // Roughly a 3x vertex reduction for the same surface; the exact figure
        // is a property of the scan, so trust this device's own read-out
        // (`arena_bytes` against the triangle count) over any number quoted
        // from a desktop fixture. Memory is why it is on here: an iPad is where
        // the arena ceiling is real.
        //
        // Sharing stays ON in the benchmark mode too, as of recon's
        // sharing-incremental change: that kernel reserves two per-block ranges
        // and now reuses them, and it retires a dead triangle through its own
        // index run for 12 bytes rather than 192. Turning it off cost 3x the
        // vertex arena -- 4177 MB measured on this device against 33 MB in the
        // normal configuration -- which made the first measurement
        // unrepresentative of anything shippable.
        //
        // Two settings still move with the mode, and both cost something the
        // read-out has to disclose. The slot count drops to one because recon
        // REFUSES a ring for an incremental extract -- a re-meshed block writes
        // into the arena the last extract filled, and a ring hands this one a
        // different slot -- through the same helper `stats_.mesh_slots` reports
        // from, so what is measured and what is displayed cannot disagree.
        // `track_block_spans` is the table it re-meshes against, and recon
        // folds that table into `arena_bytes` and its stamping loop into
        // `arena_alloc_ms`; `stats_.spans_tracked` is what says so beside them.
        mc_config.share_vertices = true;
        mc_config.slot_count = effective_mesh_slots(config);
        mc_config.track_block_spans = config.incremental_benchmark;
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

  // One reporting window per fused frame. The tiers open their own host scopes
  // and publish their device spans into it, so each row carries a GPU half no
  // wall-clock span around a fence-blocked submit can show.
  //
  // Every call below passes `metrics`, never `&stages`: null is what makes
  // FusionConfig::measure_stages an actual switch, because recon measures
  // nothing *and takes its untimed submit path* on a null. Passing the address
  // unconditionally would leave a per-dispatch query-pool round trip armed at
  // capture rate with no way to turn it off short of a rebuild.
  vr::StageMetrics stages;
  vr::StageMetrics* metrics = config_.measure_stages ? &stages : nullptr;
  if (metrics != nullptr) {
    // Seeded so the table keeps its shape frame to frame. A row that appears
    // only when its stage ran makes every row below it jump, which is what
    // makes a live read-out unreadable -- and worse in a chart indexed by
    // position, where the series silently relabel instead. The two that need it
    // most are the ones a frame routinely skips: `allocate` is short-circuited
    // whole by the occupancy guard below, so recon never reaches its own seed,
    // and `resize` runs a handful of times in a scan's life.
    //
    // Seeded in report order, which is why the breakdown is named here rather
    // than left to arrive on its own: rows are kept in first-seen order, so
    // seeding `texture` before the integrate ran would put the active-set
    // sub-row *below* texture instead of beneath the row it decomposes.
    //
    // This is also what keeps a row from ever being appended inside a
    // destructor. StageScope and GpuStageScope add their rows from `~T()`,
    // which is implicitly noexcept, so a vector growth that threw there would
    // be std::terminate rather than an unwind -- and past the fuse thread's
    // exception guard, which exists because a throw out of a thread function
    // takes the app with no message and no crash context. Every row a frame can
    // produce exists before the first scope opens, so those destructors find
    // rows rather than push_back onto them; the vector's own geometric growth
    // (eight slots for these five) leaves headroom besides.
    stages.seed(kAllocateStage);
    stages.seed(kResizeStage);
    stages.seed(kIntegrateStage);
    stages.seed(kActiveSetStage);
    // Seeded last of these and still not last in the published table: the
    // texture pass runs inside `remesh`, *after* the extract, and the extract's
    // rows are only appended at publish time. So this one is lifted out of
    // first-seen order and re-appended after them -- see the publish block. The
    // seed still has to happen here, before any scope opens, for the
    // noexcept-destructor reason above; it is the *display* position that
    // cannot be expressed by seeding.
    if (config_.texture) {
      stages.seed(kTextureStage);
    }
  }

  // Anything this frame got wrong, carried to the single publish below rather
  // than written straight into stats_. The success path used to clear
  // last_error unconditionally, which meant a frame fused with missing blocks
  // left no trace anywhere -- the read-out showed a rising vertex count and an
  // empty error line while a whole region of the scan quietly never filled.
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
    } else if (const vr::Status grown = [&] {
                 // Its own row, because nothing else covers it. recon's
                 // `allocate` row spans `allocate_from_depth`'s own dispatches
                 // and voxel_hash_map.hpp says so where it documents that row,
                 // directing a caller to give a resize between retries a row of
                 // its own. This is the *other* doubling -- the preemptive one
                 // -- and it is the more expensive half: a grow toward the
                 // 32768-bucket ceiling commits ~1.5 GiB with ~2.3 GiB
                 // transient, hundreds of milliseconds that would otherwise
                 // appear in no row the read-out prints.
                 vr::StageScope span(metrics, kResizeStage);
                 return grid_->resize(grown_to);
               }()) {
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
                                             &failures, metrics);
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
    // Accumulates into the same `resize` row as the preemptive doubling above,
    // which is the honest total: both are the same operation at the same cost,
    // and a frame that reaches this loop has usually skipped that one.
    const vr::Status grown = [&] {
      vr::StageScope span(metrics, kResizeStage);
      return grid_->resize(grown_to);
    }();
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
                                                &failures, metrics);
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
      metrics);
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
      //
      // Reads the normalized member, so it fires under the measurement mode too
      // -- which implies the flag. That case gets its own second sentence,
      // because the first one's advice does not apply there: the mode cannot
      // run without these flags, so the way out is to stop measuring rather
      // than to turn a diagnostic off.
      stats_.last_error +=
          " -- note: FusionConfig::track_dirty_blocks is on, which allocates a "
          "num_blocks*4 flag array inside integrate on every map grow";
      stats_.last_error +=
          config_.incremental_benchmark
              ? "; VI_INCREMENTAL_BENCHMARK implies it (the incremental "
                "extract "
                "reads these flags), so it cannot be turned off without "
                "leaving "
                "the measurement mode"
              : "; turning it off takes the dirty survey with it but removes "
                "that allocation from this path";
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
    remesh(frame, metrics);
  }

  // --- Publish the stage rows -----------------------------------------------
  //
  // After the remesh, not with the block above, and that order is the only one
  // that can carry a `texture` row at all: `stages` is this function's local,
  // remesh writes into it, and copying it out beforehand published a window
  // whose texture row was still the zero this function seeded.
  //
  // Its own critical section rather than widening that one, because widening it
  // would hold the lock the main thread takes four times per rendered frame
  // across the extract -- 132.7 ms on this device's own measurement.
  //
  // Only when measurement is on. `stage_count` then stays at its zero, which is
  // what the read-out branches on to fall back to `allocate_ms` /
  // `integrate_ms`
  // -- so an off switch costs the device column and no timing at all.
  if (metrics != nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Copied whole rather than picked apart: every field recon adds to a row
    // arrives without a lockstep edit here, and the read-out reads the same
    // rows the bridge publishes -- one source, not two that can disagree.
    stats_.stage_count = 0;
    stats_.stages_truncated = false;
    bool any_gpu = false;
    // Held back and re-appended after the extract below, because that is where
    // it belongs in the pipeline and the seed order cannot put it there.
    //
    // `fuse` seeds its rows at the top of the frame, before any of them has
    // run, so that the StageScope destructors find rows rather than push_back
    // onto them -- see the seed comment for why a growth there would be
    // std::terminate. Rows then keep first-seen order, which put `texture`
    // fifth, above an `extract` that is only appended down here. But the
    // texture pass runs *inside* remesh, after the extract: the published table
    // was drawing a ~4 ms row above the ~130 ms one that precedes it, on both
    // the text summary and the dashboard's stage bars.
    //
    // That order was right while texture was a no-op printing 0.0 and nobody
    // could see it. Reordering here rather than at either read-out keeps one
    // source: the bridge publishes the pipeline end to end, in order, and the
    // consumers keep drawing what they are handed.
    const vr::StageRow* texture_row = nullptr;
    for (const vr::StageRow& row : stages.rows()) {
      if (stats_.stage_count >= FusionStats::kMaxStages) {
        // Recorded rather than dropped in silence: a full array reads exactly
        // like a frame that happened to have this many rows, so a consumer
        // summing the column would under-report with nothing to notice it by.
        stats_.stages_truncated = true;
        break;
      }
      any_gpu = any_gpu || row.has_gpu;
      // By name, not by pointer identity: the row this matches was seeded from
      // kTextureStage here but recon's own scope is what fills it, and nothing
      // promises the label it carries back is the same pointer.
      if (row.name != nullptr && std::strcmp(row.name, kTextureStage) == 0) {
        texture_row = &row;
        continue;
      }
      stats_.stages[stats_.stage_count++] = row;
    }
    // A device half that was there and is not any more is a *retired* timer,
    // not a device that never had timestamps -- and only this latch can tell
    // those apart, because the aftermath looks identical in every row. See
    // FusionConfig::measure_stages for what retires one and why it is
    // permanent.
    gpu_timing_seen_ = gpu_timing_seen_ || any_gpu;
    stats_.gpu_timing_retired = gpu_timing_seen_ && !any_gpu;

    // Extract, appended so the published stages are the pipeline end to end --
    // one list the panel can draw, rather than two a reader has to rejoin.
    //
    // Here rather than beside `frames_fused`, and that is the whole of why this
    // publish moved below the remesh: `stats_.extract*` is written by `remesh`,
    // so appending these in the block above published the *previous* remesh's
    // phases as this frame's, unmarked. Above, they are fresh by construction.
    //
    // They do not report through StageMetrics -- recon wired
    // volume/tsdf/texture and left mesh's GPU column for later -- so these come
    // from ExtractTimings and carry `has_gpu = false`. That is honest rather
    // than unfortunate: the rows without a device half are exactly the part of
    // the pipeline still measured on the host alone, and a reader can see
    // which.
    //
    // The literals outlive any read (StageRow borrows its label), and the
    // "  .." prefix marks a phase as a breakdown of the extract above it, which
    // is what keeps totals from counting it twice.
    const auto push_stage = [this](const char* name, double cpu_ms) {
      if (stats_.stage_count >= FusionStats::kMaxStages) {
        stats_.stages_truncated = true;
        return;
      }
      stats_.stages[stats_.stage_count++] =
          vr::StageRow{name, cpu_ms, 0.0, false};
    };
    push_stage("extract", stats_.extract_ms);
    push_stage("  ..meshing", stats_.extract.dispatch_ms);
    push_stage("  ..compact", stats_.extract.compact_ms);
    push_stage("  ..inputs", stats_.extract.input_upload_ms);
    // Named for what it holds rather than what it usually holds. recon charges
    // the per-extract block-span stamping loop -- O(active blocks) on the host,
    // plus two mapped reads per active block on the incremental path -- to
    // `arena_alloc_ms` alongside the arena sizing this row is named after. That
    // loop runs only when the span table is on, so in that configuration this
    // row is not the same measurement the normal build's is, and comparing the
    // seven extract phases across the two silently compares different things.
    push_stage(stats_.spans_tracked ? "  ..sizing+spans" : "  ..sizing",
               stats_.extract.arena_alloc_ms);
    push_stage("  ..desc", stats_.extract.descriptor_ms);
    push_stage("  ..readback", stats_.extract.readback_ms);
    // The residual, published rather than left implicit -- the same seven cells
    // the text summary carries, for the reason it gives: these phases do not
    // sum to `extract_ms` and never did. recon's spans open after the slot
    // claim and close before the O(active_blocks) teardown of the neighbour
    // table, so the gap grows with the scan. That is the one direction in which
    // an unlabelled remainder is misread as rounding, and on the panel it read
    // as a breakdown that visibly failed to add up to the row above it.
    push_stage("  ..other",
               std::max(0.0, static_cast<double>(stats_.extract_ms) -
                                 stats_.extract.total_ms()));
    // ...and texture last, which is when it runs. Copied whole rather than
    // pushed through `push_stage`: this row is the one place in the table with
    // a real device half, and push_stage writes `has_gpu = false` because the
    // extract phases it was written for genuinely have none. Routing it through
    // there would drop the gpu column on the only stage that has one.
    if (texture_row != nullptr) {
      if (stats_.stage_count >= FusionStats::kMaxStages) {
        stats_.stages_truncated = true;
      } else {
        stats_.stages[stats_.stage_count++] = *texture_row;
      }
    }
    // No *new* `texture` row is pushed here -- the one above is recon's own,
    // moved. `fuse` seeds it at the top of the frame and the texture tier
    // reports into it through `metrics`, with a device half this host-only span
    // does not have. Building a second from `stats_.texture_ms` is how the same
    // stage ends up drawn twice with two different numbers, since that span
    // also covers the transient buffer setup and the fence: the row would be
    // the dispatch and the line beneath it the whole call, and nobody reading a
    // column of milliseconds can tell those apart. The text summary picks one
    // of the two rather than printing both -- see where it builds the table.

    // Stamped here and only here. The rows publish on this path alone, so this
    // is what the four early returns above leave standing still -- see
    // FusionStats::ms_since_stages for why a frame count cannot say it.
    last_stages_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              Clock::now().time_since_epoch())
                              .count(),
                          std::memory_order_relaxed);
  }

  // --- Sample this frame into the history -----------------------------------
  //
  // Below the remesh for the same reason the publish above is, and it is the
  // whole point of the sample: `stages` is this function's local and remesh
  // writes into it, so sampling beside `frames_fused` would total a window the
  // meshing had not been added to yet -- the spike this ring exists to catch,
  // taken just before the thing that causes it.
  //
  // Unconditional, unlike that publish: one entry per fused frame is what keeps
  // `frame` a dense index, so a consumer can align two samples by subtracting
  // them. It is emphatically not a gap indicator -- it is incremented on this
  // same path and so never skips a value; `timestamp_ns` is what carries the
  // time a stopped fuse loop spent stopped.
  //
  // With measurement off the stage totals are the zero `stages` still holds,
  // which reads the same way `stage_count` does -- no timing rather than a fast
  // frame. `extract_ms` is unaffected: it is timed by `remesh` directly and
  // does not depend on the switch.
  //
  // Its own critical section, not a widening of either neighbour: see the note
  // above on holding this lock across an extract.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    FrameSample& sample = history_[history_next_ % kHistoryCapacity];
    ++history_next_;
    sample = FrameSample{};
    sample.frame = stats_.frames_fused;
    // Breakdown rows excluded from the host total (they restate their parent's
    // time) and INCLUDED in the device total (a device span covers one
    // dispatch, never the compaction the stage ran first) -- StageMetrics'
    // totals already draw that distinction, so take it from them rather than
    // re-deriving it here and getting it differently.
    //
    // Both cover allocate, resize, integrate and texture. They do *not* cover
    // the extract, which reports through `ExtractTimings` and not the row set,
    // so it is carried alongside as its own figure rather than left out of the
    // history altogether -- see `sample.extract_ms` below.
    sample.host_ms = static_cast<float>(stages.total_cpu_ms());
    sample.device_ms = static_cast<float>(stages.total_gpu_ms());
    // Whether that device figure is a measurement or an absence. `total_gpu_ms`
    // skips every row without a span, so a queue family that reports no
    // timestamps sums to the same 0.0 as a frame that dispatched nothing, and
    // only this flag separates them. Same rule the read-out follows when it
    // prints `gpu -` instead of `0.00`.
    for (const vr::StageRow& row : stages.rows()) {
      if (row.has_gpu) {
        sample.device_timing_valid = true;
        break;
      }
    }
    // The extract, which neither total above can see. This is the figure the
    // ring was built for: at `remesh_every` 1 it lands on every frame, and it
    // is an order of magnitude larger than the fuse it sits beside.
    //
    // Read after the remesh, so on a frame that extracted this is that
    // extract's own cost. `frames_since_extract` below is what says whether it
    // is -- remesh leaves both standing when it skips or fails.
    sample.extract_ms = stats_.extract_ms;
    sample.frames_since_extract = stats_.frames_since_extract;
    sample.occupancy = occupancy;
    sample.occupancy_known = occupancy_known;
    sample.allocation_stop = allocation_stop;
    sample.triangles = stats_.triangles;
    sample.active_blocks = stats_.extract.active_blocks;
    // Stamped last and under the same lock, because the gaps worth seeing are
    // the ones that produce no frames at all and `frame` cannot show them: it
    // is incremented on this same path, so it never skips a value however long
    // the fuse loop was stopped.
    sample.timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch())
            .count());
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
  //
  // NOT RUN under the measurement mode, and that is a correctness gate rather
  // than a saving. This survey reads the integrator's dirty flags and then
  // resets them; so does the incremental extract in `remesh`, on a cadence with
  // nothing to do with this one. The fuse kernel only ORs into the flags, so
  // two owners make the window each describes drift out of step with the other:
  // at `remesh_every` 1 every extract would see the union of up to sixty fuses,
  // and at 7 the flags for frames 57-60 would be zeroed before any extract read
  // them -- those blocks reading clean and keeping triangles the fuse
  // invalidated. recon refuses the same pairing outright in its own harness
  // ("run one or the other") rather than picking for the caller, and the
  // extract is the owner here because it is the thing being measured. The
  // survey's own re-arm after a topology change is not lost with it: the reset
  // beside the extract is the same call.
  if (!config_.incremental_benchmark &&
      stats_.frames_fused % kSurveyEveryFrames == 0) {
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

void Fusion::remesh(const vr::sensor::CapturedFrame& frame,
                    vr::StageMetrics* metrics) {
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
  //
  // INERT under the measurement mode, and deliberately left so rather than
  // faked. `published_taken_` is armed only by the publish below, which that
  // mode does not reach, so `uncollected` is false for the session and every
  // remesh interval extracts. There is no consumer to throttle against and
  // simulating one would be inventing a cadence, so the honest fix is to make
  // the difference legible instead: the shipping build can let a second frame's
  // worth of dirt accumulate behind a skipped remesh, and a shorter window is
  // simply less dirt and flatters the incremental path. What each reading
  // actually covers is published as FusionStats::extract_window_frames, which
  // is why `remeshed_blocks` is never shown without it.
  if (uncollected) {
    return;
  }

  const auto t_extract = Clock::now();
  vr::mesh::ExtractTimings extract_timings{};
  //
  // The incremental overload re-meshes only the blocks whose +{0,1}^3
  // neighbourhood the fuse changed, dilating the integrator's flags on-device.
  // It falls back to a full extract on its own whenever an incremental pass
  // would be wrong -- the first one against this grid, a topology change, a
  // grown arena -- so this needs no first-frame special case.
  //
  // All THREE fields, and the epoch is the one that is easy to lose: recon
  // added `DirtyBlocks::epoch` after this branch was written (7aed36d), and a
  // brace-init that omits it still compiles -- the missing member simply
  // value-initializes to 0. But `topology_epoch()` is 0 only on a moved-from
  // map, so a zero epoch matches no live grid, recon's guard refuses the
  // incremental pass, and every extract silently falls back to the full one
  // this mode exists to measure against. Read from the integrator that
  // accumulated the flags, because that is what the token has to agree with.
  vr::Result<vr::mesh::DeviceMesh> device_mesh =
      config_.incremental_benchmark
          ? marching_cubes_->extract_device_incremental(
                *grid_, 0.0f,
                vr::mesh::DirtyBlocks{integrator_->dirty_flags_buffer(),
                                      integrator_->dirty_flags_capacity(),
                                      integrator_->dirty_epoch()},
                &extract_timings)
          : marching_cubes_->extract_device(*grid_, 0.0f, &extract_timings);
  if (!device_mesh) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "extract: " + device_mesh.status().message();
    return;
  }
  const float extract_ms = ms_since(t_extract);

  // Consumed, so cleared -- here, immediately after the extract that read them,
  // rather than on a cadence of its own. This is recon's stated contract for
  // the overload and the reason the survey above stands down in this mode: the
  // fuse kernel only ORs into the flags, so anything but "reset where they were
  // read" makes the window they describe drift out of step with the window
  // between extracts. Too long and every block reads dirty, which is a full
  // re-mesh wearing the incremental path's costs; too short and blocks that
  // really changed read clean and keep triangles the fuse invalidated.
  //
  // Reset even when the call fell back to a full pass, which is invisible from
  // here: a full pass re-meshes everything, so the flags it did not read are
  // just as spent as the ones it did. NOT reset on the failure path above --
  // nothing was re-meshed there, and clearing would drop those changes for
  // good rather than letting the next extract pick them up.
  //
  // The window is stamped with it, because `remeshed_blocks` cannot be read
  // without knowing how many fuses it accumulated over. See
  // FusionStats::extract_window_frames.
  std::uint64_t extract_window = 0;
  if (config_.incremental_benchmark) {
    integrator_->reset_dirty();
    extract_window = stats_.frames_fused - extract_window_start_;
    extract_window_start_ = stats_.frames_fused;
  }

  // Texture in place, on the device buffers marching cubes just wrote -- no
  // host round trip between the two GPU tiers. The current frame would be the
  // atlas, i.e. the live single-camera path (recon's 2026-07-07 decision).
  //
  // On now, and both halves it used to wait on are built: Published carries the
  // keyframe beside the mesh and the renderer holds a persistent ring of atlas
  // images indexed by the same slot. See FusionConfig::texture -- including its
  // warning that the flag being on is not the same as the pass running.
  float texture_ms = 0.0f;
  // The tolerance this remesh actually ran at, read once and published with the
  // result. Read once rather than twice because it is turnable mid-scan: the
  // main thread can move it between the call and the publish, and a read-out
  // reporting a value the pass did not use is worse than none -- it is the
  // number someone correlates a visible change against.
  const float threshold = occlusion_threshold_.load(std::memory_order_relaxed);
  // What the pass did, carried beside the duration because the duration cannot
  // say it: 0.0 ms is "off", "skipped", "refused before dispatching" and "ran
  // and cost nothing" all at once. See TextureState.
  //
  // "Off" under the measurement mode whatever the flag says, because that is
  // what it is: the pass and its keyframe are both skipped below, so reporting
  // anything else would name a stage this remesh did not run.
  TextureState texture_state = (config_.texture && publishes_mesh(config_))
                                   ? TextureState::NoColor
                                   : TextureState::Off;
  // Whether it *succeeded*, which is what the keyframe publish below is gated
  // on -- distinct from whether it was attempted. See there.
  bool textured_ok = false;
  // Gated on the frame actually carrying colour, not on the switch alone. A
  // frame whose colour was refused -- `convert_color` drops an unsupported
  // pixel format, an HLG or PQ transfer, a primaries set with no enumerator --
  // still fuses its depth and still reaches here, and texturing it would write
  // real `uv0` addressing an atlas this remesh never uploads. The renderer
  // would then sample whatever that slot last held: a previous keyframe's
  // image, smeared across every surface the camera can currently see. Skipping
  // leaves every `uv0` at marching cubes' sentinel, so the frame renders as
  // fused voxel colour -- which is exactly what a frame with no colour should
  // look like.
  //
  // And gated on this configuration publishing anything at all. Nothing takes
  // the mesh in the measurement mode, so the texture dispatch and the ~11 MB
  // keyframe copy below would both be computed and dropped -- work whose only
  // effect is on the fuse thread this mode exists to time. The dispatch runs
  // over the whole vertex buffer and contends for the queue the extract was
  // just measured on, so leaving it in does not merely waste effort, it moves
  // the number. `publishes_mesh` rather than the mode name, for the same reason
  // the slot floor uses it: the condition is that nothing consumes the result.
  const bool do_texture =
      config_.texture && frame.has_color() && publishes_mesh(config_);
  if (do_texture) {
    const auto t_texture = Clock::now();
    // The window reaches this tier too. `fuse` seeds the row when this flag is
    // on, so the table keeps its shape across the frames between remeshes --
    // and across a remesh that returns early above, which is the ordinary
    // steady state rather than a fault.
    const vr::Status textured =
        texturer_->texture(device_mesh.value(), frame.depth, frame.depth_camera,
                           threshold, metrics);
    textured_ok = textured.ok();
    texture_state = textured_ok ? TextureState::Ran : TextureState::Failed;
    if (!textured) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error = "texture: " + textured.message();
    }
    texture_ms = ms_since(t_texture);
  }

  // --- Stage the keyframe this mesh's uv0 index into -----------------------
  //
  // Copied rather than pointed at: `frame.color` is a view into the capture's
  // rotating staging buffers, valid only until its next poll -- and this thread
  // is what polls, so the pointer is stale by the time the renderer looks at
  // it. ~11 MB memcpy at ARKit's 1920x1440, which the capture path already
  // prices at ~0.06 ms against this device's measured unified-memory
  // bandwidth.
  //
  // Written into the entry the consumer is NOT reading; see `atlas_ring_`.
  // Outside the publish mutex deliberately: this is the largest single copy on
  // the fuse thread, and `mutex_` is the lock the main thread takes several
  // times per rendered frame. Safe because the entry being written is by
  // construction the one no consumer holds.
  const std::uint32_t* staged_atlas = nullptr;
  std::uint32_t staged_w = 0;
  std::uint32_t staged_h = 0;
  float atlas_copy_ms = 0.0f;
  // Gated on the pass having SUCCEEDED, not on the decision to attempt it.
  //
  // Those are different conditions and the difference is the whole of this
  // guard. A non-OK `texture` above bumps `stats_.errors` and falls through --
  // it does not return -- so gating on `do_texture` alone staged 11 MB and
  // published an atlas for a mesh whose `uv0` the failed pass never wrote. The
  // renderer then binds a real keyframe against a mesh that is entirely
  // sentinel, which is the pairing Published::atlas promises cannot happen, and
  // burns 11 MB host plus 11 MB of GPU copy per remesh to do it.
  //
  // Not a hypothetical: recon refuses a shared-vertex mesh until its per-vertex
  // dispatch lands, and this file asks for shared vertices (see start()), so
  // against such a revision this is *every* remesh rather than a rare one. It
  // is also the right answer on a partial failure, where some `uv0` are real
  // and some are sentinel: the atlas would be honest for one half of the mesh
  // and wrong for the other, with nothing to say which.
  if (do_texture && textured_ok) {
    const auto t_atlas = Clock::now();
    const std::uint32_t next = (atlas_slot_ + 1u) % 2u;
    const std::size_t pixels =
        static_cast<std::size_t>(frame.color_camera.width) *
        static_cast<std::size_t>(frame.color_camera.height);
    std::vector<std::uint32_t>& dst = atlas_ring_[next];
    // resize() rather than assign(): the vector keeps its capacity across
    // remeshes, so the 11 MB mapping is faulted in once per session instead of
    // once per frame -- the same reason ARKitCapture rotates its buffers by
    // swap rather than move-assigning fresh ones.
    dst.resize(pixels);
    std::memcpy(dst.data(), frame.color, pixels * sizeof(std::uint32_t));
    atlas_slot_ = next;
    staged_atlas = dst.data();
    staged_w = frame.color_camera.width;
    staged_h = frame.color_camera.height;
    // Measured, because nothing else here covers it. `texture_ms` closed above
    // and the extract span closed before that, and recon's StageMetrics only
    // ever sees what recon runs -- so this copy, the one cost this tier adds to
    // the fuse thread, was the single figure the read-out could not see. The
    // comment above prices it at ~0.06 ms; that is a claim about warm memory
    // and steady state, and a cold first-touch resize or an 11 MB vector being
    // faulted back in under pressure is exactly when it stops being true. If it
    // grows, the fuse loop slows and every published number moves with it --
    // ms_since_fuse and the frame chart included -- so this is what says why.
    atlas_copy_ms = ms_since(t_atlas);
  }

  // No host copy of the GEOMETRY. The renderer draws those very buffers --
  // interop seam B --
  // so the ~53 MB round trip per remesh that seam A cost is simply not here.
  // What replaces it is the release contract handled at the top of this
  // function: the slot this mesh lives in is not extracted into again until the
  // consumer says it has finished with it.
  std::lock_guard<std::mutex> lock(mutex_);

  // The benchmark mode stops HERE, and that is the whole of what makes its one
  // slot safe. Publishing would hand the renderer buffers the next extract
  // overwrites in place, with no release contract to hold it off -- the exact
  // failure kMinMeshSlots refuses, a destroyed or rewritten VkBuffer under a
  // live draw, raising no Status and no validation message on iOS. So the mesh
  // is measured and dropped, and the counters below still report what the
  // extract produced.
  //
  // Nothing is drawn while it runs. Not "the renderer keeps showing its last
  // mesh" -- there is no last one, because this branch is what would have given
  // it one and it never runs, so `have_mesh` on the other side of take_mesh
  // stays false for the session. The panel says so explicitly; a blank view
  // that the operator reads as a tracking failure would be the worst outcome
  // for a mode whose number depends on that operator walking the room.
  //
  // The keyframe is inside the guard with the geometry, not beside it, for the
  // reason the geometry is: they are one value. Publishing an atlas while the
  // mesh stays behind would bind THIS frame's keyframe against whatever mesh
  // the last real publish left -- a live camera image smeared across older
  // surfaces, which is precisely the pairing Published::atlas promises cannot
  // happen. Held together, the renderer keeps a consistent pair. (In this mode
  // nothing is staged either; see `do_texture`.)
  //
  // The stats it does publish are the point: `extract.remeshed_blocks` against
  // `extract.active_blocks`, over `extract_window_frames` fused frames, at the
  // device's own dirty fraction rather than a desktop fixture's.
  if (publishes_mesh(config_)) {
    mesh_ = device_mesh.value();
    published_generation_ = device_mesh.value().generation;
    published_taken_ = false;
    // Published together, under one lock, because they are one value: `uv0` is
    // a coordinate into *this* keyframe. Zeroed rather than left alone when
    // this remesh did not texture, so a consumer cannot bind the previous
    // keyframe against a mesh whose uv0 are all sentinel.
    atlas_width_ = staged_w;
    atlas_height_ = staged_h;
    published_atlas_ = staged_atlas;
    ++mesh_version_;
    stats_.mesh_version = mesh_version_;
  }
  ++stats_.remeshes;
  // recon sets both from the arena WATERMARK, not from its internally-computed
  // live count -- which it does not publish. Under an incremental extract the
  // watermark includes ranges the kernel retired in place (a dead triangle
  // becomes three indices onto one vertex rather than reclaimed space), so
  // these run above the live surface by whatever the arena has not compacted
  // yet. That is a real resident cost and belongs on the memory rows, but it is
  // not a live-geometry count, and the read-out has to say which it is showing.
  // See FusionStats::spans_tracked and the @warning on
  // FusionConfig::incremental_benchmark.
  stats_.vertices = device_mesh.value().vertex_count;
  stats_.triangles = device_mesh.value().triangle_count;
  // The window the dirty set behind this extract accumulated over, published
  // beside the fraction it is the denominator for. 0 on the normal path, where
  // no incremental extract ran and `remeshed_blocks` is 0 anyway.
  stats_.extract_window_frames = extract_window;
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
  // The three published together, because `texture_ms` alone is ambiguous in
  // both directions: 0.0 could be off, skipped, refused or instantaneous, and a
  // healthy-looking 4 ms says nothing about what the tolerance let through. The
  // state names which of those happened and the threshold is the knob it ran
  // at. Neither claims coverage -- see TextureState's warning for why a count
  // of textured vertices is not derivable at this tier.
  stats_.texture_state = texture_state;
  stats_.occlusion_threshold = threshold;
  stats_.atlas_copy_ms = atlas_copy_ms;
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
  // The atlas travels with the mesh or not at all. A caller that got a mesh and
  // had to ask separately for its keyframe could pair a mesh with the *next*
  // remesh's image, which is the one failure this pairing exists to make
  // unrepresentable -- every textured triangle would sample the wrong place.
  out.atlas = published_atlas_;
  out.atlas_width = atlas_width_;
  out.atlas_height = atlas_height_;
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

bool Fusion::set_occlusion_threshold(float metres) {
  // Refused rather than clamped, and the difference matters for a knob: a
  // clamp turns a nonsense value into a silent one, so a caller that computed
  // a NaN gets a working scan and no reason to look. `!(x >= 0)` rather than
  // `x < 0` so NaN is rejected by the comparison rather than slipping through
  // it, which is the same shape every other guard in this file uses.
  if (!(metres >= 0.0f) || !std::isfinite(metres)) {
    return false;
  }
  occlusion_threshold_.store(metres, std::memory_order_relaxed);
  return true;
}

float Fusion::occlusion_threshold() const {
  return occlusion_threshold_.load(std::memory_order_relaxed);
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
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const std::int64_t stamped = last_fuse_ns_.load(std::memory_order_relaxed);
  out.ms_since_fuse = stamped == 0
                          ? 0.0f
                          : std::chrono::duration<float, std::milli>(
                                now - std::chrono::nanoseconds(stamped))
                                .count();
  // The same computation for the stage rows, against the same `now` so the two
  // are comparable -- their *difference* is the reading that matters: this one
  // stamps only a frame that fused all the way through, so a gap between them
  // is a loop that is running while nothing completes. Read from one clock
  // sample rather than two so a slow snapshot cannot make that gap out of
  // nothing.
  const std::int64_t stages_stamped =
      last_stages_ns_.load(std::memory_order_relaxed);
  out.ms_since_stages =
      stages_stamped == 0 ? 0.0f
                          : std::chrono::duration<float, std::milli>(
                                now - std::chrono::nanoseconds(stages_stamped))
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

std::size_t Fusion::history(FrameSample* out, std::size_t capacity) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t written = static_cast<std::size_t>(
      std::min<std::uint64_t>(history_next_, kHistoryCapacity));
  // The availability query. Separated from the zero-capacity case below rather
  // than sharing its return, because the two ask different questions: a null
  // pointer asks how many there are, while a real pointer is told how many it
  // may now read. Conflating them answered `history(v.data(), v.size())` on an
  // empty vector with a count of up to kHistoryCapacity, into a buffer nothing
  // had been written to.
  if (out == nullptr) {
    return written;
  }
  if (capacity == 0) {
    return 0;
  }
  // A short buffer takes the NEWEST samples: a history is read from the present
  // backwards, and handing back the oldest would show a chart that stops
  // before the moment someone opened it to look at.
  const std::size_t take = std::min(written, capacity);
  const std::uint64_t first = history_next_ - take;
  for (std::size_t i = 0; i < take; ++i) {
    out[i] = history_[(first + i) % kHistoryCapacity];
  }
  return take;
}

}  // namespace volumetric_kit::ios_app

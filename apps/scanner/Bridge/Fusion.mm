// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "Fusion.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace volumetric_kit::ios_app {
namespace {

using Clock = std::chrono::steady_clock;

float ms_since(Clock::time_point t0) {
  return std::chrono::duration<float, std::milli>(Clock::now() - t0).count();
}

/// How many times one frame may grow the map and retry its allocation.
///
/// One doubling is not enough when the user pans onto a whole new room section,
/// and recon's reference loops retry up to five times before treating overflow
/// as fatal. Bounded rather than unbounded so a frame cannot spend the whole
/// budget resizing.
constexpr int kMaxGrowAttempts = 5;

/// Blocks per bucket, which is `VoxelGridParams::bucket_size` below.
///
/// Named once rather than written as an `8` wherever the block-table capacity
/// is derived: the occupancy guards divide by this, and a bucket size changed
/// at the grid params with the guards left restating the old one would silently
/// mis-scale the very thresholds that keep the allocate kernel out of its
/// pathological regime.
constexpr std::int32_t kBlocksPerBucket = 8;

/// The floor on `FusionConfig::mesh_slots`; see that field.
constexpr std::uint32_t kMinMeshSlots = 2;

/// How many remeshes' worth of fused frames an occupancy reading may go
/// unrefreshed before it is treated as unusable rather than current.
///
/// Several rather than one: a remesh legitimately skips its extract when the
/// renderer has not collected the last mesh (see @ref Fusion::remesh), so a
/// one-remesh window would trip on the steady state.
constexpr std::uint64_t kMaxStaleRemeshes = 4;

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
  stats_.mesh_slots = config.mesh_slots;

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
  // it -- the survey below is the whole reason the counters exist.
  vr::tsdf::TsdfIntegratorConfig integ_config;
  integ_config.track_dirty_blocks = true;
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
  std::string frame_error;
  // Whether that error is a *stage failure* rather than a report of an expected
  // outcome. Only the former reaches `stats_.errors` -- see that field: dropped
  // blocks and a volume at its ceiling are the common case on a healthy scan,
  // and counting them made the one genuine failure impossible to see.
  bool frame_stage_failed = false;

  // --- Is the occupancy reading still live? --------------------------------
  //
  // Both guards below read `stats_.extract.active_blocks`, and the only thing
  // that writes it is a *successful* extract in remesh. So a persistent extract
  // failure -- a refit that runs out of memory, a capacity past
  // maxStorageBufferRange -- freezes the numerator while the real table goes on
  // filling, and the guards then wave through exactly the regime they exist to
  // prevent. Worse, they do it silently: the read-out keeps printing the frozen
  // count as though the protection were live.
  //
  // Measured in fused frames rather than trusted, because the reading has a
  // legitimate lag: it refreshes every `remesh_every` frames, and a remesh
  // skips its extract outright when the renderer has not collected the last
  // mesh. A window several remeshes wide is therefore quiet in normal operation
  // and trips promptly when extracts actually stop.
  //
  // Past it the table is treated as full, which is the safe direction: new
  // allocation stops, everything already in the table keeps fusing, and the
  // reason says the reading is stale rather than implying a healthy volume.
  const std::uint64_t stale_after =
      kMaxStaleRemeshes * std::max<std::uint32_t>(config_.remesh_every, 1u);
  const bool occupancy_stale =
      active_blocks_measured_ &&
      stats_.frames_fused - active_blocks_at_frame_ > stale_after;

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
  constexpr float kGrowAtOccupancy = 0.7f;
  if (!occupancy_stale && stats_.extract.active_blocks > 0 &&
      config_.num_buckets < config_.max_buckets) {
    const float capacity = static_cast<float>(config_.num_buckets) *
                           static_cast<float>(kBlocksPerBucket);
    if (static_cast<float>(stats_.extract.active_blocks) >
        capacity * kGrowAtOccupancy) {
      const std::int32_t grown_to =
          std::min(config_.num_buckets * 2, config_.max_buckets);
      const vr::Status grown = grid_->resize(grown_to);
      if (grown) {
        config_.num_buckets = grown_to;
      } else {
        // Not fatal: the table is merely denser than preferred, and the
        // allocate below still works -- more slowly. Reported rather than
        // returned so the frame still fuses.
        frame_error = "preemptive resize: " + grown.message();
        frame_stage_failed = true;
      }
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
  constexpr float kRefuseAllocateAtOccupancy = 0.85f;
  const float table_capacity = static_cast<float>(config_.num_buckets) *
                               static_cast<float>(kBlocksPerBucket);
  const bool table_exhausted =
      occupancy_stale || static_cast<float>(stats_.extract.active_blocks) >
                             table_capacity * kRefuseAllocateAtOccupancy;

  // --- Allocate the blocks this frame's depth touches ----------------------
  const auto t_alloc = Clock::now();
  vr::volume::AllocFailures failures;
  vr::Result<std::uint32_t> overflow =
      table_exhausted ? vr::Result<std::uint32_t>(0u)
                      : grid_->map().allocate_from_depth(
                            frame.depth, frame.depth_camera, &failures);
  if (table_exhausted && frame_error.empty()) {
    // Two different situations, and the difference is the whole point of
    // saying which: a full volume is the documented trade working, while a
    // stale reading means the guard has no idea how full the table is and has
    // stopped allocating *because* it cannot tell. Reporting the second as
    // "volume full" would name a cause the user could act on when the real one
    // is upstream, in whatever is failing every extract.
    if (occupancy_stale) {
      frame_error =
          "occupancy unknown: no extract has measured the block table for " +
          std::to_string(stats_.frames_fused - active_blocks_at_frame_) +
          " fused frames (last read " +
          std::to_string(stats_.extract.active_blocks) +
          " blocks); not allocating new blocks until it does. See the extract "
          "error above.";
    } else {
      frame_error =
          "volume full: " + std::to_string(stats_.extract.active_blocks) +
          " of " + std::to_string(static_cast<std::int64_t>(table_capacity)) +
          " blocks at the " + std::to_string(config_.max_buckets) +
          "-bucket ceiling; not allocating new blocks (existing surface still "
          "fusing). Raise max_buckets or use a coarser voxel_size.";
    }
  }
  if (!overflow) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "allocate: " + overflow.status().message();
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
  // ones, so a doubling toward the 4096-bucket ceiling peaks around 288 MiB
  // transient for pressure that does not exist, and on a phone that is the
  // jetsam range max_buckets is chosen to stay out of.
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
    overflow = grid_->map().allocate_from_depth(frame.depth, frame.depth_camera,
                                                &failures);
    if (!overflow) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.errors;
      stats_.last_error =
          "allocate (after resize): " + overflow.status().message();
      return;
    }
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
      vr::tsdf::IntegrationMode::Classic, frame.has_color() ? &color : nullptr);
  if (!fused) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.errors;
    stats_.last_error = "integrate: " + fused.message();
    return;
  }
  const float integrate_ms = ms_since(t_integrate);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.frames_fused;
    last_pose_ = frame.depth_camera.cam_to_world;
    stats_.allocate_ms = allocate_ms;
    stats_.integrate_ms = integrate_ms;
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
  // Throttled hard: two compactions, each a dispatch plus a readback of the
  // whole active set, which at 100k+ blocks is the second most expensive thing
  // in a fuse. Every 60th fused frame is enough to characterise a scan and is
  // invisible in the frame budget. See FusionStats::survey_touched_blocks for
  // what the number does and does not prove.
  if (captured_ % 60 == 0) {
    vr::Result<std::vector<vr::volume::BlockIndex>> all =
        grid_->map().compact_active_blocks();
    if (all) {
      // The caller's already-compacted set is passed in rather than letting the
      // integrator compact a second time -- a full dispatch and readback, and
      // the second most expensive thing in a fuse at 100k+ blocks.
      vr::Result<std::vector<vr::Vec3i>> remesh =
          integrator_->dirty_remesh_blocks(*grid_, all.value().data(),
                                           all.value().size());
      if (remesh) {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.survey_active_blocks =
            static_cast<std::uint32_t>(all.value().size());
        stats_.survey_changed_blocks = integrator_->dirty_block_count();
        stats_.survey_remesh_blocks =
            static_cast<std::uint32_t>(remesh.value().size());
      }
    }
    // Re-armed every window, so each sample is ONE window's changes rather than
    // everything since the scan began -- which is the quantity an incremental
    // remesh would actually redo. Also clears the refusal a topology change
    // (remove/clear) latches, since a slot-keyed flag means nothing across one.
    integrator_->reset_dirty();
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
  return stats_;
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
  return out;
}

}  // namespace volumetric_kit::ios_app

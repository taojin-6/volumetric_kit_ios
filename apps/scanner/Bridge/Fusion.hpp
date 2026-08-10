// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file Fusion.hpp
/// @brief The reconstruction spine, driven off captured ARKit frames.
///
/// Owns recon's per-scan state — the sparse block grid, the TSDF integrator,
/// marching cubes, and the projective texturer — and turns polled
/// `sensor::CapturedFrame`s into a host mesh the renderer draws. The desktop
/// `fuse_viewer` is the reference; this is that loop with ARKit in place of a
/// Replica sequence.
///
/// @warning **No cross-library GPU wait.** iOS gives one queue in the family,
///          so recon and gfx submit through one mutex — and gfx's
///          `Swapchain::recreate` drains via `Device::wait_idle`, which takes
///          that mutex *and then* waits on the queue. Any command buffer
///          waiting on a value the sibling has not signalled yet deadlocks
///          against a rotation. So the handoff here is a **host mesh** (interop
///          seam A): readiness is checked on the host and a not-ready frame is
///          skipped, never waited on.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::ios_app {

namespace vr = volumetric_kit::recon;

/// @brief Per-scan tuning. Defaults target *first light* rather than room
///        coverage: a small map that fills quickly and grows through
///        `VoxelHashMap::resize` beats one sized for a whole room that takes
///        minutes to show anything.
struct FusionConfig {
  /// Voxel edge in metres.
  ///
  /// 1 cm is about where `sceneDepth` stops being the coarser term: the LiDAR
  /// depth map is 256x192, so past this the extra voxels mostly resolve depth
  /// noise rather than surface. It is not free -- a block is 8 voxels on an
  /// edge, so halving this halves the block *span* (16 cm -> 8 cm) and the
  /// blocks needed to shell a given surface goes up ~4x. Memory per bucket is
  /// unchanged (it is per block), so what shrinks is the scene @ref
  /// max_buckets can cover, by the same ~4x. See that field.
  float voxel_size = 0.01f;
  /// Truncation band; the conventional ~3 voxels, so it tracks @ref voxel_size.
  ///
  /// Scaled with the voxel rather than left at 6 cm, which would have made it
  /// ~6 voxels and smoothed away exactly the detail the finer voxel buys --
  /// `trunc_dist` is the distance over which a surface writes into the field.
  /// The block band is unaffected either way: `truncationBlocks` is
  /// `max(1, ceil(trunc / (8 * voxel)))`, and both 0.06/0.08 and 0.03/0.08
  /// ceil to 1, so the surface shell stays one block thick and the ~4x above
  /// is the whole of the cost.
  float trunc_dist = 0.03f;
  /// Initial hash-table shape. Grows on overflow, up to @ref max_buckets.
  ///
  /// Sized in *memory* rather than in buckets, because that is what this costs:
  /// the grid commits `num_buckets * 8 blocks * 512 voxels * 12 B` of
  /// host-visible, permanently mapped, zero-filled device memory inside
  /// @ref start -- 48 MiB per 1024 buckets -- before the first frame is drawn.
  /// 1024 is 48 MiB either way -- the figure is per block, not per metre -- so
  /// first light stays as quick as it was at 2 cm. What changed with the finer
  /// voxel is the *area* it covers before the first doubling: roughly a
  /// quarter of what 2 cm reached, so a tabletop now takes a doubling or two
  /// rather than fitting outright. That is the "fills quickly, then grows"
  /// trade still working, just starting smaller. 4096 up front was 192 MiB
  /// resident before first light, which is the opposite trade.
  std::int32_t num_buckets = 1024;
  /// Ceiling on that growth, in buckets.
  ///
  /// `VoxelBlockGrid::resize` builds the grown buffers alongside the old ones
  /// and commits only once the map resize succeeds, so each doubling costs
  /// ~1.5x the new size transiently. 32768 buckets is ~1.5 GiB resident and
  /// ~2.3 GiB at the doubling that reaches it -- an iPad-Pro number, not a
  /// phone number. Lower it for phone builds.
  ///
  /// @note **The grid is not the term that binds, and this ceiling has never
  ///       run at 32768 on device.** The value was raised from 16384 against a
  ///       grid-only derivation (805 -> 1610 MB), which is the smaller half:
  ///       recon's `MarchingCubes::plan_capacity` sizes the mesh arena from
  ///       `num_active`, so the reachable block count scales the arena ring
  ///       too, and `scanner.entitlements` records the measured split as 3089
  ///       MB of arenas beside an 805 MB grid. The arenas are what got a scan
  ///       SIGKILLed, and @ref mesh_slots was not revisited when this doubled.
  ///
  ///       So the re-derivation this note has always asked for is still owed,
  ///       against what Bridge/MemoryBudget reports on the target device. What
  ///       stands in for it meanwhile is enforcement rather than arithmetic:
  ///       @ref Fusion::fuse now checks `query_memory_budget()` headroom before
  ///       each doubling, will not re-attempt a resize at a size that already
  ///       failed, and caps one frame's doublings well below the five that now
  ///       span this whole range. Those bound the blast radius; they do not
  ///       make 32768 measured. The arenas are Metal buffers, so the working
  ///       set is the ceiling that binds, not the jetsam limit.
  ///
  /// **What this ceiling does is now enforced, which it previously was not.**
  /// It was 4096, and reaching it meant the table simply kept filling: the
  /// allocate kernel's overflow path scans every entry, so its cost per insert
  /// climbs with occupancy, and at 31480 of 32768 blocks (96%) on an M5 iPad
  /// Pro it hung the GPU outright rather than reporting anything. The stated
  /// intent -- "a scan that is missing far geometry, still running, and saying
  /// so" -- lived only in this comment. @ref Fusion::fuse now stops allocating
  /// past 85% occupancy, which is what makes that true, and is why a *lower*
  /// ceiling now costs coverage rather than stability.
  ///
  /// **At 1 cm this covers ~4x less scene than at 2 cm** -- it bounds blocks,
  /// and the finer voxel needs ~4x of them for the same surface. 16384 reaches
  /// roughly a small room at 1 cm; a 5 m walk filled it (111542 of 131072
  /// blocks). Coarsening @ref voxel_size buys area ~4x faster than raising this
  /// does, and costs no memory. Raising it further needs the
  /// `extended-virtual-addressing` entitlement, which a personal signing team
  /// cannot provision -- see apps/scanner/scanner.entitlements.
  std::int32_t max_buckets = 32768;
  /// Fuse every Nth captured frame; 1 = every frame.
  std::uint32_t fuse_every = 1;
  /// Re-extract the mesh every Nth *fused* frame; 1 = every frame.
  ///
  /// Per-frame meshing is affordable now, and was not always: recon's own
  /// profiling found `extract` costing ~55 ms/frame, of which ~50 ms was
  /// allocating a fresh worst-case vertex arena *every call* and only ~2 ms was
  /// the marching-cubes dispatch. With the arena persistent and grow-only, a
  /// 100-frame `--mesh-every 1` run went 6.3 s -> 0.8 s. So the knob stays --
  /// a large enough volume will still outrun the frame budget -- but it starts
  /// at 1, because a reconstruction that updates every frame is the point.
  std::uint32_t remesh_every = 1;
  /// Track which blocks each fuse actually changed, for the dirty-block survey.
  ///
  /// **On here, and not free anywhere.** recon's default is off and that
  /// default costs nothing: no `num_blocks * 4` host-visible flag array (which
  /// doubles with every map grow) and not one store in the fusion kernel. This
  /// app pays it because the survey in @ref Fusion::fuse is the only instrument
  /// that can say whether incremental extraction is worth building.
  ///
  /// A **field** rather than a constant at the create site, because the cost is
  /// not confined to the diagnostic. recon sizes the flag array inside
  /// `integrate`, rebuilding it beside the old one on every map grow, so a
  /// frame that cannot get that allocation fails its *integrate* -- which @ref
  /// Fusion::fuse treats as fatal to the frame, where before this was turned on
  /// the same frame fused normally. The exposure is worst where it can least be
  /// afforded: the frame after a doubling toward @ref max_buckets, which that
  /// field prices at a ~1.1 GiB transient -- the largest single allocation
  /// spike this app makes, and so the one most likely to be refused. (It was
  /// described here as "the jetsam range", which it is not: that phrasing came
  /// from a ceiling `scanner.entitlements` had recorded ~50% too low, and from
  /// calling a transient cost a limit.) Nothing retries or falls back, so
  /// turning the survey off must not need a source edit and a rebuild.
  bool track_dirty_blocks = true;
  /// Ask each tier for its host/device stage rows -- @ref FusionStats::stages.
  ///
  /// **A field for the same reason @ref track_dirty_blocks is one, and the rule
  /// stated there applies verbatim: turning this off must not need a source
  /// edit and a rebuild.** A non-null `StageMetrics*` is not merely somewhere
  /// to write timings; it routes every dispatch behind it onto recon's *timed*
  /// submit path, which resets a query pool, writes two timestamps and reads
  /// them back per command buffer. On a fused frame that is the allocate (once
  /// per grow retry, so up to `kMaxGrowAttempts + 1` times), the fusion
  /// dispatch, and the active-set compaction -- every frame, at capture rate.
  ///
  /// On, because the numbers are the point: a host span around a fence-blocked
  /// submit cannot separate a slow kernel from a stalled queue, and that
  /// distinction has been guesswork on this device. The readback rides an
  /// already-blocking wait rather than adding a second stall. Off costs only
  /// the device column -- @ref FusionStats::allocate_ms and @ref
  /// FusionStats::integrate_ms are measured either way and the read-out falls
  /// back to them.
  ///
  /// @warning Device timing can retire itself mid-run, and does not come back.
  ///          recon's tiers hold long-lived `GpuTimer`s; a failed
  ///          `vkWaitForFences` leaks the command buffer carrying that window's
  ///          queries, so the timer calls `abandon()` and every row is
  ///          host-only from then on. Nothing here can reset it, and without
  ///          @ref FusionStats::gpu_timing_retired that state is
  ///          indistinguishable from a device that never had timestamps at all.
  bool measure_stages = true;
  /// Project the current keyframe onto the mesh after each remesh.
  ///
  /// **Still off, and still load-bearing that it is.** `ProjectiveTexturer`
  /// does not annotate the triangles it wins -- it *overwrites* their vertices'
  /// `uv0`, replacing recon's `(-1, -1)` sentinel, and gfx's hybrid shader
  /// reads a non-sentinel `uv0` as "sample the atlas, ignore the vertex
  /// colour". So against the 1x1 white atlas the renderer still binds, this
  /// would render every surface the depth camera is currently looking at flat
  /// white and discard the colour the TSDF fused there.
  ///
  /// Neither half is built. @ref Fusion::Published carries the mesh and nothing
  /// else -- the colour frame it would index crossed the seam under interop
  /// seam A and does not any more -- and the consuming half, a ring of atlas
  /// images the renderer streams into and binds per slot, was never written.
  /// *This flag is what gates the feature on both of them.* Flip it in the
  /// change that lands the ring, not before: on its own it renders every
  /// surface the depth camera is looking at flat white.
  bool texture = false;

  /// @brief How many extracted meshes may be outstanding at once.
  ///
  /// Passed through to `MarchingCubesConfig::slot_count`, and **two is the
  /// floor**: @ref Fusion::start refuses anything lower rather than running
  /// with it. One is recon's own default and means a single arena reused in
  /// place, with `release_through` recording a number and changing no
  /// behaviour -- but every @ref Published::mesh here is a *borrowed*
  /// @ref vr::mesh::DeviceMesh, so at one slot the next extract overwrites the
  /// buffers an in-flight draw is reading, and a grow frees them outright
  /// (`vmaDestroyBuffer`, no fence wait). That is silent geometry corruption or
  /// a GPU fault, raised as neither a Status nor a validation message, and it
  /// is not a state worth leaving one defaulted field away.
  ///
  /// Two only *arms* the contract. A consumer drawing these buffers has to size
  /// it to its own frames in flight plus one, which is what the renderer passes
  /// (`RendererImpl::kMeshSlots`). Each slot costs a full vertex arena, so
  /// higher is not free -- see recon's `slot_count`.
  std::uint32_t mesh_slots = 2;

  /// @brief The queue families that will touch the mesh buffers.
  ///
  /// Both of them, unconditionally -- recon reduces the pair to its distinct
  /// entries and picks EXCLUSIVE where they turn out to be one family. Under
  /// the two-family queue plan a phone actually gets, they differ, and a mesh
  /// created EXCLUSIVE would be read by a family that does not own it: silently
  /// undefined, not an error.
  std::uint32_t queue_families[2] = {0, 0};
  std::uint32_t queue_family_count = 0;
};

/// @brief Why a fused frame took no new geometry in, when it took none.
///
/// A cause rather than a bool so the read-out can name one without knowing this
/// file's thresholds. See @ref FusionStats::allocation_stop.
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

/// @brief What the last fuse/remesh cost and produced, for the read-out.
struct FusionStats {
  std::uint64_t frames_fused = 0;
  std::uint64_t remeshes = 0;
  std::uint32_t vertices = 0;
  std::uint32_t triangles = 0;
  std::uint32_t mesh_version = 0;
  /// Host spans around the allocate and the integrate, measured on every fused
  /// frame whatever @ref FusionConfig::measure_stages says.
  ///
  /// These are what the read-out falls back to when @ref stage_count is 0, so
  /// turning stage measurement off costs the device column and nothing else.
  /// When rows *are* present they carry these same host spans and more --
  /// recon's `allocate` row accumulates across the grow retries this pair can
  /// only total, and each row has a device half beside it.
  float allocate_ms = 0.0f;
  float integrate_ms = 0.0f;
  float extract_ms = 0.0f;
  float texture_ms = 0.0f;

  /// @brief Upper bound on @ref stages; rows past it are dropped, not grown
  ///        into.
  static constexpr std::size_t kMaxStages = 8;

  /// @brief The per-stage host/device rows recon reported for the last fused
  ///        frame, or none when @ref FusionConfig::measure_stages is off.
  ///
  /// Empty until a frame fuses all the way through, and **not cleared by a
  /// frame that fails before then** -- see @ref ms_since_stages, which is what
  /// says how old they are.
  ///
  /// A fixed array, not a `std::vector`: this struct is copied out under the
  /// publish mutex the fuse thread takes every frame, and a vector would malloc
  /// inside that critical section -- the reason @ref FusionTraceStats exists at
  /// all.
  ///
  /// Five rows on a healthy frame -- `allocate`, `resize`, `integrate`, its
  /// `"  ..active set"` breakdown, and `texture` when @ref
  /// FusionConfig::texture is on -- so eight leaves room. **The extract is not
  /// among them:** `MarchingCubes::extract_device` reports through
  /// `mesh::ExtractTimings`, a richer per-phase struct this app already holds
  /// whole in @ref extract and prints beneath these. A row for it would be a
  /// second, coarser copy of a number already on screen.
  ///
  /// @warning `StageRow::name` is borrowed, not copied. Every row here comes
  ///          from a recon tier reporting with a string literal, or from a
  ///          literal this file passes to `StageScope`, so the pointers outlive
  ///          any read -- but a row added from a non-literal would dangle.
  vr::StageRow stages[kMaxStages]{};
  /// How many of @ref stages are real.
  std::uint32_t stage_count = 0;
  /// Whether recon reported more rows than @ref kMaxStages held.
  ///
  /// A dropped row is otherwise invisible: `stage_count == kMaxStages` reads
  /// exactly like a frame that happened to have eight. A consumer summing the
  /// column would under-report with no error and no counter to notice it by.
  bool stages_truncated = false;
  /// Milliseconds since @ref stages was last published, computed when the
  /// snapshot is taken.
  ///
  /// The staleness half, in the one unit that can carry it. The rows publish on
  /// the same fully-successful path that increments @ref frames_fused, so a
  /// frame count between them is always zero by construction and says nothing
  /// -- the frames that leave these rows behind are exactly the ones that never
  /// reach the counter. Compare @ref ms_since_fuse: that one is stamped at
  /// every `fuse` *entry*, so a large gap between the two is precisely "the
  /// loop is running and no frame is completing", which is the state a
  /// persistently failing integrate leaves the read-out in.
  ///
  /// Zero when no frame has ever published rows, where "0 ms ago" would read as
  /// current. Check @ref stage_count first.
  float ms_since_stages = 0.0f;
  /// Whether device timing was measured once and has since retired itself.
  ///
  /// recon's tiers hold long-lived `GpuTimer`s that `abandon()` on a failed
  /// fence wait, permanently and process-wide. Without this the aftermath --
  /// every row host-only, every device cell a dash -- is indistinguishable from
  /// a queue family that never reported timestamps, which is a hardware verdict
  /// rather than a fault. See @ref FusionConfig::measure_stages.
  bool gpu_timing_retired = false;
  /// @brief The last successful extract's own timings and counts, held whole.
  ///
  /// recon's struct by value rather than a hand-picked copy of its fields.
  /// Eleven of its twelve were mirrored into scalars here, which meant every
  /// field recon added or renamed wanted three lockstep edits in this app --
  /// declaration, copy, read-out -- with no compile error when one was missed;
  /// and because all seven spans are the same type, any permutation slip in the
  /// copy block compiled clean and surfaced only as a suspicious device
  /// read-out. That is the same class of gap this diff exists to close, one
  /// struct over. recon's own `fuse_viewer` holds it whole for the same reason.
  /// Costs nothing new: marching_cubes.hpp is already included above, and
  /// @ref Published already exposes a `vr::mesh::DeviceMesh` publicly.
  ///
  /// Where @ref extract_ms goes. The phases are worth the read-out space
  /// because they have *nothing* in common as optimisation targets and the
  /// total cannot tell them apart: `neighbour_lut_ms` is serial host work over
  /// the whole active set (the 2x2x2 table the sparse kernel indexes instead of
  /// probing the hash) and wants incremental extraction; `dispatch_ms` is the
  /// meshing pass and wants fewer cells. On an M5 iPad Pro at ~107k blocks this
  /// app measured extract at 132.7 ms, of which the lut was 102.2 and the
  /// dispatch 25.9.
  ///
  /// @warning `dispatch_ms` is **not** GPU time. recon submits through
  ///          `Device::submit_single_time`, which blocks on a fence, and
  ///          `ExtractTimings` documents the span as covering host record
  ///          *plus* device execution rather than either alone. Sizing a
  ///          triangle-reduction win against it overstates the ceiling by
  ///          however much of it is recording, submit and stall.
  ///
  /// @note `arena_bytes` is recon's sum across the whole ring, while
  ///       `triangle_capacity` is what this extract planned for the one slot it
  ///       wrote -- not the same scale. See @ref mesh_slots.
  vr::mesh::ExtractTimings extract{};
  /// How many slots that arena is spread over -- `FusionConfig::mesh_slots`,
  /// echoed here so the read-out needs no second source for it.
  std::uint32_t mesh_slots = 0;
  /// Block-table capacity (`num_buckets * kBlocksPerBucket`) **as of the
  /// extract that measured `extract.active_blocks`**, so the read-out's
  /// occupancy divides two figures taken at one instant.
  ///
  /// Stamped beside the block count deliberately, and that placement is load
  /// bearing. It briefly lived in `fuse`, refreshed every frame, on the theory
  /// that a capacity from before the last doubling would make occupancy read
  /// over 100%. But the numerator only ever moves on a successful extract, so
  /// refreshing the denominator alone produced the inverse fault: the grow
  /// doubles this the same frame, and a remesh that then skips or fails halves
  /// the displayed occupancy exactly when the map is under most pressure. Both
  /// halves of a ratio move together or neither does.
  std::uint32_t table_capacity = 0;
  /// Live block-table occupancy from `VoxelHashMap::load_factor` -- read every
  /// fused frame, so unlike @ref table_capacity it never lags a remesh.
  ///
  /// Re-read after a doubling, not only before one. The first cut sampled this
  /// once at the top of `fuse` and let the refuse-to-allocate guard below read
  /// it after the preemptive grow had already run, so on the frame that doubled
  /// the map the guard refused to allocate into the room it had just made --
  /// and the read-out announced a stopped scan against a 44%-full table.
  float occupancy = 0.0f;
  /// The capacity @ref occupancy is a fraction of, sampled in the same breath.
  ///
  /// Not @ref table_capacity, and the difference is the reason both exist: that
  /// one is stamped beside `extract.active_blocks` on a successful remesh, so
  /// pairing it with this numerator divides a per-fused-frame figure by a
  /// per-remesh one -- the exact two-cadence ratio its own doc forbids. Before
  /// the first successful extract it is 0, which printed `4.3% of 0 blocks`;
  /// after a doubling whose next remesh skips, it lags by a factor of two; and
  /// under a persistent extract failure it freezes for the whole session while
  /// the map doubles beneath it.
  std::uint32_t table_blocks = 0;
  /// Whether @ref occupancy is a reading rather than a fallback.
  ///
  /// False when `load_factor` failed, where @ref occupancy is forced to 1.0 so
  /// the allocate guard fails safe. That fabricated 1.0 must not be mistaken
  /// for a measurement by anything that acts on the number: it is above
  /// `kGrowThreshold`, so treating it as real also makes it a standing
  /// instruction to double the map, and it is the reason the read-out may not
  /// call the resulting stop a full volume.
  bool occupancy_known = true;
  /// Whether the scan has stopped taking *new* geometry in, and why.
  ///
  /// Its own field rather than a line in @ref last_error, because that string
  /// is most-recent-wins and shows only when @ref errors is non-zero -- and
  /// this is deliberately *not* an error (a full volume is the documented trade
  /// working). The result was that a scan stopped taking in new geometry and
  /// said so nowhere: an M5 iPad Pro ran half a scan frozen at 85% behind an
  /// `errors 0` banner, with `table` the only clue and only to someone who
  /// already knew 85% was the threshold.
  ///
  /// Covers every way allocation stops, not just the guard. The first cut set
  /// this only from the preemptive occupancy guard, which left the other half
  /// of the failure space exactly as silent as before: blocks dropped at the
  /// `max_buckets` ceiling, or to a capacity limit the per-frame grow budget
  /// could not absorb, are reported by `allocate` rather than by occupancy and
  /// can fire while the table reads 60%.
  ///
  /// A cause rather than a flag, because the read-out must not invent one. A
  /// bool here left the renderer hard-coding "(volume full)" onto it, so a
  /// failed `load_factor` -- which fabricates a 1.0 and thereby trips the guard
  /// -- printed a full volume beneath a banner naming the real upstream fault,
  /// and offered a remedy (coarsen the voxel, raise the ceiling) for a
  /// condition that was not happening. @ref Fusion::fuse withholds its own
  /// "volume full" string on that path for exactly this reason; naming the
  /// cause here is what lets the panel honour that instead of undoing it.
  AllocationStop allocation_stop = AllocationStop::None;
  /// Milliseconds since @ref Fusion::fuse last published, computed when the
  /// snapshot is taken.
  ///
  /// The freshness half that @ref occupancy and @ref allocation_stop
  /// otherwise lack, and the one pair in this struct that cannot be counted in
  /// frames: `frames_fused` stops advancing in exactly the case that needs
  /// saying. An ARKit interruption -- a call, Control Centre, the app switcher
  /// -- stops frames without stopping the display link, so the panel went on
  /// asserting ALLOCATION STOPPED in the present tense about a scan that was
  /// not allocating because it was not scanning. Compare @ref frames_since_
  /// extract and @ref frames_since_survey, which measure a lag *within* a
  /// running fuse loop and so can use its own clock.
  float ms_since_fuse = 0.0f;
  /// @brief The dirty-block survey: how much of the map a window of fusing
  ///        actually changed.
  ///
  /// The question incremental mesh extraction rests on -- re-meshing only what
  /// changed is worth its complexity only if "what changed" is a small fraction
  /// of the map. A *frustum* survey measured on Replica room0 came out at
  /// **87%**, which would kill the idea; but room0 is one small enclosed room
  /// and a 90-degree, 8 m cone from inside it contains nearly everything, so
  /// that number was the fixture rather than the truth. On device it was worse
  /// than uninformative -- wrong in both directions depending on pose (4.5%
  /// where the truth was 18.7%) -- and this replaced it outright.
  ///
  /// This field is the **whole active set** as of the survey:
  /// `compact_active_blocks().size()`, with no frustum and no frame dependence
  /// in it at all. It is the denominator @ref survey_changed_blocks and @ref
  /// survey_remesh_blocks are fractions of, and it is deliberately *not* the
  /// `active_blocks` in @ref extract: that one is refreshed by a successful
  /// remesh, this one by a successful survey, and the two cadences differ by
  /// ~60x. When they disagree, @ref survey_stale and @ref extract_stale are
  /// what say which of them stopped moving.
  ///
  /// Surveyed periodically, because one survey costs a full active-set
  /// compaction (a dispatch, a fence wait and a readback of the whole set), an
  /// O(active) dilation walk, and an O(num_blocks) host scan of the flag array.
  /// @ref survey_ms is what that actually came to.
  std::uint32_t survey_active_blocks = 0;
  /// Blocks the fuse actually CHANGED in the window -- not "was dispatched"
  /// (the dispatch covers every active block and returns early for most) and
  /// not "was in the frustum" (that is the whole view cone, which on Replica
  /// room0 read 87% and was useless).
  std::uint32_t survey_changed_blocks = 0;
  /// Those dilated into the `-x/-y/-z` octant: the set an incremental extract
  /// must actually redo, because a cell reads its corners as `base + {0,1}^3`,
  /// so a changed block invalidates every block reaching into it. Measured at
  /// 1.3-1.4x the changed set on room0.
  std::uint32_t survey_remesh_blocks = 0;
  /// How many fused frames of changes this sample accumulated.
  ///
  /// **The sample is a union over this many integrates, not one frame's work.**
  /// recon ORs the flags in and never clears them itself; @ref Fusion::fuse
  /// re-arms once per survey. So with @ref FusionConfig::remesh_every at 1 --
  /// every fused frame extracts -- the share printed beside this is a *ceiling*
  /// on what one incremental extract would redo rather than that quantity: it
  /// is what a remesh running once per window would redo. The direction is the
  /// safe one, a union being a strict superset, but reading it as a per-extract
  /// figure overstates the work and prices the optimisation against the wrong
  /// baseline.
  ///
  /// Published rather than assumed equal to the survey cadence, because it is
  /// not: a frame that takes one of @ref Fusion::fuse's error early-returns
  /// never reaches the survey, and the window then runs on into the next one.
  /// Without this, the next sample reports a double-length union through an
  /// identical-looking line.
  std::uint64_t survey_window_frames = 0;
  /// What the survey itself cost, on the fuse frame that ran one.
  ///
  /// Measured because nothing else here measures it: every other stage of @ref
  /// Fusion::fuse is timed and published, while the survey's claim to be
  /// "invisible in the frame budget" was an assertion with no number behind it
  /// -- on a device where @ref extract alone already measures 132.7 ms.
  float survey_ms = 0.0f;
  /// Whether this sample is the **first** window of the scan, which reads
  /// near-100% changed whatever the scene is.
  ///
  /// A block's first integrate always moves its weight off zero, so every block
  /// the map grew during the first window is necessarily marked -- and the map
  /// grew from empty during exactly those frames. Nothing converges inside one
  /// window either: at `max_weight` 5.0 with an inverse-square observation
  /// weight, a surface at 2-3 m needs 20-45 observations. So the first sample
  /// is ~100% on any scene, and without this flag it is indistinguishable from
  /// a steady-state one -- while being, at a 2 s log throttle against a ~1 s
  /// window, a live candidate for the first line ever recorded for a run.
  bool survey_first_window = false;
  /// @brief How far behind the current frame the survey is, and whether that is
  ///        further than its own cadence explains.
  ///
  /// What @ref frames_since_extract is for the extract, against a worse
  /// failure. The read-out gates the survey rows on `survey_active_blocks > 0`,
  /// which is a one-way latch, and both survey calls used to fail silently --
  /// so one good sample followed by a permanently failing survey (the
  /// device-lost regime this app has already hit once) reprinted a minute-old
  /// figure as this frame's number for the rest of the run. The failures are
  /// counted now, through @ref errors like every other stage; this is the half
  /// that says the printed numbers are no longer current.
  std::uint64_t frames_since_survey = 0;
  bool survey_stale = false;
  /// @brief How far behind the current frame @ref extract is, and whether that
  ///        is further than the remesh cadence explains.
  ///
  /// Everything in @ref extract is published only on remesh's fully-successful
  /// path, below the early returns for "the renderer has not collected the last
  /// mesh" and "the extract failed". Without this the read-out reprints a stale
  /// breakdown as current, and the worst of it is `dispatches`, the one value
  /// whose reading tells someone what to go and do: a 2 frozen by a failing
  /// extract sends them to debug a planner that is fine, and a frozen 1 hides
  /// one that is not.
  std::uint64_t frames_since_extract = 0;
  bool extract_stale = false;

  /// Set when a stage failed; the loop keeps running so one bad frame does not
  /// end the scan, but the reason stays visible.
  ///
  /// Most-recent-wins, and that is the whole reason @ref errors exists beside
  /// it: `fuse` republishes its own per-frame error every frame, so a failure
  /// raised by a *later* stage -- extract, texture, or the fuse thread's
  /// exception guard -- survives only until the next frame is published. At 60
  /// Hz a persistent extract failure was therefore visible for under 16 ms at a
  /// time and read as a clean scan with a frozen mesh.
  std::string last_error;
  /// How many stage failures have been raised since `start`. Monotonic, so a
  /// fault that @ref last_error cannot hold still onto is visible as a rising
  /// count -- the same shape as the renderer's mesh-upload counter, and for the
  /// same reason.
  ///
  /// **Stage failures only**, which is narrower than "frames that reported
  /// something in @ref last_error". Dropped blocks are the common case, not a
  /// failure: adjacent LiDAR pixels dilate into the same block and the kernel's
  /// bucket lock gives up after a bounded number of retries, so a healthy scan
  /// reports lost races on nearly every frame, and a full volume republishes
  /// its notice every frame it keeps fusing. Counting those made the banner
  /// read `! errors x1800` after thirty seconds of a clean scan, which buried
  /// the single extract failure this counter exists to surface.
  std::uint64_t errors = 0;
};

/// @brief The subset of @ref FusionStats the render thread's frame trace keeps.
///
/// All PODs, deliberately. @ref FusionStats carries a `std::string`, so copying
/// one out under the publish mutex mallocs inside the critical section the fuse
/// thread needs every frame -- and it is *not* a rare path: a healthy scan
/// reports lost bucket-lock races on nearly every frame, whose notice runs well
/// past libc++'s 22-character small-string buffer. The trace needs a handful of
/// scalars to fill a ring only a failure dump ever reads; it should not pay for
/// the message as well, nor take the lock a second time in the same tick.
struct FusionTraceStats {
  std::uint32_t triangles = 0;
  std::uint32_t triangle_capacity = 0;
  std::uint64_t arena_bytes = 0;
  std::uint32_t active_blocks = 0;
  float extract_ms = 0.0f;
  /// The table's own state, which this ring existed without and should not
  /// have. @ref active_blocks above is stamped by a successful extract, so in
  /// the persistent-extract-failure regime it is frozen -- and that regime is
  /// precisely what a `VK_ERROR_DEVICE_LOST` dump is read to understand. The
  /// dump for the fault the occupancy guard exists to prevent could not say how
  /// full the table was, nor whether the guard was engaged at the time.
  ///
  /// Both are PODs, so they cost this struct nothing it was designed to avoid.
  float occupancy = 0.0f;
  AllocationStop allocation_stop = AllocationStop::None;
};

/// @brief Fuses captured frames into a volume and extracts a drawable mesh.
///
/// Single-threaded from the caller's side: one thread calls @ref fuse, another
/// reads @ref take_mesh. The mesh is published under a lock and versioned, so
/// the render thread can tell "nothing new" from "a new mesh" without copying.
class Fusion {
 public:
  Fusion() = default;

  Fusion(const Fusion&) = delete;
  Fusion& operator=(const Fusion&) = delete;

  /// @brief Build the grid and the compute pipelines on the shared device.
  /// @param device     recon's adopted view of the shared `VkDevice`.
  /// @param allocator  recon's allocator on that device.
  /// @param config     Per-scan tuning.
  vr::Status start(vr::Device& device, vr::Allocator& allocator,
                   const FusionConfig& config);

  /// @brief Fuse one captured frame, and remesh if the cadence says so.
  ///
  /// A failing stage is recorded in @ref stats and skipped rather than
  /// propagated: a single dropped frame should not end a scan the user is in
  /// the middle of.
  void fuse(const vr::sensor::CapturedFrame& frame);

  /// @return The camera-to-world pose of the most recently fused frame.
  ///
  /// The render camera follows this. Without it the view stays at the world
  /// origin looking down +Z while the scan happens somewhere else entirely --
  /// which renders as a flat fill, because the camera ends up inside the
  /// surface rather than looking at it.
  vr::Mat4f last_pose() const;

  /// @brief Take the mesh if it is newer than @p known_version.
  ///
  /// The mesh is **moved** out, so a second call for the same version returns
  /// nothing even though the version has not advanced. That is the intended
  /// contract: the caller uploads what it takes before returning, and the next
  /// remesh republishes.
  ///
  /// @return The mesh and its version, or nothing when unchanged or taken.
  /// @brief A remesh's two halves, which are one value.
  ///
  /// `uv0` is a normalized coordinate into the image of the camera that
  /// textured it, and @ref texture rewrites every vertex's `uv0` on every call
  /// against the *current* frame. So a mesh and the atlas it indexes are only
  /// meaningful together: draw a mesh against a later frame's image and every
  /// textured triangle samples the wrong place. They are published, versioned,
  /// and taken as one.
  struct Published {
    /// The extractor's own buffers, borrowed. Nothing is copied to the host at
    /// all -- this is interop seam B, and the ~53 MB round trip per remesh that
    /// seam A cost is exactly what it removes.
    ///
    /// Valid until its slot is reused, which cannot happen before the consumer
    /// releases it: see @ref release_through.
    vr::mesh::DeviceMesh mesh;
    std::uint32_t version = 0;
  };

  std::optional<Published> take_mesh(std::uint32_t known_version);

  /// @brief Report that every mesh up to @p generation has been drawn, so its
  ///        slot may be extracted into again.
  ///
  /// The consumer half of the ring. Call it as the frames that drew a
  /// @ref Published::mesh retire -- for the render loop, once the frame's fence
  /// has signalled.
  ///
  /// Host-side, and that is the whole design rather than a simplification: a
  /// semaphore the extract waited on would deadlock against a swapchain
  /// rebuild, which drains the queue while holding the submit mutex (see the
  /// warning at the top of this file). Reporting completion after the fact
  /// cannot.
  ///
  /// Takes recon's @ref vr::mesh::DeviceMesh::generation, not @ref
  /// Published::version -- the two number different things, and the ring is
  /// recon's.
  ///
  /// **This records the mark; it does not hand it to recon.** The fuse thread
  /// applies it at the top of its next remesh, which is what keeps recon's
  /// extractor single-threaded. `MarchingCubes::release_through` is not atomic
  /// and its header makes serializing it against the extracting thread a caller
  /// obligation -- calling it from here would race `extract_device` on the fuse
  /// thread, and the obvious repair (hold the publish mutex across the extract)
  /// would block this thread, the *main* thread, for the length of a whole
  /// extract. Deferring costs at most one remesh of latency and no lock at all.
  void release_through(std::uint64_t generation);

  /// @brief Record a failure raised *outside* @ref fuse -- the fuse thread's
  ///        exception guard -- so it reaches the read-out like any other.
  void note_error(const std::string& message);

  FusionStats stats() const;

  /// @brief The frame trace's five scalars, without @ref FusionStats' string.
  ///
  /// See @ref FusionTraceStats: this exists so the per-frame consumer does not
  /// malloc under the mutex the fuse thread takes every frame.
  FusionTraceStats trace_stats() const;

  bool valid() const noexcept { return grid_.has_value(); }

 private:
  /// @brief Extract the mesh, and texture it when @ref FusionConfig::texture is
  ///        on.
  /// @param metrics  @ref fuse's reporting window, or null when @ref
  ///                 FusionConfig::measure_stages is off. Threaded through
  ///                 rather than opened here because the `texture` row belongs
  ///                 beside the fuse's own: one window per frame is what makes
  ///                 the rows comparable. The extract is *not* measured into it
  ///                 -- it reports through `mesh::ExtractTimings`; see @ref
  ///                 FusionStats::stages.
  void remesh(const vr::sensor::CapturedFrame& frame,
              vr::StageMetrics* metrics);

  FusionConfig config_{};
  // recon's per-scan state is create-only (no default constructor), which is
  // why these are optional rather than plain members.
  std::optional<vr::volume::VoxelBlockGrid> grid_;
  std::optional<vr::tsdf::TsdfIntegrator> integrator_;
  std::optional<vr::mesh::MarchingCubes> marching_cubes_;
  std::optional<vr::texture::ProjectiveTexturer> texturer_;

  std::uint64_t captured_ = 0;

  mutable std::mutex mutex_;
  // The published view of the extractor's buffers -- handles and counts, not
  // bytes. Copying it is copying five words.
  vr::mesh::DeviceMesh mesh_;
  std::uint32_t mesh_version_ = 0;
  // recon's generation for the mesh currently published, and whether anyone has
  // taken it. Fusion outruns the render loop routinely -- it remeshes every
  // fused frame -- so a mesh can be superseded before the renderer ever asks
  // for it, and that mesh still holds a slot.
  //
  // What `remesh` does about it is *not extract*. Publishing over it and
  // releasing the old generation is the obvious move and it is unsound: recon's
  // release_through is a monotonic high-water mark, so releasing the untaken
  // generation also retires every older one -- including the generation the
  // renderer's in-flight frames are still drawing out of. recon has a
  // single-slot primitive for this (`free_slot_of`) and keeps it private,
  // precisely because the high-water mark is the consumer's to move.
  //
  // Skipping costs nothing: the extract that would have been thrown away is
  // simply not run, so the ring never needs the extra slot and the GPU never
  // does the work. The renderer takes every frame it draws, so the skip lasts
  // one frame in the steady state.
  std::uint64_t published_generation_ = 0;
  bool published_taken_ = true;
  // The consumer's high-water mark, recorded by @ref release_through and handed
  // to recon by the fuse thread at the top of the next remesh. See that method:
  // this indirection is what keeps recon's extractor single-threaded.
  std::uint64_t consumer_released_ = 0;
  // `stats_.frames_fused` as of the last extract that published a breakdown,
  // and whether one ever has. These fed the occupancy guard too, until it moved
  // to VoxelHashMap::load_factor -- a reading that cannot go stale, so the
  // guard needs no such pair. What remains is the read-out's own concern:
  // `extract.*` publishes only on a fully-successful remesh, so without this
  // the panel reprints a frozen breakdown as current.
  std::uint64_t active_blocks_at_frame_ = 0;
  bool active_blocks_measured_ = false;
  // The `num_buckets` a preemptive resize was last refused at, or 0.
  //
  // The backoff that keeps a failing doubling from being re-attempted on every
  // fused frame. `config_.num_buckets` only advances on success, so without
  // this the trigger condition -- occupancy parked between the grow threshold
  // and the refuse threshold -- stays true indefinitely and re-asks the
  // allocator for ~1.5 GiB at capture rate, freeing it each time. Cleared on
  // any successful grow, since a resize that failed at N may well succeed at N
  // once the pressure that refused it has passed.
  std::int32_t preemptive_grow_failed_at_ = 0;
  // steady_clock nanoseconds at `fuse`'s last entry, for
  // FusionStats::ms_since_fuse. Wall clock rather than a frame count because
  // the case it exists to name is the one where the frame count stops
  // advancing; see that field.
  //
  // Atomic rather than mutex-guarded so it can be stamped once at the top of
  // `fuse` and cover every path out of it -- including the error early-returns,
  // which is the half that matters: a frame that bailed still proves the fuse
  // loop is alive, and stamping only the success path would report a *running*
  // scan as interrupted. `stats()` reads it on the main thread.
  std::atomic<std::int64_t> last_fuse_ns_{0};
  // steady_clock nanoseconds at the last publish of `stats_.stages`, for
  // FusionStats::ms_since_stages. The same shape as last_fuse_ns_ above and
  // read the same way -- but stamped on the *success* path only, which is the
  // whole point: the difference between the two is what a failing stage looks
  // like. See that field for why a frame count cannot express this.
  std::atomic<std::int64_t> last_stages_ns_{0};
  // Whether any published row has ever carried a device span.
  //
  // A one-way latch, and it has to be: what it detects is a GpuTimer that
  // retired mid-run (FusionConfig::measure_stages), which is permanent and
  // leaves no other trace. Rows arriving host-only *before* this is set are an
  // ordinary device without timestamp support, so the latch is exactly the term
  // that tells those two apart.
  bool gpu_timing_seen_ = false;
  // `stats_.frames_fused` when the current dirty window opened -- the last time
  // `fuse` called `reset_dirty`. The published window length is measured from
  // this rather than assumed to be the survey cadence, because a frame that
  // takes an error early-return never reaches the survey and the window then
  // spans two cadences. Zero is the first window, the one that reads ~100% by
  // construction; see FusionStats::survey_first_window.
  std::uint64_t dirty_window_start_ = 0;
  // `stats_.frames_fused` as of the last survey that actually published, and
  // whether one ever has. The same pair as active_blocks_at_frame_ /
  // active_blocks_measured_ above, for the same reason: a latched read-out
  // needs something that can tell a live sample from a frozen one.
  std::uint64_t survey_at_frame_ = 0;
  bool survey_measured_ = false;
  vr::Mat4f last_pose_{1.0f};
  FusionStats stats_{};
};

}  // namespace volumetric_kit::ios_app

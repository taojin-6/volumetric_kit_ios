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

// @ref AllocationStop, which this file sets and Core/AllocationStop.hpp turns
// into what the log, the trace and the panel each say about it. The enum lives
// there rather than here because those renderings are decisions made without
// the device, and this header cannot be reached from a host build: it pulls in
// all of recon.
#include "AllocationStop.hpp"
// @ref kSurveyEveryFrames and the staleness rules the read-out shares with the
// fuse loop, and @ref plan_growth / @ref guard_allocation -- the two decisions
// `fuse` makes about the block table. All pure, all on the far side of the same
// boundary and for the same reason: a threshold decided without the device
// belongs where a host test can reach it.
#include "Freshness.hpp"
#include "GrowthPolicy.hpp"

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
  ///       against what Bridge/MemoryQuery reports on the target device. What
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
  /// Track which blocks each fuse actually changed.
  ///
  /// **On here, and not free anywhere.** recon's default is off and that
  /// default costs nothing: no `num_blocks * 4` host-visible flag array (which
  /// doubles with every map grow) and not one store in the fusion kernel. This
  /// app pays it because the survey in @ref Fusion::fuse is the only instrument
  /// that can say whether incremental extraction is worth building.
  ///
  /// The flags have **two possible consumers and never both at once**: that
  /// survey, and the incremental extract under
  /// @ref incremental_benchmark. Each reads the accumulated set and then resets
  /// it, so running both would make the window each describes drift out of step
  /// with the other's -- recon refuses the pairing outright in its own harness.
  /// The survey stands down in the measurement mode; see its gate in
  /// `Fusion::fuse`. That mode also *implies* this field, so
  /// @ref Fusion::start normalizes it on the way in and everything downstream
  /// reads the stored member rather than the request.
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
  /// **On**, and both halves it used to wait on are now built:
  /// @ref Fusion::Published carries the keyframe alongside the mesh, and the
  /// renderer holds a persistent ring of atlas images indexed by the same slot
  /// as the mesh.
  ///
  /// What it does: after each remesh, `ProjectiveTexturer` projects the current
  /// frame into the mesh and writes a real `uv0` for every vertex it can see
  /// unoccluded, so those surfaces render at the colour camera's resolution
  /// rather than the voxel's -- 1920x1440 against a 1 cm grid. Everything else
  /// keeps recon's sentinel and falls back to the colour the TSDF fused.
  ///
  /// **The textured region tracks the camera and reverts behind it.** Every
  /// call overwrites every `uv0`, so a surface leaving the view returns to
  /// voxel colour. That is the live single-camera slice working as designed,
  /// not a defect -- a persistent textured reconstruction needs the
  /// multi-keyframe atlas, which is a later slice.
  ///
  /// Gated at the call site on the frame actually carrying colour, not on this
  /// flag alone: a frame whose colour the capture refused would otherwise get
  /// real `uv0` addressing an atlas that was never uploaded. See
  /// @ref Fusion::remesh.
  ///
  /// @warning **This flag being on does not mean the pass runs.** recon's
  ///          `ProjectiveTexturer::texture` refuses a mesh whose vertices are
  ///          shared between triangles, and this file asks marching cubes for
  ///          exactly that (`share_vertices`, in @ref Fusion::start -- see the
  ///          comment there for why both are on together). The per-vertex
  ///          dispatch that lifts the refusal is a recon change; against a
  ///          recon that predates it, every remesh's texture call returns
  ///          `InvalidArgument`, no keyframe is published, and the scan renders
  ///          as fused voxel colour throughout. That is not silent -- it is
  ///          @ref TextureState::Failed with the refusal in
  ///          @ref FusionStats::last_error -- but it *is* what "on" looks like
  ///          against the wrong sibling revision, so check the read-out before
  ///          concluding the flag did nothing.
  bool texture = true;

  /// @brief How far a vertex may sit from this frame's depth reading and still
  ///        count as seen, in metres.
  ///
  /// The visibility test: a vertex is textured only when
  /// `|sampled_depth - projected_z| <= this`. Everything else about projective
  /// texturing is geometry; this is the one number deciding what "the camera
  /// can see it" means.
  ///
  /// **It is not comparing geometry against truth.** It compares the *fused*
  /// surface against a *single* frame's raw depth, so it has to absorb the
  /// disagreement between them: per-frame LiDAR noise (`smoothedSceneDepth`
  /// reduces it and does not remove it), the TSDF's running average against one
  /// observation, pose error between the frames that placed the vertex and the
  /// one testing it, and voxel quantisation at @ref voxel_size.
  ///
  /// Against that, being generous costs resolution: **a depth separation
  /// smaller than this cannot be told apart**, so an object standing this far
  /// proud of a wall lets the wall behind it be textured with the object's
  /// pixels. Too tight and textured regions go patchy or flicker, worst at
  /// range; too loose and colour bleeds through foreground edges.
  ///
  /// **It does a second job that is easy to miss.** recon uses the same value
  /// as the depth-discontinuity bound in its bilinear sampler: when the 2x2
  /// taps span more than this, the sampler stops blending and takes the nearest
  /// valid tap instead, so a vertex on a silhouette is not rejected by a
  /// phantom mid-depth averaged from foreground and background. Tightening this
  /// therefore narrows visibility *and* makes that fallback fire more often.
  ///
  /// 2 cm is recon's default and is **inherited rather than measured** -- its
  /// header calls it "the ported default" from the prior engine, and nothing
  /// has re-derived it against ARKit's sensor. Two voxels at @ref voxel_size
  /// 1 cm, just under @ref trunc_dist, though it is tied to neither in code.
  ///
  /// @note A fixed metric threshold is arguably the wrong *shape*: LiDAR noise
  ///       grows with range, so this is strict at 0.5 m and permissive at 4 m.
  ///       Scaling it with the projected depth would be the principled version
  ///       -- worth doing against a measurement rather than ahead of one, which
  ///       is what this field being adjustable is for.
  ///
  /// Changeable while a scan runs; see @ref Fusion::set_occlusion_threshold.
  float occlusion_threshold = 0.02f;

  /// @brief How many extracted meshes may be outstanding at once.
  ///
  /// Passed through to `MarchingCubesConfig::slot_count`, and **two is the
  /// floor for any configuration that publishes geometry**: @ref Fusion::start
  /// refuses anything lower rather than running with it. The floor is keyed on
  /// the borrow, not on a mode name -- a configuration in which nothing takes a
  /// @ref Published::mesh has nothing to corrupt, which is why
  /// @ref incremental_benchmark can hold one slot and why relaxing it is stated
  /// as "publishes no mesh" rather than as an exemption for that flag.
  ///
  /// One is recon's own default and means a single arena reused in
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

  /// @brief Measure incremental extraction, publishing no geometry.
  ///
  /// recon's @ref vr::mesh::MarchingCubes::extract_device_incremental re-meshes
  /// only the blocks a fuse changed. **Sharing stays on** -- that kernel
  /// reserves two per-block ranges and reuses them, and retires a dead triangle
  /// through its own index run for 12 bytes rather than 192.
  ///
  /// Turning sharing off was what the first version did, and it cost 3x the
  /// vertex arena: 4177 MB measured on an iPad Pro (M5) against 33 MB for the
  /// same app normally. That made the number it produced unrepresentative of
  /// anything shippable, which is the whole reason it is no longer done.
  ///
  /// This mode changes **three** settings, not one, and the two that are easy
  /// to overlook are the two that cost something:
  ///
  /// - `slot_count` drops to 1, because recon *refuses* a ring here: a
  ///   re-meshed block writes into the arena the last extract filled, and a
  ///   ring hands this one a different slot.
  /// - `track_block_spans` turns on, which is the table the incremental pass
  ///   re-meshes against. It is grid-sized (`num_blocks * 16` device plus
  ///   `num_blocks * 8` host, doubling on every `VoxelHashMap::resize`) and
  ///   recon charges it to `ExtractTimings::arena_bytes` -- so this mode's
  ///   resident figure is **not comparable** with a normal build's. It also
  ///   charges the per-extract span stamping loop to `arena_alloc_ms`, which
  ///   this class publishes as the `..sizing` stage row.
  /// - @ref track_dirty_blocks is implied, because the flags the incremental
  ///   extract dilates on-device are exactly those. That allocation is rebuilt
  ///   inside `integrate` on every map grow and can fail the *frame* rather
  ///   than just the diagnostic; see that field for what it costs.
  ///
  /// It **does not publish a mesh**, and that is what makes one slot safe
  /// rather than the hazard @ref mesh_slots exists to refuse: nothing borrows
  /// the extractor's buffers, so nothing is drawing an arena the next extract
  /// overwrites. Nothing is drawn *at all* while it runs -- the renderer has
  /// taken no mesh, so there is no last one to keep showing. The panel says so
  /// rather than leaving a blank view to be read as a tracking failure.
  ///
  /// Because nothing collects, the back-pressure guard in @ref Fusion::remesh
  /// is inert here: the shipping build skips a remesh whose predecessor the
  /// renderer has not taken, and this mode extracts on every remesh interval.
  /// The window each reading covers is therefore published beside it as
  /// @ref FusionStats::extract_window_frames, because `remeshed_blocks` means
  /// nothing without it -- a shorter window is less dirt and flatters the
  /// incremental path.
  ///
  /// What it answers is the only question a desktop fixture cannot. room0
  /// re-meshes 81.67% of its blocks per window, capping the win there at 1.22x;
  /// what a device walk does is the **unknown this exists to measure**, not
  /// something to be told in advance. The number is
  /// `ExtractTimings::remeshed_blocks / ExtractTimings::active_blocks`, over
  /// @ref FusionStats::extract_window_frames fused frames -- not a millisecond
  /// row, and meaningful only while `ExtractTimings::incremental` is true.
  ///
  /// What it deliberately does **not** show is the in-place tearing. Seeing
  /// that needs the ring intact *and* the arena de-ringed, so that a reader
  /// holding an older generation is still drawing while a newer extract mutates
  /// -- and de-ringing the arena is the open design question this measurement
  /// exists to decide. At one slot the renderer would simply be shown nothing
  /// new.
  ///
  /// @warning An incremental extract still carries inflation for the ranges it
  ///          retires -- 1.12x measured on room0 with sharing, against 1.82x
  ///          without it. That is a working-set cost this mode pays and the
  ///          normal path does not, and it lands in
  ///          `DeviceMesh::triangle_count`, which recon sets from the arena
  ///          watermark rather than its internally-computed live count. So the
  ///          arena fill row counts retired geometry too.
  ///
  /// @note Every figure quoted above is from room0 on a desktop fixture. No
  ///       reading in the configuration this file now builds has been taken on
  ///       hardware.
  bool incremental_benchmark = false;

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

/// @brief What the last remesh's projective-texturing pass actually did.
///
/// A cause rather than a duration, for the reason @ref AllocationStop is a
/// cause rather than a bool: `texture_ms` alone cannot tell a pass that ran
/// from one that never ran, because both publish 0.0 when there was nothing to
/// measure. Every state below is reachable on a healthy device, so the read-out
/// has to name which one, and the advice differs per state.
///
/// @warning None of these says how *much* of the mesh was textured. recon's
///          `ProjectiveTexturer::texture` returns a @ref vr::Status and no
///          count, and the only way to derive one at this tier would be to read
///          the vertex arena back and tally sentinels -- the ~45 MB round trip
///          the device overload exists to avoid. So @ref Ran means the dispatch
///          completed, not that any vertex came out visible: a
///          @ref FusionConfig::occlusion_threshold too tight for the scene
///          still reports @ref Ran. Reporting coverage needs recon to count it
///          on-device; until then this distinguishes ran / skipped / refused
///          and stops there rather than implying more.
enum class TextureState : std::uint8_t {
  /// @ref FusionConfig::texture is off. The pass is not wired into this scan at
  /// all and every `uv0` stays at marching cubes' sentinel.
  Off = 0,
  /// On, but no remesh has run yet since @ref Fusion::start.
  Pending,
  /// On, and the last remesh ran the pass to completion.
  Ran,
  /// Skipped because the frame carried no colour -- `convert_color` refuses an
  /// unsupported pixel format, HLG and PQ, so this is reachable on real
  /// hardware rather than only at session start. Not a fault: the mesh renders
  /// as fused voxel colour, which is what a colourless frame should look like.
  NoColor,
  /// The pass was attempted and refused or failed; the reason is in
  /// @ref FusionStats::last_error. Distinct from @ref NoColor because this one
  /// *is* a fault -- and it is the state a build against a recon that still
  /// refuses shared-vertex meshes sits in permanently, which otherwise looks
  /// exactly like texturing that runs and produces nothing.
  Failed,
};

/// @brief One fused frame, kept so a chart can show what a snapshot cannot.
///
/// The read-out publishes the *latest* values and the UI polls them a few times
/// a second, which is enough for a trend and structurally unable to show a
/// spike: fusion runs on its own thread, so a poll at any rate samples whatever
/// was last published and never the frame in between. A 154 ms meshing frame
/// among 20 ms neighbours is invisible to it, and that is exactly the frame
/// worth seeing.
///
/// So the sample is taken where the frame is produced, not where the UI asks.
///
/// Assembled field by field, which is the opposite of the rule the fuse loop
/// follows for @ref FusionStats::extract ("recon's struct, whole -- not a
/// field-by-field transcription"). That rule is about *transcription*: copying
/// a struct one member at a time so a field added upstream goes missing. This
/// is a **reduction** -- 240 of these are resident for the whole scan, so it
/// carries what a chart plots and deliberately not the strings, the error
/// counters, or the four separate arena figures beside them. A field added to
/// `FusionStats` is meant to be absent here until someone decides a chart needs
/// it; that is the point, not an oversight. Keep it that way, and keep the
/// footprint in mind when adding one.
///
/// Deliberately **totals, not per-stage rows**. A stacked-by-stage history
/// would need the row set carried per entry, and the question a history answers
/// -- "which frame was slow, and was it host or device" -- is answered by three
/// numbers.
///
/// @warning The per-stage split for *this* sample is generally **not**
///          retrievable. @ref FusionStats::stages holds one frame's rows and is
///          overwritten every fused frame, so by the time a spike is on screen
///          its breakdown is long gone -- the only sample whose rows can still
///          be read is the newest, which is never the one anyone opened the
///          chart for. Read that as the deliberate limit of this type: it says
///          *which frame, and which of three coarse phases*. Anything finer
///          needs the row set carried per entry, which is the cost this
///          rejected. Do not document it as a two-place lookup; that lookup
///          does not work.
struct FrameSample {
  /// `FusionStats::frames_fused` as of this sample.
  ///
  /// Sequential by construction and therefore **not** a gap indicator: the
  /// increment and this append are on one path, so every frame that fuses gets
  /// an index and every frame that does not never reaches here. A capture
  /// dropout draws as adjacent indices at normal spacing. @ref timestamp_ns is
  /// the half that can actually show it.
  std::uint64_t frame = 0;
  /// Monotonic nanoseconds (`steady_clock`) at which this sample was taken.
  ///
  /// Carried because @ref frame cannot express elapsed time and the gaps worth
  /// seeing are exactly the ones that produce no frames: an ARKit interruption
  /// stops the fuse loop without stopping the display link, and a chart indexed
  /// by frame number alone renders the resumption as the very next frame.
  std::uint64_t timestamp_ns = 0;
  /// Summed host milliseconds across the frame's stages, breakdown rows
  /// excluded -- they restate time the stage above already counted.
  ///
  /// @warning Not a host-only measurement. recon documents `StageRow::cpu_ms`
  ///          as wall clock around a *blocking* submit, so for a compute stage
  ///          this covers record, submit, the fence stall and device execution
  ///          -- an end-to-end cost. It therefore already contains most of
  ///          @ref device_ms, and stacking the two double-counts. Draw them
  ///          overlaid, or draw the host figure alone; a "host vs device"
  ///          split built by subtraction reads a device stall as host time.
  float host_ms = 0.0f;
  /// Summed device milliseconds. Breakdowns ARE counted here: a device span
  /// covers one dispatch and never the compaction the stage ran first, so
  /// skipping them deletes that time rather than deduplicating it.
  ///
  /// Meaningful only where @ref device_timing_valid is set. A row carrying no
  /// GPU span contributes nothing, so a queue family that reports no timestamps
  /// and a frame whose device did no work are both a flat zero here.
  float device_ms = 0.0f;
  /// Whether any stage this frame carried a real device measurement.
  ///
  /// The distinction @ref device_ms cannot make on its own, and the one the
  /// read-out already makes in text by printing `gpu -` rather than `0.00`. A
  /// chart that omits this draws a zero device line on a device that was never
  /// able to measure one, which reads as "all the time is host".
  bool device_timing_valid = false;
  /// Milliseconds the extract took, when one ran on this frame.
  ///
  /// Carried separately because it is **not** in @ref host_ms or
  /// @ref device_ms: `MarchingCubes::extract_device` reports through
  /// `vr::mesh::ExtractTimings` rather than the stage row set, so neither total
  /// can see it. It is also the largest cost in a remesh frame -- 132.7 ms
  /// against ~20 ms of fuse on this device's own measurement -- and with
  /// @ref FusionConfig::remesh_every at its default of 1 it is on every frame.
  /// A history without it is a history that cannot show the spike this type was
  /// built for.
  ///
  /// Zero when this frame ran no extract, or when one ran and failed. Use
  /// @ref frames_since_extract to tell those from a genuinely fast one.
  float extract_ms = 0.0f;
  /// Live block-table occupancy at this frame.
  ///
  /// Paired with @ref occupancy_known for the reason @ref FrameTrace pairs its
  /// own two: an unreadable `load_factor` publishes a *fabricated* 1.0 here, so
  /// a chart plotting this column alone draws a hard climb to 100% that never
  /// happened -- at exactly the moment the reader is trying to find out what
  /// did. See @ref FusionStats::occupancy_known.
  float occupancy = 0.0f;
  /// Whether @ref occupancy was read or fabricated.
  bool occupancy_known = true;
  /// Both stamped by the last *successful* remesh. That is this frame's when
  /// one ran and succeeded, and an older frame's whenever it skipped (the
  /// renderer has not collected the last mesh) or the extract failed -- so
  /// these two series flat-line while every other one keeps moving. @ref
  /// frames_since_extract is how far back they actually reach.
  std::uint32_t triangles = 0;
  std::uint32_t active_blocks = 0;
  /// Fused frames since the extract that stamped @ref triangles and
  /// @ref active_blocks; `0` when this frame's own remesh refreshed them.
  ///
  /// Mirrored from @ref FusionStats::frames_since_extract rather than left to
  /// the live read-out, because a history is read long after the fact: without
  /// it a consumer taking `samples.last.triangles` cannot tell a steady scan
  /// from an extract that has been failing for a minute, which is exactly the
  /// regime worth catching.
  std::uint64_t frames_since_extract = 0;
  /// Whether the scan took new geometry in on this frame, and if not, why.
  ///
  /// The cause rather than a bool, matching @ref FrameTrace beside it: the
  /// advice a reader acts on differs per cause, and a history that recorded
  /// only *that* it stopped would need the live read-out open at the same
  /// moment to say which -- which is the coupling this ring exists to remove.
  AllocationStop allocation_stop = AllocationStop::None;
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
  /// The host span around `ProjectiveTexturer::texture`, or 0 when the pass did
  /// not run. **Read it with @ref texture_state, never alone**: 0.0 here means
  /// "off", "skipped" and "refused before dispatching" as well as "ran and cost
  /// nothing", and those want different actions.
  ///
  /// Wider than the `texture` stage row beside it, deliberately and not
  /// redundantly: this covers the whole call including the transient buffer
  /// setup and the fence, while recon's row is the dispatch it wraps. The text
  /// summary prints one or the other rather than both -- see the note where it
  /// builds the stage table, and @ref Fusion::remesh's, for why two numbers for
  /// one pass is the failure being avoided.
  float texture_ms = 0.0f;
  /// What that duration means. See @ref TextureState -- and its warning about
  /// what @ref TextureState::Ran does and does not claim.
  TextureState texture_state = TextureState::Off;
  /// The tolerance the last texture pass actually ran at, in metres.
  ///
  /// Published because @ref Fusion::set_occlusion_threshold is turnable
  /// mid-scan: without it the read-out shows the effect of a knob whose value
  /// is only in the caller, and the procedure
  /// @ref FusionConfig::occlusion_threshold documents -- turn it while pointing
  /// at one surface and watch where texturing stops -- has nothing to correlate
  /// the watching against.
  float occlusion_threshold = 0.0f;
  /// The host span around the ~11 MB keyframe copy the fuse thread makes per
  /// textured remesh, in ms.
  ///
  /// Measured because it is otherwise invisible: it sits outside recon's stage
  /// spans, outside @ref extract_ms and outside @ref texture_ms, so a copy that
  /// costs more than the ~0.06 ms the capture path prices it at would slow the
  /// fuse loop with every published figure showing no cause. Zero on a remesh
  /// that published no keyframe.
  float atlas_copy_ms = 0.0f;

  /// @brief Upper bound on @ref stages; rows past it are recorded as dropped in
  ///        @ref stages_truncated, not grown into.
  ///
  /// Sixteen because the published set is the WHOLE pipeline, not just what
  /// reports through StageMetrics: allocate, resize and integrate and their
  /// breakdowns come from the tiers, and extract plus its phases are appended
  /// from @ref extract so the end-to-end sequence is one list rather than two a
  /// reader has to rejoin.
  static constexpr std::size_t kMaxStages = 16;

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
  /// Five rows on a healthy frame come from the tiers -- `allocate`, `resize`,
  /// `integrate`, its `"  ..active set"` breakdown, and `texture` when @ref
  /// FusionConfig::texture is on. **The extract is appended to them rather than
  /// reported through them:** `MarchingCubes::extract_device` reports through
  /// `mesh::ExtractTimings`, a richer per-phase struct this app holds whole in
  /// @ref extract, and the appended rows are that struct flattened so a reader
  /// sees one pipeline instead of rejoining two lists. Those rows carry
  /// `has_gpu = false` -- honestly, not unfortunately: they are exactly the
  /// part of the pipeline still measured on the host alone.
  ///
  /// They are appended **after the remesh**, which is why the publish sits
  /// below it: @ref extract is written by `remesh`, so appending them beside
  /// @ref frames_fused published the previous remesh's phases as this frame's.
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
  /// How many slots that arena is spread over -- the count the **extractor**
  /// was built with, echoed here so the read-out needs no second source for it.
  ///
  /// Not always `FusionConfig::mesh_slots`: @ref
  /// FusionConfig::incremental_benchmark overrides it to 1 on the way into
  /// `MarchingCubesConfig`, and reporting the request instead had the read-out
  /// saying "3 slots" while one was in use -- the single line that would have
  /// shown the mode was on, saying it was off.
  std::uint32_t mesh_slots = 0;
  /// Whether the extractor is keeping recon's per-block span table.
  ///
  /// On only under @ref FusionConfig::incremental_benchmark, and published
  /// because it is what makes two other rows here non-comparable with a normal
  /// build: recon folds the table into `extract.arena_bytes` and charges its
  /// per-extract stamping loop to `arena_alloc_ms`, which this class publishes
  /// as the `..sizing` stage row. The read-out says so beside those figures
  /// rather than leaving them to be compared across builds.
  bool spans_tracked = false;
  /// Fused frames the dirty set behind the last extract had accumulated over.
  ///
  /// The denominator `extract.remeshed_blocks` is meaningless without: the
  /// fraction of the surface a fuse moved is a function of how much fusing
  /// happened since the previous extract, so a shorter window is simply less
  /// dirt and flatters the incremental path. 1 in the steady state under
  /// @ref FusionConfig::incremental_benchmark, which extracts on every remesh
  /// interval because nothing collects and the back-pressure guard is
  /// therefore inert; the shipping build can accumulate more.
  ///
  /// 0 when no incremental extract has run.
  std::uint64_t extract_window_frames = 0;
  /// Whether this scan is running as a measurement rather than as a scan.
  ///
  /// Echoes @ref FusionConfig::incremental_benchmark. The read-out needs it for
  /// something no other field can say: in this mode nothing is published, so
  /// @ref mesh_version stays 0 while @ref remeshes climbs, and a panel reading
  /// "remesh 4900 (v0)" is otherwise indistinguishable from a wedged publish
  /// path. It is also what lets the panel explain an empty view.
  bool incremental_benchmark = false;
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
  /// Whether @ref occupancy is a reading rather than a fallback, carried for
  /// the same reason @ref FusionStats::occupancy_known is: a failed
  /// `load_factor` publishes a fabricated 1.0 so the allocate guard fails
  /// safe, and a dump that printed it as `occ=100.0%` invented a measurement
  /// in the one artifact a device loss leaves behind.
  bool occupancy_known = true;
  AllocationStop allocation_stop = AllocationStop::None;
  /// Milliseconds since @ref Fusion::fuse last published, computed at read
  /// time like @ref FusionStats::ms_since_fuse and `0.0f` when no frame has
  /// fused at all.
  ///
  /// The qualifier @ref allocation_stop needs. It is a latch `Fusion` never
  /// clears, so an ARKit interruption -- which stops fuse frames without
  /// stopping the display link -- leaves a stale cause stamped onto render
  /// frames that allocated nothing. The live renderings drop the claim
  /// entirely (see @ref reportable_allocation_stop); the trace keeps both
  /// numbers, because what the cause *was* is exactly what a forensic dump is
  /// opened to find.
  float ms_since_fuse = 0.0f;
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
    /// @brief The keyframe image @ref mesh's `uv0` index into, as packed RGB
    ///        (one `uint32` per pixel, R in the low byte) -- or null when this
    ///        mesh was not textured.
    ///
    /// The atlas half of the pair this struct's own documentation describes: a
    /// mesh and the image its `uv0` address are only meaningful together, and
    /// drawing one against a later frame's image samples the wrong place
    /// everywhere it was textured.
    ///
    /// **Valid only until the next @ref take_mesh**, which is a shorter promise
    /// than @ref mesh carries. It points into a small ring this class owns, and
    /// the consumer is expected to copy out of it during the same call --
    /// which the renderer does, into the atlas image for the slot it just put
    /// the mesh in. Two entries is enough for that discipline and is what the
    /// ring holds: the fuse thread cannot publish again until the previous mesh
    /// is taken, so at most one unread entry exists, and the entry being copied
    /// is never the one being written.
    ///
    /// Null rather than stale whenever the texture pass did not both run and
    /// succeed, which is two cases and not one. `convert_color` refuses an
    /// unsupported pixel format, HLG and PQ, so a colourless frame is reachable
    /// on a real device rather than only at session start, and @ref
    /// Fusion::remesh skips texturing entirely on one. A pass that *was*
    /// attempted and failed publishes nothing either: it is gated on the
    /// returned @ref vr::Status, not on the decision to try, because a refusal
    /// leaves `uv0` untouched and staging 11 MB against it would assert a
    /// pairing that does not hold. Either way no coordinate points at an atlas
    /// that was never uploaded, and @ref FusionStats::texture_state says which
    /// of the two happened.
    const std::uint32_t* atlas = nullptr;
    std::uint32_t atlas_width = 0;
    std::uint32_t atlas_height = 0;
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

  /// @brief Change the projective-texturing visibility tolerance mid-scan.
  ///
  /// Live rather than start-only because the value it replaces was a literal at
  /// the call site: tuning it meant a source edit and a rebuild, which is
  /// exactly the shape @ref FusionConfig::track_dirty_blocks and
  /// @ref FusionConfig::measure_stages are fields to avoid. A tolerance is
  /// judged by looking at a scan, and a scan is not a thing you can hold still
  /// across a rebuild -- the point is to turn it while pointing at the same
  /// surface and watch where texturing stops.
  ///
  /// Non-finite and negative values are refused rather than stored: both make
  /// `|d - z| <= threshold` false for every vertex (every comparison with NaN
  /// is false), so nothing would ever be textured and the read-out would show a
  /// working texture pass producing no coverage. Zero is *allowed* -- it means
  /// "only an exact depth match", which is a legitimate, if useless, end of the
  /// range and is distinguishable from the refusal.
  ///
  /// Thread-safe against a running fuse loop, which is the whole point: this is
  /// called from the main thread while @ref remesh reads it on the fuse thread.
  /// An atomic rather than the publish mutex, so a knob cannot contend with the
  /// lock the render loop already takes several times a frame.
  ///
  /// @return `false` if @p metres was refused, leaving the previous value.
  bool set_occlusion_threshold(float metres);

  /// @return The tolerance @ref remesh is currently applying.
  float occlusion_threshold() const;

  FusionStats stats() const;

  /// @brief How many fused frames the history holds.
  ///
  /// Public because a caller sizing a buffer for @ref history needs it, and a
  /// number repeated at the call site drifts below this one and silently
  /// truncates -- which reads as a shorter scan, not as a bug.
  ///
  /// ~4 s at 60 fused fps, and considerably longer at the rates this actually
  /// runs (20-130 ms/frame measured on device), which is the range where a
  /// spike is still findable by eye.
  static constexpr std::size_t kHistoryCapacity = 240;

  /// @brief Copy the frame history, oldest first, into @p out.
  ///
  /// Oldest-first because that is chart order, and doing the rotation here
  /// keeps the ring's wrap-around out of every consumer -- including the Swift
  /// one, where getting it wrong draws a plausible graph with a discontinuity
  /// somewhere in the middle.
  ///
  /// @param out       Destination, or null to query the available count without
  ///                  writing anything.
  /// @param capacity  How many @p out holds. A smaller buffer receives the
  ///                  **newest** that many samples, since a history is read
  ///                  from the present backwards.
  /// @return How many samples were written: `0` for a zero @p capacity, and for
  ///         a null @p out the count that *would* be written into an
  ///         unconstrained buffer. Those two are deliberately not the same
  ///         answer -- a caller passing a real pointer is told what it may now
  ///         read, so `history(v.data(), v.size())` on an empty vector cannot
  ///         come back with a count it would then read past.
  std::size_t history(FrameSample* out, std::size_t capacity) const;

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
  /// @brief The keyframe images published meshes index into; see
  ///        @ref Published::atlas.
  ///
  /// @warning Not the renderer's ring of the same name. `AtlasRing.hpp` holds a
  ///          second `atlas ring` in this namespace -- `kRingSlots` VkImages
  ///          plus mapped staging buffers, written on the render thread, one
  ///          per mesh slot. This is the producer side and that is the consumer
  ///          side; the depth argument below is about `take_mesh` handing out
  ///          an entry while the next remesh fills the other, and says nothing
  ///          at all about how deep that one has to be.
  ///
  /// Two, alternating, and the argument for two rather than one is the whole
  /// reason this is a ring at all. `take_mesh` marks the mesh taken and returns
  /// while the consumer is still copying out of the entry it was handed, so a
  /// single buffer would let the very next remesh overwrite the bytes being
  /// read. Alternating means the entry being written is never the entry being
  /// read: a third publish needs a second take first (the uncollected guard in
  /// @ref remesh), and the consumer is single-threaded, so it has finished with
  /// the older entry before it asks for the newer one.
  ///
  /// A copy rather than a borrow, because `CapturedFrame::color` points into
  /// the capture's rotating staging buffers and is documented valid only until
  /// its next poll -- and the poll is on this thread, so the pointer would go
  /// stale the moment this one loops. ~11 MB at ARKit's 1920x1440, memcpy'd
  /// once per remesh on the fuse thread.
  std::vector<std::uint32_t> atlas_ring_[2];
  std::uint32_t atlas_slot_ = 0;
  /// The entry the currently-published mesh indexes into, and its dimensions;
  /// null / 0 when the last remesh did not texture. Guarded by @ref mutex_ with
  /// the mesh they belong to -- see @ref Published::atlas for why they are one
  /// value.
  const std::uint32_t* published_atlas_ = nullptr;
  std::uint32_t atlas_width_ = 0;
  std::uint32_t atlas_height_ = 0;
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
  // The live projective-texturing tolerance, seeded from
  // FusionConfig::occlusion_threshold at start(). Atomic because the main
  // thread turns it while the fuse thread reads it once per remesh; see
  // set_occlusion_threshold for why it is not under mutex_.
  std::atomic<float> occlusion_threshold_{0.02f};
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
  // The same idea for the OTHER consumer of those flags. `stats_.frames_fused`
  // when the dirty set the next incremental extract will read began
  // accumulating -- the last time `remesh` reset it. Separate from
  // `dirty_window_start_` because the two never run in the same scan (see the
  // survey's gate) and collapsing them would make each look like it had been
  // maintained by the other. Published as FusionStats::extract_window_frames,
  // which is the denominator `extract.remeshed_blocks` is meaningless without.
  std::uint64_t extract_window_start_ = 0;
  // `stats_.frames_fused` as of the last survey that actually published, and
  // whether one ever has. The same pair as active_blocks_at_frame_ /
  // active_blocks_measured_ above, for the same reason: a latched read-out
  // needs something that can tell a live sample from a frozen one.
  std::uint64_t survey_at_frame_ = 0;
  bool survey_measured_ = false;
  vr::Mat4f last_pose_{1.0f};
  // The frame history, written on the fuse thread and read under mutex_ with
  // everything else it publishes.
  //
  // Fixed and non-allocating, like FrameTrace in the renderer: this is appended
  // on every fused frame, and a container that could reallocate there would put
  // a malloc on the hot path to serve a chart.
  //
  // 240 entries is ~4 s at 60 fused fps and considerably longer at the rates
  // this actually runs (20-130 ms/frame measured on device), which is the range
  // where a spike is still findable by eye.
  FrameSample history_[kHistoryCapacity]{};
  std::uint64_t history_next_ = 0;

  FusionStats stats_{};
};

}  // namespace volumetric_kit::ios_app

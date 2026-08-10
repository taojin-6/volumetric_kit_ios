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

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
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
  /// ~1.5x the new size transiently. 16384 buckets is ~768 MiB resident and
  /// ~1.1 GiB at the doubling that reaches it -- an iPad-Pro number, not a
  /// phone number. Lower it for phone builds.
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
  std::int32_t max_buckets = 16384;
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

/// @brief What the last fuse/remesh cost and produced, for the read-out.
struct FusionStats {
  std::uint64_t frames_fused = 0;
  std::uint64_t remeshes = 0;
  std::uint32_t vertices = 0;
  std::uint32_t triangles = 0;
  std::uint32_t mesh_version = 0;
  float allocate_ms = 0.0f;
  float integrate_ms = 0.0f;
  float extract_ms = 0.0f;
  float texture_ms = 0.0f;
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
  /// @brief The dirty-block survey: how much of the map one frame can see.
  ///
  /// The question incremental mesh extraction rests on -- re-meshing only what
  /// changed is worth its complexity only if "what changed" is a small fraction
  /// of the map. Measured on Replica room0 it came out at **87%**, which would
  /// kill the idea; but room0 is one small enclosed room and a 90-degree, 8 m
  /// cone from inside it contains nearly everything, so that number may be the
  /// fixture rather than the truth. This device walks a real space with a
  /// short-range LiDAR depth, which is the case that matters.
  ///
  /// `touched` is the active blocks inside the current depth camera's frustum.
  /// It **over-estimates** what an integrate actually writes -- the frustum is
  /// the whole view cone, while only voxels within +/-trunc_dist of the
  /// measured depth are touched -- so a *low* number here is conclusive and a
  /// high one is not. Surveyed periodically, because each survey costs two full
  /// active-set compactions (a dispatch and a readback apiece).
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
/// past libc++'s 22-character small-string buffer. The trace needs five scalars
/// to fill a ring only a failure dump ever reads; it should not pay for the
/// message as well, nor take the lock a second time in the same tick.
struct FusionTraceStats {
  std::uint32_t triangles = 0;
  std::uint32_t triangle_capacity = 0;
  std::uint64_t arena_bytes = 0;
  std::uint32_t active_blocks = 0;
  float extract_ms = 0.0f;
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
  void remesh(const vr::sensor::CapturedFrame& frame);

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
  // `stats_.frames_fused` as of the last extract that actually measured
  // occupancy. The anti-hang guards in @ref fuse read `stats_.active_blocks`,
  // which only a successful extract refreshes, so this is what tells a live
  // reading from one frozen by a persistent extract failure -- at which point
  // the guards would otherwise be reading a number that stopped tracking
  // reality and waving a filling table through.
  std::uint64_t active_blocks_at_frame_ = 0;
  bool active_blocks_measured_ = false;
  vr::Mat4f last_pose_{1.0f};
  FusionStats stats_{};
};

}  // namespace volumetric_kit::ios_app

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
  /// 4096 is 192 MiB resident, and ~288 MiB transiently at the doubling that
  /// reaches it: `VoxelBlockGrid::resize` builds the grown buffers alongside
  /// the old ones and commits only once the map resize succeeds. Past that a
  /// phone is into jetsam range, where the app *disappears* rather than
  /// surfacing the OutOfMemory that recon is written to report. Refusing to
  /// grow instead leaves a scan that is missing far geometry, still running,
  /// and saying so in @ref FusionStats::last_error.
  ///
  /// **At 1 cm this ceiling covers ~4x less scene than it did at 2 cm** -- it
  /// bounds blocks, and the finer voxel needs ~4x of them for the same
  /// surface. So this reaches roughly a large tabletop rather than a room. It
  /// is deliberately *not* raised to compensate: 16384 would be 768 MiB
  /// resident and ~1.1 GiB at the doubling, and the jetsam argument above is
  /// the reason the number is what it is. Raise it for a room-scale scan on a
  /// device with the headroom (an iPad Pro, not a phone) -- and know that the
  /// failure it prevents is graceful and reported, not a crash.
  std::int32_t max_buckets = 4096;
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
  /// @ref Fusion::Published now carries the colour frame across the seam beside
  /// the mesh whose `uv0` indexes it, which is the producing half. The
  /// consuming half -- a ring of atlas images the renderer streams into and
  /// binds per slot -- is not built yet, and *this flag is what gates the
  /// feature on it*. Flip it in the change that lands the ring.
  bool texture = false;
};

/// @brief What the last fuse/remesh cost and produced, for the read-out.
struct FusionStats {
  std::uint64_t frames_fused = 0;
  std::uint64_t remeshes = 0;
  std::uint32_t active_blocks = 0;
  std::uint32_t vertices = 0;
  std::uint32_t triangles = 0;
  std::uint32_t mesh_version = 0;
  float allocate_ms = 0.0f;
  float integrate_ms = 0.0f;
  float extract_ms = 0.0f;
  float texture_ms = 0.0f;
  /// Set when a stage failed; the loop keeps running so one bad frame does not
  /// end the scan, but the reason stays visible.
  std::string last_error;
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
    vr::mesh::Mesh mesh;
    std::uint32_t version = 0;
    /// The colour frame that textured @ref mesh, canonical-encoded and packed
    /// RGBA8. Empty when the frame carried no colour, or texturing is off.
    std::vector<std::uint32_t> atlas;
    std::uint32_t atlas_width = 0;
    std::uint32_t atlas_height = 0;

    bool has_atlas() const noexcept { return !atlas.empty(); }
  };

  std::optional<Published> take_mesh(std::uint32_t known_version);

  /// @brief Record a failure raised *outside* @ref fuse -- the fuse thread's
  ///        exception guard -- so it reaches the read-out like any other.
  void note_error(const std::string& message);

  FusionStats stats() const;

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
  vr::mesh::Mesh mesh_;
  /// Published beside mesh_ and taken with it -- see Published. Refilled on the
  /// fuse thread and moved out by the consumer, so neither the copy in nor the
  /// hand-off costs anything under the lock.
  std::vector<std::uint32_t> atlas_;
  std::uint32_t atlas_width_ = 0;
  std::uint32_t atlas_height_ = 0;
  std::uint32_t mesh_version_ = 0;
  vr::Mat4f last_pose_{1.0f};
  FusionStats stats_{};
};

}  // namespace volumetric_kit::ios_app

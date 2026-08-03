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
  /// Voxel edge in metres. 2 cm is the coarse end of useful for LiDAR depth
  /// this noisy, and keeps the block budget (and the remesh) affordable.
  float voxel_size = 0.02f;
  /// Truncation band; the conventional ~3 voxels.
  float trunc_dist = 0.06f;
  /// Initial hash-table shape. Grows on overflow.
  std::int32_t num_buckets = 4096;
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
  bool texture = true;
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
  /// @return The mesh and its version, or nothing when unchanged.
  std::optional<std::pair<vr::mesh::Mesh, std::uint32_t>> take_mesh(
      std::uint32_t known_version);

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
  std::uint32_t mesh_version_ = 0;
  vr::Mat4f last_pose_{1.0f};
  FusionStats stats_{};
};

}  // namespace volumetric_kit::ios_app

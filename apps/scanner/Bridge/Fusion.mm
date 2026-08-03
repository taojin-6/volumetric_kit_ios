// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "Fusion.hpp"

#include <chrono>

namespace volumetric_kit::ios_app {
namespace {

using Clock = std::chrono::steady_clock;

float ms_since(Clock::time_point t0) {
  return std::chrono::duration<float, std::milli>(Clock::now() - t0).count();
}

}  // namespace

vr::Status Fusion::start(vr::Device& device, vr::Allocator& allocator,
                         const FusionConfig& config) {
  config_ = config;

  vr::volume::VoxelGridParams grid{};
  grid.voxel_size = config.voxel_size;
  grid.block_size = 8;
  grid.voxels_per_block = 512;  // 8^3
  grid.trunc_dist = config.trunc_dist;
  grid.bucket_size = 8;
  grid.num_buckets = config.num_buckets;
  grid.num_blocks = config.num_buckets * 8;
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

  vr::Result<vr::tsdf::TsdfIntegrator> integrator =
      vr::tsdf::TsdfIntegrator::create(device, allocator);
  if (!integrator) {
    return integrator.status();
  }
  integrator_.emplace(std::move(integrator).value());

  vr::Result<vr::mesh::MarchingCubes> mc =
      vr::mesh::MarchingCubes::create(device, allocator);
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

  // --- Allocate the blocks this frame's depth touches ----------------------
  const auto t_alloc = Clock::now();
  vr::Result<std::uint32_t> overflow =
      grid_->map().allocate_from_depth(frame.depth, frame.depth_camera);
  if (!overflow) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_error = "allocate: " + overflow.status().message();
    return;
  }
  if (overflow.value() > 0) {
    // The table filled. Grow and retry once: resize preserves block indices, so
    // everything already fused keeps its tsdf/weight/color at the same offset.
    const vr::Status grown = grid_->resize(config_.num_buckets * 2);
    if (grown) {
      config_.num_buckets *= 2;
      (void)grid_->map().allocate_from_depth(frame.depth, frame.depth_camera);
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_error = "resize: " + grown.message();
      return;
    }
  }
  const float allocate_ms = ms_since(t_alloc);

  // --- Fuse depth, and colour when the frame carries it --------------------
  const auto t_integrate = Clock::now();
  vr::tsdf::ColorFrame color{};
  color.pixels = frame.color;
  color.cam = frame.color_camera;
  const vr::Status fused = integrator_->integrate(
      *grid_, frame.depth, frame.depth_camera, /*max_weight=*/5.0f,
      vr::tsdf::IntegrationMode::Classic, frame.has_color() ? &color : nullptr);
  if (!fused) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_error = "integrate: " + fused.message();
    return;
  }
  const float integrate_ms = ms_since(t_integrate);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.frames_fused;
    stats_.allocate_ms = allocate_ms;
    stats_.integrate_ms = integrate_ms;
    stats_.last_error.clear();
  }

  if (stats_.frames_fused % config_.remesh_every == 0) {
    remesh(frame);
  }
}

void Fusion::remesh(const vr::sensor::CapturedFrame& frame) {
  const auto t_extract = Clock::now();
  vr::Result<vr::mesh::DeviceMesh> device_mesh =
      marching_cubes_->extract_device(*grid_);
  if (!device_mesh) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_error = "extract: " + device_mesh.status().message();
    return;
  }
  const float extract_ms = ms_since(t_extract);

  // Texture in place, on the device buffers marching cubes just wrote -- no
  // host round trip between the two GPU tiers. The current frame *is* the
  // atlas, so this is the live single-camera path (recon's 2026-07-07
  // decision), and the renderer binds that same image.
  float texture_ms = 0.0f;
  if (config_.texture) {
    const auto t_texture = Clock::now();
    const vr::Status textured = texturer_->texture(
        device_mesh.value(), frame.depth, frame.depth_camera);
    if (!textured) {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_error = "texture: " + textured.message();
    }
    texture_ms = ms_since(t_texture);
  }

  // One host copy, after both GPU passes -- the mesh crosses to the host once
  // rather than once per tier.
  vr::Result<vr::mesh::Mesh> host =
      marching_cubes_->download(device_mesh.value());
  if (!host) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_error = "download: " + host.status().message();
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  mesh_ = std::move(host).value();
  ++mesh_version_;
  ++stats_.remeshes;
  stats_.vertices = static_cast<std::uint32_t>(mesh_.vertices.size());
  stats_.triangles = static_cast<std::uint32_t>(mesh_.indices.size() / 3);
  stats_.mesh_version = mesh_version_;
  stats_.extract_ms = extract_ms;
  stats_.texture_ms = texture_ms;
}

std::optional<std::pair<vr::mesh::Mesh, std::uint32_t>> Fusion::take_mesh(
    std::uint32_t known_version) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mesh_version_ == known_version || mesh_.vertices.empty()) {
    return std::nullopt;
  }
  // Copied, not moved: the fuse thread keeps publishing into mesh_, and the
  // render thread may take several frames to finish with what it got.
  return std::make_pair(mesh_, mesh_version_);
}

FusionStats Fusion::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace volumetric_kit::ios_app

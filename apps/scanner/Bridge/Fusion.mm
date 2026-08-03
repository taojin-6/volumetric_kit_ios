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
      vr::mesh::MarchingCubes::create(device, allocator, [&] {
        vr::mesh::MarchingCubesConfig mc_config;
        // The renderer binds these buffers as geometry rather than being handed
        // a host copy, so it needs its own usage bits on the same allocation --
        // only it knows which, which is why this tier takes them.
        mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
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

  // --- Allocate the blocks this frame's depth touches ----------------------
  const auto t_alloc = Clock::now();
  vr::Result<std::uint32_t> overflow =
      grid_->map().allocate_from_depth(frame.depth, frame.depth_camera);
  if (!overflow) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_error = "allocate: " + overflow.status().message();
    return;
  }
  // The table filled. Grow and retry *while* it keeps filling: resize preserves
  // block indices, so everything already fused keeps its tsdf/weight/color at
  // the same offset, and another doubling is cheap next to the frame it would
  // otherwise fuse with holes.
  int grow_attempts = 0;
  while (overflow.value() > 0 && grow_attempts < kMaxGrowAttempts) {
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
      stats_.last_error = "resize: " + grown.message();
      return;
    }
    config_.num_buckets = grown_to;
    ++grow_attempts;
    // Checked, not discarded. This is the same call as above and its return is
    // the only thing that says whether the grown map actually absorbed the
    // frame; dropping it fuses against a grid still missing those blocks and
    // reports success.
    overflow =
        grid_->map().allocate_from_depth(frame.depth, frame.depth_camera);
    if (!overflow) {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_error =
          "allocate (after resize): " + overflow.status().message();
      return;
    }
  }
  if (overflow.value() > 0 && frame_error.empty()) {
    frame_error = "allocate: " + std::to_string(overflow.value()) +
                  " blocks dropped after " + std::to_string(grow_attempts) +
                  " grow(s); some geometry will be missing";
  }
  // Integrated anyway when blocks were dropped: what *did* allocate still
  // fuses, and a partial frame beats none. The error above is what keeps it
  // from reading as a clean one.
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
    last_pose_ = frame.depth_camera.cam_to_world;
    stats_.allocate_ms = allocate_ms;
    stats_.integrate_ms = integrate_ms;
    // Assigned, not cleared: a frame that fused with dropped blocks says so.
    stats_.last_error = frame_error;
  }

  if (stats_.frames_fused % config_.remesh_every == 0) {
    remesh(frame);
  }
}

void Fusion::remesh(const vr::sensor::CapturedFrame& frame) {
  const auto t_extract = Clock::now();
  vr::mesh::ExtractTimings extract_timings{};
  vr::Result<vr::mesh::DeviceMesh> device_mesh =
      marching_cubes_->extract_device(*grid_, 0.0f, &extract_timings);
  if (!device_mesh) {
    std::lock_guard<std::mutex> lock(mutex_);
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
      stats_.last_error = "texture: " + textured.message();
    }
    texture_ms = ms_since(t_texture);
  }

  // No host copy. The renderer draws these very buffers -- interop seam B --
  // so the ~53 MB round trip per remesh that seam A cost is simply not here.
  // What replaces it is the release contract below: the slot this mesh lives in
  // is not extracted into again until the consumer says it has finished.
  std::lock_guard<std::mutex> lock(mutex_);

  // A mesh published but never taken still holds a slot. Fusion outruns the
  // render loop routinely -- it remeshes every fused frame -- so without this
  // the ring drains to nothing within a second and every later extract fails
  // with "every output slot is still outstanding".
  if (!published_taken_ && published_generation_ != 0) {
    marching_cubes_->release_through(published_generation_);
  }
  mesh_ = device_mesh.value();
  published_generation_ = device_mesh.value().generation;
  published_taken_ = false;
  ++mesh_version_;
  ++stats_.remeshes;
  stats_.vertices = mesh_.vertex_count;
  stats_.triangles = mesh_.triangle_count;
  stats_.mesh_version = mesh_version_;
  stats_.extract_ms = extract_ms;
  stats_.triangle_capacity = extract_timings.triangle_capacity;
  stats_.active_blocks = extract_timings.active_blocks;
  stats_.arena_bytes = extract_timings.arena_bytes;
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
  if (!marching_cubes_) {
    return;
  }
  marching_cubes_->release_through(generation);
}

void Fusion::note_error(const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
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

}  // namespace volumetric_kit::ios_app

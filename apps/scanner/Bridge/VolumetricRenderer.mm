// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Objective-C++ seam. This is the one translation unit where a
// CAMetalLayer* and a vg::app::WindowedApp are both first-class, which is the
// entire reason the bridge is .mm rather than Swift.

#import "VolumetricRenderer.h"

#import <Metal/Metal.h>

// The definition, which the header above deliberately does not carry: this is
// the one unit that sends `VolumetricCapture` a message (-captureHandle) rather
// than just naming the pointer type.
#import "ARKitCapture.h"

#import "AllocationStop.hpp"
#import "AtlasRing.hpp"
#import "BridgeStrings.hpp"
#import "FrameTrace.hpp"
#import "Fusion.hpp"
// `occupancy_thresholds` and the constants behind it. Named rather than left
// to Fusion.hpp's copy, on this file's own rule about transitive includes.
#import "GrowthPolicy.hpp"
#import "MemoryQuery.hpp"
#import "OrbitCamera.hpp"
#import "Readout.hpp"
#import "RendererErrors.hpp"
#import "RendererImpl.hpp"
#import "SharedDevice.hpp"
#import "StatTone.hpp"
#import "ViewOrientation.hpp"

#include <os/log.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
// <cstdio> and <cstdarg> were here for `fusionSummary` and the read-out's cell
// builders, which now live in Readout.mm and name them there. Nothing in this
// file formats any more.
#include <cstring>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
// `frameHistory` sizes a std::vector to hand to Fusion::history. Named here
// rather than left to Fusion.hpp's copy, for the reason recorded above
// <cstdio>: a transitive include is not a dependency, and reordering the
// imports above would turn this into a hard error in a file that already
// learned that once.
#include <vector>

// glm::half_pi, for the viewport turn below. Included rather than left to
// matrix_transform.hpp, which happens to pull it in today -- the same reliance
// the <cstdio> note above records going wrong.
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "triangle_frag.spv.hpp"
#include "triangle_vert.spv.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"

#include "volumetric_kit/gfx/assets/mesh.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/graphics_pipeline.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/shader.hpp"
#include "volumetric_kit/gfx/core/texture.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
// vr::StageRow, named directly by -initWithRow: and by the read-out's row loop.
// Reached transitively through Fusion.hpp today, which is the pattern the
// <cstdio> note above records going wrong: pruning that header's includes, or
// recon relocating the type, breaks this file with an unknown-type error in a
// translation unit nobody edited.
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"

namespace vg = volumetric_kit::gfx;
namespace vr = volumetric_kit::recon;
namespace app = volumetric_kit::ios_app;

// --- The recon/gfx vertex layout, pinned across repos ------------------------
// This TU is the only place `recon::mesh::Vertex` and `gfx::assets::Vertex` are
// both visible, which makes it the only place they can be compared. Each repo
// pins its own struct against its own literals -- recon in mesh.hpp, gfx in
// hybrid_mesh_pipeline.cpp -- and neither pins against the other, so a
// self-consistent change on either side satisfies its own asserts and lands.
//
// These assertions used to be justified by a host `memcpy` and were removed
// with it. That is exactly backwards: under interop seam B nothing is copied,
// gfx builds its vertex input description from *its* offsets and
// vkCmdDrawIndexedIndirect reads *recon's* buffer through them. A repacked or
// reordered vertex is then read at the wrong offsets with no size mismatch to
// catch it -- garbage positions and normals, no compile error, no Status, no
// validation message. The siblings track `main` unpinned, so a broken pairing
// arrives on an ordinary upstream commit; this is what turns it into a build
// failure naming the field that moved.
namespace {
using RVertex = vr::mesh::Vertex;
using GVertex = vg::assets::Vertex;
static_assert(sizeof(RVertex) == sizeof(GVertex),
              "recon and gfx vertex layouts have diverged: size");
static_assert(offsetof(RVertex, position) == offsetof(GVertex, position),
              "recon and gfx vertex layouts have diverged: position");
static_assert(offsetof(RVertex, normal) == offsetof(GVertex, normal),
              "recon and gfx vertex layouts have diverged: normal");
static_assert(offsetof(RVertex, tangent) == offsetof(GVertex, tangent),
              "recon and gfx vertex layouts have diverged: tangent");
static_assert(offsetof(RVertex, uv0) == offsetof(GVertex, uv0),
              "recon and gfx vertex layouts have diverged: uv0");
static_assert(offsetof(RVertex, color) == offsetof(GVertex, color),
              "recon and gfx vertex layouts have diverged: color");
static_assert(std::is_standard_layout<RVertex>::value &&
                  std::is_standard_layout<GVertex>::value,
              "offsetof above requires standard-layout vertex types");
}  // namespace

NSErrorDomain const VolumetricRendererErrorDomain =
    @"io.taojin.volumetrickit.renderer";
NSErrorUserInfoKey const VolumetricRendererVulkanResultKey =
    @"VolumetricRendererVulkanResult";

namespace {

// Metal's recommended working-set ceiling in bytes, or 0 when unavailable.
//
// The third of the three ceilings this app runs under, and by the numbers in
// scanner.entitlements the *lowest* of them: two thirds of physical RAM against
// a jetsam limit near the whole of it. It is also the one the voxel grid and
// the mesh arenas are charged against, since both are Metal buffers underneath
// MoltenVK -- so a comfortable jetsam percentage on its own is the reassuring
// half of the picture, which is why the read-out prints this beside it.
//
// Deliberately not in Bridge/MemoryQuery: that file reads the jetsam ledger
// through Mach and stays plain C++, and these are separate subsystems that
// scanner.entitlements is explicit about keeping apart. Read from Metal rather
// than from VMA's HeapStats::budget_bytes, which is a heuristic until
// VK_EXT_memory_budget is enabled -- an open TODO in both sibling libraries,
// and exactly the kind of estimate this read-out exists to stop relying on.
//
// Cached: iOS has one GPU and this value does not move, so the device is
// created once rather than at the polling rate. `recommendedMaxWorkingSetSize`
// is ios(16.0), which is this app's deployment target (see
// cmake/ios.toolchain.cmake).
std::uint64_t gpu_working_set_bytes() {
  static const std::uint64_t cached = []() -> std::uint64_t {
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    return device == nil ? 0
                         : static_cast<std::uint64_t>(
                               device.recommendedMaxWorkingSetSize);
  }();
  return cached;
}

std::string api_version_string(std::uint32_t v) {
  return std::to_string(VK_API_VERSION_MAJOR(v)) + "." +
         std::to_string(VK_API_VERSION_MINOR(v)) + "." +
         std::to_string(VK_API_VERSION_PATCH(v));
}

// --- Viewport orientation ----------------------------------------------------
// The turn itself is in Core/ViewOrientation.hpp, where it is pure and host
// tested -- including the on-device measurement that settled it, which is
// recorded there in full.
//
// What cannot move is this: the correspondence between the enum Swift sees and
// the enum the turn is defined on. The mapping is computed by *subtracting raw
// values*, so a VolumetricViewOrientation that stopped agreeing with its core
// counterpart would turn every scan by a silent multiple of 90 degrees -- and
// would do it without touching either file that looks responsible for the
// angle. Pinned here, at the seam where the two meet.
static_assert(static_cast<NSInteger>(VolumetricViewOrientationLandscapeLeft) ==
                  static_cast<NSInteger>(app::ViewOrientation::LandscapeLeft),
              "VolumetricViewOrientation and app::ViewOrientation disagree: "
              "landscape-left");
static_assert(static_cast<NSInteger>(VolumetricViewOrientationPortrait) ==
                  static_cast<NSInteger>(app::ViewOrientation::Portrait),
              "VolumetricViewOrientation and app::ViewOrientation disagree: "
              "portrait");
static_assert(static_cast<NSInteger>(VolumetricViewOrientationLandscapeRight) ==
                  static_cast<NSInteger>(app::ViewOrientation::LandscapeRight),
              "VolumetricViewOrientation and app::ViewOrientation disagree: "
              "landscape-right");
static_assert(
    static_cast<NSInteger>(VolumetricViewOrientationPortraitUpsideDown) ==
        static_cast<NSInteger>(app::ViewOrientation::PortraitUpsideDown),
    "VolumetricViewOrientation and app::ViewOrientation disagree: upside-down");

}  // namespace

namespace {

/// Fill one @ref app::ReadoutInputs from the renderer's state.
///
/// The counters live here rather than in `FusionStats` because the upload is
/// the render thread's stage and the memory warning arrives on the UI thread --
/// neither is in the fusion's snapshot, which is exactly why the panel was once
/// missing both.
///
/// @p s and @p budget are **borrowed into the returned struct**, which outlives
/// this call -- so both have to be named locals in the caller. `lifetimebound`
/// is what enforces that: without it the obvious one-liner form,
/// `readout_inputs(*_impl, _impl->fusion.stats(), app::query_memory_budget())`,
/// compiles clean under `-Wall -Wextra` and reads the whole panel out of two
/// destroyed temporaries, because lifetime extension does not reach through a
/// constructor parameter into a reference member.
app::ReadoutInputs readout_inputs(const app::RendererImpl& impl,
                                  const app::FusionStats& s
                                  [[clang::lifetimebound]],
                                  const app::MemoryBudget& budget
                                  [[clang::lifetimebound]]) {
  app::ReadoutInputs in{s, budget};
  in.gpu_working_set_bytes = gpu_working_set_bytes();
  in.mesh_upload_failures = impl.mesh_upload_failures;
  in.mesh_upload_error = impl.mesh_upload_error;
  in.atlas_failures = impl.atlas_failures;
  in.atlas_error = impl.atlas_error;
  in.memory_warnings = impl.memory_warnings;
  in.memory_warning_footprint_bytes = impl.memory_warning_footprint_bytes;
  return in;
}

}  // namespace

@implementation VolumetricRenderer {
  std::unique_ptr<app::RendererImpl> _impl;
  // Retained, not borrowed. The fuse thread dereferences the raw
  // ICameraCapture* this object owns, and the two are siblings on the view
  // controller with no specified destruction order -- so "must outlive the
  // renderer" as a header sentence is not a mechanism. Held from
  // -startFusionWithCapture: until the join in -stopFusion, which is the exact
  // window in which the pointer is read.
  VolumetricCapture* _capture;
}

- (nullable instancetype)initWithLayer:(CAMetalLayer*)layer
                                 error:(NSError**)error {
  self = [super init];
  if (self == nil) {
    return nil;
  }
  _impl = std::make_unique<app::RendererImpl>();

  // --- One VkDevice, adopted by both libraries ------------------------------
  // Not an optimisation: a VkBuffer is valid only on the VkDevice that created
  // it, so the zero-copy mesh handoff needs *one* device. Two devices on this
  // same GPU would still cost a round trip through host memory.
  const vr::Status built = _impl->shared.build((__bridge const void*)layer,
                                               "volumetric_kit_ios scanner");
  if (!built) {
    app::set_error(error, built, "SharedDevice");
    return nil;
  }

  vg::app::WindowedAppConfig config;
  config.app_name = "volumetric_kit_ios scanner";
  // Ignored by adopt (the embedder owns the instance), left for documentation.
  config.enable_validation = false;
  config.swapchain.extent = {
      static_cast<std::uint32_t>(layer.drawableSize.width),
      static_cast<std::uint32_t>(layer.drawableSize.height)};
  // FIFO rather than the MAILBOX default: on a phone, tearing-free vsync at the
  // display's cadence is what we want, and MAILBOX keeps the GPU busy producing
  // frames that are then discarded -- straight thermal cost for no benefit.
  config.swapchain.preferred_present_mode = VK_PRESENT_MODE_FIFO_KHR;
  // A depth attachment, which the triangle did not need but a reconstruction
  // does: without it near geometry does not occlude far, and the mesh renders
  // as whichever triangle happened to be emitted last. The swapchain owns one
  // per image, so it is depth-safe at any frames-in-flight count.
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  // Set rather than left at gfx's default: RendererImpl::kMeshSlots is sized
  // against this number, and a default that changed in the other repo would
  // silently turn the mesh ring into a use-after-free.
  config.frames_in_flight = app::RendererImpl::kFramesInFlight;

  vg::Result<vg::app::WindowedApp> app = vg::app::WindowedApp::adopt(
      _impl->shared.gfx_payload(), config,
      [self](VkInstance instance) -> vg::Result<VkSurfaceKHR> {
        // The surface already exists -- picking a present-capable device
        // required one -- so hand over the bootstrap's rather than making a
        // second. Ownership transfers with it, which is why the bootstrap
        // releases it: destroying it twice is a use-after-free at teardown.
        if (instance != self->_impl->shared.instance()) {
          return vg::Status::invalid_argument(
              "surface factory: adopted a different VkInstance than the "
              "bootstrap created the surface on");
        }
        return self->_impl->shared.release_surface();
      });
  if (!app) {
    app::set_error(error, app.status(), "WindowedApp::adopt");
    return nil;
  }
  _impl->app = std::move(app).value();

  // recon adopts the same device. Nothing consumes it until fusion lands, but
  // adopting now is the point of this slice -- and a mismatch between what the
  // bootstrap enabled and what recon requires must fail at bring-up, where the
  // message is actionable, rather than at the first dispatch.
  vr::Result<vr::Device> recon_device =
      vr::Device::adopt(_impl->shared.recon_payload(), {});
  if (!recon_device) {
    app::set_error(error, recon_device.status(), "recon Device::adopt");
    return nil;
  }
  _impl->recon_device.emplace(std::move(recon_device).value());

  // recon allocates its volume, mesh arena and staging buffers from its own
  // VMA allocator on the shared device -- separate accounting from gfx's, one
  // device underneath.
  vr::Result<vr::Allocator> recon_allocator =
      vr::Allocator::create(_impl->shared.instance(), *_impl->recon_device);
  if (!recon_allocator) {
    // The vr::Status overload, not a flatten through vg::Status::unsupported:
    // an allocator failure on a user's phone is out-of-memory or a VkResult,
    // and reporting it as "unsupported" reads as a capability the driver lacks.
    app::set_error(error, recon_allocator.status(), "recon Allocator::create");
    return nil;
  }
  _impl->recon_allocator.emplace(std::move(recon_allocator).value());

  VkDevice device = _impl->app.device().handle();
  vg::Result<vg::ShaderModule> vert = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_vert_spv),
      vi_triangle_vert_spv_size);
  if (!vert) {
    app::set_error(error, vert.status(), "vertex ShaderModule::create");
    return nil;
  }
  _impl->vertex_shader = std::move(vert).value();

  vg::Result<vg::ShaderModule> frag = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_frag_spv),
      vi_triangle_frag_spv_size);
  if (!frag) {
    app::set_error(error, frag.status(), "fragment ShaderModule::create");
    return nil;
  }
  _impl->fragment_shader = std::move(frag).value();

  vg::GraphicsPipelineDesc desc;
  desc.vertex_shader = &_impl->vertex_shader;
  desc.fragment_shader = &_impl->fragment_shader;
  desc.layout = _impl->app.swapchain().layout();
  // Procedural vertices: positions come from gl_VertexIndex, so no vertex
  // buffer and no input bindings.
  desc.vertex_bindings = nullptr;
  desc.vertex_binding_count = 0;

  vg::Result<vg::GraphicsPipeline> pipeline =
      vg::GraphicsPipeline::create(device, desc);
  if (!pipeline) {
    app::set_error(error, pipeline.status(), "GraphicsPipeline::create");
    return nil;
  }
  _impl->pipeline = std::move(pipeline).value();

  // The renderer's hybrid path: it samples the projective-texturing atlas where
  // a triangle carries a real uv0, and falls back to the per-vertex colour the
  // TSDF fused elsewhere. That is exactly what recon's mesh emits.
  vg::Result<vg::pipelines::HybridMeshPipeline> mesh_pipeline =
      vg::pipelines::HybridMeshPipeline::create(
          device, _impl->app.swapchain().layout());
  if (!mesh_pipeline) {
    app::set_error(error, mesh_pipeline.status(), "HybridMeshPipeline::create");
    return nil;
  }
  _impl->mesh_pipeline.emplace(std::move(mesh_pipeline).value());

  // --- The atlas set, without which nothing draws ---------------------------
  // Not optional and not a detail: HybridMeshPipeline::submit returns early on
  // a VK_NULL_HANDLE atlas, recording no bind, no push constant and no draw. A
  // frame missing this one field clears the screen and presents it, which is
  // indistinguishable from a working renderer looking at empty space -- and is
  // exactly what the scanner did until now.
  //
  // 1x1 white, which is what gfx prescribes for the vertex-colour path: the
  // fragment shader samples the atlas unconditionally, but only where a
  // triangle carries a real uv0. Where projective texturing did not win a
  // camera, uv0 is the (-1,-1) sentinel and the TSDF's per-vertex colour is
  // used instead -- so these texels are never selected there.
  //
  // "Never selected" holds only while FusionConfig::texture is off, which is
  // why it is off: the texturer overwrites uv0 with a real coordinate on every
  // triangle it wins, and every one of those would then sample this one white
  // texel instead of its fused colour. The pair moves together.
  static const std::uint8_t kWhite[4] = {255, 255, 255, 255};
  vg::ImageUploadDesc atlas_desc;
  atlas_desc.extent = {1, 1};
  atlas_desc.format = VK_FORMAT_R8G8B8A8_UNORM;
  atlas_desc.pixels = kWhite;
  atlas_desc.size = sizeof(kWhite);
  vg::Result<vg::Texture> atlas_texture = vg::upload_texture(
      _impl->app.device(), _impl->app.allocator(), atlas_desc);
  if (!atlas_texture) {
    app::set_error(error, atlas_texture.status(), "atlas upload_texture");
    return nil;
  }
  _impl->atlas_texture = std::move(atlas_texture).value();

  vg::Result<vg::Sampler> atlas_sampler = vg::Sampler::create(device);
  if (!atlas_sampler) {
    app::set_error(error, atlas_sampler.status(), "atlas Sampler::create");
    return nil;
  }
  _impl->atlas_sampler.emplace(std::move(atlas_sampler).value());

  // Sized for the whole ring plus the white fallback, allocated once here. A
  // pool grown or re-created later would have to be done mid-frame, on the
  // thread that is recording -- and the sets it hands out are bound by frames
  // still in flight, so freeing one is a use-after-free with no diagnostic on
  // this configuration.
  const std::uint32_t kAtlasSets = app::RendererImpl::kMeshSlots + 1;
  const VkDescriptorPoolSize atlas_pool_size{
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kAtlasSets};
  vg::Result<vg::DescriptorPool> atlas_pool =
      vg::DescriptorPool::create(device, &atlas_pool_size, 1, kAtlasSets);
  if (!atlas_pool) {
    app::set_error(error, atlas_pool.status(), "atlas DescriptorPool::create");
    return nil;
  }
  _impl->atlas_pool = std::move(atlas_pool).value();

  vg::Result<vg::DescriptorSet> atlas_set = _impl->atlas_pool.allocate(
      _impl->mesh_pipeline->descriptor_set_layout(0));
  if (!atlas_set) {
    app::set_error(error, atlas_set.status(), "atlas DescriptorPool::allocate");
    return nil;
  }
  _impl->atlas_set = std::move(atlas_set).value();
  _impl->atlas_set.write_combined_image_sampler(
      0, _impl->atlas_texture.view(), _impl->atlas_sampler->handle(),
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  // The ring's sets, here and only here -- the images they will point at do not
  // exist yet, and are built on the first textured mesh once ARKit has stated
  // its colour size (see build_atlas_ring).
  //
  // Allocated up front because a set taken from this pool cannot be given back:
  // gfx passes no flags to DescriptorPool::create and its header says the kit
  // does not free sets individually. Allocating them lazily inside
  // build_atlas_ring meant a build that failed halfway had permanently spent
  // the sets its completed slots took, out of a pool holding exactly
  // kMeshSlots + 1 -- so the retry that function's caller promises could never
  // succeed, and a transient memory-pressure refusal became a flat-white
  // reconstruction for the life of the process. Taken once here, the count is
  // exact by construction and no later path can exhaust it.
  //
  // A failure here fails bring-up, which is the honest place for it: this is a
  // fixed, small allocation made before the first frame, so it not being
  // available is a configuration fault rather than the transient the per-frame
  // path has to tolerate.
  for (std::size_t i = 0; i < app::RendererImpl::kMeshSlots; ++i) {
    vg::Result<vg::DescriptorSet> slot_set = _impl->atlas_pool.allocate(
        _impl->mesh_pipeline->descriptor_set_layout(0));
    if (!slot_set) {
      app::set_error(error, slot_set.status(),
                     "atlas ring DescriptorPool::allocate");
      return nil;
    }
    _impl->atlas.slots[i].set = std::move(slot_set).value();
  }

  app::FusionConfig fusion_config;
  // One slot per frame in flight, plus one. The renderer draws the extractor's
  // buffers in place now, so an extract must never land on geometry a pending
  // frame is still reading -- and the ring is what makes that impossible rather
  // than merely unlikely.
  fusion_config.mesh_slots = app::RendererImpl::kMeshSlots;
  // Both families, always. Under the two-families plan a phone actually gets,
  // recon writes these buffers on one and gfx reads them on the other, and an
  // EXCLUSIVE buffer read by a family that does not own it is undefined with no
  // error. recon collapses the pair to EXCLUSIVE wherever they are the same
  // family, so this needs no branch on the plan.
  // Measurement mode, and a COMPILE-TIME one: cmake
  // -DVI_INCREMENTAL_BENCHMARK=ON.
  //
  // It was an environment variable first, and that silently did not work. iOS
  // resumes a running process rather than cold-starting it, so a relaunch --
  // even `devicectl ... --terminate-existing` -- re-ran no `getenv`, and the
  // app kept whatever mode the FIRST launch after install had picked up. Runs
  // two through four read as benchmark mode and were measuring the full path;
  // the only tell was `verts == 3 * tris`, i.e. that sharing was off. A toggle
  // whose failure mode is "quietly measured the wrong thing" is worse than no
  // toggle.
  //
  // A define cannot drift across a *resume*: the mode is in the binary, so
  // installing it is what switches it, and there is no runtime state for a
  // resume to carry. It also matches what this is -- a build you launch to read
  // a number, not something a user should reach by tapping.
  //
  // It is not stateless, though, and the remaining state is one configure up
  // rather than one launch: `option()` caches, so an ON latches into the build
  // tree and outlives the configure that set it. The build file warns on every
  // configure that carries it, because the repair someone reaches for -- re-run
  // the README's configure line -- is the one that leaves it on.
  // See FusionConfig::incremental_benchmark.
#ifdef VI_INCREMENTAL_BENCHMARK
  fusion_config.incremental_benchmark = true;
#else
  fusion_config.incremental_benchmark = false;
#endif

  fusion_config.queue_families[0] = _impl->shared.compute_family();
  fusion_config.queue_families[1] = _impl->shared.graphics_family();
  fusion_config.queue_family_count = 2;

  const vr::Status fusion_started = _impl->fusion.start(
      *_impl->recon_device, *_impl->recon_allocator, fusion_config);
  if (!fusion_started) {
    // Likewise: Fusion::start commits the volume, so its usual failure is an
    // OutOfMemory that must reach Swift as one.
    app::set_error(error, fusion_started, "Fusion::start");
    return nil;
  }

  return self;
}

- (BOOL)renderFrameWithDrawableSize:(CGSize)size error:(NSError**)error {
  const VkExtent2D extent{static_cast<std::uint32_t>(size.width),
                          static_cast<std::uint32_t>(size.height)};

  vg::Result<std::optional<vg::windowing::Frame>> frame =
      _impl->app.begin_frame(extent);
  if (!frame) {
    // The fault happened in an *earlier* frame; this is only where it is
    // noticed. Dump what those frames were doing before the error propagates.
    // Through `describe` rather than `message()`: for every gfx fence wait
    // the message is the bare string "vkWaitForFences" -- the call, not its
    // result -- so a lost device and a slow one produced byte-identical
    // dumps, in the log this ring exists to be read from and where the
    // NSError built on the next line is long gone.
    _impl->trace.dump(app::describe(frame.status(), "begin_frame").c_str());
    app::set_error(error, frame.status(), "begin_frame");
    return NO;
  }
  if (!frame.value()) {
    // No drawable this tick (the view is off-screen or mid-rebuild). The
    // protocol working as designed, not a failure.
    return YES;
  }
  const vg::windowing::Frame& f = *frame.value();

  // Claimed here, not at the top of the tick: the window should hold the last
  // kCapacity frames that actually *drew*, not the last kCapacity calls. A
  // rotation, a Slide Over resize or a return from background yields no
  // drawable for far longer than the ring is deep, so claiming per tick flushed
  // the whole window with blank entries -- and a device loss noticed just after
  // one dumped 24 empty lines and none of the frames that could have caused it.
  app::FrameTrace::Entry& trace = _impl->trace.begin_frame_entry();

  // Take the newest mesh, if fusion published one since the last upload. Never
  // wait for it: the render loop draws the previous mesh rather than stalling,
  // which is what keeps presentation smooth while a remesh is in flight.
  //
  // Take the newest mesh, if fusion published one since the last. Nothing is
  // uploaded and nothing is copied: interop seam B hands over the extractor's
  // own VkBuffers, which the draw below reads in place. What used to be here
  // was a ~53 MB download plus a blocking one-shot upload, every remesh, for
  // geometry that never left the device.
  std::optional<app::Fusion::Published> fresh;
  if (!_impl->mesh_unusable) {
    fresh = _impl->fusion.take_mesh(_impl->uploaded_version);
  }
  if (fresh) {
    const vr::mesh::DeviceMesh& m = fresh->mesh;
    // Every generation take_mesh hands over becomes this renderer's to release,
    // drawn or not -- recorded before the test below so the release logic can
    // still retire one it refuses to draw.
    _impl->newest_taken_generation = m.generation;
    // Advanced whichever way the test goes. It used to move only on the
    // accepting branch, so a mesh that could not be bound was re-taken on every
    // following tick: a 60 Hz storm of release calls and failure counts that
    // never terminated, which made the "! upload xN" banner a tick counter
    // rather than a count of anything.
    _impl->uploaded_version = fresh->version;

    // Verified, not assumed. recon reports the usage *and the sharing mode* its
    // buffers were actually created with precisely because Vulkan cannot be
    // asked, and binding one that lacks a usage bit is a validation-layer-only
    // diagnostic -- undefined behaviour in the shipping configuration, which is
    // this one.
    //
    // The sharing mode is the term that can actually vary here, and the one
    // with the most at stake: reading an EXCLUSIVE buffer from a family that
    // does not own it is undefined outright, where a missing usage bit is at
    // least a diagnostic -- and on Apple, where Metal has no queue-ownership
    // concept, it is undefined in the way that appears to work. Checked only
    // when the families actually differ, which is what the queue plan decides
    // at bring-up and what -initWithLayer: passes to Fusion::start; recon
    // collapses the pair to EXCLUSIVE wherever they are the same family, and
    // that is correct rather than a failure.
    const bool cross_family =
        _impl->shared.graphics_family() != _impl->shared.compute_family();
    const bool sharing_ok =
        !cross_family || m.sharing_mode == VK_SHARING_MODE_CONCURRENT;
    const bool bindable =
        m.valid() && sharing_ok &&
        (m.vertex_usage & VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) != 0 &&
        (m.index_usage & VK_BUFFER_USAGE_INDEX_BUFFER_BIT) != 0 &&
        (m.indirect_usage & VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT) != 0;
    if (bindable) {
      // A slot per frame in flight, holding the *generation* rather than the
      // geometry: the buffers are recon's, and this only has to remember which
      // one to release once this frame's fence has signalled.
      _impl->mesh_slot = (_impl->mesh_slot + 1) % app::RendererImpl::kMeshSlots;
      _impl->mesh_slots[_impl->mesh_slot] = m;
      _impl->have_mesh = true;

      // --- Publish the keyframe this mesh's uv0 index into -----------------
      //
      // Into the SAME slot the mesh just went into. They are one value: the
      // uv0 recon wrote address this particular image, so a mesh drawn against
      // any other keyframe samples the wrong place on every textured triangle
      // -- and the result looks like a plausible photograph of somewhere else,
      // not like an error.
      // Gated on `draw_mesh` the property, which is the only half of the draw
      // decision known this early -- `have_mesh` is being set just above, and
      // the full `draw_mesh` is computed 120 lines below, after the camera
      // work. That is far enough down that the whole atlas path used to run
      // unconditionally: with drawMesh = NO, every accepted mesh still paid an
      // 11 MB main-thread memcpy and a recorded 11 MB GPU copy for an image no
      // frame would bind. At remesh_every = 1 and 60 Hz that is ~660 MB/s of
      // each, on the thread holding the display-link deadline, to serve a
      // render path deliberately switched off.
      //
      // Skipping is safe only together with the slot bookkeeping below: a slot
      // left unwritten while drawing is off must not be bound when drawing
      // comes back on, or the first frame after the switch samples a keyframe
      // several meshes stale. `atlas.slot_written` is what carries that, and
      // clearing it is why the else-branch exists rather than the skip being a
      // bare `if`.
      if (_impl->draw_mesh && fresh->atlas != nullptr) {
        // Built on the first textured mesh rather than at bring-up, because the
        // size is ARKit's to state: `imageResolution` is not known until a
        // frame has arrived, and guessing 1920x1440 would be a constant that
        // silently mis-sizes the ring on any device that reports otherwise.
        if (!_impl->atlas.ready) {
          // The sampler is engaged from bring-up onwards -- a failure there
          // returns nil -- but it is passed rather than reached for, so the
          // ring never dereferences an optional it cannot see being filled,
          // and its null check is a real guard rather than a decorative one.
          const vg::Status built = app::build_atlas_ring(
              _impl->atlas, _impl->app.allocator(),
              _impl->atlas_sampler ? _impl->atlas_sampler->handle()
                                   : VK_NULL_HANDLE,
              fresh->atlas_width, fresh->atlas_height);
          if (!built.ok()) {
            // Not fatal to the frame, and deliberately not latched: the ring is
            // an allocation of ~kMeshSlots * 11 MB of images plus as much again
            // in mapped staging buffers, so a refusal here is much more likely
            // to be transient memory pressure than a permanent fault. The mesh
            // still draws, with the white atlas bound and every textured
            // triangle sampling white -- so the failure is visible rather than
            // silent, and the next remesh retries. The retry can now actually
            // succeed; see build_atlas_ring for what used to stop it.
            //
            // Its own counter, not the mesh-upload pair: that one latches and
            // means the geometry is undrawable forever. See `atlas_failures`.
            ++_impl->atlas_failures;
            _impl->atlas_error = "ring: " + std::string(built.message());
          }
        }
        // Checked, not assumed: a ring built for one colour size and handed
        // another would read past the staged image. ARKit does not change
        // `imageResolution` mid-session, which is exactly why an unchecked
        // mismatch would be a latent read overrun rather than a visible bug.
        const bool extent_ok = _impl->atlas.ready &&
                               fresh->atlas_width == _impl->atlas.width &&
                               fresh->atlas_height == _impl->atlas.height;
        if (extent_ok) {
          app::AtlasSlot& slot = _impl->atlas.slots[_impl->mesh_slot];
          // The same expression that sized the buffer, called rather than
          // restated: the two live in different translation units now, and a
          // format change applied to only one of them writes past the mapping.
          const VkDeviceSize bytes =
              app::atlas_staging_bytes(_impl->atlas.width, _impl->atlas.height);
          // The one host copy on this path. `Published::atlas` is valid only
          // until the next take_mesh, so it is consumed here, in the same call
          // that received it, rather than remembered.
          std::memcpy(slot.staging.mapped(), fresh->atlas,
                      static_cast<std::size_t>(bytes));
          // Recorded here, above `f.target->begin` below, and that position is
          // load-bearing rather than incidental: vkCmdCopyBufferToImage may not
          // be recorded inside a render pass instance, and this build ships
          // without the validation layer that would say so. See the
          // precondition on record_atlas_upload.
          app::record_atlas_upload(
              f.cmd, slot.staging.handle(), slot.texture.image(),
              _impl->atlas.width, _impl->atlas.height,
              _impl->atlas.slot_in_undefined_layout[_impl->mesh_slot]);
          // Two flags, deliberately: the image has now been written, so it is
          // no longer in UNDEFINED and never will be again until the ring is
          // rebuilt -- while `slot_written`, which is about bindability, gets
          // cleared below for reasons that leave the layout exactly as it is.
          _impl->atlas.slot_in_undefined_layout[_impl->mesh_slot] = false;
          _impl->atlas.slot_written[_impl->mesh_slot] = true;
        } else if (_impl->atlas.ready) {
          // A textured mesh whose keyframe could not be staged. Skipping the
          // upload alone is not enough and was the bug: `fresh->atlas` being
          // non-null means Fusion *did* texture this mesh, so every vertex
          // carries a real uv0 -- and the bind below would hand it whichever
          // keyframe this slot last held, kMeshSlots meshes ago. That renders
          // as a plausible photograph of somewhere else painted over the live
          // scan, which is the single failure the mesh/slot pairing exists to
          // make unrepresentable.
          //
          // Marking the slot unwritten is what makes it bind the white fallback
          // instead: wrong-looking, obviously, and honestly.
          //
          // Deliberately not a rebuild. Re-running build_atlas_ring here would
          // free images that frames still in flight are binding -- a
          // use-after-free needing a queue drain to make safe, on the thread
          // that must not stall. ARKit does not resize mid-session, so this
          // path is defensive; if it ever fires it stays degraded for the rest
          // of the scan, and says so rather than recovering quietly.
          _impl->atlas.slot_written[_impl->mesh_slot] = false;
          ++_impl->atlas_failures;
          _impl->atlas_error =
              "keyframe is " + std::to_string(fresh->atlas_width) + "x" +
              std::to_string(fresh->atlas_height) + " but the ring was built " +
              std::to_string(_impl->atlas.width) + "x" +
              std::to_string(_impl->atlas.height) +
              "; textured meshes render white for the rest of this scan";
        }
      } else if (fresh->atlas != nullptr) {
        // Drawing is off, so the upload above was skipped. The mesh in this
        // slot still has real uv0, so the slot must not stay bindable -- see
        // the gate's note. The next remesh reaching it while drawing is on
        // writes it again.
        _impl->atlas.slot_written[_impl->mesh_slot] = false;
      }
      // The message is deliberately NOT cleared here. `mesh_upload_failures` is
      // a running total like every other counter on this read-out, and clearing
      // only the message left the banner gated on a count that never resets:
      // one transient failure followed by a recovery read "! upload x1: " --
      // for the rest of the process, an alarm with no content. Count and reason
      // stay together, so the line means "this happened N times, most recently
      // for this reason" whether or not it is still happening.
    } else {
      // Deliberately *not* released here. release_through is a monotonic
      // high-water mark, so handing it this generation would retire every older
      // one with it -- including the generation this very frame goes on to
      // draw, since have_mesh and mesh_slot still name it. recon would then be
      // free to claim that slot, and a grow frees its buffers outright, under a
      // live draw. The ordinary in-flight logic below retires it instead, which
      // by construction never releases past something still being read.
      //
      // Latched, because this cannot heal: the usage bits come from two
      // constants in Fusion::start and the sharing mode from the queue plan, so
      // a mesh that is unusable once is unusable every time. Collecting the
      // ones that follow would only walk recon's ring to exhaustion, one
      // undrawable generation at a time, and bury the reason under a count.
      _impl->mesh_unusable = true;
      ++_impl->mesh_upload_failures;
      _impl->mesh_upload_error =
          sharing_ok
              ? "mesh is not bindable as geometry (usage bits or handles "
                "missing)"
              : "mesh buffers are VK_SHARING_MODE_EXCLUSIVE but recon and gfx "
                "are on different queue families; binding them would be "
                "undefined";
    }
  }

  vg::RenderTargetBeginInfo begin{};
  begin.load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  begin.clear_color = {{0.05f, 0.06f, 0.09f, 1.0f}};
  f.target->begin(f.cmd, begin);

  // The newest fused pose, taken every frame rather than only on remesh, so a
  // following camera tracks smoothly between mesh updates. Kept up to date
  // outside the draw branch as well, because it is also what a gesture seeds
  // the turntable from -- and the user can reach for the screen before the
  // first mesh ever arrives.
  _impl->camera_to_world = _impl->fusion.last_pose();
  // Back to the OpenGL camera convention. recon's pose is CV (+Z forward, +Y
  // down) because that is what its projection wants, but glm::perspective maps
  // -Z forward -- so feeding it the CV pose directly puts the entire scene
  // *behind* the camera and renders nothing but backfaces. cv_from_gl_camera is
  // an involution, so applying it again is the conversion back.
  glm::mat4 device_pose = vr::sensor::cv_from_gl_camera(_impl->camera_to_world);
  // ...and then round to the viewport. ARKit fixes the camera basis to the
  // *sensor*: +X along the long axis of the device, +Y along the short axis,
  // +Z out of the screen (ARCamera.transform). That frame does not turn when
  // the interface does, so rendering the pose straight into a portrait drawable
  // puts the scan on its side -- which reads as a broken reconstruction rather
  // than a misaligned render camera.
  //
  // What the turn *is* is not decided here. See `kSensorBasisOrientation` in
  // Core/ViewOrientation.hpp, which is the only place in this app that turns an
  // orientation into an angle, and carries the derivation and the device check
  // with it. The tests in tests/view_orientation_test.cpp pin the result.
  //
  // It does have to sit *here*, after cv_from_gl_camera and before either
  // camera sees the pose. Fusion is unaffected -- the pose and the intrinsics
  // are mutually consistent in the sensor frame -- but OrbitCamera::take_over
  // seeds the turntable's heading from device_pose_[1], the up column this turn
  // rewrites, whenever the aim is steeper than about 45 degrees. The
  // turntable's steady state is roll-free because view() imposes world up, but
  // its seed is not, so a wrong turn here is laundered into yaw_ on the first
  // drag rather than dropped. Moving this inside the follow branch would fix
  // follow mode and leave that seed reading a raw sensor pose.
  //
  // Skipped only when the angle itself is zero. The guard used to test the enum
  // *value*, which was the same test only while the zero sat at raw 0 -- so
  // moving the zero would have silently skipped the one orientation that now
  // needs the largest turn of the four.
  // app::viewport_turn defines this and the host tests pin it; the renderer
  // holds the pure-C++ enum, so there is nothing to convert here.
  if (const float turn = app::viewport_turn(_impl->view_orientation);
      turn != 0.0f) {
    device_pose = glm::rotate(device_pose, turn, glm::vec3(0.0f, 0.0f, 1.0f));
  }
  _impl->camera.set_device_pose(device_pose);

  // `have_mesh` is false for the whole session under VI_INCREMENTAL_BENCHMARK:
  // it is armed only by a successful take below, and that mode publishes
  // nothing for a take to find. So this is not "keeps drawing the last mesh" --
  // there is no last mesh, and the scene stays empty from launch. The stats
  // panel says so in as many words, because an empty view is otherwise read as
  // ARKit having lost tracking, and this mode's number depends on whoever is
  // holding the iPad believing the scan is working and walking the room.
  const bool draw_mesh = _impl->draw_mesh && _impl->have_mesh;

  // --- Retire the generations no frame in flight is reading any more --------
  //
  // Outside the draw branch, and that placement is the correction rather than a
  // tidy-up: every release the consumer owes recon used to sit inside
  // `if (draw_mesh)`, so setting the public `drawMesh` property to NO stopped
  // all of them permanently. The renderer went on collecting meshes --
  // take_mesh runs regardless, and marking one taken is exactly what stops
  // fusion reusing its slot -- released nothing, and after kMeshSlots extracts
  // recon refused every one that followed. Turning drawing back on could not
  // recover it either: the renderer only ever released what it drew, and by
  // then it could no longer obtain anything to draw.
  //
  // Park the generation this frame draws -- zero when it draws none -- then
  // release everything strictly older than the oldest generation any frame
  // still in flight is reading.
  //
  // Releasing "what the frame that just retired drew" is what this used to
  // do, and it is wrong whenever one generation spans more than one frame --
  // which is the normal case, not a corner: mesh_slot advances only when
  // take_mesh yields a *new* mesh, so at 60 Hz with a remesh every few fused
  // frames the same generation is drawn many frames running. Two frames in
  // flight then both hold it, begin_frame has waited on only the older one's
  // fence, and releasing on that fence hands recon a slot the newer frame is
  // still reading. recon is then free to pick it -- and a grow *frees* its
  // buffers outright, so the in-flight draw reads a destroyed VkBuffer. That
  // is a GPU fault surfacing as VK_ERROR_DEVICE_LOST out of the next
  // vkWaitForFences, and it needs the arena to actually grow, which is why it
  // only shows up once the scan gets large.
  //
  // The min is over the whole array because every entry names a frame still
  // in flight once the current one is recorded; anything older than all of
  // them is finished everywhere. release_through is a monotonic high-water
  // mark, so a repeated or lower value is harmless.
  const std::size_t recording =
      _impl->frame_slot % app::RendererImpl::kFramesInFlight;
  _impl->frame_generations[recording] =
      draw_mesh ? _impl->mesh_slots[_impl->mesh_slot].generation : 0;
  ++_impl->frame_slot;

  std::uint64_t oldest_in_flight = 0;
  for (const std::uint64_t g : _impl->frame_generations) {
    if (g != 0 && (oldest_in_flight == 0 || g < oldest_in_flight)) {
      oldest_in_flight = g;
    }
  }
  // Generations are pre-incremented from 0, so 1 is the first real one and
  // there is nothing below it to release. An all-zero array means no frame in
  // flight holds a generation at all -- drawing is off, or nothing has been
  // drawn yet -- and then everything collected so far is finished by
  // definition. That fallback is the only thing that drains the ring while
  // drawMesh is NO, and the only thing that lets it refill when it goes back
  // on.
  const std::uint64_t released_through = oldest_in_flight > 0
                                             ? oldest_in_flight - 1
                                             : _impl->newest_taken_generation;
  if (released_through > 0) {
    _impl->fusion.release_through(released_through);
  }
  trace.released_through = released_through;

  // Fusion's half of the entry, outside the draw branch.
  //
  // Ten of the twelve fields were written only when a mesh was drawn, and the
  // dump's format is fixed -- so a frame that never sampled the allocator
  // printed `alloc=ok arena=0 blocks=0 occ=0.0%`, byte-identical to one that
  // measured an idle allocator and an empty arena. Those are the regimes a
  // device-lost dump is *read* in: before the first successful take, after a
  // latched upload failure, or under VI_INCREMENTAL_BENCHMARK, which publishes
  // no geometry for the whole session. `arena_bytes` carries a reading
  // instruction on FrameTrace::Entry -- compare it against the previous entry
  // -- and a phantom drop to 0 at every drew=1 -> drew=0 boundary breaks
  // exactly the comparison that would find the use-after-free described above.
  //
  // The same placement mistake as the release logic thirty lines up, which sat
  // inside this branch until it stopped recon reusing anything at all.
  //
  // The narrow accessor, not stats(): this runs every frame, and FusionStats
  // carries a std::string whose copy would malloc inside the mutex the fuse
  // thread takes on every one of its own frames. A handful of scalars is all
  // the ring holds. See Fusion::trace_stats.
  const app::FusionTraceStats fused = _impl->fusion.trace_stats();
  trace.triangles = fused.triangles;
  trace.triangle_capacity = fused.triangle_capacity;
  trace.arena_bytes = fused.arena_bytes;
  trace.active_blocks = fused.active_blocks;
  trace.occupancy = fused.occupancy;
  trace.occupancy_known = fused.occupancy_known;
  // Latched rather than passed through reportable_allocation_stop, unlike the
  // three live renderings: this is the forensic copy, and `ms_since_fuse`
  // beside it is what qualifies the cause. Discarding what the cause *was* is
  // the wrong trade in the one artifact a device loss leaves behind.
  trace.stop = fused.allocation_stop;
  trace.ms_since_fuse = fused.ms_since_fuse;
  trace.extract_ms = fused.extract_ms;

  if (draw_mesh) {
    // recon's buffers, named rather than copied. LiveMesh owns nothing and
    // reads the index count GPU-side out of the indirect command, so the count
    // never crosses the CPU either.
    const vr::mesh::DeviceMesh& live_src = _impl->mesh_slots[_impl->mesh_slot];
    vg::pipelines::LiveMesh live;
    live.vertices = live_src.vertices;
    live.indices = live_src.indices;
    live.indirect = live_src.indirect;
    const vg::pipelines::HybridMeshDraw draw{live};

    // The three fields that are genuinely about the mesh this frame drew.
    trace.drew_mesh = true;
    trace.generation = live_src.generation;
    trace.mesh_slot = _impl->mesh_slot;

    // Both ends clamped, not just the denominator. A zero *width* drawable is
    // just as reachable as a zero height -- an orientation change, an iPad
    // Slide Over resize -- and it does not divide by zero, it yields aspect 0,
    // which makes glm::perspective's first term 1/(0 * tanHalfFovy) = +inf and
    // NaNs every clip-space x. The mesh then vanishes into a clear-coloured
    // frame that still returns YES: indistinguishable on screen from "fusion
    // produced nothing", which is the exact ambiguity the atlas comment above
    // exists to design out. In Debug, glm asserts instead and CI aborts.
    const float aspect = static_cast<float>(std::max(extent.width, 1u)) /
                         static_cast<float>(std::max(extent.height, 1u));
    // The same FOV the camera scales a pan by, and the same clip range its
    // zoom-out limit is derived from -- see kVerticalFov and kFarClip. A pan
    // computed against a different FOV slides out from under the finger, and a
    // far plane nearer than the camera can pull the pivot clips the scan away.
    glm::mat4 proj = glm::perspective(app::kVerticalFov, aspect, app::kNearClip,
                                      app::kFarClip);
    // GL and Vulkan clip space differ in *two* ways, and glm targets GL.
    //
    // Depth range: GL's NDC z runs [-1, 1], Vulkan's [0, 1]. That one is a glm
    // compile-time switch, GLM_FORCE_DEPTH_ZERO_TO_ONE, set on this target in
    // CMakeLists.txt -- glm is header-only, so it has to be defined for every
    // TU that instantiates a projection, not included from somewhere. Without
    // it Vulkan's 0 <= z_clip <= w_clip test discards everything nearer than
    // the harmonic mean 2fn/(f+n), which at kNearClip = 0.05 / kFarClip = 50 is
    // ~0.1 m: the effective near plane is double what kNearClip says, and
    // zooming the turntable in to kMinDistance puts the pivot inside the clip
    // volume and the scan disappears.
    //
    // Framebuffer Y: Vulkan's +Y is down where GL's is up. glm has no mode for
    // that one, so it stays arithmetic here.
    proj[1][1] *= -1.0f;

    vg::pipelines::HybridMeshFrame frame_info{};
    frame_info.extent = extent;
    // Either the device pose or the turntable, depending on whether the user
    // has taken the camera over.
    frame_info.view_proj = proj * _impl->camera.view();
    // Required. Leave it null and submit() records nothing whatsoever.
    // The keyframe belonging to the mesh this frame is about to draw, or the
    // 1x1 white fallback before the ring exists. Indexed by `mesh_slot` because
    // the two are one value -- see AtlasSlot.
    //
    // The set must be non-null either way -- HybridMeshPipeline::submit records
    // no bind, no push constant and no draw at all on a null one, which is
    // indistinguishable from a working renderer looking at empty space.
    //
    // Gated on the slot having been WRITTEN, not on the ring existing.
    // build_atlas_ring creates all kMeshSlots images at once and each starts in
    // VK_IMAGE_LAYOUT_UNDEFINED, but only the slot a textured mesh lands in is
    // ever uploaded -- so between the first textured mesh and the ring coming
    // fully round, `atlas.ready` is true for slots that have never been
    // written. Binding one declares SHADER_READ_ONLY_OPTIMAL for an image in
    // UNDEFINED, and gfx's hybrid_mesh.frag samples the atlas before it
    // branches, so the read happens unconditionally: undefined behaviour, with
    // no validation layer on this configuration to name it. Reachable as soon
    // as one remesh publishes no keyframe -- a colour the capture refused, a
    // texture pass that failed -- between two that do.
    //
    // On those frames this is also *correct* rather than merely safe: a mesh
    // Fusion did not texture has every uv0 at recon's sentinel, so the shader
    // takes the vertex-colour branch and never looks at what is bound here.
    const bool atlas_bindable =
        _impl->atlas.ready && _impl->atlas.slot_written[_impl->mesh_slot];
    frame_info.atlas = atlas_bindable
                           ? _impl->atlas.slots[_impl->mesh_slot].set.handle()
                           : _impl->atlas_set.handle();
    frame_info.draws = &draw;
    frame_info.draw_count = 1;
    _impl->mesh_pipeline->submit(f.cmd, frame_info);
  } else {
    vkCmdBindPipeline(f.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                      _impl->pipeline.handle());

    // Viewport and scissor are dynamic state, so they follow the swapchain
    // through a rotation without rebuilding the pipeline. (The hybrid pipeline
    // sets its own from HybridMeshFrame::extent.)
    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(f.cmd, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = extent;
    vkCmdSetScissor(f.cmd, 0, 1, &scissor);

    vkCmdDraw(f.cmd, 3, 1, 0, 0);
  }

  f.target->end(f.cmd);

  const vg::Status end = _impl->app.end_frame(f);
  if (!end.ok()) {
    // A stale swapchain is the normal signal that the drawable changed size;
    // the next begin_frame rebuilds. Only a genuine error propagates.
    if (!vg::windowing::swapchain_stale(end)) {
      // Dumped here as well as out of begin_frame. gfx reports a vkQueueSubmit
      // or present failure from this call, and a VK_ERROR_DEVICE_LOST is a
      // normal way MoltenVK surfaces one -- `swapchain_stale` matches only
      // OUT_OF_DATE and SUBOPTIMAL, so a lost device took this path and threw
      // the whole ring away in silence. ScannerViewController suspends the
      // loop on the NO below, so the 24 entries would die with _impl.
      _impl->trace.dump(app::describe(end, "end_frame").c_str());
      app::set_error(error, end, "end_frame");
      return NO;
    }
    return YES;
  }
  ++_impl->frames_presented;
  return YES;
}

- (void)dealloc {
  // Explicit, and in this order: the fuse thread must be joined before the
  // capture it polls is released, and both before ~RendererImpl drains the
  // queues and unwinds Vulkan. ARC releases the ivars after this returns and
  // says nothing about their order relative to each other.
  [self stopFusion];
}

- (void)startFusionWithCapture:(VolumetricCapture*)capture {
  if (_impl->fusing.load()) {
    return;
  }
  // Retained for the life of the thread below, which reads the pointer this
  // unwraps on every iteration.
  _capture = capture;
  _impl->capture =
      static_cast<vr::sensor::ICameraCapture*>([capture captureHandle]);
  _impl->fusing.store(true);

  // The impl pointer, deliberately, and not `self`: this TU is compiled
  // -fobjc-arc, so a by-copy capture of `self` would be __strong and the retain
  // would live until the lambda is destroyed -- which needs the thread to exit,
  // which needs `fusing` to go false. That is renderer -> _impl -> fuse_thread
  // -> lambda -> renderer, a cycle nothing outside the loop can break, and
  // -dealloc would never run at all. The impl outlives the thread by
  // construction: ~RendererImpl joins before destroying anything.
  app::RendererImpl* impl = _impl.get();
  _impl->fuse_thread = std::thread([impl] {
    while (impl->fusing.load()) {
      // Contained here rather than allowed to escape. A throw out of a thread
      // function is std::terminate -- the app vanishes with no message and no
      // crash context -- and the reachable one is real: every remesh builds a
      // fresh ~50 MB host vertex vector inside marching_cubes->download, on a
      // phone that is already holding the volume. Per iteration, so Fusion's
      // contract still holds: one bad frame is recorded and skipped, it does
      // not end a scan the user is in the middle of.
      try {
        vr::Result<std::optional<vr::sensor::CapturedFrame>> got =
            impl->capture->poll();
        if (!got || !got.value()) {
          // Nothing new: sleep briefly rather than spin. ARKit delivers at
          // 60 Hz and this loop is otherwise free to burn a core doing nothing.
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          continue;
        }
        impl->fusion.fuse(*got.value());
      } catch (const std::exception& e) {
        impl->fusion.note_error(std::string("fuse thread: ") + e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } catch (...) {
        impl->fusion.note_error("fuse thread: unknown exception");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  });
}

- (void)beginStopFusion {
  if (_impl) {
    // The flag only. `capture` stays valid and the thread stays joinable --
    // -stopFusion is still what makes either of those untrue.
    _impl->fusing.store(false);
  }
}

- (void)stopFusion {
  if (_impl) {
    _impl->stop_fusing();
  }
  // After the join, never before: the thread dereferences the capture handle
  // this retain keeps alive.
  _capture = nil;
}

// Written out rather than left to auto-synthesis. A `@property` with no
// accessors gets an ivar of its own, so `renderer.drawMesh = NO` would set a
// field nothing reads and leave `_impl->draw_mesh` at its default -- silently
// disabling the one A/B this app has for telling a dead render path from
// misplaced geometry.
- (BOOL)drawMesh {
  return _impl->draw_mesh;
}

- (void)setDrawMesh:(BOOL)drawMesh {
  _impl->draw_mesh = drawMesh;
}

- (float)textureOcclusionThreshold {
  return _impl->fusion.occlusion_threshold();
}

- (void)setTextureOcclusionThreshold:(float)metres {
  // The return is deliberately dropped: an Objective-C setter cannot report,
  // and Fusion refuses rather than clamps, so a rejected value leaves the
  // previous one in force. Reading the property back is what tells a caller
  // which happened -- named in the header, since a silently-ignored setter is
  // otherwise indistinguishable from one that worked.
  (void)_impl->fusion.set_occlusion_threshold(metres);
}

#pragma mark - Camera

- (void)orbitByFractionX:(float)dx y:(float)dy {
  _impl->camera.orbit(dx, dy);
}

- (void)panByFractionX:(float)dx y:(float)dy {
  _impl->camera.pan(dx, dy);
}

- (void)zoomByScale:(float)scale {
  _impl->camera.zoom(scale);
}

- (void)followDevice {
  _impl->camera.follow_device();
}

// The one seam between the Objective-C enum Swift sets and the pure-C++ one the
// renderer holds. A cast rather than a switch because the two mirror each other
// value for value -- which is not an assumption: the static_asserts above pin
// all four, so a reordering of either enum is a build failure here rather than
// a scan silently turned by a multiple of 90 degrees.
- (VolumetricViewOrientation)viewOrientation {
  return static_cast<VolumetricViewOrientation>(_impl->view_orientation);
}

- (void)setViewOrientation:(VolumetricViewOrientation)viewOrientation {
  _impl->view_orientation = static_cast<app::ViewOrientation>(viewOrientation);
}

- (BOOL)followingDevice {
  return _impl->camera.following();
}

- (float)cameraDistance {
  return _impl->camera.distance();
}

+ (NSUInteger)frameHistoryCapacity {
  return static_cast<NSUInteger>(app::Fusion::kHistoryCapacity);
}

+ (VolumetricGaugeThresholds)gaugeThresholds {
  // Transcribed field by field rather than cast from the C++ pairs. The two
  // structs are laid out identically today and a reinterpret_cast would work
  // today, which is exactly the kind of coupling that survives a field being
  // inserted on one side and starts painting the memory bar with the arena's
  // thresholds -- a wrong colour that looks like a right one.
  //
  // Assigned by name rather than positionally, which is the half that was
  // missing: all three members are the same type, so a brace list is as
  // order-dependent as the cast it replaced. Swapping `arenaFill` and `memory`
  // in the header, or `warn` and `critical` in the pair, compiled with no
  // diagnostic and drew each bar against the other's tiers.
  const auto pair = [](app::ToneThresholds t) {
    return VolumetricToneThresholds{t.warn, t.critical};
  };
  VolumetricGaugeThresholds out;
  // The one pair that is not a constant on this side: its critical tier is the
  // allocate guard and its warn tier is recon's, so it is derived from both.
  out.occupancy =
      pair(app::occupancy_thresholds(app::Fusion::grow_threshold()));
  out.arenaFill = pair(app::kArenaFillThresholds);
  out.memory = pair(app::kMemoryThresholds);
  return out;
}

+ (VolumetricStatTone)toneForFraction:(double)fraction
                           thresholds:(VolumetricToneThresholds)thresholds {
  return static_cast<VolumetricStatTone>(app::tone_for(
      fraction, app::ToneThresholds(thresholds.warn, thresholds.critical)));
}

- (uint32_t)blockCapacity {
  return _impl->fusion.stats().table_capacity;
}

- (uint64_t)memoryFootprintBytes {
  return app::query_memory_budget().footprint_bytes;
}

- (uint64_t)memoryLimitBytes {
  const app::MemoryBudget budget = app::query_memory_budget();
  // 0 when the OS declined to answer or is over the limit, which the dashboard
  // reads as "no ceiling known" rather than drawing a full bar -- see
  // MemoryBudget::limit_known.
  return budget.limit_known ? budget.limit_bytes : 0;
}

- (uint64_t)gpuWorkingSetBytes {
  return gpu_working_set_bytes();
}

- (NSArray<VolumetricFrameSample*>*)frameHistory {
  // Asked how many there are before allocating room for them, rather than
  // value-initialising the full ring on every poll: for most of a scan's first
  // seconds -- and for the whole of a short one -- that zero-filled the other
  // 236 slots to return four. The null-buffer query is the accessor's own
  // documented way to ask.
  //
  // Two lock acquisitions, so the ring may gain entries between them. That is
  // bounded rather than racy: `history` clamps its copy to the capacity handed
  // in, so a grown ring simply yields its newest `available` samples and the
  // buffer cannot be overrun. The alternative -- one call holding the lock
  // across an allocation -- is the thing the fuse thread must not wait on.
  //
  // Still the fusion's bound and never a number repeated here: one that drifted
  // below the real capacity would silently truncate and read as a shorter scan.
  const std::size_t available = _impl->fusion.history(nullptr, 0);
  std::vector<app::FrameSample> samples(available);
  const std::size_t count =
      available == 0 ? 0
                     : _impl->fusion.history(samples.data(), samples.size());
  NSMutableArray<VolumetricFrameSample*>* out =
      [NSMutableArray arrayWithCapacity:count];
  for (std::size_t i = 0; i < count; ++i) {
    [out addObject:[[VolumetricFrameSample alloc] initWithSample:samples[i]]];
  }
  return out;
}

- (NSArray<VolumetricStageRow*>*)stageRows {
  return app::stage_rows(_impl->fusion.stats());
}

- (NSArray<VolumetricStatSection*>*)statSections {
  const app::FusionStats s = _impl->fusion.stats();
  const app::MemoryBudget budget = app::query_memory_budget();
  return app::stat_sections(readout_inputs(*_impl, s, budget));
}

- (VolumetricDashboardSnapshot*)dashboardSnapshot {
  // Three reads, once each, and everything the panel shows is derived from
  // them. Assembling the same panel from the individual properties took five
  // `FusionStats` copies and three `task_info` traps, with the fuse thread
  // writing in between -- see VolumetricDashboardSnapshot for what that let the
  // headline and the Volume card disagree about. `ReadoutInputs` is what makes
  // that arithmetic impossible rather than merely discouraged.
  //
  // The history is its own source and its own lock; it is read once here rather
  // than folded in, because the ring and the stats are different structures and
  // no single lock covers both.
  const app::FusionStats s = _impl->fusion.stats();
  const app::MemoryBudget budget = app::query_memory_budget();
  return app::dashboard_snapshot(readout_inputs(*_impl, s, budget),
                                 self.frameHistory);
}

- (NSString*)fusionSummary {
  // Its own read, which is why a caller wanting both this and the panel on one
  // tick should take `dashboardSnapshot` and read `.summary` off it instead --
  // that is this same text, rendered from the rows that snapshot carries. Kept
  // for callers that want only the transcript.
  const app::FusionStats s = _impl->fusion.stats();
  const app::MemoryBudget budget = app::query_memory_budget();
  return app::fusion_summary(readout_inputs(*_impl, s, budget));
}
- (void)noteMemoryWarning {
  if (!_impl) {
    return;
  }
  _impl->memory_warnings += 1;
  // Sampled here rather than left to the next poll, because this is the one
  // moment the OS has told us the number matters and the next tick is up to
  // half a second away -- long enough for the allocation that provoked the
  // warning to have been freed again.
  const app::MemoryBudget budget = app::query_memory_budget();
  if (budget.valid) {
    _impl->memory_warning_footprint_bytes = budget.footprint_bytes;
  }
  // os_log_error rather than os_log: this is the only pre-jetsam signal the app
  // gets, and the error level is what survives into `log collect` at the
  // default capture settings -- the read-out's own mirror is os_log, which a
  // killed process may not have flushed a copy of at the relevant moment.
  os_log_error(
      OS_LOG_DEFAULT, "vk-scan: memory warning #%llu, footprint %llu MB",
      static_cast<unsigned long long>(_impl->memory_warnings),
      static_cast<unsigned long long>(_impl->memory_warning_footprint_bytes /
                                      (1024ULL * 1024ULL)));
}

- (void)waitIdle {
  if (!_impl) {
    return;
  }
  if (_impl->app.valid()) {
    (void)_impl->app.wait_idle();
  }
  // gfx idles only the queues it was assigned, and recon's Device exposes no
  // wait at all -- so on a two-family plan recon's queue is one nobody else
  // would ever drain. The bootstrap owns them both and waits on both.
  _impl->shared.wait_idle();
}

- (NSString*)deviceName {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(_impl->app.device().physical_device(), &props);
  return app::to_ns_string(props.deviceName);
}

- (NSString*)apiVersion {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(_impl->app.device().physical_device(), &props);
  return app::to_ns_string(api_version_string(props.apiVersion));
}

- (NSString*)sharedDeviceSummary {
  return app::to_ns_string(_impl->shared.summary());
}

- (BOOL)sharesOneDevice {
  if (!_impl->app.valid() || !_impl->recon_device) {
    return NO;
  }
  const VkDevice bootstrap = _impl->shared.device();
  if (bootstrap == VK_NULL_HANDLE) {
    return NO;
  }
  // The handle comparison is a post-condition, not a discovery: both wrappers
  // were handed this same field and cannot come back differing. It is worth
  // reporting only alongside owns_device(), which is false purely because each
  // went through `adopt` -- a library that quietly created a device of its own
  // is the divergence this can actually catch.
  return _impl->app.device().handle() == bootstrap &&
         _impl->recon_device->handle() == bootstrap &&
         !_impl->app.device().owns_device() &&
         !_impl->recon_device->owns_device();
}

- (uint64_t)framesPresented {
  return _impl->frames_presented;
}

@end

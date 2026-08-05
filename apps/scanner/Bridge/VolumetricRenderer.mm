// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Objective-C++ seam. This is the one translation unit where a
// CAMetalLayer* and a vg::app::WindowedApp are both first-class, which is the
// entire reason the bridge is .mm rather than Swift.

#import "VolumetricRenderer.h"

#import "Fusion.hpp"
#import "OrbitCamera.hpp"
#import "SharedDevice.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>

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
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"

namespace vg = volumetric_kit::gfx;
namespace vr = volumetric_kit::recon;
namespace app = volumetric_kit::ios_app;

NSErrorDomain const VolumetricRendererErrorDomain =
    @"io.taojin.volumetrickit.renderer";
NSErrorUserInfoKey const VolumetricRendererVulkanResultKey =
    @"VolumetricRendererVulkanResult";

namespace {

// Never nil, so a `nonnull` property cannot hand Swift a null it traps on:
// `stringWithUTF8String:` returns nil for invalid UTF-8, and Vulkan promises
// only that VkPhysicalDeviceProperties::deviceName is a NUL-terminated
// char[256] -- a driver may put any bytes in it, and Swift imports the property
// as a non-optional String.
NSString* to_ns_string(const std::string& text) {
  if (NSString* utf8 = [NSString stringWithUTF8String:text.c_str()]) {
    return utf8;
  }
  // Latin-1 maps every byte to a code point, so this cannot fail in turn.
  NSString* latin1 = [[NSString alloc] initWithBytes:text.data()
                                              length:text.size()
                                            encoding:NSISOLatin1StringEncoding];
  return latin1 != nil ? latin1 : @"(unprintable)";
}

VolumetricRendererError error_code(vg::Status::Code domain) {
  switch (domain) {
    case vg::Status::Code::Ok:
      return VolumetricRendererErrorUnknown;
    case vg::Status::Code::InvalidArgument:
      return VolumetricRendererErrorInvalidArgument;
    case vg::Status::Code::NotFound:
      return VolumetricRendererErrorNotFound;
    case vg::Status::Code::Unsupported:
      return VolumetricRendererErrorUnsupported;
    case vg::Status::Code::OutOfMemory:
      return VolumetricRendererErrorOutOfMemory;
    case vg::Status::Code::IoError:
      return VolumetricRendererErrorIoError;
    case vg::Status::Code::Vulkan:
      return VolumetricRendererErrorVulkan;
  }
  return VolumetricRendererErrorUnknown;
}

VolumetricRendererError error_code(vr::Status::Code domain) {
  switch (domain) {
    case vr::Status::Code::Ok:
      return VolumetricRendererErrorUnknown;
    case vr::Status::Code::InvalidArgument:
      return VolumetricRendererErrorInvalidArgument;
    case vr::Status::Code::NotFound:
      return VolumetricRendererErrorNotFound;
    case vr::Status::Code::Unsupported:
      return VolumetricRendererErrorUnsupported;
    case vr::Status::Code::OutOfMemory:
      return VolumetricRendererErrorOutOfMemory;
    case vr::Status::Code::IoError:
      return VolumetricRendererErrorIoError;
    // recon keeps its backend neutral, but here the backend *is* Vulkan and the
    // detail code is the VkResult.
    case vr::Status::Code::Backend:
      return VolumetricRendererErrorVulkan;
  }
  return VolumetricRendererErrorUnknown;
}

// Surface a library Status as an NSError so Swift sees a native failure instead
// of a status code it would have to interpret. The two libraries' Status types
// are structurally alike (domain + optional backend code + message) but neither
// imports the other, so each overload below reduces its own to these values.
void set_error(NSError** error, const char* stage, VolumetricRendererError code,
               std::optional<VkResult> vk_result, const std::string& message) {
  if (error == nullptr) {
    return;
  }
  NSMutableDictionary* info = [NSMutableDictionary dictionary];
  std::string described = std::string(stage) + ": " + message;
  if (vk_result) {
    described += " (";
    described += std::string(vg::to_string(*vk_result));
    described += ")";
    info[VolumetricRendererVulkanResultKey] = @(static_cast<int>(*vk_result));
  }
  info[NSLocalizedDescriptionKey] = to_ns_string(described);
  *error = [NSError errorWithDomain:VolumetricRendererErrorDomain
                               code:code
                           userInfo:info];
}

void set_error(NSError** error, const vg::Status& status, const char* stage) {
  const bool vulkan = status.domain() == vg::Status::Code::Vulkan;
  set_error(error, stage, error_code(status.domain()),
            vulkan ? std::optional<VkResult>(status.code()) : std::nullopt,
            status.message());
}

// Carried through rather than flattened into `unsupported`: a device-creation
// failure on a user's phone should name its VkResult, not read as a capability
// the driver lacks.
void set_error(NSError** error, const vr::Status& status, const char* stage) {
  const bool backend = status.domain() == vr::Status::Code::Backend;
  set_error(
      error, stage, error_code(status.domain()),
      backend ? std::optional<VkResult>(static_cast<VkResult>(status.detail()))
              : std::nullopt,
      status.message());
}

std::string api_version_string(std::uint32_t v) {
  return std::to_string(VK_API_VERSION_MAJOR(v)) + "." +
         std::to_string(VK_API_VERSION_MINOR(v)) + "." +
         std::to_string(VK_API_VERSION_PATCH(v));
}

}  // namespace

// Everything C++ lives here so the header stays Objective-C only and Swift
// never sees a move-only type.
//
// --- Teardown ----------------------------------------------------------------
// Two rules, and they pull in opposite directions, which is why the order below
// is written out rather than left to intuition:
//
//   1. A gfx Buffer/Texture's producing Allocator must outlive it, and that
//      allocator belongs to `app`. So the atlas image and the mesh ring, both
//      allocated from it, must be declared *after* `app` to be destroyed
//      *before* it.
//   2. gfx warns that resources destroyed while the frame loop still has frames
//      in flight referencing them is VUID-vkDestroyPipeline-pipeline-00765 and
//      friends. Rule 1 puts them in exactly that position.
//
// gfx resolves this by prescribing an explicit `wait_idle()` after the render
// loop and before anything unwinds -- not by declaration order. So that is what
// ~RendererImpl does: a destructor *body* runs before any member is destroyed,
// which makes the drain structural in the one place that can actually order it.
// An earlier attempt made it structural by declaring `app` last so it tore down
// first; that cannot work once anything is allocated from its allocator, and
// appending seven such members after it had already inverted it.
struct RendererImpl {
  // Declared first, destroyed last: everything below borrows the VkDevice this
  // owns and destroys nothing, so it has to outlive all of them.
  volumetric_kit::ios_app::SharedDevice shared;
  // recon's view of the same VkDevice -- what the volume, the integrator and
  // marching cubes below all allocate and dispatch on. Adopted at bring-up
  // rather than lazily, so a mismatch between what the bootstrap enabled and
  // what recon requires fails where the message is actionable.
  // optional, not a plain member: recon's Device is create-or-adopt only and
  // has no public default constructor -- which is the invariant working, not an
  // inconvenience. There is no such thing as an empty one to default-construct.
  std::optional<volumetric_kit::recon::Device> recon_device;
  std::optional<volumetric_kit::recon::Allocator> recon_allocator;

  // --- Reconstruction ------------------------------------------------------
  app::Fusion fusion;
  std::thread fuse_thread;
  std::atomic<bool> fusing{false};
  // Borrowed, not owned -- but the owning VolumetricCapture is retained by the
  // renderer for exactly as long as this is non-null, and the thread that reads
  // it is joined before that retain is dropped. See -startFusionWithCapture:.
  vr::sensor::ICameraCapture* capture = nullptr;

  // --- Render state (plain values; destruction order does not matter) -------
  std::uint64_t frames_presented = 0;
  std::uint32_t uploaded_version = 0;
  bool have_mesh = false;
  bool draw_mesh = true;
  // Failures of the mesh upload, which is the one stage with no Status to
  // propagate: it happens on the render thread, after fusion has already
  // reported success. Without these a mesh that never reaches the GPU reads as
  // a rising vertex count next to a frozen screen.
  std::uint64_t mesh_upload_failures = 0;
  std::string mesh_upload_error;
  // The pose the newest mesh was fused at; the camera follows the scan until
  // the user's fingers take it over.
  vr::Mat4f camera_to_world{1.0f};
  app::OrbitCamera camera;
  VolumetricViewOrientation view_orientation =
      VolumetricViewOrientationPortrait;

  // --- gfx, and everything built on its device + allocator ------------------
  // `app` comes first here and the resources it backs follow, so reverse
  // destruction frees them before the allocator that produced them. The queue
  // drain that makes that safe is in ~RendererImpl, not in this ordering.
  vg::app::WindowedApp app;
  // CPU-ahead depth, named rather than defaulted: kMeshSlots below is only
  // correct relative to it.
  static constexpr std::uint32_t kFramesInFlight = 2;

  vg::ShaderModule vertex_shader;
  vg::ShaderModule fragment_shader;
  vg::GraphicsPipeline pipeline;
  std::optional<vg::pipelines::HybridMeshPipeline> mesh_pipeline;
  // The atlas binding the hybrid pipeline requires of every frame. Its fragment
  // shader samples set 0 unconditionally, so submit() records *nothing at all*
  // without one -- see the bring-up comment where these are filled in.
  vg::Texture atlas_texture;
  // optional for the same reason recon's Device is: Sampler keeps its default
  // constructor private, so it is create-only and there is no empty one to
  // default-construct. The other three do expose an empty state.
  std::optional<vg::Sampler> atlas_sampler;
  vg::DescriptorPool atlas_pool;
  vg::DescriptorSet atlas_set;
  // A ring, not one slot: replacing a GpuMesh the GPU may still be reading is a
  // use-after-free, and at per-frame meshing that would be every frame. One
  // slot more than the frames in flight means a mesh uploaded now is untouched
  // again by the time the ring comes back round -- the cheap version of the
  // mesh-slot ring the interop decision describes, and no wait_idle per frame.
  //
  // Derived from kFramesInFlight rather than written as a literal, which is the
  // whole correctness argument: a mesh must outlive every frame that can still
  // be reading it. The literal 3 held only because gfx's WindowedAppConfig
  // happens to default frames_in_flight to 2 -- a value in another repo that
  // this file neither set nor read. It sets it now (see -initWithLayer:), so
  // the two cannot drift.
  static constexpr std::size_t kMeshSlots = kFramesInFlight + 1;
  vg::pipelines::GpuMesh mesh_slots[kMeshSlots];
  std::size_t mesh_slot = 0;

  ~RendererImpl() {
    // Before anything else, and before any member is destroyed: the fuse thread
    // polls `capture` and submits recon work on the shared queue, so it has to
    // be stopped while everything it touches is still alive. A joinable
    // std::thread reaching a member destructor would be std::terminate.
    stop_fusing();
    // Then drain. gfx idles only the queues it was assigned and recon's Device
    // exposes no wait at all, so the bootstrap -- which owns both -- is what
    // makes this cover the whole device.
    if (app.valid()) {
      (void)app.wait_idle();
    }
    shared.wait_idle();
  }

  void stop_fusing() {
    fusing.store(false);
    if (fuse_thread.joinable()) {
      fuse_thread.join();
    }
    capture = nullptr;
  }

  RendererImpl() = default;
  RendererImpl(const RendererImpl&) = delete;
  RendererImpl& operator=(const RendererImpl&) = delete;
};

@implementation VolumetricRenderer {
  std::unique_ptr<RendererImpl> _impl;
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
  _impl = std::make_unique<RendererImpl>();

  // --- One VkDevice, adopted by both libraries ------------------------------
  // Not an optimisation: a VkBuffer is valid only on the VkDevice that created
  // it, so the zero-copy mesh handoff needs *one* device. Two devices on this
  // same GPU would still cost a round trip through host memory.
  const vr::Status built = _impl->shared.build((__bridge const void*)layer,
                                               "volumetric_kit_ios scanner");
  if (!built) {
    set_error(error, built, "SharedDevice");
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
  config.frames_in_flight = RendererImpl::kFramesInFlight;

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
    set_error(error, app.status(), "WindowedApp::adopt");
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
    set_error(error, recon_device.status(), "recon Device::adopt");
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
    set_error(error, recon_allocator.status(), "recon Allocator::create");
    return nil;
  }
  _impl->recon_allocator.emplace(std::move(recon_allocator).value());

  VkDevice device = _impl->app.device().handle();
  vg::Result<vg::ShaderModule> vert = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_vert_spv),
      vi_triangle_vert_spv_size);
  if (!vert) {
    set_error(error, vert.status(), "vertex ShaderModule::create");
    return nil;
  }
  _impl->vertex_shader = std::move(vert).value();

  vg::Result<vg::ShaderModule> frag = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_frag_spv),
      vi_triangle_frag_spv_size);
  if (!frag) {
    set_error(error, frag.status(), "fragment ShaderModule::create");
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
    set_error(error, pipeline.status(), "GraphicsPipeline::create");
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
    set_error(error, mesh_pipeline.status(), "HybridMeshPipeline::create");
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
    set_error(error, atlas_texture.status(), "atlas upload_texture");
    return nil;
  }
  _impl->atlas_texture = std::move(atlas_texture).value();

  vg::Result<vg::Sampler> atlas_sampler = vg::Sampler::create(device);
  if (!atlas_sampler) {
    set_error(error, atlas_sampler.status(), "atlas Sampler::create");
    return nil;
  }
  _impl->atlas_sampler.emplace(std::move(atlas_sampler).value());

  const VkDescriptorPoolSize atlas_pool_size{
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
  vg::Result<vg::DescriptorPool> atlas_pool =
      vg::DescriptorPool::create(device, &atlas_pool_size, 1, 1);
  if (!atlas_pool) {
    set_error(error, atlas_pool.status(), "atlas DescriptorPool::create");
    return nil;
  }
  _impl->atlas_pool = std::move(atlas_pool).value();

  vg::Result<vg::DescriptorSet> atlas_set = _impl->atlas_pool.allocate(
      _impl->mesh_pipeline->descriptor_set_layout(0));
  if (!atlas_set) {
    set_error(error, atlas_set.status(), "atlas DescriptorPool::allocate");
    return nil;
  }
  _impl->atlas_set = std::move(atlas_set).value();
  _impl->atlas_set.write_combined_image_sampler(
      0, _impl->atlas_texture.view(), _impl->atlas_sampler->handle(),
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  const vr::Status fusion_started = _impl->fusion.start(
      *_impl->recon_device, *_impl->recon_allocator, app::FusionConfig{});
  if (!fusion_started) {
    // Likewise: Fusion::start commits the volume, so its usual failure is an
    // OutOfMemory that must reach Swift as one.
    set_error(error, fusion_started, "Fusion::start");
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
    set_error(error, frame.status(), "begin_frame");
    return NO;
  }
  if (!frame.value()) {
    // No drawable this tick (the view is off-screen or mid-rebuild). The
    // protocol working as designed, not a failure.
    return YES;
  }
  const vg::windowing::Frame& f = *frame.value();

  // Take the newest mesh, if fusion published one since the last upload. Never
  // wait for it: the render loop draws the previous mesh rather than stalling,
  // which is what keeps presentation smooth while a remesh is in flight.
  //
  // Done *before* the render scope opens. The upload below is gfx's blocking
  // one-shot overload -- begin, record, submit, vkWaitForFences(UINT64_MAX) --
  // and sandwiching a synchronous queue round trip between target->begin and
  // target->end held a command buffer open across it for no reason. This does
  // not make the upload non-blocking; see the note on -startFusionWithCapture:.
  if (std::optional<app::Fusion::Published> fresh =
          _impl->fusion.take_mesh(_impl->uploaded_version)) {
    vg::assets::Mesh gfx_mesh;
    // A bulk copy, not a field-by-field rebuild: recon's mesh::Vertex *is*
    // gfx::assets::Vertex since the 2026-08-02 layout decision. Size alone does
    // not say so -- a reorder within the 64 bytes would pass it and misread
    // every vertex -- so the field offsets are pinned too, matching the
    // assertions recon's own to_gfx_mesh carries.
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
    static_assert(std::is_trivially_copyable<RVertex>::value &&
                      std::is_trivially_copyable<GVertex>::value,
                  "vertex bulk copy requires trivially copyable layouts");
    gfx_mesh.vertices.resize(fresh->mesh.vertices.size());
    std::memcpy(gfx_mesh.vertices.data(), fresh->mesh.vertices.data(),
                fresh->mesh.vertices.size() * sizeof(GVertex));
    // Moved: the indices are the one half of the mesh that needs no conversion,
    // and at room scale the copy this replaces was ~3 MB per frame.
    gfx_mesh.indices = std::move(fresh->mesh.indices);

    vg::Result<vg::pipelines::GpuMesh> uploaded = vg::pipelines::upload_mesh(
        _impl->app.device(), _impl->app.allocator(), gfx_mesh);
    if (uploaded) {
      _impl->mesh_slot = (_impl->mesh_slot + 1) % RendererImpl::kMeshSlots;
      _impl->mesh_slots[_impl->mesh_slot] = std::move(uploaded).value();
      _impl->uploaded_version = fresh->version;
      _impl->have_mesh = true;
      _impl->mesh_upload_error.clear();
    } else {
      // Recorded, not dropped. This is the one stage between "fusion says it
      // produced geometry" and "the geometry is on screen", so a silent failure
      // here shows a rising vertex count next to a frozen mesh -- or next to
      // the bring-up triangle -- with nothing anywhere naming the upload.
      ++_impl->mesh_upload_failures;
      _impl->mesh_upload_error = uploaded.status().message();
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
  // *sensor*: "the x-axis always points along the long axis of the device, from
  // the front-facing camera toward the Home button", +Y along the short axis,
  // +Z out of the screen (ARCamera.transform). That frame does not turn when
  // the interface does, so in portrait the camera's +X is viewport-*down* and
  // its +Y is viewport-right -- and the scan renders on its side, which reads
  // as a broken reconstruction rather than a misaligned render camera.
  //
  // Fusion is unaffected either way (the pose and the intrinsics are mutually
  // consistent in the sensor frame), so the correction belongs here and nowhere
  // else. A rotation about the camera's own +Z carries its basis onto the
  // viewport's: portrait needs +90 degrees, and each further quarter turn of
  // the interface adds another -- which is what VolumetricViewOrientation's
  // values count. Landscape-left is the sensor's own basis and needs none.
  if (const float quarter_turns = static_cast<float>(_impl->view_orientation)) {
    constexpr float kHalfPi = 1.5707964f;
    device_pose = glm::rotate(device_pose, quarter_turns * kHalfPi,
                              glm::vec3(0.0f, 0.0f, 1.0f));
  }
  _impl->camera.set_device_pose(device_pose);

  const bool draw_mesh = _impl->draw_mesh && _impl->have_mesh;
  if (draw_mesh) {
    vg::pipelines::HybridMeshDraw draw{};
    draw.geometry = &_impl->mesh_slots[_impl->mesh_slot];

    const float aspect = static_cast<float>(extent.width) /
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
    frame_info.atlas = _impl->atlas_set.handle();
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
      set_error(error, end, "end_frame");
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
  RendererImpl* impl = _impl.get();
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

- (VolumetricViewOrientation)viewOrientation {
  return _impl->view_orientation;
}

- (void)setViewOrientation:(VolumetricViewOrientation)viewOrientation {
  _impl->view_orientation = viewOrientation;
}

- (BOOL)followingDevice {
  return _impl->camera.following();
}

- (float)cameraDistance {
  return _impl->camera.distance();
}

- (NSString*)fusionSummary {
  const app::FusionStats s = _impl->fusion.stats();
  // The upload is the render thread's stage, so it is not in FusionStats -- but
  // it sits between "fusion produced geometry" and "geometry is on screen", so
  // it belongs on the same read-out.
  std::string upload;
  if (_impl->mesh_upload_failures > 0) {
    upload = "\n  ! upload x" + std::to_string(_impl->mesh_upload_failures) +
             ": " + _impl->mesh_upload_error;
  }
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "fused %llu / remesh %llu  v%u\n"
                "  mesh      %u verts / %u tris\n"
                "  allocate  %.1f ms\n"
                "  integrate %.1f ms\n"
                "  extract   %.1f ms\n"
                "  texture   %.1f ms%s%s%s",
                static_cast<unsigned long long>(s.frames_fused),
                static_cast<unsigned long long>(s.remeshes), s.mesh_version,
                s.vertices, s.triangles, s.allocate_ms, s.integrate_ms,
                s.extract_ms, s.texture_ms,
                s.last_error.empty() ? "" : "\n  ! ", s.last_error.c_str(),
                upload.c_str());
  // Through the nil-guarding helper, like every other string property here: the
  // buffer carries a library message, and `fusionSummary` is imported as a
  // non-optional Swift String that traps on the nil `stringWithUTF8String:`
  // returns for invalid UTF-8.
  return to_ns_string(buf);
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
  return to_ns_string(props.deviceName);
}

- (NSString*)apiVersion {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(_impl->app.device().physical_device(), &props);
  return to_ns_string(api_version_string(props.apiVersion));
}

- (NSString*)sharedDeviceSummary {
  return to_ns_string(_impl->shared.summary());
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

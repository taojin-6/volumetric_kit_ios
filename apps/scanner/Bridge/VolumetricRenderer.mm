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
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <glm/gtc/matrix_transform.hpp>
#include "triangle_frag.spv.hpp"
#include "triangle_vert.spv.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"

#include "volumetric_kit/gfx/assets/mesh.hpp"
#include "volumetric_kit/gfx/core/graphics_pipeline.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/shader.hpp"
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
struct RendererImpl {
  // Declared first, destroyed last: everything below borrows the VkDevice this
  // owns and destroys nothing, so it has to outlive all of them.
  volumetric_kit::ios_app::SharedDevice shared;
  // recon's view of the same VkDevice. Unused until fusion lands, but adopted
  // here because *proving both libraries share one device* is what this slice
  // is for -- and because a failure to adopt must surface at bring-up, not
  // later.
  // optional, not a plain member: recon's Device is create-or-adopt only and
  // has no public default constructor -- which is the invariant working, not an
  // inconvenience. There is no such thing as an empty one to default-construct.
  std::optional<volumetric_kit::recon::Device> recon_device;
  std::optional<volumetric_kit::recon::Allocator> recon_allocator;
  vg::ShaderModule vertex_shader;
  vg::ShaderModule fragment_shader;
  vg::GraphicsPipeline pipeline;
  // Declared LAST, so reverse member destruction tears it down FIRST. gfx warns
  // that resources created after the app destruct before it while its frame
  // loop may still have frames in flight referencing them -- destroying a
  // VkPipeline a submitted frame still uses is
  // VUID-vkDestroyPipeline-pipeline-00765. The app's own teardown drains the
  // loop, so putting it here orders that drain ahead of the objects it protects
  // rather than after them. -dealloc waits as well; this makes the ordering
  // structural instead of remembered.
  vg::app::WindowedApp app;
  std::uint64_t frames_presented = 0;

  // --- Reconstruction ------------------------------------------------------
  app::Fusion fusion;
  std::thread fuse_thread;
  std::atomic<bool> fusing{false};
  vr::sensor::ICameraCapture* capture = nullptr;

  std::optional<vg::pipelines::HybridMeshPipeline> mesh_pipeline;
  // A ring, not one slot: replacing a GpuMesh the GPU may still be reading is a
  // use-after-free, and at per-frame meshing that would be every frame. With
  // two frames in flight, a mesh uploaded now is untouched again by the time
  // the ring comes back round -- which is the cheap version of the mesh-slot
  // ring the interop decision describes, and avoids a wait_idle per frame.
  static constexpr std::size_t kMeshSlots = 3;
  vg::pipelines::GpuMesh mesh_slots[kMeshSlots];
  std::size_t mesh_slot = 0;
  std::uint32_t uploaded_version = 0;
  bool have_mesh = false;
  bool draw_mesh = true;
  // The pose the newest mesh was fused at; the camera follows the scan until
  // the user's fingers take it over.
  vr::Mat4f camera_to_world{1.0f};
  app::OrbitCamera camera;
};

@implementation VolumetricRenderer {
  std::unique_ptr<RendererImpl> _impl;
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
    set_error(error,
              vg::Status::unsupported(recon_allocator.status().message()),
              "recon Allocator::create");
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

  const vr::Status fusion_started = _impl->fusion.start(
      *_impl->recon_device, *_impl->recon_allocator, app::FusionConfig{});
  if (!fusion_started) {
    set_error(error, vg::Status::unsupported(fusion_started.message()),
              "Fusion::start");
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

  vg::RenderTargetBeginInfo begin{};
  begin.load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  begin.clear_color = {{0.05f, 0.06f, 0.09f, 1.0f}};
  f.target->begin(f.cmd, begin);

  // Take the newest mesh, if fusion published one since the last upload. Never
  // wait for it: the render loop draws the previous mesh rather than stalling,
  // which is what keeps presentation smooth while a remesh is in flight.
  if (std::optional<std::pair<vr::mesh::Mesh, std::uint32_t>> fresh =
          _impl->fusion.take_mesh(_impl->uploaded_version)) {
    vg::assets::Mesh gfx_mesh;
    // A bulk copy, not a field-by-field rebuild: recon's mesh::Vertex *is*
    // gfx::assets::Vertex since the 2026-08-02 layout decision.
    static_assert(sizeof(vr::mesh::Vertex) == sizeof(vg::assets::Vertex),
                  "recon and gfx vertex layouts have diverged");
    gfx_mesh.vertices.resize(fresh->first.vertices.size());
    std::memcpy(gfx_mesh.vertices.data(), fresh->first.vertices.data(),
                fresh->first.vertices.size() * sizeof(vg::assets::Vertex));
    gfx_mesh.indices = fresh->first.indices;

    vg::Result<vg::pipelines::GpuMesh> uploaded = vg::pipelines::upload_mesh(
        _impl->app.device(), _impl->app.allocator(), gfx_mesh);
    if (uploaded) {
      _impl->mesh_slot = (_impl->mesh_slot + 1) % RendererImpl::kMeshSlots;
      _impl->mesh_slots[_impl->mesh_slot] = std::move(uploaded).value();
      _impl->uploaded_version = fresh->second;
      _impl->have_mesh = true;
    }
  }

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
  _impl->camera.set_device_pose(
      vr::sensor::cv_from_gl_camera(_impl->camera_to_world));

  const bool draw_mesh = _impl->draw_mesh && _impl->have_mesh;
  if (draw_mesh) {
    vg::pipelines::HybridMeshDraw draw{};
    draw.mesh = &_impl->mesh_slots[_impl->mesh_slot];

    const float aspect = static_cast<float>(extent.width) /
                         static_cast<float>(std::max(extent.height, 1u));
    // The same FOV the camera scales a pan by -- see kVerticalFov. A pan
    // computed against a different one slides out from under the finger.
    glm::mat4 proj = glm::perspective(app::kVerticalFov, aspect, 0.05f, 20.0f);
    // Vulkan's clip space has +Y down where GL's has it up, and glm targets GL.
    proj[1][1] *= -1.0f;

    vg::pipelines::HybridMeshFrame frame_info{};
    frame_info.extent = extent;
    // Either the device pose or the turntable, depending on whether the user
    // has taken the camera over.
    frame_info.view_proj = proj * _impl->camera.view();
    frame_info.draws = &draw;
    frame_info.draw_count = 1;
    (void)_impl->mesh_pipeline->submit(f.cmd, frame_info);
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
  // gfx's prescribed teardown: idle before anything created after the app is
  // destroyed. The app's own destructor drains too, but only once destruction
  // has already begun -- this puts the wait ahead of every member.
  [self waitIdle];
}

- (void)startFusionWithCapture:(VolumetricCapture*)capture {
  if (_impl->fusing.load()) {
    return;
  }
  _impl->capture =
      static_cast<vr::sensor::ICameraCapture*>([capture captureHandle]);
  _impl->fusing.store(true);
  _impl->fuse_thread = std::thread([self] {
    while (self->_impl->fusing.load()) {
      vr::Result<std::optional<vr::sensor::CapturedFrame>> got =
          self->_impl->capture->poll();
      if (!got || !got.value()) {
        // Nothing new: sleep briefly rather than spin. ARKit delivers at 60 Hz
        // and this loop is otherwise free to burn a core doing nothing.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      self->_impl->fusion.fuse(*got.value());
    }
  });
}

- (void)stopFusion {
  _impl->fusing.store(false);
  if (_impl->fuse_thread.joinable()) {
    _impl->fuse_thread.join();
  }
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

- (BOOL)followingDevice {
  return _impl->camera.following();
}

- (float)cameraDistance {
  return _impl->camera.distance();
}

- (NSString*)fusionSummary {
  const app::FusionStats s = _impl->fusion.stats();
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "fused %llu / remesh %llu  v%u\n"
                "  mesh      %u verts / %u tris\n"
                "  allocate  %.1f ms\n"
                "  integrate %.1f ms\n"
                "  extract   %.1f ms\n"
                "  texture   %.1f ms%s%s",
                static_cast<unsigned long long>(s.frames_fused),
                static_cast<unsigned long long>(s.remeshes), s.mesh_version,
                s.vertices, s.triangles, s.allocate_ms, s.integrate_ms,
                s.extract_ms, s.texture_ms,
                s.last_error.empty() ? "" : "\n  ! ", s.last_error.c_str());
  return [NSString stringWithUTF8String:buf];
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

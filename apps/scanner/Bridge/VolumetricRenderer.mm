// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Objective-C++ seam. This is the one translation unit where a
// CAMetalLayer* and a vg::app::WindowedApp are both first-class, which is the
// entire reason the bridge is .mm rather than Swift.

#import "VolumetricRenderer.h"

#import "Fusion.hpp"
#import "OrbitCamera.hpp"
#import "SharedDevice.hpp"

#include <os/log.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
// FrameTrace::dump uses std::fprintf / std::snprintf / std::fflush, and
// fusionSummary uses std::snprintf. It compiled only because some gfx or recon
// header happens to pull <cstdio> in transitively today.
#include <cstdio>
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

// --- Frame trace -------------------------------------------------------------
// A device loss is reported by the *next* vkWaitForFences, so by the time the
// error surfaces the frame that faulted is already gone and nothing on the
// stack says what it did. This keeps the last few frames' worth of the state
// that could plausibly cause a GPU fault and dumps it when the loss is
// detected.
//
// A ring rather than per-frame logging: at 60 Hz an os_log per frame is both
// noise and a perturbation, and only the frames immediately before the fault
// matter. Written from the render thread and read from the render thread, so no
// locking.
struct FrameTrace {
  struct Entry {
    std::uint64_t frame = 0;
    std::uint64_t generation = 0;  // recon generation this frame drew
    std::size_t mesh_slot = 0;
    std::uint64_t released_through = 0;  // what we told recon it may reuse
    std::uint32_t triangles = 0;
    std::uint32_t triangle_capacity = 0;
    std::uint64_t arena_bytes = 0;  // grew? compare against the previous entry
    std::uint32_t active_blocks = 0;
    float extract_ms = 0.0f;
    bool drew_mesh = false;
  };

  static constexpr std::size_t kCapacity = 24;
  Entry entries[kCapacity];
  std::uint64_t next = 0;

  Entry& begin_frame_entry() {
    Entry& e = entries[next % kCapacity];
    e = Entry{};
    e.frame = next;
    ++next;
    return e;
  }

  // Oldest-first, so the last line is the frame closest to the fault.
  //
  // Both channels on purpose: os_log is what survives a run with no debugger
  // attached (readable afterwards via `log collect`), and stderr is what
  // reaches `devicectl process launch --console` live. os_log alone goes
  // nowhere near the console, which is the mistake worth not repeating.
  void dump(const char* why) const {
    const std::uint64_t count = std::min<std::uint64_t>(next, kCapacity);
    os_log_error(OS_LOG_DEFAULT,
                 "vk-trace: %{public}s -- last %llu frames:", why,
                 static_cast<unsigned long long>(count));
    std::fprintf(stderr, "vk-trace: %s -- last %llu frames:\n", why,
                 static_cast<unsigned long long>(count));
    for (std::uint64_t i = 0; i < count; ++i) {
      const Entry& e = entries[(next - count + i) % kCapacity];
      char line[256];
      std::snprintf(
          line, sizeof(line),
          "f=%llu drew=%d gen=%llu slot=%zu released<=%llu tris=%u/%u "
          "arena=%llu blocks=%u extract=%.1fms",
          static_cast<unsigned long long>(e.frame), e.drew_mesh ? 1 : 0,
          static_cast<unsigned long long>(e.generation), e.mesh_slot,
          static_cast<unsigned long long>(e.released_through), e.triangles,
          e.triangle_capacity, static_cast<unsigned long long>(e.arena_bytes),
          e.active_blocks, static_cast<double>(e.extract_ms));
      os_log_error(OS_LOG_DEFAULT, "vk-trace: %{public}s", line);
      std::fprintf(stderr, "vk-trace: %s\n", line);
    }
    std::fflush(stderr);
  }
};

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
  // +1, which is both what recon's `slot_count` prescribes ("the consumer's
  // frames in flight plus one") and what the ring now needs. The extra slot was
  // buying headroom around two ordering faults on the producing side, since
  // fixed in Fusion::remesh: the consumer's release was applied *after* the
  // extract it exists to make room for, so the ring always ran a slot shallower
  // than its depth, and a mesh nobody had collected was published over rather
  // than left alone, which put a third generation outstanding at once.
  //
  // With both gone the outstanding set is exactly the generations named in
  // frame_generations plus the one being extracted. Each slot is a full vertex
  // arena that never shrinks, so the padding was not free -- against the arena
  // sizes this app reaches, it was a few hundred megabytes resident to cover an
  // ordering choice.
  static constexpr std::size_t kMeshSlots = kFramesInFlight + 1;
  // The meshes in flight -- borrowed views of recon's buffers, not storage.
  // Copying one is copying a few handles.
  vr::mesh::DeviceMesh mesh_slots[kMeshSlots];
  // The generation each in-flight frame drew. Read as a *set*: what may be
  // released is everything older than the oldest entry, not the entry belonging
  // to the frame that just retired -- one generation is normally drawn by
  // several consecutive frames, so the retired frame's generation is often
  // still being read by a newer one. begin_frame's fence wait is the only
  // completion signal gfx gives (no fence is exposed, and no semaphore may
  // cross the seam), and it says a frame finished, not that a generation did.
  std::uint64_t frame_generations[kFramesInFlight] = {};
  std::size_t frame_slot = 0;
  std::size_t mesh_slot = 0;
  // The newest generation take_mesh has handed over, drawn or not. What the
  // release logic falls back to when *no* frame is holding a generation: the
  // per-frame minimum says nothing then, and without this the generations taken
  // while drawing was off were never released at all -- `drawMesh = NO`
  // exhausted recon's ring within kMeshSlots extracts and turning drawing back
  // on could not recover, because the renderer only released what it drew and
  // could no longer obtain anything to draw.
  std::uint64_t newest_taken_generation = 0;
  // Latched when a published mesh cannot be bound as geometry. That is a
  // configuration fault, not a transient: the usage bits come from two
  // constants in Fusion::start and the sharing mode from the queue plan, so a
  // mesh that is unusable once is unusable every time. Latching stops the
  // renderer collecting meshes it cannot draw -- which at 60 Hz was a storm of
  // failure counts, and which would otherwise walk recon's ring to exhaustion
  // one uncollectable generation at a time.
  bool mesh_unusable = false;
  // Diagnostic only: what the last few frames drew, dumped when a device loss
  // (or any begin_frame failure) is detected. See FrameTrace.
  FrameTrace trace;

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

  app::FusionConfig fusion_config;
  // One slot per frame in flight, plus one. The renderer draws the extractor's
  // buffers in place now, so an extract must never land on geometry a pending
  // frame is still reading -- and the ring is what makes that impossible rather
  // than merely unlikely.
  fusion_config.mesh_slots = RendererImpl::kMeshSlots;
  // Both families, always. Under the two-families plan a phone actually gets,
  // recon writes these buffers on one and gfx reads them on the other, and an
  // EXCLUSIVE buffer read by a family that does not own it is undefined with no
  // error. recon collapses the pair to EXCLUSIVE wherever they are the same
  // family, so this needs no branch on the plan.
  fusion_config.queue_families[0] = _impl->shared.compute_family();
  fusion_config.queue_families[1] = _impl->shared.graphics_family();
  fusion_config.queue_family_count = 2;

  const vr::Status fusion_started = _impl->fusion.start(
      *_impl->recon_device, *_impl->recon_allocator, fusion_config);
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
    // The fault happened in an *earlier* frame; this is only where it is
    // noticed. Dump what those frames were doing before the error propagates.
    _impl->trace.dump(frame.status().message().c_str());
    set_error(error, frame.status(), "begin_frame");
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
  FrameTrace::Entry& trace = _impl->trace.begin_frame_entry();

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
      _impl->mesh_slot = (_impl->mesh_slot + 1) % RendererImpl::kMeshSlots;
      _impl->mesh_slots[_impl->mesh_slot] = m;
      _impl->have_mesh = true;
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
      _impl->frame_slot % RendererImpl::kFramesInFlight;
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

    // The narrow accessor, not stats(): this runs every frame, and FusionStats
    // carries a std::string whose copy would malloc inside the mutex the fuse
    // thread takes on every one of its own frames. Five scalars is all the ring
    // holds. See Fusion::trace_stats.
    const app::FusionTraceStats s = _impl->fusion.trace_stats();
    trace.drew_mesh = true;
    trace.generation = live_src.generation;
    trace.mesh_slot = _impl->mesh_slot;
    trace.triangles = s.triangles;
    trace.triangle_capacity = s.triangle_capacity;
    trace.arena_bytes = s.arena_bytes;
    trace.active_blocks = s.active_blocks;
    trace.extract_ms = s.extract_ms;

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
  // Shown as a count with the reason, not the reason alone. `last_error` is
  // most-recent-wins and `fuse` republishes its own every fused frame, so a
  // repeating extract or texture failure surfaced for under 16 ms at a time and
  // the scan read as clean with a mesh that had simply stopped updating. The
  // count is what holds still long enough to be read.
  std::string errors;
  if (s.errors > 0) {
    errors = "\n  ! errors x" + std::to_string(s.errors) +
             (s.last_error.empty() ? "" : ": " + s.last_error);
  }
  // The phase rows, built here as label/value pairs rather than as eight more
  // positional varargs on the format below. That format already couples ~19
  // conversions to their arguments by position, and a run of same-typed floats
  // is the one shift -Wformat cannot see: it prints every later phase under the
  // wrong label, silently, with nothing in the output to reveal it. Binding
  // each label to its value at one site removes the class of mistake, and
  // recon's own viewer builds these same seven the same way.
  //
  // Names picked not to collide with quantities already on this read-out:
  // `inputs` rather than "upload" (there is an `! upload` banner), `sizing`
  // rather than "arena" (there is an arena section below), `meshing` rather
  // than "dispatch" (that word is a *count* on the extract line). The unit is
  // declared once in the row label instead of seven times in the values.
  //
  // `other` is the residual, printed rather than left implicit: these phases do
  // not sum to extract_ms and never did. recon's spans open after the slot
  // claim and close before the O(active_blocks) teardown of the neighbour
  // table, so the gap grows with the scan -- the one direction in which an
  // unlabelled remainder would be misread as rounding.
  struct PhaseCell {
    const char* label;
    double ms;
  };
  const PhaseCell phases[] = {
      {"compact", s.extract.compact_ms},
      {"inputs", s.extract.input_upload_ms},
      {"sizing", s.extract.arena_alloc_ms},
      {"desc", s.extract.descriptor_ms},
      {"meshing", s.extract.dispatch_ms},
      {"read", s.extract.readback_ms},
      {"other",
       std::max(0.0, static_cast<double>(s.extract_ms) - s.extract.total_ms())},
  };
  std::string phase_rows;
  for (std::size_t i = 0; i < sizeof(phases) / sizeof(phases[0]); ++i) {
    // Fixed widths, because this string is rebuilt continuously: without them a
    // value gaining a digit shifts every label to its right, and columns that
    // move cannot be read. Two decimals because three of these sat under
    // 0.05 ms on the measured device and at one decimal were indistinguishable
    // from "not measured" -- including `sizing`, the phase that prices a refit.
    char cell[32];
    std::snprintf(cell, sizeof(cell), "%-8s%6.2f", phases[i].label,
                  phases[i].ms);
    // The first cell continues the row label; every fourth after it opens a new
    // row in the same 12-column gutter every other line on this read-out uses.
    if (i == 0) {
      phase_rows += ' ';
    } else if (i % 4 == 0) {
      phase_rows += "\n            ";
    } else {
      phase_rows += "  ";
    }
    phase_rows += cell;
  }

  // `pass` rather than `dispatch`: the number is how many times the surface was
  // meshed, and `dispatch` is a duration below. Read it as cost, not as a
  // verdict on the capacity planner -- the refit triggers against the slot's
  // *retained* grow-only arena, not against the plan, so a plan that
  // undershoots badly still reports one pass whenever an earlier peak left the
  // arena large enough to absorb it.
  std::string extract_note =
      "  (" + std::to_string(s.extract.dispatches) + " pass)";
  if (s.extract_stale) {
    // Everything in `s.extract` comes from the last *successful* remesh, so say
    // so when that is no longer this frame. Without it a breakdown frozen by a
    // failing extract reads as current.
    extract_note += "  [stale " + std::to_string(s.frames_since_extract) + "f]";
  }

  // The dirty-block survey, built here and placed *beside* `table` in the body
  // below rather than appended after the whole thing.
  //
  // Beside `table` because it is a fraction of that block count and means very
  // little anywhere else. Not appended, for two reasons the appended version
  // demonstrated: the body deliberately ends without a trailing newline, so a
  // row added after it rendered glued onto the texture line; and the tail is
  // what a clipped overlay loses first -- the banners were moved to the top for
  // exactly that reason, and the statusLabel is bottom-constrained on a phone
  // -- which put this PR's headline measurement in the first place to go.
  std::string dirty_rows;
  if (s.survey_active_blocks > 0) {
    // Named markers, because each makes the sample mean something other than
    // what it looks like: the first window reads ~100% on any scene (the map
    // grew from empty inside it), and a stale sample is an old one that the
    // gate above -- a one-way latch on `survey_active_blocks > 0` -- cannot
    // take back off the screen.
    std::string note;
    if (s.survey_first_window) {
      note += "  [first window: grew from empty]";
    }
    if (s.survey_stale) {
      note += "  [stale " + std::to_string(s.frames_since_survey) + "f]";
    }
    char cell[320];
    if (s.survey_changed_blocks == 0) {
      // The steady state, not a degenerate case: recon documents a scan
      // revisiting converged surface at `max_weight` as marking nothing. This
      // is the branch that used to print "0.0%, 0.0x saved" -- the best reading
      // available, reported as no benefit at all, on the column a reader scans.
      // There is no ratio to print here, so the sentence is the result.
      std::snprintf(cell, sizeof(cell),
                    "\n  dirty     nothing changed in %llu fused frames"
                    "  %.1f ms"
                    "\n            (%u blocks active at the survey)%s",
                    static_cast<unsigned long long>(s.survey_window_frames),
                    s.survey_ms, s.survey_active_blocks, note.c_str());
    } else {
      // Dilation (remesh / changed), not a "saved" factor. The share of the map
      // is printed beside it and a speedup would be exactly `100 / share`, so
      // the pair carried one number twice; and recon's own read-out refuses to
      // call it a speedup at all, because only the marching-cubes dispatch
      // scales with the block count while compact walks every table slot, the
      // arena is sized by the whole surface, and readback copies all of it.
      // Dilation is the one ratio here that is not a restatement: it is the
      // cost of the marching-cubes stencil, 1.3-1.4x on room0.
      //
      // The window is printed because the sample is a union across it, not one
      // frame's work -- see FusionStats::survey_window_frames.
      std::snprintf(
          cell, sizeof(cell),
          // The cost trails the row like every other stage on this read-out.
          // It is here at all because the survey was the only stage in `fuse`
          // with no timer around it, while asserting in a comment that it was
          // invisible in the frame budget -- on a device where `extract` alone
          // measures 132.7 ms.
          "\n  dirty     %u changed -> %u to remesh  (%.2fx dilation)  %.1f ms"
          "\n            %.1f%% of %u blocks active at the survey, "
          "%llu-frame window%s",
          s.survey_changed_blocks, s.survey_remesh_blocks,
          static_cast<double>(s.survey_remesh_blocks) /
              static_cast<double>(s.survey_changed_blocks),
          s.survey_ms,
          100.0 * static_cast<double>(s.survey_remesh_blocks) /
              static_cast<double>(s.survey_active_blocks),
          s.survey_active_blocks,
          static_cast<unsigned long long>(s.survey_window_frames),
          note.c_str());
    }
    dirty_rows = cell;
  }

  // Sized for two full library messages plus the fixed body: the error and the
  // upload lines can both be present and both carry a `Status::message()`.
  //
  // Truncation, if it ever happened, could only cut the *tail*, and the tail is
  // now the phase rows and the fixed body rather than the banners -- those
  // moved to the top precisely so a clipped read-out keeps its failures.
  // snprintf's return is discarded, so a cut would be silent either way; the
  // buffer is sized generously rather than checked because it is stack memory
  // and the realistic worst case measures well under half of it.
  char buf[2048];
  std::snprintf(
      buf, sizeof(buf),
      // Banners directly under the header, not at the end. They were last,
      // which put the only two lines naming an actual failure at the bottom of
      // an overlay that already runs past the safe area on a landscape iPhone
      // -- so they were the first things clipped. Nothing below them is worth
      // more screen than they are.
      "fused %llu / remesh %llu  v%u%s%s\n"
      "  mesh      %u verts / %u tris\n"
      "  allocate  %.1f ms\n"
      "  integrate %.1f ms\n"
      "  extract   %.1f ms%s\n"
      // recon's phase split, listed rather than grouped into host/GPU totals:
      // several are genuinely both (`compact` is a dispatch plus its readback
      // stall, and `meshing` covers host record *and* device execution per
      // ExtractTimings' own doc), so a two-bucket summary here would be a guess
      // presented as a measurement. They also have unrelated fixes: the lut is
      // serial host work over the whole active set, while meshing is the pass
      // itself. So the point is to see which one is large. Built above, because
      // eight more same-typed varargs in this list is a silent mislabel waiting
      // to happen.
      "  phases/ms%s\n"
      // The two arena numbers are on different scales, so each says which:
      // triangle_capacity is what the last extract planned for the one slot it
      // wrote, while arena_bytes is recon's sum across the whole ring. Printed
      // as one quantity they read as a single arena and overstated it by the
      // slot count.
      "  arena     %u tris planned for this slot (%.2f%% full)\n"
      "            %.1f MB across %u slots / %u blocks\n"
      "  table     %u / %u blocks (%.1f%% occupied)%s\n"
      "  texture   %.1f ms",
      static_cast<unsigned long long>(s.frames_fused),
      static_cast<unsigned long long>(s.remeshes), s.mesh_version,
      errors.c_str(), upload.c_str(), s.vertices, s.triangles, s.allocate_ms,
      s.integrate_ms, s.extract_ms, extract_note.c_str(), phase_rows.c_str(),
      s.extract.triangle_capacity,
      s.extract.triangle_capacity > 0
          ? 100.0 * static_cast<double>(s.triangles) /
                static_cast<double>(s.extract.triangle_capacity)
          : 0.0,
      static_cast<double>(s.extract.arena_bytes) / (1024.0 * 1024.0),
      s.mesh_slots, s.extract.active_blocks, s.extract.active_blocks,
      s.table_capacity,
      s.table_capacity > 0
          ? 100.0 * static_cast<double>(s.extract.active_blocks) /
                static_cast<double>(s.table_capacity)
          : 0.0,
      dirty_rows.c_str(), s.texture_ms);
  // Mirror the read-out to os_log, throttled.
  //
  // os_log and nothing else. stderr would be a third copy of a string that
  // already reaches a console twice over: ScannerViewController interpolates
  // this very property into its status text and `print`s it to stdout every
  // 0.5 s, and `devicectl device process launch --console` connects both
  // standard streams. What os_log adds is the part neither stream has -- it
  // survives a run with no console and no debugger attached, readable
  // afterwards via `log collect`, which is what a scan whose numbers settle a
  // question needs. FrameTrace::dump writes both because a crash dump has no
  // Swift tick behind it that has already printed; a healthy read-out does.
  //
  // Throttled by wall clock rather than by call: this is a property the Swift
  // view polls at its own refresh rate, which is not a cadence this file
  // controls. Two seconds is slow enough to stay readable in a console and fast
  // enough to show an arena growing.
  {
    static std::chrono::steady_clock::time_point last_logged{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last_logged >= std::chrono::seconds(2)) {
      last_logged = now;
      // One os_log per line, not one call for the whole buffer. os/log.h caps
      // dynamic content -- `%s` and `%@` -- at 1024 bytes per logged line and
      // truncates the rest before it is written to disk, and `buf` is
      // deliberately twice that, sized for two full library messages.
      // FrameTrace::dump splits for this reason; handing the whole buffer over
      // while citing that as precedent would silently drop the tail, and the
      // tail is where this read-out's numbers now live.
      //
      // Split in place and put back, so the string this function returns is
      // unchanged: `buf` is a local, and each newline is restored before the
      // next line is read.
      char* line = buf;
      while (*line != '\0') {
        char* end = std::strchr(line, '\n');
        if (end != nullptr) {
          *end = '\0';
        }
        os_log(OS_LOG_DEFAULT, "vk-scan: %{public}s", line);
        if (end == nullptr) {
          break;
        }
        *end = '\n';
        line = end + 1;
      }
    }
  }
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

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file RendererImpl.hpp
/// @brief The renderer's C++ state, in a header so the units that operate on it
///        can live in their own files.
///
/// Internal to the bridge: nothing here crosses into Swift, and
/// VolumetricRenderer.h stays Objective-C only. It is a header rather than a
/// detail of VolumetricRenderer.mm because a function that takes a
/// `RendererImpl&` -- building the atlas ring, bringing the device up -- cannot
/// be compiled in another translation unit otherwise, and those are exactly the
/// units worth moving out of the renderer.
///
/// @warning The member **order** below is load-bearing. See the teardown note
///          on @ref RendererImpl.

#import "VolumetricRenderer.h"

#import "FrameTrace.hpp"
#import "Fusion.hpp"
#import "OrbitCamera.hpp"
#import "SharedDevice.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/graphics_pipeline.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/shader.hpp"
#include "volumetric_kit/gfx/core/texture.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"

namespace volumetric_kit::ios_app {

namespace vg = volumetric_kit::gfx;
namespace vr = volumetric_kit::recon;

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
  Fusion fusion;
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
  // The atlas path's own counter and reason, deliberately NOT the pair above.
  //
  // They report opposite things and sharing one channel made them
  // indistinguishable. `mesh_upload_failures` has exactly one other writer,
  // which latches `mesh_unusable` first, so it fires once per process and means
  // "the geometry can never be drawn". The atlas path does not latch -- a ring
  // refusal is degraded rendering, not a dead one -- so on a persistent failure
  // it counts at frame rate, and every iteration overwrote the shared message.
  // A genuine unbindable-mesh fault arriving afterwards had its reason erased
  // on the very next frame, and on the Alerts card, which prints the count
  // without the message, a retryable atlas refusal and a permanently fatal
  // mesh were the same red row.
  //
  // Separate counters, and the atlas one carries its reason to the panel rather
  // than only into the text summary.
  std::uint64_t atlas_failures = 0;
  std::string atlas_error;
  // Memory warnings the OS pushed at us, and what the process was holding the
  // last time one arrived. See -noteMemoryWarning. Plain values rather than
  // atomics for the same reason the counter above is: UIKit delivers the
  // warning on the main thread and `fusionSummary` is built on it too.
  std::uint64_t memory_warnings = 0;
  std::uint64_t memory_warning_footprint_bytes = 0;
  // The pose the newest mesh was fused at; the camera follows the scan until
  // the user's fingers take it over.
  vr::Mat4f camera_to_world{1.0f};
  OrbitCamera camera;
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
  //
  // 1x1 white, and still here: it is what a frame binds before the first
  // textured mesh arrives, whenever a remesh published no keyframe (a colour
  // the capture refused, a texture pass that failed), and whenever this frame's
  // mesh slot holds an atlas image nothing has written yet. That last case is
  // what `atlas_slot_written` decides -- see where frame_info.atlas is chosen,
  // because binding an unwritten slot is undefined behaviour rather than merely
  // a wrong colour.
  //
  // Never *selected* by the shader on those frames, because Fusion leaves every
  // uv0 at the sentinel when it does not texture -- but the set must be
  // non-null regardless or the frame records no draw at all.
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

  /// @brief One keyframe image, paired with the mesh slot whose `uv0` index it.
  ///
  /// Persistent, and that is the design rather than an optimisation. gfx's
  /// `upload_texture` creates a fresh image plus a staging buffer and blocks on
  /// a fence -- right for an asset loaded once, wrong at remesh rate.
  /// `FusionConfig::remesh_every` is 1, so that shape would mean an 11 MB image
  /// creation and a blocking graphics submit *every frame*, on the queue whose
  /// submit mutex the warning at the top of Fusion.hpp is entirely about. These
  /// are built once at the colour camera's size and rewritten in place: one
  /// host memcpy into a mapped staging buffer, then a copy recorded into the
  /// frame's own command buffer. Nothing is allocated, submitted or waited on
  /// per frame.
  struct AtlasSlot {
    vg::Texture texture;
    /// Host-visible and persistently mapped: the render thread writes here and
    /// the GPU copies out inside the frame already being recorded, so there is
    /// no second submit and no fence.
    vg::Buffer staging;
    vg::DescriptorSet set;
  };
  /// Indexed by @ref mesh_slot, deliberately: a mesh and the keyframe its `uv0`
  /// address are one value, so slot i's atlas belongs to slot i's mesh. Binding
  /// them crossed samples the wrong place on every textured triangle -- and
  /// looks like a plausible image, not like an error.
  AtlasSlot atlas_slots[kMeshSlots];
  /// The colour camera's dimensions the ring was built for, or 0 before the
  /// first textured mesh arrives. ARKit does not change `imageResolution`
  /// mid-session, but a ring built for one size and fed another would read past
  /// the staged image; the upload checks this rather than assuming it.
  std::uint32_t atlas_width = 0;
  std::uint32_t atlas_height = 0;
  /// Whether @ref atlas_slots hold real images yet; until then every frame
  /// binds the 1x1 white set.
  bool atlas_ready = false;
  /// Which slots have been written at least once. A slot that has not is still
  /// in `VK_IMAGE_LAYOUT_UNDEFINED`, so its first barrier must not claim to
  /// transition *from* `SHADER_READ_ONLY_OPTIMAL` -- undefined behaviour that a
  /// validation layer would name and that MoltenVK, which this ships without,
  /// simply acts on.
  bool atlas_slot_written[kMeshSlots] = {};
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

}  // namespace volumetric_kit::ios_app

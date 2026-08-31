// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Objective-C++ seam. This is the one translation unit where a
// CAMetalLayer* and a vg::app::WindowedApp are both first-class, which is the
// entire reason the bridge is .mm rather than Swift.

#import "VolumetricRenderer.h"

#import <Metal/Metal.h>

#import "AllocationStopDisplay.hpp"
#import "AtlasRing.hpp"
#import "BridgeStrings.hpp"
#import "FrameTrace.hpp"
#import "Fusion.hpp"
#import "MemoryQuery.hpp"
#import "OrbitCamera.hpp"
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
// FrameTrace::dump uses std::fprintf / std::snprintf / std::fflush, and
// fusionSummary uses std::snprintf. It compiled only because some gfx or recon
// header happens to pull <cstdio> in transitively today.
#include <cstdarg>
#include <cstdio>
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

// What this file was split into, brought in by name.
//
// Named individually rather than with a `using namespace`, so this list *is*
// the inventory of what left: a symbol that stops resolving names itself, and
// adding one back here is a deliberate line rather than an invisible
// consequence of a wildcard.
//
// Needed at all because the call sites are inside `@implementation`, which is
// global scope. These used to sit in this file's anonymous namespace, where
// unqualified lookup found them; they now live in a real namespace, where it
// does not.
using volumetric_kit::ios_app::allocation_stop_note;
using volumetric_kit::ios_app::allocation_stop_text;
using volumetric_kit::ios_app::allocation_stop_value;
using volumetric_kit::ios_app::build_atlas_ring;
using volumetric_kit::ios_app::FrameTrace;
using volumetric_kit::ios_app::record_atlas_upload;
using volumetric_kit::ios_app::RendererImpl;
using volumetric_kit::ios_app::set_error;
using volumetric_kit::ios_app::to_ns_string;

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

/// The turn for the orientation Swift handed us, in radians about the **GL**
/// camera's +Z. See `app::viewport_turn`, which defines it and which the tests
/// pin.
float viewport_turn(VolumetricViewOrientation orientation) {
  return app::viewport_turn(static_cast<app::ViewOrientation>(orientation));
}

}  // namespace

// The stage-row value type. At file scope, not inside the renderer's
// @implementation -- Objective-C has no nested implementations.
@interface VolumetricStageRow ()
- (instancetype)initWithRow:(const volumetric_kit::recon::StageRow&)row;
@end

@interface VolumetricFrameSample ()
- (instancetype)initWithSample:(const app::FrameSample&)sample;
@end

@interface VolumetricStatRow ()
- (instancetype)initWithLabel:(NSString*)label
                        value:(NSString*)value
                         tone:(VolumetricStatTone)tone;
@end

@implementation VolumetricStatRow
- (instancetype)initWithLabel:(NSString*)label
                        value:(NSString*)value
                         tone:(VolumetricStatTone)tone {
  if ((self = [super init])) {
    _label = [label copy];
    _value = [value copy];
    _tone = tone;
  }
  return self;
}
@end

@interface VolumetricStatSection ()
- (instancetype)initWithTitle:(NSString*)title
                         rows:(NSArray<VolumetricStatRow*>*)rows;
@end

@implementation VolumetricStatSection
- (instancetype)initWithTitle:(NSString*)title
                         rows:(NSArray<VolumetricStatRow*>*)rows {
  if ((self = [super init])) {
    _title = [title copy];
    _rows = [rows copy];
  }
  return self;
}
@end

@interface VolumetricDashboardSnapshot ()
// The budget whole, not the one field the panel used to draw. The gauges need
// the ceiling, the peak and both refusal flags, and passing them individually
// would be five more parameters that can be threaded in the wrong order --
// while the struct is already a single coherent reading from one `task_info`
// call, which is the property the whole snapshot exists to preserve.
- (instancetype)initWithStages:(NSArray<VolumetricStageRow*>*)stages
                      sections:(NSArray<VolumetricStatSection*>*)sections
                       history:(NSArray<VolumetricFrameSample*>*)history
                         stats:(const app::FusionStats&)stats
                        budget:(const app::MemoryBudget&)budget
                    workingSet:(std::uint64_t)workingSet;
@end

@implementation VolumetricDashboardSnapshot
- (instancetype)initWithStages:(NSArray<VolumetricStageRow*>*)stages
                      sections:(NSArray<VolumetricStatSection*>*)sections
                       history:(NSArray<VolumetricFrameSample*>*)history
                         stats:(const app::FusionStats&)stats
                        budget:(const app::MemoryBudget&)budget
                    workingSet:(std::uint64_t)workingSet {
  if ((self = [super init])) {
    _stages = [stages copy];
    _sections = [sections copy];
    _history = [history copy];
    // From the same `stats` the sections were built from, which is the whole
    // point of this object: the headline meter and the Volume card are the same
    // fraction, and reading it twice let them disagree.
    _occupancy = stats.occupancy;
    _occupancyKnown = stats.occupancy_known ? YES : NO;
    _triangles = stats.triangles;
    _vertices = stats.vertices;
    _allocationStop = allocation_stop_value(stats.allocation_stop);
    _allocationStopReason =
        stats.allocation_stop == app::AllocationStop::None
            ? nil
            : to_ns_string(allocation_stop_text(stats.allocation_stop).advice);

    // The gauge figures. `table_blocks` rather than `table_capacity` beside the
    // occupancy, and `table_capacity` only beside `active_blocks` -- the two
    // pairs are stamped at different cadences and each is coherent only with
    // its own partner. See FusionStats::table_blocks.
    _triangleCapacity = stats.extract.triangle_capacity;
    _tableBlocks = stats.table_blocks;
    _activeBlocks = stats.extract.active_blocks;
    _extractStale = stats.extract_stale ? YES : NO;

    // No extract residual here. `max(0, extract_ms - extract.total_ms())` was
    // computed on this line and published as its own figure, but recon already
    // pushes that exact expression as the `"  ..other"` stage row -- so the
    // panel drew it as a bar and then listed it again underneath a heading
    // reading "not in the bars above". One measurement, one publication: the
    // bar. See Fusion.mm's push_stage("  ..other", ...).
    _atlasCopyMs = stats.atlas_copy_ms;
    _framesFused = stats.frames_fused;
    _msSinceStages = stats.ms_since_stages;
    // Published beside `ms_since_stages` because it is the only thing that one
    // means anything against -- see the property docs. Sending the first
    // without the second is what left the panel unable to make the comparison
    // the transcript makes three hundred lines away in this same file.
    _msSinceFuse = stats.ms_since_fuse;
    _stagesTruncated = stats.stages_truncated ? YES : NO;
    _gpuTimingRetired = stats.gpu_timing_retired ? YES : NO;

    _memoryFootprintBytes = budget.footprint_bytes;
    _gpuWorkingSetBytes = workingSet;
    // Zero when the ceiling is not a real one. `limit_known` is false both for
    // a kernel too old to report the remainder and for a process already at the
    // limit, and in the second case `limit_bytes` is a live number that means
    // the opposite of what it looks like -- it has collapsed onto the
    // footprint. Publishing 0 makes a consumer that forgets to check draw
    // nothing rather than draw 100%.
    _memoryLimitBytes = budget.limit_known ? budget.limit_bytes : 0;
    _memoryPeakBytes = budget.peak_footprint_bytes;
    _memoryValid = budget.valid ? YES : NO;
    _memoryAtLimit = budget.at_limit ? YES : NO;
  }
  return self;
}
@end

namespace {

// Small helpers so building a section reads as a list of figures rather than a
// wall of alloc/init.
// Measured rather than truncated, and never nil.
//
// Both halves were bugs. The longest value on this read-out is the error row,
// which carries `FusionStats::last_error` -- and `Fusion::fuse` appends a ~215
// character advisory to that on the `track_dirty_blocks` path, so a fixed 256
// byte buffer cut it with the return value discarded. A cut lands wherever the
// limit falls, including mid-UTF-8 in a driver or recon message, and
// `stringWithUTF8String:` answers **nil** for the result -- which `[nil copy]`
// then stores in a `nonnull` property, so the trap surfaces in Swift at the
// first read rather than here. `to_ns_string` is this file's own answer to that
// second half and simply was not being used; see its comment.
// How many fused frames a survey may be missing for before the read-out calls
// it a failure rather than a first window.
//
// A window and a half, the same margin `Fusion` uses to stop presenting a
// *published* sample as current: the first survey is due at
// `kSurveyEveryFrames`, and naming a fault the moment that frame goes by would
// name one on the frame the sample is being taken. Derived from Fusion's own
// constant rather than restated, because a second 60 here is a number that
// drifts the first time the real one moves.
constexpr std::uint64_t kSurveyFirstWindowFrames =
    app::kSurveyEveryFrames + app::kSurveyEveryFrames / 2;

NSString* fmt(const char* format, ...) __attribute__((format(printf, 1, 2)));
NSString* fmt(const char* format, ...) {
  // Sized for the common row, which is far shorter than this; the heap path is
  // for the error row and anything else that grows without a fixed bound.
  char stack[256];
  va_list args;
  va_start(args, format);
  va_list measure;
  va_copy(measure, args);
  const int needed = std::vsnprintf(stack, sizeof(stack), format, measure);
  va_end(measure);
  if (needed < 0) {
    // An output error rather than a length. Nothing useful to print, and a
    // partially-filled buffer here is not NUL-terminated by any guarantee.
    va_end(args);
    return @"(unprintable)";
  }
  if (static_cast<std::size_t>(needed) < sizeof(stack)) {
    va_end(args);
    return to_ns_string(stack);
  }
  std::vector<char> heap(static_cast<std::size_t>(needed) + 1);
  std::vsnprintf(heap.data(), heap.size(), format, args);
  va_end(args);
  return to_ns_string(heap.data());
}

void add(NSMutableArray<VolumetricStatRow*>* rows, NSString* label,
         NSString* value, VolumetricStatTone tone = VolumetricStatToneNeutral) {
  [rows addObject:[[VolumetricStatRow alloc] initWithLabel:label
                                                     value:value
                                                      tone:tone]];
}

VolumetricStatSection* make_section(NSString* title,
                                    NSArray<VolumetricStatRow*>* rows) {
  return [[VolumetricStatSection alloc] initWithTitle:title rows:rows];
}

// A fraction's tone against two thresholds, so every meter in the app agrees
// about when a number stops being routine. The rule is in Core/StatTone.hpp,
// where it is host tested; this is only the ObjC-typed name the panel uses.
//
// Pinned value-for-value like the orientation enum above, and for the same
// reason: the two enums are converted by a cast, so a reordering would silently
// recolour every gauge rather than failing.
static_assert(static_cast<NSInteger>(VolumetricStatToneNeutral) ==
                  static_cast<NSInteger>(app::StatTone::Neutral),
              "VolumetricStatTone and app::StatTone disagree: neutral");
static_assert(static_cast<NSInteger>(VolumetricStatToneGood) ==
                  static_cast<NSInteger>(app::StatTone::Good),
              "VolumetricStatTone and app::StatTone disagree: good");
static_assert(static_cast<NSInteger>(VolumetricStatToneWarn) ==
                  static_cast<NSInteger>(app::StatTone::Warn),
              "VolumetricStatTone and app::StatTone disagree: warn");
static_assert(static_cast<NSInteger>(VolumetricStatToneCritical) ==
                  static_cast<NSInteger>(app::StatTone::Critical),
              "VolumetricStatTone and app::StatTone disagree: critical");

VolumetricStatTone tone_for(double fraction, double warn, double crit) {
  return static_cast<VolumetricStatTone>(app::tone_for(fraction, warn, crit));
}

}  // namespace

@implementation VolumetricFrameSample
- (instancetype)initWithSample:(const app::FrameSample&)sample {
  if ((self = [super init])) {
    _frame = sample.frame;
    _hostMs = sample.host_ms;
    _deviceMs = sample.device_ms;
    _timestampNs = sample.timestamp_ns;
    _deviceTimingValid = sample.device_timing_valid ? YES : NO;
    _extractMs = sample.extract_ms;
    _occupancy = sample.occupancy;
    _occupancyKnown = sample.occupancy_known ? YES : NO;
    _triangles = sample.triangles;
    _activeBlocks = sample.active_blocks;
    _framesSinceExtract = sample.frames_since_extract;
    _allocationStop = allocation_stop_value(sample.allocation_stop);
    // Derived rather than carried, so the two cannot disagree about the same
    // frame the way two independently-assigned fields eventually do.
    _allocationStopped =
        sample.allocation_stop != app::AllocationStop::None ? YES : NO;
  }
  return self;
}
@end

@implementation VolumetricStageRow
- (instancetype)initWithRow:(const volumetric_kit::recon::StageRow&)row {
  if ((self = [super init])) {
    // Copied, unlike the C++ row which borrows: an NSString outliving the
    // literal costs nothing here, and it frees the Swift side from the
    // lifetime rule StageRow carries.
    //
    // Through to_ns_string like every other string this file hands Swift, not
    // through a bare stringWithUTF8String:. `name` is inside
    // NS_ASSUME_NONNULL_BEGIN, so Swift imports it as a non-optional String
    // that traps on the nil that call returns for invalid UTF-8 -- and the
    // labels are library literals this file does not choose, which is the same
    // reason deviceName goes through it. Today they are ASCII; FusionStats'
    // own warning about a row named from somewhere else is the case this
    // covers.
    _name = to_ns_string(row.name != nullptr ? row.name : "");
    _cpuMs = row.cpu_ms;
    _gpuMs = row.gpu_ms;
    _hasGpu = row.has_gpu ? YES : NO;
  }
  return self;
}
@end

@interface VolumetricRenderer ()
// The panel's two builders, taking the snapshot rather than each fetching one.
// This is what lets `dashboardSnapshot` assemble the whole read-out from a
// single `FusionStats` copy and a single `query_memory_budget`, while the
// individual properties keep working for a caller that wants one figure.
- (NSArray<VolumetricStageRow*>*)stageRowsForStats:(const app::FusionStats&)s;
- (NSArray<VolumetricStatSection*>*)
    sectionsForStats:(const app::FusionStats&)s
              budget:(const app::MemoryBudget&)budget
          workingSet:(std::uint64_t)workingSet;
@end

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

  // Sized for the whole ring plus the white fallback, allocated once here. A
  // pool grown or re-created later would have to be done mid-frame, on the
  // thread that is recording -- and the sets it hands out are bound by frames
  // still in flight, so freeing one is a use-after-free with no diagnostic on
  // this configuration.
  const std::uint32_t kAtlasSets = RendererImpl::kMeshSlots + 1;
  const VkDescriptorPoolSize atlas_pool_size{
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kAtlasSets};
  vg::Result<vg::DescriptorPool> atlas_pool =
      vg::DescriptorPool::create(device, &atlas_pool_size, 1, kAtlasSets);
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
  for (std::size_t i = 0; i < RendererImpl::kMeshSlots; ++i) {
    vg::Result<vg::DescriptorSet> slot_set = _impl->atlas_pool.allocate(
        _impl->mesh_pipeline->descriptor_set_layout(0));
    if (!slot_set) {
      set_error(error, slot_set.status(),
                "atlas ring DescriptorPool::allocate");
      return nil;
    }
    _impl->atlas_slots[i].set = std::move(slot_set).value();
  }

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
      // several meshes stale. `atlas_slot_written` is what carries that, and
      // clearing it is why the else-branch exists rather than the skip being a
      // bare `if`.
      if (_impl->draw_mesh && fresh->atlas != nullptr) {
        // Built on the first textured mesh rather than at bring-up, because the
        // size is ARKit's to state: `imageResolution` is not known until a
        // frame has arrived, and guessing 1920x1440 would be a constant that
        // silently mis-sizes the ring on any device that reports otherwise.
        if (!_impl->atlas_ready) {
          const vg::Status built = [&] {
            return build_atlas_ring(*_impl, fresh->atlas_width,
                                    fresh->atlas_height);
          }();
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
        const bool extent_ok = _impl->atlas_ready &&
                               fresh->atlas_width == _impl->atlas_width &&
                               fresh->atlas_height == _impl->atlas_height;
        if (extent_ok) {
          RendererImpl::AtlasSlot& slot = _impl->atlas_slots[_impl->mesh_slot];
          const std::size_t bytes =
              static_cast<std::size_t>(_impl->atlas_width) *
              _impl->atlas_height * sizeof(std::uint32_t);
          // The one host copy on this path. `Published::atlas` is valid only
          // until the next take_mesh, so it is consumed here, in the same call
          // that received it, rather than remembered.
          std::memcpy(slot.staging.mapped(), fresh->atlas, bytes);
          record_atlas_upload(f.cmd, slot.staging.handle(),
                              slot.texture.image(), _impl->atlas_width,
                              _impl->atlas_height,
                              !_impl->atlas_slot_written[_impl->mesh_slot]);
          _impl->atlas_slot_written[_impl->mesh_slot] = true;
        } else if (_impl->atlas_ready) {
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
          _impl->atlas_slot_written[_impl->mesh_slot] = false;
          ++_impl->atlas_failures;
          _impl->atlas_error =
              "keyframe is " + std::to_string(fresh->atlas_width) + "x" +
              std::to_string(fresh->atlas_height) + " but the ring was built " +
              std::to_string(_impl->atlas_width) + "x" +
              std::to_string(_impl->atlas_height) +
              "; textured meshes render white for the rest of this scan";
        }
      } else if (fresh->atlas != nullptr) {
        // Drawing is off, so the upload above was skipped. The mesh in this
        // slot still has real uv0, so the slot must not stay bindable -- see
        // the gate's note. The next remesh reaching it while drawing is on
        // writes it again.
        _impl->atlas_slot_written[_impl->mesh_slot] = false;
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
  if (const float turn = viewport_turn(_impl->view_orientation); turn != 0.0f) {
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
    // thread takes on every one of its own frames. A handful of scalars is all
    // the ring holds. See Fusion::trace_stats.
    const app::FusionTraceStats s = _impl->fusion.trace_stats();
    trace.drew_mesh = true;
    trace.generation = live_src.generation;
    trace.mesh_slot = _impl->mesh_slot;
    trace.triangles = s.triangles;
    trace.triangle_capacity = s.triangle_capacity;
    trace.arena_bytes = s.arena_bytes;
    trace.active_blocks = s.active_blocks;
    trace.occupancy = s.occupancy;
    trace.stop = s.allocation_stop;
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
    // fully round, `atlas_ready` is true for slots that have never been
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
        _impl->atlas_ready && _impl->atlas_slot_written[_impl->mesh_slot];
    frame_info.atlas = atlas_bindable
                           ? _impl->atlas_slots[_impl->mesh_slot].set.handle()
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

+ (NSUInteger)frameHistoryCapacity {
  return static_cast<NSUInteger>(app::Fusion::kHistoryCapacity);
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
  return [self stageRowsForStats:_impl->fusion.stats()];
}

- (NSArray<VolumetricStageRow*>*)stageRowsForStats:(const app::FusionStats&)s {
  NSMutableArray<VolumetricStageRow*>* rows =
      [NSMutableArray arrayWithCapacity:s.stage_count];
  for (std::uint32_t i = 0; i < s.stage_count; ++i) {
    [rows addObject:[[VolumetricStageRow alloc] initWithRow:s.stages[i]]];
  }
  // Immutable, because the property says `copy` and a caller reading that
  // attribute is entitled to a value that cannot change under it. Eight
  // elements at a few hertz is not the cost worth keeping a live mutable array
  // to save.
  return [rows copy];
}

// TODO(scanner): fusionSummary still formats its own text with snprintf, so
// this and the log line are two independent formatters over one set of figures
// and can drift. Rendering the summary FROM these sections is the fix; it is
// not done here because that string carries layout this does not (banner
// ordering, the clipped-tail argument) and folding it in deserves its own
// change rather than a footnote to a UI one.
- (NSArray<VolumetricStatSection*>*)statSections {
  return [self sectionsForStats:_impl->fusion.stats()
                         budget:app::query_memory_budget()
                     workingSet:gpu_working_set_bytes()];
}

- (NSArray<VolumetricStatSection*>*)
    sectionsForStats:(const app::FusionStats&)s
              budget:(const app::MemoryBudget&)budget
          workingSet:(std::uint64_t)workingSet {
  const double mb = 1024.0 * 1024.0;
  NSMutableArray<VolumetricStatSection*>* out = [NSMutableArray array];

  // --- Alerts: the two signals that are not in FusionStats at all -----------
  //
  // First, because `fusionSummary` puts them first and for the same reason: a
  // mesh whose uploads are failing looks like a clean scan -- the fused and
  // remesh counters keep climbing while the geometry on screen stops changing
  // -- and a memory warning is the only notice the OS gives before jetsam.
  //
  // These live on the bridge, not on the fusion: the upload is the render
  // thread's stage and the warning arrives on the UI thread, so neither is in
  // the snapshot above. That is exactly why the panel was missing them.
  {
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    if (_impl->mesh_upload_failures > 0) {
      // With the reason, not just the count. The message was reaching only the
      // text summary, so on this panel -- the primary read-out -- a mesh that
      // cannot be bound as geometry was an unexplained red row. It is also
      // latched and permanent, which the row now says outright rather than
      // leaving to be inferred from a count that never moves again.
      add(r, @"mesh upload",
          _impl->mesh_upload_error.empty()
              ? fmt("%llu failed",
                    (unsigned long long)_impl->mesh_upload_failures)
              : fmt("%llu failed: %s",
                    (unsigned long long)_impl->mesh_upload_failures,
                    _impl->mesh_upload_error.c_str()),
          VolumetricStatToneCritical);
    }
    if (_impl->atlas_failures > 0) {
      // Its own row, beside that one rather than merged into it, because the
      // two mean opposite things: this one is retryable and leaves the scan
      // rendering in voxel colour, that one is fatal and leaves it not
      // rendering geometry at all. Sharing a row made a transient memory-
      // pressure refusal look like an unbindable mesh, and -- because this path
      // counts at frame rate while that one fires once -- let a persistent
      // atlas failure erase a genuine mesh fault's reason on the next frame.
      //
      // Warn rather than Critical, matching what it costs: the geometry still
      // draws, textured surfaces fall back to white or to fused voxel colour.
      add(r, @"atlas",
          _impl->atlas_error.empty()
              ? fmt("%llu failed", (unsigned long long)_impl->atlas_failures)
              : fmt("%llu failed: %s",
                    (unsigned long long)_impl->atlas_failures,
                    _impl->atlas_error.c_str()),
          VolumetricStatToneWarn);
    }
    if (_impl->memory_warnings > 0) {
      // The footprint sampled *at* the warning, not now: this is the one
      // reading taken when the OS said it mattered, and the next poll is up to
      // half a second later -- long enough for the allocation that provoked it
      // to have been freed again.
      add(r, @"memory warning",
          _impl->memory_warning_footprint_bytes > 0
              ? fmt("x%llu  (last at %.0f MB held)",
                    (unsigned long long)_impl->memory_warnings,
                    _impl->memory_warning_footprint_bytes / mb)
              : fmt("x%llu", (unsigned long long)_impl->memory_warnings),
          VolumetricStatToneCritical);
    }
    // The fusion's own error counter, here rather than on the section that
    // describes the geometry. It was a row on `Fusion` -- beside the frame and
    // vertex counts, in a card a reader consults for scale rather than for
    // health -- which put the two halves of "something is wrong" on opposite
    // sides of the panel: the bridge's failures here, the fusion's four cards
    // away. A fault is a fault whichever thread noticed it.
    if (s.errors > 0) {
      // The count with the reason *when there is one*, not the count with a
      // dangling separator. `last_error` is most-recent-wins and `fuse` assigns
      // it on every fused frame from outside the failure guard (Fusion.mm's
      // `stats_.last_error = frame_error;`), so a clean frame after a failure
      // blanks the message while the counter -- the half that does not get
      // overwritten -- keeps standing. Unguarded, one survey refusal at default
      // config grew a permanent critical card reading "fusion  1 errors  " with
      // nothing after it, for the life of the process. `fusionSummary` has
      // guarded this same case all along.
      add(r, @"fusion",
          s.last_error.empty()
              ? fmt("%llu errors", (unsigned long long)s.errors)
              : fmt("%llu errors: %s", (unsigned long long)s.errors,
                    s.last_error.c_str()),
          VolumetricStatToneCritical);
    }
    if (r.count > 0) [out addObject:make_section(@"Alerts", r)];
  }

  // --- Scene: the geometry, and the arena holding it -------------------------
  //
  // One section where there were three. `Fusion` carried `%u verts / %u tris`,
  // `Extract` carried the same pair back as `emitted`, and `Arena` carried the
  // triangle count a third time as the numerator of its fill ratio -- with the
  // headline's `M tris` above them making four. They are one quantity measured
  // once: `stats_.vertices` and `stats_.triangles` are assigned from the mesh
  // the extract just wrote. The only case in which the two differed was an
  // extract too stale to have written it, which is a *staleness* fact and now
  // reads as one.
  {
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    // First row on the first card, because it is the answer to the question
    // the screen is provoking. This build publishes no geometry, so the view is
    // empty from launch however well the scan is going -- and the operator
    // walking the room is the one supplying the coverage being measured. Left
    // unmarked, the natural reading is that tracking has failed and the natural
    // response is to stop walking, which ends the measurement.
    if (s.incremental_benchmark) {
      add(r, @"mode", @"MEASURING -- no geometry drawn",
          VolumetricStatToneWarn);
    }
    // Under an incremental extract these are recon's arena watermark, which
    // still counts ranges the kernel retired in place, so they run above the
    // live surface. Marked rather than silently compared against a normal
    // build's. See FusionStats::spans_tracked.
    add(r, @"mesh",
        s.spans_tracked ? fmt("%u verts / %u tris  (incl. retired)", s.vertices,
                              s.triangles)
                        : fmt("%u verts / %u tris", s.vertices, s.triangles));
    // The arena's fill against the plan for the slot this extract wrote. Its
    // own row rather than folded into the mesh line above: the numerator is the
    // same triangle count, but the question is whether the next remesh has room
    // rather than how much surface there is.
    //
    // OCCUPANCY, not live-surface fill, and the distinction is the whole of
    // this row under an incremental extract. `s.triangles` is recon's arena
    // watermark, which still counts triangles the kernel retired in place, so
    // the ratio runs above the live surface by however much the arena has not
    // compacted away -- recon's own occupancy ceiling is what eventually forces
    // a full pass to compact it. Left driving the tone, because the resident
    // bytes are genuinely held either way, but named so a warn colour is not
    // read as "the live surface is about to overflow".
    const double fill = s.extract.triangle_capacity > 0
                            ? (double)s.triangles / s.extract.triangle_capacity
                            : 0.0;
    add(r, @"arena",
        s.spans_tracked ? fmt("%.1f%% of %u tris  (incl. retired)",
                              100.0 * fill, s.extract.triangle_capacity)
                        : fmt("%.1f%% of %u tris", 100.0 * fill,
                              s.extract.triangle_capacity),
        tone_for(fill, 0.9, 0.98));
    // recon folds the per-block span table into `arena_bytes` -- grid-sized,
    // and doubling on every VoxelHashMap::resize -- so this figure is not
    // comparable with a normal build's. The table is the measurement's own
    // cost, which makes saying so part of reporting the measurement.
    add(r, @"resident",
        s.spans_tracked ? fmt("%.0f MB across %u slot%s  (incl. span table)",
                              s.extract.arena_bytes / mb, s.mesh_slots,
                              s.mesh_slots == 1 ? "" : "s")
                        : fmt("%.0f MB across %u slots",
                              s.extract.arena_bytes / mb, s.mesh_slots));
    // The publish counter, or the reason there is not one. Under the
    // measurement mode nothing publishes, so the version stays 0 while the
    // remesh count climbs; printed as "v0 after 4900 remeshes" that reads
    // exactly like a wedged publish path, which is the one thing a diagnostic
    // build must not be ambiguous about. Named instead, since "not published"
    // is the mode working.
    add(r, @"version",
        s.incremental_benchmark ? fmt("not published, after %llu remeshes",
                                      (unsigned long long)s.remeshes)
                                : fmt("v%u after %llu remeshes", s.mesh_version,
                                      (unsigned long long)s.remeshes));
    // `pass` rather than `dispatch`: the number is how many times the surface
    // was meshed, and `dispatch` is a duration on the latency bars. Read it as
    // cost, not as a verdict on the capacity planner -- the refit triggers
    // against the slot's retained grow-only arena, so a plan that undershoots
    // still reports one pass whenever an earlier peak left room to absorb it.
    //
    // The staleness marker rides on this row rather than on its own, because
    // this is the row it invalidates: everything in `s.extract` above --
    // capacity, arena bytes, and the mesh counts the extract stamped -- comes
    // from the last *successful* remesh, and a breakdown frozen by a failing
    // extract otherwise reads as current.
    add(r, @"extract",
        s.extract_stale
            ? fmt("%u pass  [stale %llu frames]", s.extract.dispatches,
                  (unsigned long long)s.frames_since_extract)
            : fmt("%u pass", s.extract.dispatches),
        s.extract_stale || s.extract.dispatches > 1
            ? VolumetricStatToneWarn
            : VolumetricStatToneNeutral);
    // The two rows the measurement mode exists to produce, and without which it
    // cannot be told from the thing it is measured against. recon falls back to
    // a full extract silently and by design -- a topology change, a grown
    // arena, flags it will not vouch for -- so "did this call re-mesh only the
    // changed blocks" is reported rather than inferred. `dispatches` on the row
    // above reads 1 on both paths and cannot answer it.
    //
    // Shown only when the mode is on, because on the normal path `incremental`
    // is false by construction and a permanent "full" row is noise.
    if (s.incremental_benchmark) {
      add(r, @"incremental",
          s.extract.incremental ? @"yes" : @"NO -- fell back to full",
          s.extract.incremental ? VolumetricStatToneNeutral
                                : VolumetricStatToneWarn);
      // The fraction the whole feature trades against. Meaningless on a full
      // pass, which reports 0 rather than restating active_blocks -- so the
      // fallback prints the window alone instead of a fake 0%.
      //
      // The window travels with it because the fraction is a function of how
      // much fusing happened since the last extract: the same scan re-meshes
      // twice as much of itself over two frames as over one, and a number
      // quoted without it can be compared with anything. See
      // FusionStats::extract_window_frames.
      if (s.extract.incremental && s.extract.active_blocks > 0) {
        const double frac =
            (double)s.extract.remeshed_blocks / (double)s.extract.active_blocks;
        add(r, @"re-meshed",
            fmt("%.1f%% of %u blocks  (%llu fused frame%s)", 100.0 * frac,
                s.extract.active_blocks,
                (unsigned long long)s.extract_window_frames,
                s.extract_window_frames == 1 ? "" : "s"));
      } else {
        add(r, @"re-meshed",
            fmt("- (%llu fused frame%s)",
                (unsigned long long)s.extract_window_frames,
                s.extract_window_frames == 1 ? "" : "s"));
      }
    }
    add(r, @"fused", fmt("%llu frames", (unsigned long long)s.frames_fused));
    // The texture pass, as a state rather than a duration -- the latency bars
    // already carry its timing, and this row exists to say which of the four
    // things a 0.0 ms there means. See app::TextureState, and its warning about
    // what "ran" does not claim.
    //
    // The tolerance travels with it because it is the knob the state is the
    // only feedback for: FusionConfig::occlusion_threshold documents finding
    // the value by turning it while pointing at one surface and watching where
    // texturing stops, and until this row existed the panel showed neither the
    // value being turned nor whether the pass was running at all.
    switch (s.texture_state) {
      case app::TextureState::Off:
        add(r, @"texture", @"off");
        break;
      case app::TextureState::Pending:
        add(r, @"texture", @"on, no remesh yet");
        break;
      case app::TextureState::NoColor:
        add(r, @"texture", @"skipped -- no colour on this frame",
            VolumetricStatToneWarn);
        break;
      case app::TextureState::Failed:
        add(r, @"texture", @"failed", VolumetricStatToneCritical);
        break;
      case app::TextureState::Ran:
        add(r, @"texture", fmt("%.3f m tolerance", s.occlusion_threshold),
            VolumetricStatToneGood);
        break;
    }
    [out addObject:make_section(@"Scene", r)];
  }

  // --- The stage rows are NOT a section --------------------------------------
  //
  // They used to be. `Pipeline` formatted every stage into `"%6.2f ms   gpu
  // %6.2f"` here, at a few hertz, and the panel then threw the whole section
  // away -- `DashboardView` special-cased the title and drew bars from
  // `stageRows` instead. Two formatters over one set of figures, one of them
  // feeding nothing, and a title match holding them together. The bars are the
  // only rendering, so `stageRows` is the only publication.

  // --- Block table: the hash map, and the ceiling that stops a scan ----------
  {
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    // `occupancy_known` gates the figure rather than decorating it. When
    // `load_factor` fails the fusion forces occupancy to 1.0 so the guard
    // refuses, and printing that as "100.0% of N blocks" in critical red is a
    // full volume reported on a volume that may be nearly empty.
    //
    // `table_blocks`, NOT `table_capacity`. Both are block-table capacities and
    // the wrong one was here: `occupancy` is read from `load_factor` every
    // fused frame, while `table_capacity` is stamped beside
    // `extract.active_blocks` on a successful remesh, so the row divided a
    // per-frame numerator by a per-remesh denominator -- the pairing
    // FusionStats::table_blocks exists to prevent and names outright. It reads
    // `4.3% of 0 blocks` before the first extract, halves after a doubling
    // whose next remesh skips, and freezes for the session under a persistent
    // extract failure while the map doubles beneath it.
    if (s.occupancy_known) {
      add(r, @"occupied",
          fmt("%.1f%% of %u blocks", 100.0 * s.occupancy, s.table_blocks),
          tone_for(s.occupancy, 0.7, 0.85));
    } else {
      add(r, @"occupied", fmt("unreadable  (of %u blocks)", s.table_blocks),
          VolumetricStatToneWarn);
    }
    // The cause, not one of the four causes. See `allocation_stop_text`: the
    // advice for a full volume is actively wrong for the other two, and this
    // row used to assert it for all of them.
    if (s.allocation_stop != app::AllocationStop::None) {
      add(r, @"state",
          fmt("ALLOCATION STOPPED — %s",
              allocation_stop_text(s.allocation_stop).headline),
          VolumetricStatToneCritical);
    }
    // The other capacity, with its own partner and its own cadence stated. Two
    // block counts sat on this card reading as one quantity measured twice --
    // `occupied` above moves every fused frame, this one only when a remesh
    // succeeds. Saying which instant each belongs to is what stops a reader
    // subtracting them.
    //
    // The denominator is guarded for the same reason `occupied`'s is, one row
    // up: `table_capacity` is stamped only on a fully-successful remesh and is
    // 0 until the first one lands, so printing it unconditionally reproduced
    // `0 of 0 blocks` here -- the identical artefact this card was rewritten to
    // remove -- from launch, and for the whole session on a device whose
    // extract keeps failing.
    if (s.table_capacity > 0) {
      add(r, @"active",
          fmt("%u of %u blocks at last remesh", s.extract.active_blocks,
              s.table_capacity));
    } else {
      add(r, @"active", @"no remesh yet");
    }
    [out addObject:make_section(@"Block table", r)];
  }

  // --- Dirty: the survey, off the card whose cadence it does not share -------
  //
  // It was the last row of `Volume`, under an `active` row it does not share a
  // denominator with: `survey_active_blocks` is the whole active set as of the
  // last survey, `extract.active_blocks` is the set as of the last remesh, and
  // the two cadences differ by ~60x. Adjacent and near-identically labelled,
  // they read as the same number disagreeing with itself.
  {
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    if (s.survey_active_blocks == 0) {
      // A row rather than a missing card -- a card that simply is not there
      // reflows the grid the moment one lands. But *which* row, because the
      // gate is a one-way latch and a single neutral "no sample yet" was the
      // sole output for three states that want three different responses.
      //
      // Under the measurement mode the survey never runs at all: the block
      // below is gated on `!incremental_benchmark`, so nothing is failing and
      // nothing is coming. After the first window has had time to land and has
      // not, the surveys are failing -- reachable at default config, and
      // reachable indefinitely with `track_dirty_blocks` off, which the survey
      // block is *not* gated on: it still pays for a compaction, a fence and a
      // full readback every window, and `dirty_remesh_blocks` then refuses.
      // That case used to read as the neutral first-window state forever.
      if (s.incremental_benchmark) {
        add(r, @"survey", @"not run under the measurement mode");
      } else if (s.frames_fused < kSurveyFirstWindowFrames) {
        add(r, @"survey", @"no sample yet");
      } else {
        add(r, @"survey",
            fmt("failing — %llu fused frames, no sample",
                (unsigned long long)s.frames_fused),
            VolumetricStatToneWarn);
      }
    } else {
      // The same three markers `fusionSummary` attaches, because each makes the
      // sample mean something other than what it looks like and the gate above
      // is a one-way latch that cannot take a stale sample back off the screen.
      // Without them this row showed a first window's ~100% -- which any scene
      // produces, the map having grown from empty inside it -- and indefinitely
      // stale samples, both presented as this frame's.
      //
      // On their own row, and the tone on every row they qualify. They used to
      // ride on `changed` alone, which was fine while the whole sample was one
      // sentence and wrong the moment it became four rows: the denominator and
      // the window are equally first-window artefacts and equally frozen by a
      // stale sample, and they rendered neutral and unmarked beside a `changed`
      // that carried both caveats.
      std::string note;
      if (s.survey_first_window) {
        note += "first window: grew from empty";
      }
      if (s.survey_stale) {
        if (!note.empty()) note += ", ";
        note += "stale " + std::to_string(s.frames_since_survey) + "f";
      }
      const VolumetricStatTone survey_tone =
          s.survey_stale ? VolumetricStatToneWarn : VolumetricStatToneNeutral;
      if (!note.empty()) {
        add(r, @"sample", fmt("%s", note.c_str()), survey_tone);
      }
      if (s.survey_changed_blocks == 0) {
        // The steady state, not a degenerate case -- recon documents a scan
        // revisiting converged surface at `max_weight` as marking nothing.
        // There is no ratio to print, so the sentence is the result: this is
        // the branch that used to read "0.0%", the best available outcome
        // reported as no benefit at all.
        add(r, @"changed", @"nothing", survey_tone);
      } else {
        add(r, @"changed",
            fmt("%u -> %u remesh (%.1f%%)", s.survey_changed_blocks,
                s.survey_remesh_blocks,
                100.0 * s.survey_remesh_blocks /
                    (double)s.survey_active_blocks),
            survey_tone);
      }
      // The denominator, on its own row and named as the survey's own. It was
      // only ever inside the percentage above, which meant the one figure that
      // says what the fraction is *of* could not be read off the panel at all.
      add(r, @"active", fmt("%u blocks surveyed", s.survey_active_blocks),
          survey_tone);
      // Published rather than assumed equal to the survey cadence: a frame that
      // takes one of `fuse`'s error early-returns never reaches the survey, and
      // the window runs on into the next one. Without it, a double-length union
      // reports through an identical-looking line.
      add(r, @"window",
          fmt("%llu fused frames", (unsigned long long)s.survey_window_frames),
          survey_tone);
      // What the survey cost. Measured because nothing else measures it, and on
      // this card rather than among the latency bars because it runs on one
      // frame in the window rather than on every frame they describe.
      //
      // The one figure on this card that is NOT from the sample above it.
      // `survey_ms` is published on every attempt, including the failed ones --
      // deliberately, since a failure still paid for the compaction -- while
      // the three rows above refresh only when one succeeds. Unlabelled, a
      // failing survey's partial time (the compaction alone; the O(num_blocks)
      // host scan and the dilation walk never ran) sat under three rows
      // describing a sample from four seconds earlier and read as its cost.
      add(r, @"cost",
          s.survey_stale ? fmt("%.2f ms  (last attempt)", s.survey_ms)
                         : fmt("%.2f ms", s.survey_ms),
          survey_tone);
    }
    [out addObject:make_section(@"Dirty", r)];
  }

  // --- Memory: what the two gauges beside these rows cannot draw -------------
  //
  // The `jetsam` and `gpu working set` ratios are gone from here, and only
  // those: the panel now draws each as a filled bar from the typed fields on
  // the snapshot, with its own numerator and ceiling written beneath it.
  // Keeping the rows as well would have put the same two fractions on one card
  // twice, which is the pattern this whole change is about. What stays is
  // everything a bar cannot carry -- a reading that failed, a ceiling that is
  // not a ceiling, and the two figures that are not ratios at all.
  {
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    const std::uint64_t ws = workingSet;
    // The refusal branches `fusionSummary` also has, and for the same reason:
    // with only `ws > 0` and `limit_known`, a failed `task_info` left every
    // figure at its zero and the card rendered "0 / 4096 MB (0.0%)" in the
    // neutral tone -- a fabricated healthy reading, printed on the same tick
    // the log line correctly said the reading was unavailable. The gauges
    // refuse to draw in both of these states, so the row is the only thing on
    // the card that can say what happened.
    if (!budget.valid) {
      // The kernel's own code rather than a bare "unavailable". MemoryBudget
      // enumerates two reasons a reading can fail and they want different
      // responses; this row is read when something has already gone wrong.
      add(r, @"held",
          fmt("unavailable  [task_info kr=%d]", budget.task_info_status),
          VolumetricStatToneWarn);
    } else if (budget.at_limit) {
      // The last state before jetsam. The kernel clamps the remainder to 0
      // here, so deriving a ceiling yields limit == footprint and draws a tidy
      // 100% under a ceiling that rose to meet the number it was measuring.
      // This gets a sentence, not a ratio.
      add(r, @"held",
          fmt("%.0f MB — no headroom left before jetsam",
              budget.footprint_bytes / mb),
          VolumetricStatToneCritical);
    } else if (!budget.limit_known) {
      // Valid, under the limit, and the ceiling still unreported: a kernel too
      // old to carry `limit_bytes_remaining`. The footprint is real and the
      // jetsam gauge has no denominator, so the figure comes through here.
      add(r, @"held",
          fmt("%.0f MB  (ceiling not reported)", budget.footprint_bytes / mb));
    }
    // Whether the panel will draw any bar on this card at all, which is what
    // decides how much of the reading has to be carried as text.
    //
    // A gauge needs a real numerator -- the footprint, so `valid` -- and a real
    // denominator. `memoryLimitBytes` is already published as 0 both when the
    // kernel cannot report the ceiling and when the process is at it, so it is
    // the whole test for the jetsam bar; the working set is an independent
    // Metal reading and stands or falls on its own.
    const bool jetsam_gauge = budget.valid && budget.limit_known;
    const bool gpu_gauge = budget.valid && ws > 0;
    // Reported when no bar will carry it, because it is a separate reading that
    // did not fail -- suppressing it on another reading's fault drops a figure
    // that is still obtainable, and it is the *lower* of the two ceilings on
    // this hardware. This used to be gated on `!budget.valid` alone, which held
    // only while the gauge beside it was suppressed at the limit as well: in
    // the `valid && at_limit` state both vanished and the card fell back to a
    // sentence about jetsam and the device's RAM, with the ceiling that
    // actually binds nowhere on it.
    if (ws > 0 && !gpu_gauge) {
      add(r, @"gpu working set", fmt("%.0f MB recommended", ws / mb));
    }
    // Likewise the high-water mark. The panel ticks it onto whichever bars it
    // draws, which is the one place it means something -- a bare "peak 1620 MB"
    // beside a fill a reader cannot compare it to is the number without the
    // comparison that makes it worth keeping. But with no bar drawn there is no
    // tick, and this is the only figure on the card that survives the gap
    // between polls: a `resize` doubling spikes for well under one 2 Hz
    // interval, and the states with no gauge -- a refused `task_info`, and the
    // last window before jetsam -- are exactly the ones where having seen the
    // spike matters most. It went missing there, while `fusionSummary` kept
    // printing it, so panel and transcript disagreed in the pre-jetsam window.
    if (budget.peak_footprint_bytes > 0 && !jetsam_gauge && !gpu_gauge) {
      add(r, @"peak", fmt("%.0f MB", budget.peak_footprint_bytes / mb),
          budget.at_limit ? VolumetricStatToneCritical
                          : VolumetricStatToneNeutral);
    }
    if (budget.device_ram_bytes > 0) {
      add(r, @"device RAM", fmt("%.0f MB", budget.device_ram_bytes / mb));
    }
    [out addObject:make_section(@"Memory", r)];
  }

  return out;
}

- (VolumetricDashboardSnapshot*)dashboardSnapshot {
  // Three reads, once each, and everything the panel shows is derived from
  // them. Assembling the same panel from the individual properties took five
  // `FusionStats` copies and three `task_info` traps, with the fuse thread
  // writing in between -- see VolumetricDashboardSnapshot for what that let the
  // headline and the Volume card disagree about.
  //
  // The history is its own source and its own lock; it is read once here rather
  // than folded in, because the ring and the stats are different structures and
  // no single lock covers both.
  const app::FusionStats s = _impl->fusion.stats();
  const app::MemoryBudget budget = app::query_memory_budget();
  const std::uint64_t ws = gpu_working_set_bytes();
  return [[VolumetricDashboardSnapshot alloc]
      initWithStages:[self stageRowsForStats:s]
            sections:[self sectionsForStats:s budget:budget workingSet:ws]
             history:self.frameHistory
               stats:s
              budget:budget
          workingSet:ws];
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
  // The atlas ring's own banner, on its own counter. Separate from the line
  // above because a ring refusal is retryable and leaves the scan rendering in
  // voxel colour, while an upload failure is latched and leaves it with no
  // geometry at all -- see `atlas_failures` for how sharing one channel let the
  // frame-rate one erase the once-per-process one's reason.
  if (_impl->atlas_failures > 0) {
    upload += "\n  ! atlas x" + std::to_string(_impl->atlas_failures) + ": " +
              _impl->atlas_error;
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
  // The OS's own warning, which is the only part of this read-out that is not a
  // poll: see -noteMemoryWarning. A banner rather than a row on the memory
  // block below, because it is an event with a time rather than a level -- and
  // because it means the system has already decided memory is short, which puts
  // it with the failures and not with the gauges. The footprint at the moment
  // it fired is carried along; the polled one beside it will usually have
  // moved.
  std::string memory_warnings;
  if (_impl->memory_warnings > 0) {
    memory_warnings =
        "\n  ! memory warning x" + std::to_string(_impl->memory_warnings);
    if (_impl->memory_warning_footprint_bytes > 0) {
      memory_warnings += " (last at " +
                         std::to_string(_impl->memory_warning_footprint_bytes /
                                        (1024ULL * 1024ULL)) +
                         " MB)";
    }
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
  // The publish counter, or the reason there is not one. See the format string.
  const std::string version_cell = s.incremental_benchmark
                                       ? std::string("(not published)")
                                       : "v" + std::to_string(s.mesh_version);

  // The measurement mode's own row, and the log is the copy that matters for it
  // -- `devicectl ... --console` is what a reading is actually taken from, and
  // the panel is the thing being cross-checked. Says the mode is on, whether
  // recon took the incremental path (it falls back silently, and a run that
  // measured the fallback is worthless), and the fraction with the window it
  // accumulated over. Empty on the normal path, so the transcript is unchanged
  // there.
  std::string bench_row;
  if (s.incremental_benchmark) {
    char cell[192];
    if (s.extract.incremental && s.extract.active_blocks > 0) {
      std::snprintf(
          cell, sizeof(cell),
          "  measuring incremental extraction -- NO geometry published\n"
          "            re-meshed %.1f%% of %u blocks over %llu fused "
          "frame(s)\n",
          100.0 * static_cast<double>(s.extract.remeshed_blocks) /
              static_cast<double>(s.extract.active_blocks),
          s.extract.active_blocks,
          static_cast<unsigned long long>(s.extract_window_frames));
    } else {
      std::snprintf(
          cell, sizeof(cell),
          "  measuring incremental extraction -- NO geometry published\n"
          "            FELL BACK to a full extract (%llu fused frame(s))\n",
          static_cast<unsigned long long>(s.extract_window_frames));
    }
    bench_row = cell;
  }

  // The `table` row's figure and its suffix, built here rather than as four
  // more varargs down in the body -- the same argument the phase rows make, and
  // this row now has a branch in it.
  //
  // Both halves of the ratio come from `fuse`, sampled after every resize that
  // frame: `occupancy` from the map's own heap counter and `table_blocks` from
  // the bucket count that produced it. Deliberately *not* `table_capacity`,
  // which is stamped beside `extract.active_blocks` on a successful remesh --
  // pairing that denominator with this numerator divided a per-fused-frame
  // figure by a per-remesh one, so the row read "4.3% of 0 blocks" before the
  // first extract, halved after a doubling whose remesh skipped, and under a
  // persistent extract failure froze the denominator for the session while the
  // map doubled beneath it. See FusionStats::table_blocks.
  std::string table_row;
  {
    char cell[192];
    if (!s.occupancy_known) {
      // The number in `occupancy` is a fabricated 1.0 that makes the allocate
      // guard fail safe; printing it as a reading would be inventing a
      // measurement. See FusionStats::occupancy_known.
      std::snprintf(cell, sizeof(cell), "occupancy unreadable, of %u blocks",
                    s.table_blocks);
    } else {
      std::snprintf(cell, sizeof(cell), "%.1f%% of %u blocks",
                    100.0 * static_cast<double>(s.occupancy), s.table_blocks);
    }
    table_row = cell;
    // A stopped scan is a present-tense claim, so it has to stop being made
    // when the fuse loop stops running. `allocation_stop` and `occupancy` are
    // per-frame values latched into a snapshot that outlives the frame, and an
    // ARKit interruption -- a call, Control Centre, the app switcher -- stops
    // frames without stopping the display link, so the panel went on announcing
    // a full volume for the length of a phone call, about a scan that was not
    // allocating because it was not scanning.
    //
    // A second is generous against a 60 Hz capture decimated to whatever
    // `fuse_every` is, and deliberately so: this should fire on an
    // interruption, not on a slow frame.
    constexpr float kFuseStaleAfterMs = 1000.0f;
    if (s.ms_since_fuse > kFuseStaleAfterMs) {
      char note[96];
      std::snprintf(note, sizeof(note),
                    "  -- not fusing (%.1f s since a frame)",
                    static_cast<double>(s.ms_since_fuse) / 1000.0);
      table_row += note;
    } else {
      table_row += allocation_stop_note(s.allocation_stop);
    }
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

  // What the OS will actually allow, asked of the OS.
  //
  // This is the number every sizing knob in FusionConfig is really chosen
  // against -- voxel_size, max_buckets, mesh_slots -- and until now it appeared
  // nowhere the app could see: only as a figure in scanner.entitlements that
  // nothing measured. Its absence is why the run that motivated
  // increased-memory-limit could only be diagnosed after the fact, from a
  // process that had already vanished.
  //
  // Two rows, because there are two ceilings and the *lower* one is the one
  // nothing here used to print. The jetsam limit kills the process; the GPU
  // working set is roughly two thirds of it and is what the voxel grid and the
  // mesh arenas are really charged against. A footprint that reads as a
  // comfortable fraction of the jetsam limit is half again as much of the
  // working set, and reporting only the first is reassuring in exactly the
  // direction that gets a scan killed.
  //
  // The working-set row compares the *footprint* against that ceiling rather
  // than a GPU-only figure. That overstates GPU pressure, because the footprint
  // also charges host allocations Metal never sees -- which is the safe
  // direction for an instrument whose job is to warn, and it needs no second
  // allocator to agree with. `device RAM` rides along on the same row for scale
  // and is not a ceiling anyone should size against; see
  // Core/MemoryBudget.hpp.
  //
  // Assembled by concatenation, with every number adjacent to the words naming
  // it, rather than as one format string. The version this replaces bound four
  // same-typed `%.0f`/`%.1f` conversions to their arguments by position, which
  // is the one mistake -Wformat cannot see: transposing the limit and the
  // device's RAM printed a fully plausible row whose verdict was inverted, and
  // left the printed fraction disagreeing with the printed percentage. Same
  // reasoning as the phase cells above; the difference is that this actually
  // removes the class rather than relocating it into a smaller format.
  const auto mb = [](std::uint64_t bytes) {
    char cell[32];
    std::snprintf(cell, sizeof(cell), "%.0f",
                  static_cast<double>(bytes) / (1024.0 * 1024.0));
    return std::string(cell);
  };
  const auto pct = [](std::uint64_t part, std::uint64_t whole) {
    char cell[32];
    std::snprintf(cell, sizeof(cell), "%.1f",
                  whole > 0 ? 100.0 * static_cast<double>(part) /
                                  static_cast<double>(whole)
                            : 0.0);
    return std::string(cell);
  };
  std::string memory_rows;
  {
    const app::MemoryBudget budget = app::query_memory_budget();
    const std::uint64_t working_set = gpu_working_set_bytes();

    std::string held;
    if (!budget.valid) {
      // The kernel's own code, not a bare "unavailable". MemoryBudget
      // enumerates two reasons a reading can fail and they want different
      // responses; a placeholder that cannot tell them apart asks the reader to
      // guess, and this row is read when something has already gone wrong.
      held = "unavailable [task_info kr=" +
             std::to_string(budget.task_info_status) + "]";
    } else if (budget.at_limit) {
      // The kernel clamps the remainder to 0 here, so there is no ceiling to
      // derive -- deriving one anyway yields limit == footprint and prints a
      // tidy 100% under a ceiling that rose to meet the number it was
      // measuring. This is the last state before jetsam; it gets a banner, not
      // a ratio.
      held = "! " + mb(budget.footprint_bytes) +
             " MB held, no headroom left before jetsam";
    } else if (budget.limit_known) {
      held = mb(budget.footprint_bytes) + " / " + mb(budget.limit_bytes) +
             " MB jetsam (" + pct(budget.footprint_bytes, budget.limit_bytes) +
             "%)";
    } else {
      held =
          mb(budget.footprint_bytes) + " MB held, jetsam ceiling not reported";
    }
    // The high-water mark, which is the only part of this row that can survive
    // the gap between polls. See MemoryBudget::peak_footprint_bytes: a resize
    // spike lasts well under one polling interval, so the sampled footprint
    // beside it is the steady state and reads as comfortable no matter what the
    // scan just survived.
    if (budget.peak_footprint_bytes > 0) {
      held += ", peak " + mb(budget.peak_footprint_bytes) + " MB";
    }

    std::string context;
    if (working_set > 0) {
      context = budget.valid
                    ? mb(budget.footprint_bytes) + " / " + mb(working_set) +
                          " MB gpu working set (" +
                          pct(budget.footprint_bytes, working_set) + "%)"
                    : mb(working_set) + " MB gpu working set";
    }
    // Printed even when task_info refused, because it is a separate reading
    // that did not fail: suppressing the whole block on one sub-failure would
    // drop two figures that are still obtainable.
    if (budget.device_ram_bytes > 0) {
      if (!context.empty()) {
        context += ", ";
      }
      context += "device RAM " + mb(budget.device_ram_bytes) + " MB";
    }

    memory_rows = "  memory    " + held + "\n";
    if (!context.empty()) {
      memory_rows += "            " + context + "\n";
    }
  }

  // The fused stages, from the rows recon reported rather than from the host
  // scalars beside them.
  //
  // Both halves, because the gap is the finding: the host figure is wall clock
  // around a fence-blocked submit -- host record, submit, the fence stall and
  // device execution together -- while `gpu` is the dispatch alone. A stage
  // whose device share is small is not a slow kernel and will not be fixed by
  // a faster one, and on this device that distinction has been guesswork.
  //
  // But subtract the two ONLY on a row with no breakdown beneath it. The halves
  // nest differently: a host scope spans a whole call, a device span covers one
  // dispatch, so an indented `..` row's kernel ran inside its parent's host
  // span while reporting its device time on its own line. `integrate` measured
  // 18.86 ms host / 10.61 gpu with `..active set` at 6.98 / 6.39 beneath it --
  // reading the 8.25 ms difference as submit overhead sends the reader after a
  // stall that is 6.39 ms of measured kernel printed on the next line.
  // recon encodes this asymmetry deliberately; see StageMetrics::
  // kBreakdownPrefix and total_gpu_ms, which skips breakdowns in one total and
  // not the other for exactly this reason.
  //
  // Built as a block rather than as more varargs, for the reason the phase
  // cells are: same-typed floats in a positional list are the one mislabel
  // -Wformat cannot see. A row with no device span prints a dash rather than
  // 0.00 -- "not measured" and "measured, and fast" are different claims, and
  // `has_gpu` is what tells them apart.
  //
  // A sub-table in the 12-column gutter, exactly as `phases/ms` is, rather than
  // rows sitting directly in the outer table. That is what lets the label field
  // be 14 wide: recon's breakdown rows carry its `"  .."` prefix on top of
  // their own name, so `  ..active set` is 14 characters and will not fit the
  // 10 the outer gutter leaves. In a `%-9s` field it did not truncate, it
  // *shifted* -- its two figures printed five columns right of every
  // neighbouring row's, on the one row this whole change exists to surface --
  // and widening the outer table instead would move every other line on the
  // read-out to suit one library label.
  //
  // The prefix survives into the display deliberately: indentation is how the
  // breakdown says it decomposes the row above rather than standing beside it,
  // which is the distinction the paragraph above is about.
  std::string stage_rows;
  const auto stage_gutter = [&stage_rows](bool first) {
    stage_rows += first ? " " : "\n            ";
  };
  // Whether the table already carries the texture pass, which decides below
  // whether the standalone `texture` line is printed at all. See there.
  bool have_texture_stage = false;
  for (std::uint32_t i = 0; i < s.stage_count; ++i) {
    const vr::StageRow& row = s.stages[i];
    if (row.name != nullptr && std::strcmp(row.name, "texture") == 0) {
      have_texture_stage = true;
    }
    // Fixed widths for the reason the phase cells have them: this string is
    // rebuilt continuously, and a value gaining a digit would shift every
    // column to its right.
    char cell[64];
    if (row.has_gpu) {
      std::snprintf(cell, sizeof(cell), "%-14s%7.2f   gpu %7.2f", row.name,
                    row.cpu_ms, row.gpu_ms);
    } else {
      std::snprintf(cell, sizeof(cell), "%-14s%7.2f   gpu %7s", row.name,
                    row.cpu_ms, "-");
    }
    stage_gutter(i == 0);
    stage_rows += cell;
  }
  if (s.stage_count == 0) {
    // The fallback, which is what keeps `FusionConfig::measure_stages` a switch
    // rather than a regression: these two scalars are measured on every fused
    // frame either way. No `gpu` column at all, because there is genuinely no
    // measurement to put in one -- a dash there would report a row that was
    // never asked for as one that was asked for and came back empty.
    char cell[64];
    std::snprintf(cell, sizeof(cell), "%-14s%7.2f", "allocate",
                  static_cast<double>(s.allocate_ms));
    stage_gutter(true);
    stage_rows += cell;
    std::snprintf(cell, sizeof(cell), "%-14s%7.2f", "integrate",
                  static_cast<double>(s.integrate_ms));
    stage_gutter(false);
    stage_rows += cell;
  } else {
    // What the rows above are, when they are not this frame's. They publish
    // only on the fully-successful fuse path, so four early returns leave the
    // last good set standing -- the same hazard `extract_stale` and
    // `survey_stale` cover, and the reason FusionStats carries an age for them.
    //
    // Measured *against* `ms_since_fuse` rather than against a wall clock of
    // its own, because the pair is the reading and neither half means much
    // alone. An ARKit interruption stops both, and announcing stale timings
    // there would blame the fusion for a phone call -- the same fault
    // `ms_since_fuse` was added to stop this read-out making one row above. The
    // difference isolates the other case: frames arriving, none completing.
    //
    // A second of that, for the reason the fuse-liveness note uses a second --
    // generous against a 60 Hz capture, so this fires on a stage that is
    // failing rather than on a slow frame.
    constexpr float kStagesStaleAfterMs = 1000.0f;
    if (s.ms_since_stages > s.ms_since_fuse + kStagesStaleAfterMs) {
      char note[96];
      std::snprintf(note, sizeof(note), "\n            (%.1f s old)",
                    static_cast<double>(s.ms_since_stages) / 1000.0);
      stage_rows += note;
    }
    if (s.stages_truncated) {
      stage_rows += "\n            (more stages than this holds)";
    }
    // Named, because the aftermath is silent by construction: every row goes
    // host-only and every gpu cell becomes a dash, which is exactly what a
    // device with no timestamp support looks like. One is a fault, the other is
    // hardware. See FusionConfig::measure_stages.
    if (s.gpu_timing_retired) {
      stage_rows +=
          "\n            (device timing retired after a fence failure -- host "
          "only for the rest of this run)";
    }
  }

  // --- The texture pass, in one line rather than two ------------------------
  //
  // The state leads, because the duration cannot carry it: 0.0 ms is "off",
  // "skipped for a colourless frame", "refused before dispatching" and "ran and
  // cost nothing" all at once, and those want different actions. It is also the
  // only thing that distinguishes a texture pass a sibling revision refuses on
  // every remesh from one that runs and produces nothing. See app::TextureState
  // -- including its warning that "ran" is not a claim about how much of the
  // mesh came out textured, which nothing at this tier can say.
  //
  // The wall-clock span prints ONLY when the stage table does not already carry
  // a `texture` row. With measure_stages on it does, fed by recon's own
  // timestamp span around the dispatch -- while `texture_ms` wraps the whole
  // call including the transient buffer setup and the fence. Two different
  // numbers for one pass, a few lines apart under the same word, read as a
  // measurement fault and make anyone totalling the stage column double-count
  // it. Fusion's publish declines to push a second row for exactly this reason;
  // printing one unconditionally here reintroduced it at the render layer.
  // Invisible while texture was off, because both said 0.0.
  std::string texture_row = "  texture   ";
  switch (s.texture_state) {
    case app::TextureState::Off:
      texture_row += "off";
      break;
    case app::TextureState::Pending:
      texture_row += "on, no remesh yet";
      break;
    case app::TextureState::NoColor:
      texture_row += "skipped -- this frame carried no colour";
      break;
    case app::TextureState::Failed:
      texture_row += "FAILED -- reason in the errors line above";
      break;
    case app::TextureState::Ran: {
      char cell[128];
      std::snprintf(cell, sizeof(cell), "ran at %.3f m tolerance",
                    static_cast<double>(s.occlusion_threshold));
      texture_row += cell;
      if (!have_texture_stage) {
        std::snprintf(cell, sizeof(cell), ",  %.1f ms (whole call)",
                      static_cast<double>(s.texture_ms));
        texture_row += cell;
      }
      // The fuse thread's ~11 MB keyframe copy, which is in no stage row and no
      // other total -- see FusionStats::atlas_copy_ms.
      std::snprintf(cell, sizeof(cell), "\n            keyframe copy %.2f ms",
                    static_cast<double>(s.atlas_copy_ms));
      texture_row += cell;
      break;
    }
  }

  // Sized for two full library messages plus the fixed body: the error and the
  // upload lines can both be present and both carry a `Status::message()`.
  //
  // 4096 rather than 2048. The worst case measured at 1766 bytes when this was
  // sized, which is 86% of 2048 rather than the "well under half" the previous
  // note claimed, and the memory block above adds up to ~150 more. That left
  // less margin than the comment described, against a buffer whose overflow is
  // silent: snprintf's return is discarded, so a cut would not be reported.
  // Doubling it is stack memory on a thread that has megabytes of it, and it
  // restores the headroom the comment always assumed.
  //
  // Truncation, if it ever happened, could only cut the *tail*, and the tail is
  // now the phase rows and the fixed body rather than the banners or the memory
  // block -- those are at the top precisely so a clipped read-out keeps them.
  char buf[4096];
  std::snprintf(
      buf, sizeof(buf),
      // Banners directly under the header, not at the end. They were last,
      // which put the only two lines naming an actual failure at the bottom of
      // an overlay that already runs past the safe area on a landscape iPhone
      // -- so they were the first things clipped. Nothing below them is worth
      // more screen than they are.
      // `%s` for the version rather than `v%u`: the measurement mode publishes
      // nothing, so the counter stays 0 while remeshes climb, and "v0" beside a
      // five-figure remesh count reads as a wedged publish path in the one copy
      // of the read-out that has a device history. Built above.
      "fused %llu / remesh %llu  %s%s%s%s\n"
      // The measurement banner, directly under the counts it reframes. Empty
      // string on the normal path.
      "%s"
      // Directly under the banners, and above every capacity row, because this
      // is the only quantity on the read-out whose ceiling is enforced by
      // something outside the app and the only one whose breach takes the
      // process with it. It sat second-from-last, which is the position the
      // comment above `dirty_rows` warns about in as many words: the
      // statusLabel is bottom-constrained and clips silently, and at 12 pt this
      // body plus the ARKit rows below it overruns a landscape iPhone by enough
      // that the tail is simply not on screen. With an `! errors` banner
      // present -- the OOM-adjacent case where the reading matters most -- the
      // old position was off the bottom of every iPhone. Nothing below it is
      // worth more screen than it is, which is the same argument that moved the
      // banners up.
      //
      // Carries its own newlines and may be two rows, so it is spliced in as a
      // block rather than given a row of its own here.
      "%s"
      "  mesh      %u verts / %u tris\n"
      // The fuse's own stages, host and device. A labelled group with its rows
      // in the 12-column gutter, like `phases/ms` below and for the same two
      // reasons: its widest label does not fit the outer table, and same-typed
      // doubles in a positional vararg list are the one mislabel -Wformat
      // cannot see. Built above, where the branches are.
      "  stages/ms%s\n"
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
      // Occupancy from the map itself, not from the extract's block count:
      // that number refreshes only on a successful remesh, and this row is
      // where someone looks to see whether a scan is still taking geometry in.
      // The suffix is the whole point -- 85% is a threshold, and a threshold a
      // reader has to already know is one they will miss. Built above, because
      // the figure has a branch in it and the suffix has four cases.
      "  table     %s%s\n"
      // Built above: the state decides the shape of this line, and whether the
      // call span appears at all depends on the stage table beside it.
      "%s",
      static_cast<unsigned long long>(s.frames_fused),
      static_cast<unsigned long long>(s.remeshes), version_cell.c_str(),
      errors.c_str(), upload.c_str(), memory_warnings.c_str(),
      bench_row.c_str(), memory_rows.c_str(), s.vertices, s.triangles,
      stage_rows.c_str(), s.extract_ms, extract_note.c_str(),
      phase_rows.c_str(), s.extract.triangle_capacity,
      s.extract.triangle_capacity > 0
          ? 100.0 * static_cast<double>(s.triangles) /
                static_cast<double>(s.extract.triangle_capacity)
          : 0.0,
      static_cast<double>(s.extract.arena_bytes) / (1024.0 * 1024.0),
      s.mesh_slots, s.extract.active_blocks, table_row.c_str(),
      dirty_rows.c_str(), texture_row.c_str());
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

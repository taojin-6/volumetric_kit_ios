// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "Readout.hpp"

#import "AllocationStopDisplay.hpp"
#import "BridgeStrings.hpp"
#import "StatTone.hpp"

#include <os/log.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "volumetric_kit/recon/core/stage_metrics.hpp"

namespace app = volumetric_kit::ios_app;

// The stage-row value type. At file scope, not inside the renderer's
// @implementation -- Objective-C has no nested implementations.
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
    // Not `stats.allocation_stop` itself: the cause is a latch Fusion never
    // clears, and this snapshot feeds Dashboard.swift's persistent banner --
    // which went on telling the user to abandon the scan and restart at a
    // coarser voxel size for the length of a phone call, about a volume that
    // was not full. The log's `table` row was guarded and these were not; the
    // rule is now in one place, in Core/AllocationStop.hpp.
    const app::AllocationStop stop = app::reportable_allocation_stop(
        stats.allocation_stop, stats.ms_since_fuse);
    _allocationStop = app::allocation_stop_value(stop);
    _allocationStopReason =
        stop == app::AllocationStop::None
            ? nil
            : app::to_ns_string(app::allocation_stop_text(stop).advice);

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
    return app::to_ns_string(stack);
  }
  std::vector<char> heap(static_cast<std::size_t>(needed) + 1);
  std::vsnprintf(heap.data(), heap.size(), format, args);
  va_end(args);
  return app::to_ns_string(heap.data());
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

VolumetricStatTone panel_tone(double fraction, double warn, double crit) {
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
    _allocationStop = app::allocation_stop_value(sample.allocation_stop);
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
    _name = app::to_ns_string(row.name != nullptr ? row.name : "");
    _cpuMs = row.cpu_ms;
    _gpuMs = row.gpu_ms;
    _hasGpu = row.has_gpu ? YES : NO;
  }
  return self;
}
@end

namespace volumetric_kit::ios_app {

namespace {

/// One row as a console line: a tone mark, a fixed-width label, the value.
///
/// Fixed width because this string is rebuilt continuously -- without it a
/// value gaining a digit shifts every label to its right, and columns that move
/// cannot be read. The mark is what carries the panel's colour into a stream
/// that has none: a reader watching the console sees the same alarm the panel
/// paints, rather than having to know which figures matter.
std::string text_row(VolumetricStatRow* row) {
  const char* mark = "   ";
  switch (row.tone) {
    case VolumetricStatToneCritical:
      mark = " ! ";
      break;
    case VolumetricStatToneWarn:
      mark = " ~ ";
      break;
    case VolumetricStatToneGood:
    case VolumetricStatToneNeutral:
      mark = "   ";
      break;
  }
  char line[512];
  std::snprintf(line, sizeof(line), "%s%-18s%s", mark, row.label.UTF8String,
                row.value.UTF8String);
  return std::string(line) + "\n";
}

/// The panel, as the log's text.
///
/// **This is the collapse.** The summary used to be a second exhaustive
/// formatter over `FusionStats` -- its own switches, its own thresholds, its
/// own ~19-conversion format string -- sitting beside the one that builds these
/// sections. Two formatters over one set of figures can disagree, and these two
/// had: the panel divided a live figure one way and the log another. There is
/// now one place a figure is decided and one place it is worded, and the log is
/// a rendering of the panel rather than a parallel account of the same scan.
std::string render_text(NSArray<VolumetricStatSection*>* sections,
                        NSArray<VolumetricStageRow*>* stages) {
  std::string out;
  for (VolumetricStatSection* section in sections) {
    out += section.title.UTF8String;
    out += "\n";
    for (VolumetricStatRow* row in section.rows) {
      out += text_row(row);
    }
  }
  // The stage grid, which is not a section: it is a per-stage host/device split
  // the panel charts rather than lists, and it has no label/value shape to
  // borrow. Rendered here so the log still carries it.
  if (stages.count > 0) {
    out += "Stages\n";
    for (VolumetricStageRow* stage in stages) {
      char line[512];
      if (stage.hasGpu) {
        std::snprintf(line, sizeof(line), "   %-18s%6.2f ms cpu  %6.2f ms gpu",
                      stage.name.UTF8String, stage.cpuMs, stage.gpuMs);
      } else {
        // No device span was measured for this row. Deliberately not printed as
        // a zero: see VolumetricStageRow, where `hasGpu` is documented as a
        // measurement fact and explicitly not a capability report.
        std::snprintf(line, sizeof(line), "   %-18s%6.2f ms cpu",
                      stage.name.UTF8String, stage.cpuMs);
      }
      out += line;
      out += "\n";
    }
  }
  return out;
}

}  // namespace

NSArray<VolumetricStageRow*>* stage_rows(const app::FusionStats& s) {
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

NSArray<VolumetricStatSection*>* stat_sections(const ReadoutInputs& in) {
  // Named locally so the builders below read as they always have; the change
  // is where the values come from, not what they are.
  const app::FusionStats& s = in.stats;
  const app::MemoryBudget& budget = in.budget;
  const std::uint64_t workingSet = in.gpu_working_set_bytes;
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
    if (in.mesh_upload_failures > 0) {
      // With the reason, not just the count. The message was reaching only the
      // text summary, so on this panel -- the primary read-out -- a mesh that
      // cannot be bound as geometry was an unexplained red row. It is also
      // latched and permanent, which the row now says outright rather than
      // leaving to be inferred from a count that never moves again.
      add(r, @"mesh upload",
          in.mesh_upload_error.empty()
              ? fmt("%llu failed", (unsigned long long)in.mesh_upload_failures)
              : fmt("%llu failed: %s",
                    (unsigned long long)in.mesh_upload_failures,
                    in.mesh_upload_error.c_str()),
          VolumetricStatToneCritical);
    }
    if (in.atlas_failures > 0) {
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
          in.atlas_error.empty()
              ? fmt("%llu failed", (unsigned long long)in.atlas_failures)
              : fmt("%llu failed: %s", (unsigned long long)in.atlas_failures,
                    in.atlas_error.c_str()),
          VolumetricStatToneWarn);
    }
    if (in.memory_warnings > 0) {
      // The footprint sampled *at* the warning, not now: this is the one
      // reading taken when the OS said it mattered, and the next poll is up to
      // half a second later -- long enough for the allocation that provoked it
      // to have been freed again.
      add(r, @"memory warning",
          in.memory_warning_footprint_bytes > 0
              ? fmt("x%llu  (last at %.0f MB held)",
                    (unsigned long long)in.memory_warnings,
                    in.memory_warning_footprint_bytes / mb)
              : fmt("x%llu", (unsigned long long)in.memory_warnings),
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
        panel_tone(fill, 0.9, 0.98));
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
          panel_tone(s.occupancy, 0.7, 0.85));
    } else {
      add(r, @"occupied", fmt("unreadable  (of %u blocks)", s.table_blocks),
          VolumetricStatToneWarn);
    }
    // The cause, not one of the four causes. See `allocation_stop_text`: the
    // advice for a full volume is actively wrong for the other two, and this
    // row used to assert it for all of them.
    // Through reportable_allocation_stop like the banner and the `table` row:
    // ALLOCATION STOPPED is a present-tense claim, and the latch behind it
    // outlives the fuse loop that set it.
    if (const app::AllocationStop stop =
            app::reportable_allocation_stop(s.allocation_stop, s.ms_since_fuse);
        stop != app::AllocationStop::None) {
      add(r, @"state",
          fmt("ALLOCATION STOPPED — %s",
              app::allocation_stop_text(stop).headline),
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

  // --- Extract phases -------------------------------------------------------
  //
  // Added when the log and the panel were collapsed onto one model. The text
  // summary carried this grid and the panel did not, so a collapse that simply
  // rendered the sections would have dropped it -- and the right answer to "the
  // text has something the model lacks" is to put it in the model, not to keep
  // a second formatter alive for it. The panel gains the breakdown as a result.
  //
  // `other` is the residual, printed rather than left implicit: these phases do
  // not sum to extract_ms and never did. recon's spans open after the slot
  // claim and close before the O(active_blocks) teardown of the neighbour
  // table, so the gap grows with the scan -- the one direction in which an
  // unlabelled remainder would be misread as rounding.
  {
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
        {"other", std::max(0.0, static_cast<double>(s.extract_ms) -
                                    s.extract.total_ms())},
    };
    NSMutableArray<VolumetricStatRow*>* r = [NSMutableArray array];
    for (const PhaseCell& cell : phases) {
      // Two decimals because three of these sat under 0.05 ms on the measured
      // device and at one decimal were indistinguishable from "not measured" --
      // including `sizing`, the phase that prices a refit.
      add(r, [NSString stringWithUTF8String:cell.label],
          fmt("%.2f ms", cell.ms));
    }
    [out addObject:make_section(@"Extract phases", r)];
  }

  return out;
}

NSString* fusion_summary(const ReadoutInputs& in) {
  const std::string text = render_text(stat_sections(in), stage_rows(in.stats));

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
      // truncates the rest before it is written to disk. FrameTrace::dump
      // splits for this reason; handing the whole read-out over while citing
      // that as precedent would silently drop the tail, and the tail is where
      // this read-out's numbers live.
      std::string::size_type start = 0;
      while (start < text.size()) {
        const std::string::size_type end = text.find('\n', start);
        const std::string line = text.substr(
            start, end == std::string::npos ? std::string::npos : end - start);
        if (!line.empty()) {
          os_log(OS_LOG_DEFAULT, "vk-scan: %{public}s", line.c_str());
        }
        if (end == std::string::npos) {
          break;
        }
        start = end + 1;
      }
    }
  }
  // Through the nil-guarding helper, like every other string property here: the
  // text carries library messages, and `fusionSummary` is imported as a
  // non-optional Swift String that traps on the nil `stringWithUTF8String:`
  // returns for invalid UTF-8.
  return to_ns_string(text);
}

VolumetricDashboardSnapshot* dashboard_snapshot(
    const ReadoutInputs& in, NSArray<VolumetricFrameSample*>* history) {
  return [[VolumetricDashboardSnapshot alloc]
      initWithStages:stage_rows(in.stats)
            sections:stat_sections(in)
             history:history
               stats:in.stats
              budget:in.budget
          workingSet:in.gpu_working_set_bytes];
}

}  // namespace volumetric_kit::ios_app

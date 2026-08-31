// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file Readout.hpp
/// @brief The scan read-out: what the panel shows and what the log prints.
///
/// One model, two renderings. `stat_sections` builds the grouped figures the
/// SwiftUI panel draws; `fusion_summary` is those same sections joined into
/// text -- which is what VolumetricRenderer.h has always said it was, and is
/// now what it does. Before this file they were two independent formatters over
/// one `FusionStats`, roughly 1200 lines of parallel switch and format, and
/// they had already drifted about how a live figure is divided.
///
/// Objective-C++ deliberately: everything here returns objects Swift holds, so
/// there is no pure half to lift into Core/. What *is* pure -- the stop causes,
/// the tone thresholds -- already lives there and is called from here.

#import <Foundation/Foundation.h>

#import "VolumetricRenderer.h"

#import "Fusion.hpp"
#import "MemoryQuery.hpp"

#include <cstdint>
#include <string>

namespace volumetric_kit::ios_app {

/// @brief Everything the read-out reads, sampled once.
///
/// A snapshot rather than a renderer reference, and that is the point: the
/// figures come from three sources the fuse thread moves independently -- the
/// fusion's stats, the kernel's memory ledger, and counters the render and UI
/// threads own -- so a builder that fetched each one itself would compose a
/// panel from several instants. `VolumetricDashboardSnapshot` exists because
/// assembling the panel from the individual properties cost five `FusionStats`
/// copies and three `task_info` traps; filling one of these once makes that
/// arithmetic impossible rather than merely discouraged.
///
/// @warning Holds **references** to @ref stats and @ref budget. It is built,
///          used and dropped inside one call; nothing here outlives the
///          statement that made it.
struct ReadoutInputs {
  /// The two borrowed readings are constructor arguments and the rest are
  /// assigned by name, deliberately. Aggregate-initialising all nine
  /// positionally is the mistake `VolumetricDashboardSnapshot` already warns
  /// about -- same-typed counters threaded in the wrong order, which compiles
  /// and reports the atlas's failures as the mesh's.
  ReadoutInputs(const FusionStats& s, const MemoryBudget& b)
      : stats(s), budget(b) {}

  /// The fusion's own figures, one copy taken by the caller.
  const FusionStats& stats;
  /// The kernel's memory reading, from one `task_info`.
  const MemoryBudget& budget;
  /// Metal's recommended working-set ceiling, or 0 when unavailable.
  std::uint64_t gpu_working_set_bytes = 0;

  // --- Counters that are not in FusionStats --------------------------------
  // The upload is the render thread's stage and the memory warning arrives on
  // the UI thread, so neither is in the fusion's snapshot. That is exactly why
  // the panel was once missing them.

  /// Latched: a published mesh that cannot be bound as geometry.
  std::uint64_t mesh_upload_failures = 0;
  std::string mesh_upload_error;
  /// Not latched -- a ring refusal is degraded rendering, not a dead one -- so
  /// this counts at frame rate and keeps its own reason. Sharing one channel
  /// with the mesh fault let a transient atlas refusal erase a permanent
  /// mesh fault's message on the very next frame.
  std::uint64_t atlas_failures = 0;
  std::string atlas_error;
  /// The OS's own warning, and what the process held when it last fired.
  std::uint64_t memory_warnings = 0;
  std::uint64_t memory_warning_footprint_bytes = 0;
};

/// @brief The last fused frame's stages, for charting. Empty when nothing has
///        fused yet, or for the whole run when `measure_stages` is off.
NSArray<VolumetricStageRow*>* stage_rows(const FusionStats& s);

/// @brief The grouped figures the panel draws: Alerts, Scene, Block table,
///        Dirty, Memory.
NSArray<VolumetricStatSection*>* stat_sections(const ReadoutInputs& in);

/// @brief The same sections, joined into the log's text.
///
/// Rendered from @ref stat_sections rather than formatted a second time, so the
/// panel and the log cannot disagree about a figure. Also mirrors itself to
/// os_log, throttled to one copy every two seconds and split per line -- see
/// the note at the split, which is a size limit and not a style choice.
NSString* fusion_summary(const ReadoutInputs& in);

/// @brief The whole panel from one fill of @ref ReadoutInputs.
VolumetricDashboardSnapshot* dashboard_snapshot(
    const ReadoutInputs& in, NSArray<VolumetricFrameSample*>* history);

}  // namespace volumetric_kit::ios_app

// The value types' private initializers, shared with VolumetricRenderer.mm --
// `frameHistory` builds samples there because it needs the Fusion object rather
// than its stats snapshot.
@interface VolumetricStageRow ()
- (instancetype)initWithRow:(const volumetric_kit::recon::StageRow&)row;
@end

@interface VolumetricFrameSample ()
- (instancetype)initWithSample:
    (const volumetric_kit::ios_app::FrameSample&)sample;
@end

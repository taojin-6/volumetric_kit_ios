// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// The live scan dashboard: everything the read-out carries, visible at once.
///
/// SwiftUI rather than an ImGui overlay, and not out of preference. ImGui
/// renders *inside* the Vulkan pass, so on a device where meshing already costs
/// over 100 ms the overlay competes with the reconstruction for the same queue
/// and lands in every GPU capture. SwiftUI composites on the UI layer -- the OS
/// draws it and the recon pass pays nothing. gfx also ships only the Vulkan
/// backend by design (the platform half is the consumer's), so ImGui here would
/// have meant writing a UIKit input backend first.
///
/// **Nothing is hidden behind a tap.** An earlier cut moved the timings into a
/// sheet on the theory that a scanner is a consumer product; it is a tool, and
/// hiding half of what it measures behind a gesture makes it a worse one. Every
/// figure the text read-out carried is on screen. What changed is that it is
/// *grouped*, and that the figures a reader has to act on carry visual state
/// instead of waiting to be noticed in a column of digits.
///
/// **Each figure appears once.** The panel grew a card per source rather than a
/// card per question, and the same quantity ended up on four of them: the
/// triangle count was in the headline, in `Fusion`, again in `Extract` as
/// `emitted`, and a fourth time as the numerator of `Arena`'s fill. Sections are
/// grouped by what a reader is trying to find out now -- the geometry, the hash
/// map, the survey, the memory position -- and a figure lives on exactly one of
/// them.
///
/// The grouping and the figures come from the bridge (`statSections`); the
/// *order* they are read in and the gauges drawn over them come from this file.
/// That split is deliberate: the bridge knows which numbers are coherent with
/// each other, and only a view knows what fits on a screen. It *is* now a
/// guarantee against drift, which the previous note here was right to say it
/// was not: the log used to format its own text from the same stats and the two
/// had in fact drifted -- the panel divided live occupancy by a remesh-cadence
/// capacity while the transcript, three hundred lines away in the same file,
/// divided it by the right one. The transcript is rendered from these sections
/// now (`VolumetricDashboardSnapshot.summary`), so there is one place a figure
/// is decided and one place it is worded.
///
/// Which leaves one asymmetry worth knowing about: the log has no bars, so the
/// bridge publishes every figure and marks the ones a gauge here draws. This
/// file skips those rows; the transcript prints them. See
/// `StatItem.drawnAsGauge`.

import Charts
import SwiftUI

// MARK: - Model

/// One stage's split, mirrored out of `VolumetricStageRow`.
struct StageBar: Identifiable {
  let id: String
  let name: String
  let hostMs: Double
  let deviceMs: Double
  let hasGPU: Bool
  /// recon marks a breakdown of the row above it by indenting the label.
  var isBreakdown: Bool { name.hasPrefix(" ") }
  var trimmed: String { name.trimmingCharacters(in: .whitespaces) }
}

/// One fused frame, mirrored out of `VolumetricFrameSample`.
struct FrameSample: Identifiable {
  let id: UInt64
  let hostMs: Double
  let deviceMs: Double
  /// The meshing cost, which is **not** in either figure above.
  ///
  /// `hostMs`/`deviceMs` are `StageMetrics` totals, and the extract does not
  /// report there -- it reports through `ExtractTimings`. It is also the
  /// dominant cost of a remesh frame (~132.7 ms against ~20 ms of fuse on this
  /// device's own measurement) and `remesh_every` defaults to 1, so leaving it
  /// out made the chart plot roughly an eighth of the frame it claimed to
  /// show -- and hid the exact spike the ring was built to catch.
  let extractMs: Double
  /// Whether `deviceMs` is a measurement or an absence. `total_gpu_ms` skips
  /// rows without a span, so a queue family that reports no timestamps sums to
  /// the same 0.0 as a frame that dispatched nothing.
  let deviceTimingValid: Bool
  /// The fuse half. The device span sits *inside* the host span, so this is the
  /// larger of the two and never their sum.
  var fuseMs: Double { max(hostMs, deviceMs) }
  /// The whole frame. The extract runs *after* the fuse and is measured
  /// separately, so it adds rather than overlapping.
  var totalMs: Double { fuseMs + extractMs }
}

/// A labelled figure, mirrored out of `VolumetricStatRow`.
struct StatItem: Identifiable {
  let id = UUID()
  let label: String
  let value: String
  let tone: VolumetricStatTone
  /// Whether a gauge on this card already draws this figure, in which case the
  /// panel does not also list it as a row. Carried from the bridge rather than
  /// decided here: the bridge publishes every figure it has, because the log
  /// renders the same model and has no gauges. See `VolumetricStatRow`.
  let drawnAsGauge: Bool
}

/// A named group, mirrored out of `VolumetricStatSection`.
struct StatGroup: Identifiable {
  let id: String
  let items: [StatItem]
}

/// What the dashboard draws, refreshed at the display cadence.
///
/// Sampling and display are different rates on purpose: `history` is sampled
/// per *fused frame* on the fusion thread, because that thread runs at its own
/// pace and a poll from here -- at any frequency -- sees only whatever was last
/// published. Display stays slow because a number flickering at 120 Hz is
/// unreadable no matter how cheap it is to draw.
@MainActor
final class DashboardModel: ObservableObject {
  @Published var scanning = false
  @Published var elapsed: TimeInterval = 0
  @Published var fps: Double = 0

  @Published var triangles: Int = 0
  /// Beside `triangles` rather than derived from it: the extract emits an
  /// indexed mesh, so the ratio is a property of the surface being scanned.
  @Published var vertices: Int = 0
  @Published var occupancy: Double = 0
  /// Whether `occupancy` is a reading rather than a fallback. The fusion forces
  /// it to 1.0 when `load_factor` fails so the guard refuses, and drawing that
  /// as a full meter reports a full volume on one that may be nearly empty.
  @Published var occupancyKnown = true
  /// Why new geometry stopped going in, phrased as what to do about it, or nil
  /// when it has not. Carries the *cause* -- the advice for a full volume is
  /// actively wrong for the other two the fusion distinguishes.
  @Published var allocationStopReason: String?

  var allocationStopped: Bool { allocationStopReason != nil }

  @Published var trackingText = ""
  @Published var trackingHealthy = true
  @Published var framesIn = 0
  @Published var framesDropped = 0

  /// The memory position, as figures rather than as sentences.
  ///
  /// These were published and drawn nowhere. Two of them existed on this model
  /// through the whole life of the panel while the card beside them rendered
  /// pre-formatted text from the bridge, so the app paid for the reading every
  /// tick and showed a gauge of it never. They are what the two meters are
  /// filled from now.
  ///
  /// `limit` and `workingSet` are two different ceilings and the *smaller* one
  /// binds -- on this hardware that is the working set, which is why both are
  /// drawn rather than whichever is larger. Zero means "not known", which is
  /// never zero bytes: `valid` and `atLimit` are what say a bar must not be
  /// drawn at all.
  /// There is no `atLimit` here, deliberately. The bridge already folds it into
  /// `limit` -- which it publishes as 0 both when the kernel cannot report a
  /// ceiling and when the process is at it -- so a second flag beside these is
  /// a duplicate test that can only be got wrong, and was: gating the *whole*
  /// card on it blanked the working-set bar and the peak in the one window
  /// before jetsam they exist for. `limit > 0` is the entire question.
  @Published var memoryUsedBytes: UInt64 = 0
  @Published var memoryLimitBytes: UInt64 = 0
  @Published var memoryWorkingSetBytes: UInt64 = 0
  @Published var memoryPeakBytes: UInt64 = 0
  @Published var memoryValid = true

  /// Triangles the last extract planned room for, and the fill against it.
  @Published var triangleCapacity: Int = 0
  /// Whether `triangleCapacity` -- and so the arena fill drawn from it -- comes
  /// from a remesh older than this frame. Both halves of that ratio are stamped
  /// only on a successful extract, so under a persistent failure the bar freezes
  /// at whatever it last read while the surface it claims to describe keeps
  /// growing.
  @Published var extractStale = false

  @Published var stages: [StageBar] = []
  /// The one cost of a fused frame that no stage bar covers.
  ///
  /// One, not two. The extract's unaccounted remainder used to sit beside this
  /// and did not belong: recon publishes it as the `"  ..other"` row, so it is
  /// already a bar, and listing it here drew it twice and then said it was in
  /// neither. The keyframe copy really is outside every span there is.
  @Published var atlasCopyMs: Double = 0
  /// Fused frames so far. What tells an empty `stages` that no frame has
  /// completed yet from one whose measurement is switched off.
  @Published var framesFused: UInt64 = 0
  /// How old the bars are, what to measure that against, and the two ways they
  /// can be less than they seem.
  @Published var msSinceStages: Double = 0
  /// Never compared to a threshold on its own -- see `staleness`.
  @Published var msSinceFuse: Double = 0
  /// Whether the stage rows have fallen behind the fuse loop. Decided by the
  /// bridge against `kFuseStaleAfterMs`, not re-derived here from
  /// `msSinceStages` and `msSinceFuse` -- that comparison used to be written
  /// out in this file against a literal `1000`, a third copy of a rule the log
  /// and the panel print side by side.
  @Published var stagesStale = false
  @Published var stagesTruncated = false
  @Published var gpuTimingRetired = false

  @Published var history: [FrameSample] = []
  @Published var groups: [StatGroup] = []
  /// True prose, so these stay text. Split three ways because they answer three
  /// questions and were concatenated into one block: what hardware this is,
  /// what the renderer is doing with it, and what ARKit is feeding it.
  @Published var deviceLines: [String] = []
  @Published var renderLines: [String] = []
  @Published var captureLines: [String] = []

  /// Whether the arena fill gauge has a denominator to divide by. Before the
  /// first successful extract the capacity is 0, and a bar drawn from that
  /// reports a full arena on one that holds nothing.
  var arenaFillKnown: Bool { triangleCapacity > 0 }
  var arenaFill: Double {
    triangleCapacity > 0 ? Double(triangles) / Double(triangleCapacity) : 0
  }

  /// A failure worth interrupting for. Distinct from the volume being full,
  /// which is the documented trade working and gets a calmer banner.
  @Published var failure: String?

  var dropFraction: Double {
    framesIn > 0 ? Double(framesDropped) / Double(framesIn) : 0
  }
}

// MARK: - The dashboard

struct DashboardView: View {
  @ObservedObject var model: DashboardModel
  /// Told to UIKit rather than kept here alone: the host's height constraint
  /// has to follow, because a ScrollView has no intrinsic height and would
  /// otherwise keep its expanded size around a collapsed body.
  let onExpandedChange: (Bool) -> Void

  @AppStorage("dashboardExpanded") private var expanded = true

  /// Adaptive rather than a fixed column count: the same panel has to work on
  /// a landscape iPad, where six groups fit across, and a portrait phone, where
  /// one does. A hardcoded grid would be right on exactly one of them.
  private let columns = [GridItem(.adaptive(minimum: 290), spacing: 12)]

  /// The tiers every gauge below draws against, from the bridge.
  ///
  /// Read from `VolumetricRenderer` rather than written out here, because a
  /// gauge is not a second view of a row -- `drawnAsGauge` suppresses the row
  /// wherever a bar exists, so on those figures these numbers are the only
  /// thing deciding what colour a reader sees, and the bridge has already toned
  /// the matching row with them. Four literal pairs used to sit in this file,
  /// mirroring four in `Readout.mm`; the arena pair had already drifted once.
  ///
  /// Static because they are compile-time constants on the far side too: this
  /// is a value read once, not a per-tick bridge call.
  private static let thresholds = VolumetricRenderer.gaugeThresholds

  var body: some View {
    VStack(alignment: .leading, spacing: 0) {
      // Always visible, collapsed or not: the state line is the one thing worth
      // seeing without deciding to look.
      headline.padding(12)

      // OUTSIDE the collapse, deliberately. `expanded` is @AppStorage, so it
      // persists across launches: with the banners inside it, one collapse
      // meant a later run could lose the renderer, set `failure`, and show a
      // headline reading "Paused 00:00 / 0.00 M tris" with nothing red on
      // screen and no way to learn why short of expanding a panel the reader
      // has no reason to suspect. A banner that a saved preference can hide is
      // not a banner. Same for the volume stop, whose only other collapsed-state
      // signal is the occupancy meter.
      if model.allocationStopReason != nil || model.failure != nil {
        VStack(alignment: .leading, spacing: 8) {
          if let reason = model.allocationStopReason { volumeStopped(reason) }
          if let failure = model.failure { failureBanner(failure) }
        }
        .padding(.horizontal, 12)
        .padding(.bottom, 12)
      }

      if expanded {
        ScrollView {
          VStack(alignment: .leading, spacing: 10) {
            LazyVGrid(columns: columns, alignment: .leading, spacing: 10) {
              Card("Timeline") { timeline }
              // The stage bars, built here from `stages` and from no section.
              // The bridge used to publish the same figures twice -- as bars
              // and as a `Pipeline` section this loop then had to recognise by
              // title and throw away. One source, one card, no title match.
              Card("Latency") { latency }
              // Sections in the order a reader works through them, not in the
              // order the bridge appends them. See `sectionOrder`.
              ForEach(orderedGroups) { group in
                Card(group.id) {
                  VStack(alignment: .leading, spacing: 7) {
                    gauges(for: group.id)
                    rows(group.items)
                  }
                }
              }
              if !model.deviceLines.isEmpty {
                Card("Device") { lines(model.deviceLines) }
              }
              if !model.renderLines.isEmpty {
                Card("Render") { lines(model.renderLines) }
              }
              if !model.captureLines.isEmpty {
                Card("Capture") { lines(model.captureLines) }
              }
            }
          }
          .padding(.horizontal, 12)
          .padding(.bottom, 12)
        }
      }
    }
    .onAppear { onExpandedChange(expanded) }
    .onChange(of: expanded) { onExpandedChange($0) }
    // Both non-interactive, deliberately. The panel sits over the
    // reconstruction and the orbit/pan/zoom recognizers live on the view
    // underneath; a full-bleed background that hit-tests claims every touch in
    // the panel's rectangle whether or not anything is there to receive it,
    // which is what killed the camera across the top half of the screen. With
    // these transparent to touches, PassthroughContainer can ask the subtree
    // what it actually wants and let the rest through.
    .background(
      RoundedRectangle(cornerRadius: 16)
        .fill(.ultraThinMaterial)
        .allowsHitTesting(false)
    )
    .overlay(
      RoundedRectangle(cornerRadius: 16)
        .strokeBorder(Color.white.opacity(0.10))
        .allowsHitTesting(false)
    )
  }

  // The one line that has to be readable without looking: capturing or not,
  // how much surface, how full the volume is.
  //
  // Two layouts, because one does not fit. Laid out end to end at default text
  // size these pieces want roughly 660 pt -- the status pair, a 34 pt triangle
  // count, the meter, and a tracking chip whose longest real value is "limited
  // (insufficient features)" -- against the ~373 pt a portrait iPhone offers.
  // SwiftUI resolves that by compressing the most compressible child, and the
  // meter was it: a GeometryReader accepts a zero-width proposal, so the one
  // element carrying the 85% threshold silently became 0 pt wide while the
  // labels merely truncated. `ViewThatFits` picks the stacked layout instead,
  // and the meter now declares a minimum width so the wide variant honestly
  // reports that it does not fit.
  private var headline: some View {
    ViewThatFits(in: .horizontal) {
      HStack(alignment: .center, spacing: 14) {
        status
        Divider().frame(height: 22)
        triangleCount
        occupancy.frame(maxWidth: 150)
        Spacer(minLength: 0)
        HStack(spacing: 6) {
          chips
          expandButton
        }
      }

      VStack(alignment: .leading, spacing: 8) {
        HStack(alignment: .center, spacing: 12) {
          status
          Spacer(minLength: 0)
          expandButton
        }
        HStack(alignment: .center, spacing: 12) {
          triangleCount
          Spacer(minLength: 0)
          chips
        }
        occupancy
      }
    }
    .padding(.horizontal, 4)
  }

  private var status: some View {
    HStack(spacing: 7) {
      Circle()
        .fill(model.scanning ? Color.red : Color.secondary)
        .frame(width: 8, height: 8)
      Text(model.scanning ? "Scanning" : "Paused")
        .font(.title3.weight(.semibold))
        .fixedSize()
      Text(Self.clock(model.elapsed))
        .font(.title3.monospacedDigit())
        .foregroundStyle(.secondary)
        .fixedSize()
    }
  }

  private var triangleCount: some View {
    HStack(alignment: .firstTextBaseline, spacing: 5) {
      Text(Self.millions(model.triangles))
        .font(.system(size: 34, weight: .bold, design: .rounded))
        .monospacedDigit()
      Text("M tris").font(.footnote).foregroundStyle(.secondary)
    }
    .fixedSize()
  }

  private var occupancy: some View {
    VStack(alignment: .leading, spacing: 3) {
      // The same two tiers the bridge's `occupied` row is toned on, so the
      // headline and the Block table card cannot disagree about the same
      // fraction. The tick stays on the 85% line -- the one that means act.
      Meter(
        fraction: model.occupancy, warn: Self.thresholds.occupancy.warn,
        critical: Self.thresholds.occupancy.critical,
        known: model.occupancyKnown)
      // The unknown case is a *refusal to report*, not a reading of zero. The
      // fusion forces occupancy to 1.0 when load_factor fails so its guard
      // refuses; printing that as "volume 100% full" is a full volume claimed
      // on a volume nobody measured.
      Text(
        model.occupancyKnown
          ? "volume \(Int(model.occupancy * 100))% full"
          : "volume unreadable"
      )
      .font(.footnote).foregroundStyle(.secondary)
      .fixedSize()
    }
  }

  @ViewBuilder private var chips: some View {
    Chip(model.trackingText, tone: model.trackingHealthy ? .good : .warn)
    Chip("\(Int(model.fps)) fps", tone: .neutral)
    if model.dropFraction > 0.2 {
      Chip("\(Int(model.dropFraction * 100))% dropped", tone: .warn)
    }
  }

  private var expandButton: some View {
    Button {
      withAnimation(.easeInOut(duration: 0.18)) { expanded.toggle() }
    } label: {
      Image(
        systemName: expanded
          ? "chevron.up.circle.fill"
          : "chevron.down.circle.fill"
      )
      .font(.title3)
      .symbolRenderingMode(.hierarchical)
    }
    .buttonStyle(.plain)
    .foregroundStyle(.secondary)
    .accessibilityLabel(expanded ? "Hide details" : "Show details")
  }

  // Stated as an instruction, and the instruction comes from the bridge because
  // it depends on the cause. A full volume wants coarser voxels; an unreadable
  // occupancy and a dropped-block failure are not full volumes at all, and the
  // advice for one sends the reader after a limit they have not reached.
  private func volumeStopped(_ advice: String) -> some View {
    Banner(tint: .orange, icon: "exclamationmark.triangle.fill") {
      Text("New geometry is not being added.")
        .font(.footnote.weight(.semibold))
      Text(advice).font(.footnote).foregroundStyle(.secondary)
    }
  }

  private func failureBanner(_ text: String) -> some View {
    Banner(tint: .red, icon: "xmark.octagon.fill") {
      Text(text).font(.caption).foregroundStyle(.red)
    }
  }

  private var timeline: some View {
    VStack(alignment: .leading, spacing: 5) {
      if model.history.count > 1 {
        FrameChart(samples: model.history)
        HStack {
          Text("\(model.history.count) fused frames")
          Spacer()
          if let peak = model.history.map(\.totalMs).max() {
            Text(String(format: "peak %.0f ms", peak)).monospacedDigit()
          }
        }
        .font(.footnote).foregroundStyle(.secondary)
      } else {
        Text("collecting…").font(.footnote).foregroundStyle(.secondary)
      }
    }
  }

  /// The end-to-end breakdown: every stage recon reported, then what it did not
  /// report, then how much of either to believe.
  ///
  /// The three parts are separate because they are separately wrong when they
  /// are wrong. The bars are a measurement; the rows beneath them are costs
  /// that sit inside no bar and would make the column silently under-report the
  /// frame; the markers say the whole card describes an older frame than the
  /// one on screen.
  private var latency: some View {
    VStack(alignment: .leading, spacing: 5) {
      if model.stages.isEmpty {
        // Which of the two, because they want different responses and a zero
        // stage count alone cannot tell them apart. Before the first frame
        // fuses all the way through there are no rows and nothing is switched
        // off; announcing the config flag there sends a reader after a knob
        // instead of the fault in front of them -- and on a device whose first
        // fuse never completes, that is what the card said for the whole run.
        Text(model.framesFused == 0 ? "no fused frame yet" : "stage timing off")
          .font(.footnote).foregroundStyle(.secondary)
      } else {
        ForEach(model.stages) { stage in
          StageRowView(stage: stage, scale: stageScale)
        }
        HStack(spacing: 10) {
          LegendSwatch(color: .secondary, text: "host")
          LegendSwatch(color: .accentColor, text: "device")
        }
        .font(.footnote).foregroundStyle(.secondary).padding(.top, 2)
        // Inside the branch that drew the bars, because it says "not in the
        // bars above" and there is no above without them.
        outsideTheBars
      }
      staleness
    }
  }

  /// The cost of the same fused frame that no bar above covers.
  ///
  /// The keyframe copy runs on the fuse thread outside every span there is, so
  /// a reader summing the column gets a frame cost missing it entirely.
  ///
  /// It is the only one. `extract residual` was listed here too and was never
  /// outside anything: recon publishes `max(0, extract_ms - extract.total_ms())`
  /// as the `"  ..other"` stage row, the bridge recomputed the byte-identical
  /// expression into a second field, and this heading then asserted the
  /// opposite of the truth over a figure already drawn as a bar six rows up.
  /// A reader summing the column double-counted it -- and since that term grows
  /// with `active_blocks`, the over-count grew with the scan.
  @ViewBuilder private var outsideTheBars: some View {
    if model.atlasCopyMs > 0.005 {
      VStack(alignment: .leading, spacing: 3) {
        Text("not in the bars above")
          .font(.system(size: 10.5, weight: .medium))
          .foregroundStyle(.tertiary)
        HStack(alignment: .firstTextBaseline) {
          Text("keyframe copy").foregroundStyle(.secondary)
          Spacer(minLength: 8)
          Text(String(format: "%.2f ms", model.atlasCopyMs)).monospacedDigit()
        }
        .font(.footnote)
      }
      .padding(.top, 3)
    }
  }

  /// What makes the figures above less than they look.
  ///
  /// In milliseconds, not frames: the rows publish on the same path that
  /// increments the fused counter, so a frame count between them is zero by
  /// construction and the frames that leave the rows behind are exactly the
  /// ones that never reach the counter.
  ///
  /// Measured *against `msSinceFuse`* rather than against a wall clock, which
  /// is the comparison the transcript has always made and this card did not.
  /// An ARKit interruption stops both clocks together -- limited tracking, a
  /// phone call -- and an absolute threshold announces stale timings for its
  /// whole duration, blaming the fusion for the camera. The difference isolates
  /// the case that is a fault: frames arriving, none completing.
  @ViewBuilder private var staleness: some View {
    let stale = model.stagesStale
    if stale || model.stagesTruncated || model.gpuTimingRetired {
      VStack(alignment: .leading, spacing: 2) {
        if stale {
          Text(String(format: "measured %.1f s ago", model.msSinceStages / 1000))
        }
        // An under-report with no other symptom: a full row array reads exactly
        // like a pipeline that happened to have that many stages.
        if model.stagesTruncated {
          Text("more stages than the snapshot holds — rows are missing")
        }
        // A fault, and distinct from a device that never reported timestamps at
        // all. Both leave every device bar absent; only this one is a failure.
        if model.gpuTimingRetired {
          Text("device timing retired after a failed fence — host only")
        }
      }
      .font(.footnote)
      .foregroundStyle(.orange)
      .fixedSize(horizontal: false, vertical: true)
      .padding(.top, 2)
    }
  }

  /// The read order, independent of the order the bridge appends sections.
  ///
  /// Pinned because the grid is adaptive and a card that comes and goes moves
  /// every card after it: `Alerts` appears only when something has failed, and
  /// an unpinned order let the arrival of a fault reflow the panel a reader was
  /// mid-way through. Unknown titles sort last rather than being dropped -- a
  /// section this file has not been taught about is still a section the bridge
  /// meant to publish.
  private static let sectionOrder = [
    "Alerts", "Scene", "Fuse timing", "Extract phases", "Block table", "Dirty",
    "Memory",
  ]

  private var orderedGroups: [StatGroup] {
    model.groups.sorted { a, b in
      let ra = Self.sectionOrder.firstIndex(of: a.id) ?? Self.sectionOrder.count
      let rb = Self.sectionOrder.firstIndex(of: b.id) ?? Self.sectionOrder.count
      return ra == rb ? a.id < b.id : ra < rb
    }
  }

  /// The bars a card carries above its figures, where it has any.
  ///
  /// Only where a *position against a ceiling* is the question. Occupancy is
  /// deliberately not here: the headline already carries that meter with its
  /// 85% tick, and drawing it twice on one screen is the duplication this
  /// change exists to remove.
  @ViewBuilder private func gauges(for section: String) -> some View {
    switch section {
    case "Scene":
      // Against the plan for the slot this extract wrote -- whether the next
      // remesh has room, which the triangle count alone cannot answer.
      //
      // Marked when the extract that stamped it is not this frame's. Numerator
      // and denominator are both written only on a successful remesh, so a
      // persistent extract failure freezes the bar at whatever it last read
      // while the surface keeps growing -- and the bar is the thing a reader
      // glances at to decide whether the next remesh has room. The `extract`
      // row directly beneath it has reported this all along; the gauge above it
      // carried no signal at all.
      if model.arenaFillKnown {
        Gauge(
          label: "arena", fraction: model.arenaFill,
          warn: Self.thresholds.arenaFill.warn,
          critical: Self.thresholds.arenaFill.critical,
          caption: Self.ratio(
            model.triangles, model.triangleCapacity, unit: "tris"),
          stale: model.extractStale)
      }
    case "Memory":
      // Both ceilings, because the smaller one binds and it is not always the
      // same one. Each is gated on its *own* denominator rather than on one
      // shared flag: `memoryLimitBytes` is already published as 0 when the
      // ceiling is not a real one -- including at the limit, where the kernel
      // clamps the remainder and a derived ceiling collapses onto the footprint
      // -- so the jetsam bar needs no separate at-limit test, while the working
      // set is an independent Metal reading that did not fail and has no reason
      // to vanish with it. Gating both on `!memoryAtLimit` blanked the whole
      // card in exactly the pre-jetsam window it exists for.
      //
      // The peak ticks the bars rather than sitting under them as a row. It is
      // the only figure on this card that survives the gap between polls -- a
      // `resize` doubling spikes for well under one 2 Hz interval and is gone
      // before the next sample -- so against the fill it says how much closer
      // to the ceiling this scan has already been than it looks right now.
      //
      // When no bar is drawn there is no tick, and the *bridge* prints it as a
      // row -- not this file, on the same condition. A copy here would render
      // both in the one state that reaches it (valid, no ceiling of either
      // kind), which is the duplication this change exists to remove.
      if model.memoryValid {
        if model.memoryLimitBytes > 0 {
          Gauge(
            label: "jetsam",
            fraction: Double(model.memoryUsedBytes) / Double(model.memoryLimitBytes),
            warn: Self.thresholds.memory.warn,
            critical: Self.thresholds.memory.critical,
            caption: Self.megabytes(model.memoryUsedBytes, model.memoryLimitBytes),
            mark: Self.fraction(model.memoryPeakBytes, model.memoryLimitBytes))
        }
        if model.memoryWorkingSetBytes > 0 {
          Gauge(
            label: "gpu",
            fraction: Double(model.memoryUsedBytes) / Double(model.memoryWorkingSetBytes),
            warn: Self.thresholds.memory.warn,
            critical: Self.thresholds.memory.critical,
            caption: Self.megabytes(model.memoryUsedBytes, model.memoryWorkingSetBytes),
            mark: Self.fraction(model.memoryPeakBytes, model.memoryWorkingSetBytes))
        }
      }
    default:
      EmptyView()
    }
  }

  /// One scale across the rows, so their lengths compare to each other rather
  /// than each filling its own width.
  private var stageScale: Double {
    max(model.stages.map { max($0.hostMs, $0.deviceMs) }.max() ?? 1, 0.001)
  }

  /// The rows a card lists, which is every figure the bridge published for it
  /// except the ones a gauge above already draws.
  ///
  /// Filtered here rather than withheld by the bridge, because the bridge's
  /// model feeds the log too and the log has no bars: gating the figures on
  /// "a gauge will carry this" left the transcript's Memory block at one
  /// `device RAM` row through the whole healthy state.
  private func rows(_ items: [StatItem]) -> some View {
    VStack(alignment: .leading, spacing: 3) {
      ForEach(items.filter { !$0.drawnAsGauge }) { item in
        HStack(alignment: .firstTextBaseline) {
          Text(item.label.trimmingCharacters(in: .whitespaces))
            .foregroundStyle(.secondary)
          Spacer(minLength: 8)
          Text(item.value)
            .monospacedDigit()
            .multilineTextAlignment(.trailing)
            .foregroundStyle(Self.color(for: item.tone))
        }
        .font(.footnote)
      }
    }
  }

  private func lines(_ text: [String]) -> some View {
    VStack(alignment: .leading, spacing: 2) {
      ForEach(text, id: \.self) { line in
        Text(line)
          .font(.footnote.monospaced())
          .foregroundStyle(.secondary)
          .fixedSize(horizontal: false, vertical: true)
      }
    }
    .frame(maxWidth: .infinity, alignment: .leading)
  }

  private static func color(for tone: VolumetricStatTone) -> Color {
    switch tone {
    case .good: return .green
    case .warn: return .orange
    case .critical: return .red
    default: return .primary
    }
  }

  private static func clock(_ seconds: TimeInterval) -> String {
    let s = Int(seconds)
    return String(format: "%02d:%02d", s / 60, s % 60)
  }

  private static func millions(_ n: Int) -> String {
    String(format: "%.2f", Double(n) / 1_000_000)
  }

  /// A count and the count it is a fraction of, at the scale a gauge caption
  /// can hold: the arena runs to millions of triangles and the raw digits do
  /// not fit beside a bar on a phone.
  ///
  /// Both formatted at **one** unit, chosen from the whole. Choosing per number
  /// rendered the two halves of a single ratio in different units -- `900 k /
  /// 1.50 M tris` -- on a caption whose entire purpose is to be compared, and
  /// dropped the numerator's precision below 1 M while keeping two decimals
  /// above it, so the figure jumped from `999 k` to `1.00 M` across one
  /// triangle. The denominator picks, because it is the one that does not move.
  private static func ratio(_ part: Int, _ whole: Int, unit: String) -> String {
    whole >= 1_000_000
      ? String(
        format: "%.2f M / %.2f M %@", Double(part) / 1_000_000,
        Double(whole) / 1_000_000, unit)
      : String(
        format: "%.0f k / %.0f k %@", Double(part) / 1_000,
        Double(whole) / 1_000, unit)
  }

  /// A gauge's two figures and its fraction, in one line under the bar.
  ///
  /// Both numerator and denominator, never the percentage alone: the fraction
  /// is what the bar already draws, and a reader deciding whether to coarsen
  /// the voxel size or drop a mesh slot needs the megabytes.
  private static func megabytes(_ used: UInt64, _ total: UInt64) -> String {
    guard total > 0 else { return mb(used) }
    return String(
      format: "%@ / %@  (%.0f%%)", mb(used), mb(total),
      100 * Double(used) / Double(total))
  }

  private static func mb(_ bytes: UInt64) -> String {
    String(format: "%.0f MB", Double(bytes) / (1024.0 * 1024.0))
  }

  /// A ratio for a gauge tick, or nil where there is nothing to divide by.
  /// Returning nil rather than 0 keeps an absent reading from drawing a tick at
  /// the origin, which reads as a high-water mark of zero.
  private static func fraction(_ part: UInt64, _ whole: UInt64) -> Double? {
    whole > 0 && part > 0 ? Double(part) / Double(whole) : nil
  }
}

// MARK: - Pieces

private struct StageRowView: View {
  let stage: StageBar
  let scale: Double

  var body: some View {
    HStack(spacing: 7) {
      Text(stage.trimmed)
        .font(.footnote.monospaced())
        .foregroundStyle(stage.isBreakdown ? .secondary : .primary)
        .padding(.leading, stage.isBreakdown ? 9 : 0)
        .frame(width: 92, alignment: .leading)

      GeometryReader { geo in
        ZStack(alignment: .leading) {
          Capsule().fill(.quaternary)
          // Device drawn OVER host, not beside it: the device span happens
          // inside the host span, so laying them end to end would draw a total
          // that does not exist.
          Capsule().fill(.secondary)
            .frame(width: geo.size.width * min(stage.hostMs / scale, 1))
          if stage.hasGPU {
            Capsule().fill(Color.accentColor)
              .frame(width: geo.size.width * min(stage.deviceMs / scale, 1))
          }
        }
      }
      .frame(height: 7)

      Text(String(format: "%.1f", stage.hostMs))
        .font(.footnote.monospacedDigit())
        .foregroundStyle(.secondary)
        .frame(width: 44, alignment: .trailing)
    }
  }
}

private struct FrameChart: View {
  let samples: [FrameSample]

  /// Split rather than totalled, because the two halves answer different
  /// questions and only one of them moves: the fuse sits near 20 ms all scan,
  /// and the meshing is what spikes. Stacked, so the bar height is still the
  /// whole frame -- a reader comparing bar to bar is comparing frame cost, and
  /// the colour says which half caused the difference.
  var body: some View {
    Chart(samples) { sample in
      BarMark(x: .value("Frame", sample.id), y: .value("ms", sample.fuseMs))
        .foregroundStyle(by: .value("half", "fuse"))
      BarMark(x: .value("Frame", sample.id), y: .value("ms", sample.extractMs))
        .foregroundStyle(by: .value("half", "mesh"))
    }
    .chartForegroundStyleScale([
      "fuse": Color.accentColor.opacity(0.85), "mesh": Color.orange.opacity(0.8),
    ])
    .chartLegend(position: .bottom, spacing: 4)
    .chartYAxis { AxisMarks(position: .leading) }
    .chartXAxis(.hidden)
    .frame(height: 92)
  }
}

private enum Tone { case neutral, good, warn }

private struct Chip: View {
  let text: String
  let tone: Tone

  init(_ text: String, tone: Tone) {
    self.text = text
    self.tone = tone
  }

  var body: some View {
    Text(text)
      .font(.footnote)
      .padding(.horizontal, 6)
      .padding(.vertical, 2)
      .background(bg, in: RoundedRectangle(cornerRadius: 5))
      .foregroundStyle(fg)
  }

  private var fg: Color {
    switch tone {
    case .neutral: return .secondary
    case .good: return .green
    case .warn: return .orange
    }
  }
  private var bg: Color {
    tone == .neutral ? Color.primary.opacity(0.07) : fg.opacity(0.15)
  }
}

/// A filled bar carrying its own threshold.
///
/// The tick is the point: a scan can sit at 85% with allocation stopped and
/// every other figure looking healthy, and a reader who has to already know
/// what 85% means is one who will miss it.
private struct Meter: View {
  let fraction: Double
  /// The two tiers, matching the bridge's own `tone_for(value, warn, critical)`
  /// on the rows these bars were migrated from.
  ///
  /// Two, because collapsing them to one dropped a step and a colour: a
  /// footprint at 75% of the jetsam ceiling used to render warn-toned and went
  /// back to a plain accent bar, and 85%+ went from critical red to orange. On
  /// the Scene card the `arena` row kept `tone_for(fill, 0.9, 0.98)` while its
  /// gauge had a single 0.9 tier, so at 0.99 the row was red directly beneath a
  /// merely-orange bar drawn from the same two numbers.
  let warn: Double
  let critical: Double
  /// False when the figure could not be read. Drawn as an empty track with no
  /// fill and no tick, because a fabricated 1.0 rendered as a full bar is the
  /// most alarming reading on the panel produced by the absence of a reading.
  let known: Bool
  /// A high-water mark to tick alongside the threshold, or nil for a bar whose
  /// history is not kept.
  ///
  /// Drawn rather than written out because it answers a question about *this*
  /// bar: how far past the fill the same quantity has already been. The memory
  /// footprint is sampled at roughly 2 Hz and the allocation that gets a scan
  /// killed is a spike lasting well under one interval -- so the fill is the
  /// steady state and this is the only part of the gauge that saw the event.
  var mark: Double? = nil

  /// The fill colour for a fraction, on the same two tiers the rows use.
  static func tint(_ fraction: Double, warn: Double, critical: Double) -> Color {
    if fraction >= critical { return .red }
    if fraction >= warn { return .orange }
    return .accentColor
  }

  var body: some View {
    GeometryReader { geo in
      // Ticks are inset by their own width so a mark at 1.0 lands *on* the end
      // of the track rather than starting at it and drawing past the end.
      // Offsetting by the full width put the one reading the tick exists to
      // show -- a peak over the ceiling, which is routine when a lifetime
      // high-water is measured against a limit re-derived every poll -- off the
      // bar entirely.
      let span = max(geo.size.width - Self.tickWidth, 0)
      ZStack(alignment: .leading) {
        Capsule().fill(.quaternary)
        if known {
          Capsule()
            .fill(Self.tint(fraction, warn: warn, critical: critical))
            .frame(width: geo.size.width * min(max(fraction, 0), 1))
          // Behind the threshold tick and in a quieter colour: it is context
          // for the fill, while the threshold is the line that means act.
          if let mark, mark > fraction {
            Rectangle()
              .fill(Color.primary.opacity(0.45))
              .frame(width: Self.tickWidth)
              .offset(x: span * min(max(mark, 0), 1))
          }
          Rectangle()
            .fill(Color.orange.opacity(0.9))
            .frame(width: Self.tickWidth)
            .offset(x: span * critical)
        }
      }
    }
    // A minimum, not just a height. A GeometryReader accepts a zero-width
    // proposal without complaint, which made this the first thing SwiftUI
    // compressed out of an overfull headline -- silently, and it is the element
    // the headline exists to carry. With a floor, an enclosing ViewThatFits is
    // told the row does not fit instead of being handed a 0 pt meter.
    .frame(minWidth: 80, idealWidth: 120, minHeight: 6, maxHeight: 6)
  }

  private static let tickWidth: Double = 1.5
}

/// A labelled `Meter` with its two figures written beneath it.
///
/// The card-level counterpart to the headline's bare meter: same bar, same
/// threshold tick, but carrying the label and the quantities a reader needs in
/// order to act on the position. It exists because the rows it replaced could
/// not draw a position at all -- `"1234 / 4096 MB (30.1%)"` says where the
/// footprint is only to someone willing to divide, and the 85% line it is
/// approaching was nowhere on the card.
private struct Gauge: View {
  let label: String
  let fraction: Double
  /// Both tiers, carried through to the bar. See `Meter.warn`.
  let warn: Double
  let critical: Double
  let caption: String
  /// The high-water fraction, ticked on the bar. See `Meter.mark`.
  var mark: Double? = nil
  /// Whether the two figures come from an older frame than the one on screen.
  /// Said on the bar itself, because the bar is what a reader glances at -- a
  /// staleness note on a row underneath does not reach someone reading a fill.
  var stale: Bool = false

  var body: some View {
    VStack(alignment: .leading, spacing: 3) {
      HStack(alignment: .firstTextBaseline) {
        Text(label).foregroundStyle(.secondary)
        if stale {
          Text("stale").font(.system(size: 10)).foregroundStyle(.orange)
        }
        Spacer(minLength: 8)
        Text(caption).monospacedDigit()
          .foregroundStyle(captionTint)
      }
      .font(.footnote)
      Meter(
        fraction: fraction, warn: warn, critical: critical, known: true,
        mark: mark)
    }
    .opacity(stale ? 0.65 : 1)
  }

  private var captionTint: Color {
    fraction >= warn
      ? Meter.tint(fraction, warn: warn, critical: critical) : .secondary
  }
}

private struct LegendSwatch: View {
  let color: Color
  let text: String
  var body: some View {
    HStack(spacing: 4) {
      RoundedRectangle(cornerRadius: 2).fill(color).frame(width: 7, height: 7)
      Text(text)
    }
  }
}

private struct Banner<Content: View>: View {
  let tint: Color
  let icon: String
  @ViewBuilder var content: Content

  init(tint: Color, icon: String, @ViewBuilder content: () -> Content) {
    self.tint = tint
    self.icon = icon
    self.content = content()
  }

  var body: some View {
    HStack(alignment: .top, spacing: 8) {
      Image(systemName: icon).foregroundStyle(tint)
      VStack(alignment: .leading, spacing: 2) { content }
      Spacer(minLength: 0)
    }
    .padding(10)
    .background(tint.opacity(0.12), in: RoundedRectangle(cornerRadius: 10))
    .overlay(
      RoundedRectangle(cornerRadius: 10).strokeBorder(tint.opacity(0.35))
    )
  }
}

/// One titled group. Fixed-height-free and self-sizing, so the grid packs them
/// by content rather than to the tallest.
private struct Card<Content: View>: View {
  let title: String
  @ViewBuilder var content: Content

  init(_ title: String, @ViewBuilder content: () -> Content) {
    self.title = title
    self.content = content()
  }

  var body: some View {
    VStack(alignment: .leading, spacing: 6) {
      Text(title.uppercased())
        .font(.system(size: 11.5, weight: .semibold))
        .tracking(0.8)
        .foregroundStyle(.tertiary)
      content
    }
    .padding(9)
    .frame(maxWidth: .infinity, alignment: .leading)
    .background(Color.primary.opacity(0.05), in: RoundedRectangle(cornerRadius: 9))
  }
}

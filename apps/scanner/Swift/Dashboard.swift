// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// The live scan dashboard.
///
/// SwiftUI rather than an ImGui overlay, and the reason is not preference.
/// ImGui renders *inside* the Vulkan pass, so on a device where meshing already
/// costs over 100 ms the overlay competes with the reconstruction for the same
/// queue and lands in every GPU capture. SwiftUI composites on the UI layer, so
/// the OS draws it and the recon pass pays nothing. gfx also ships only the
/// Vulkan backend by design -- the platform half is the consumer's -- so ImGui
/// here would mean writing a UIKit input backend first.
///
/// **What is on screen is a product decision, not a layout one.** Someone
/// walking a room cannot act on `integrate 18.9 ms`; they can act on "the
/// volume is full, finish here". So the primary surface carries four answers --
/// am I capturing, is it working, how much room is left, will it survive -- and
/// every millisecond lives in a sheet they never have to open. That cut is what
/// lets the four things that matter be large enough to read at arm's length
/// while moving.

import Charts
import SwiftUI

// MARK: - Model

/// One stage's split, mirrored out of `VolumetricStageRow` for SwiftUI.
struct StageBar: Identifiable {
  let id: String
  let name: String
  let hostMs: Double
  let deviceMs: Double
  let hasGPU: Bool
  /// A breakdown of the row above it rather than a stage of its own; recon
  /// marks these by indenting the label.
  var isBreakdown: Bool { name.hasPrefix(" ") }
}

/// One fused frame, mirrored out of `VolumetricFrameSample`.
struct FrameSample: Identifiable {
  let id: UInt64
  let hostMs: Double
  let deviceMs: Double
  var totalMs: Double { max(hostMs, deviceMs) }
}

/// What the dashboard draws, refreshed at the display cadence.
///
/// Sampling and display are deliberately different rates. The history is
/// sampled per *fused frame* on the fusion thread, because that thread runs at
/// its own pace and a poll from here -- at any frequency -- sees only whatever
/// was last published. Display stays slow because a number flickering at 120 Hz
/// is unreadable no matter how cheap it is to draw.
@MainActor
final class DashboardModel: ObservableObject {
  @Published var scanning = false
  @Published var elapsed: TimeInterval = 0
  @Published var fps: Double = 0

  @Published var triangles: Int = 0
  @Published var occupancy: Double = 0
  @Published var activeBlocks: Int = 0
  @Published var blockCapacity: Int = 0
  @Published var allocationStopped = false

  @Published var trackingText = ""
  @Published var trackingHealthy = true
  @Published var framesIn = 0
  @Published var framesDropped = 0

  @Published var memoryUsedBytes: UInt64 = 0
  @Published var memoryCeilingBytes: UInt64 = 0

  @Published var stages: [StageBar] = []
  @Published var history: [FrameSample] = []

  /// A failure worth interrupting for. Distinct from the volume being full,
  /// which is the documented trade working and gets its own, calmer, banner.
  @Published var failure: String?

  /// The share of captured frames that never reached fusion.
  var dropFraction: Double {
    framesIn > 0 ? Double(framesDropped) / Double(framesIn) : 0
  }
}

// MARK: - Primary surface

struct DashboardView: View {
  @ObservedObject var model: DashboardModel
  @State private var showDiagnostics = false

  var body: some View {
    VStack(alignment: .leading, spacing: 10) {
      captureState
      coverage
      if model.allocationStopped { volumeFull }
      if let failure = model.failure { failureBanner(failure) }
      chips
    }
    .padding(.horizontal, 14)
    .frame(maxWidth: 520, alignment: .leading)
    .sheet(isPresented: $showDiagnostics) {
      DiagnosticsSheet(model: model)
    }
  }

  // "Am I capturing?" -- first, because every other number is meaningless if
  // the answer is no.
  private var captureState: some View {
    Panel {
      HStack(spacing: 9) {
        Circle()
          .fill(model.scanning ? Color.red : Color.secondary)
          .frame(width: 9, height: 9)
        Text(model.scanning ? "Scanning" : "Paused")
          .font(.callout.weight(.semibold))
        Spacer()
        Text(Self.clock(model.elapsed))
          .font(.callout.monospacedDigit())
          .foregroundStyle(.secondary)
      }
    }
  }

  // "Is it working?" and "how much room is left?" -- one panel, because on this
  // pipeline they are the same question: the surface stops growing when the
  // volume fills, and seeing them apart invites reading a flat line as a stalled
  // scan rather than a full one.
  private var coverage: some View {
    Panel {
      VStack(alignment: .leading, spacing: 9) {
        HStack(alignment: .firstTextBaseline, spacing: 6) {
          Text(Self.millions(model.triangles))
            .font(.system(size: 40, weight: .bold, design: .rounded))
            .monospacedDigit()
          Text("M triangles captured")
            .font(.caption)
            .foregroundStyle(.secondary)
        }

        Meter(fraction: model.occupancy, threshold: 0.85)

        HStack {
          Text("volume \(Int(model.occupancy * 100))% full")
          Spacer()
          Text(
            "\(Self.thousands(model.activeBlocks)) / \(Self.thousands(model.blockCapacity)) blocks"
          )
          .monospacedDigit()
        }
        .font(.caption2)
        .foregroundStyle(.secondary)

        if model.history.count > 1 {
          Trend(samples: model.history)
        }
      }
    }
  }

  // Stated as an instruction, not a percentage. The meter above already says
  // 85%; what a percentage cannot say is that new geometry has stopped being
  // taken in and what to do about it.
  private var volumeFull: some View {
    Panel(tint: .orange) {
      Label {
        VStack(alignment: .leading, spacing: 2) {
          Text("The volume is full.").font(.footnote.weight(.semibold))
          Text(
            "Existing surface keeps refining, but new areas will not be "
              + "added. Finish here, or restart with a coarser voxel size."
          )
          .font(.caption2)
          .foregroundStyle(.secondary)
        }
      } icon: {
        Image(systemName: "exclamationmark.triangle.fill")
          .foregroundStyle(.orange)
      }
    }
  }

  private func failureBanner(_ text: String) -> some View {
    Panel(tint: .red) {
      Label(text, systemImage: "xmark.octagon.fill")
        .font(.caption)
        .foregroundStyle(.red)
    }
  }

  // The remaining health signals, small because they are only interesting when
  // one of them is wrong -- which is what the colour is for.
  private var chips: some View {
    Panel {
      HStack(spacing: 7) {
        Chip(model.trackingText, tone: model.trackingHealthy ? .good : .warn)
        Chip("\(Int(model.fps)) fps", tone: .neutral)
        if model.dropFraction > 0.2 {
          Chip("\(Int(model.dropFraction * 100))% dropped", tone: .warn)
        }
        Chip(
          Self.memory(model.memoryUsedBytes, model.memoryCeilingBytes),
          tone: Self.memoryTone(
            model.memoryUsedBytes,
            model.memoryCeilingBytes))
        Spacer(minLength: 0)
        Button {
          showDiagnostics = true
        } label: {
          Image(systemName: "waveform.path.ecg")
            .font(.caption)
        }
        .buttonStyle(.plain)
        .foregroundStyle(.secondary)
        .accessibilityLabel("Pipeline diagnostics")
      }
    }
  }

  // MARK: formatting

  private static func clock(_ seconds: TimeInterval) -> String {
    let s = Int(seconds)
    return String(format: "%02d:%02d", s / 60, s % 60)
  }

  private static func millions(_ n: Int) -> String {
    String(format: "%.1f", Double(n) / 1_000_000)
  }

  private static func thousands(_ n: Int) -> String {
    n >= 1000 ? "\(n / 1000)k" : "\(n)"
  }

  private static func memory(_ used: UInt64, _ ceiling: UInt64) -> String {
    let gb = { (b: UInt64) in Double(b) / 1_073_741_824 }
    // A ceiling of zero means the OS declined to answer (or the process is
    // already over it), so say the footprint alone rather than divide by it.
    guard ceiling > 0 else { return String(format: "%.1f GB", gb(used)) }
    return String(format: "%.1f / %.1f GB", gb(used), gb(ceiling))
  }

  private static func memoryTone(_ used: UInt64, _ ceiling: UInt64) -> Tone {
    guard ceiling > 0 else { return .neutral }
    let f = Double(used) / Double(ceiling)
    return f >= 0.85 ? .crit : (f >= 0.7 ? .warn : .neutral)
  }
}

// MARK: - Diagnostics

struct DiagnosticsSheet: View {
  @ObservedObject var model: DashboardModel
  @Environment(\.dismiss) private var dismiss

  var body: some View {
    NavigationStack {
      List {
        Section("Per fused frame") {
          ForEach(model.stages) { stage in
            StageRowView(stage: stage, scale: peak)
          }
        }
        Section("Frame history") {
          FrameChart(samples: model.history)
            .frame(height: 150)
            .listRowInsets(
              EdgeInsets(
                top: 10, leading: 10,
                bottom: 10, trailing: 10))
        }
      }
      .navigationTitle("Pipeline")
      .navigationBarTitleDisplayMode(.inline)
      .toolbar {
        ToolbarItem(placement: .confirmationAction) {
          Button("Done") { dismiss() }
        }
      }
    }
    .presentationDetents([.medium, .large])
  }

  /// Bars share one scale, so their lengths are comparable to each other rather
  /// than each filling its own row.
  private var peak: Double {
    max(model.stages.map { max($0.hostMs, $0.deviceMs) }.max() ?? 1, 0.001)
  }
}

private struct StageRowView: View {
  let stage: StageBar
  let scale: Double

  var body: some View {
    HStack(spacing: 10) {
      Text(stage.name.trimmingCharacters(in: .whitespaces))
        .font(.caption.monospaced())
        .foregroundStyle(stage.isBreakdown ? .secondary : .primary)
        .padding(.leading, stage.isBreakdown ? 12 : 0)
        .frame(width: 96, alignment: .leading)

      GeometryReader { geo in
        ZStack(alignment: .leading) {
          Capsule().fill(.quaternary)
          // Device drawn over host rather than stacked beside it: the device
          // span happens INSIDE the host span, so laying them end to end would
          // draw a total that does not exist.
          Capsule().fill(.tertiary)
            .frame(width: geo.size.width * stage.hostMs / scale)
          if stage.hasGPU {
            Capsule().fill(Color.accentColor)
              .frame(width: geo.size.width * stage.deviceMs / scale)
          }
        }
      }
      .frame(height: 8)

      Text(String(format: "%.1f", stage.hostMs))
        .font(.caption.monospacedDigit())
        .foregroundStyle(.secondary)
        .frame(width: 42, alignment: .trailing)
    }
    .padding(.vertical, 1)
  }
}

private struct FrameChart: View {
  let samples: [FrameSample]

  var body: some View {
    Chart(samples) { sample in
      BarMark(
        x: .value("Frame", sample.id),
        y: .value("ms", sample.totalMs)
      )
      .foregroundStyle(Color.accentColor.opacity(0.85))
    }
    .chartYAxis { AxisMarks(position: .leading) }
    .chartXAxis(.hidden)
    .overlay(alignment: .topTrailing) {
      // The spike is the reason this exists, so name it rather than leaving it
      // to be read off an axis.
      if let peak = samples.map(\.totalMs).max() {
        Text(String(format: "peak %.0f ms", peak))
          .font(.caption2.monospacedDigit())
          .foregroundStyle(.secondary)
      }
    }
  }
}

// MARK: - Pieces

private enum Tone { case neutral, good, warn, crit }

private struct Chip: View {
  let text: String
  let tone: Tone

  init(_ text: String, tone: Tone) {
    self.text = text
    self.tone = tone
  }

  var body: some View {
    Text(text)
      .font(.caption2)
      .padding(.horizontal, 7)
      .padding(.vertical, 3)
      .background(background, in: RoundedRectangle(cornerRadius: 6))
      .foregroundStyle(foreground)
  }

  private var foreground: Color {
    switch tone {
    case .neutral: return .secondary
    case .good: return .green
    case .warn: return .orange
    case .crit: return .red
    }
  }

  private var background: Color {
    tone == .neutral ? Color.white.opacity(0.08) : foreground.opacity(0.15)
  }
}

/// A filled bar carrying its own threshold.
///
/// The tick is the whole point: a scan can sit at 85% with allocation stopped
/// and every other figure looking healthy, and a reader who has to already know
/// what 85% means is a reader who will miss it.
private struct Meter: View {
  let fraction: Double
  let threshold: Double

  var body: some View {
    GeometryReader { geo in
      ZStack(alignment: .leading) {
        Capsule().fill(.quaternary)
        Capsule()
          .fill(fraction >= threshold ? Color.orange : Color.accentColor)
          .frame(width: geo.size.width * min(fraction, 1))
        Rectangle()
          .fill(Color.orange.opacity(0.9))
          .frame(width: 1.5)
          .offset(x: geo.size.width * threshold)
      }
    }
    .frame(height: 7)
  }
}

/// Surface captured over the recent past. Flat means nothing new is going in,
/// which reads very differently beside a full meter than beside an empty one.
private struct Trend: View {
  let samples: [FrameSample]

  var body: some View {
    Chart(samples) { sample in
      AreaMark(
        x: .value("Frame", sample.id),
        y: .value("ms", sample.totalMs)
      )
      .foregroundStyle(Color.accentColor.opacity(0.25))
      LineMark(
        x: .value("Frame", sample.id),
        y: .value("ms", sample.totalMs)
      )
      .foregroundStyle(Color.accentColor)
    }
    .chartXAxis(.hidden)
    .chartYAxis(.hidden)
    .frame(height: 30)
  }
}

/// The translucent card every group sits in.
private struct Panel<Content: View>: View {
  var tint: Color?
  @ViewBuilder var content: Content

  init(tint: Color? = nil, @ViewBuilder content: () -> Content) {
    self.tint = tint
    self.content = content()
  }

  var body: some View {
    content
      .padding(.horizontal, 13)
      .padding(.vertical, 11)
      .frame(maxWidth: .infinity, alignment: .leading)
      .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 13))
      .overlay(
        RoundedRectangle(cornerRadius: 13)
          .strokeBorder(tint?.opacity(0.35) ?? Color.white.opacity(0.10))
      )
  }
}

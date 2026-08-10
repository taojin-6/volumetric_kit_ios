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
/// The grouping comes from the bridge (`statSections`), not from this file --
/// the same sections the log line is rendered from, so the screen and the
/// transcript cannot drift apart.

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
  /// The device span sits *inside* the host span, so the frame's cost is the
  /// larger of the two and never their sum.
  var totalMs: Double { max(hostMs, deviceMs) }
}

/// A labelled figure, mirrored out of `VolumetricStatRow`.
struct StatItem: Identifiable {
  let id = UUID()
  let label: String
  let value: String
  let tone: VolumetricStatTone
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
  @Published var occupancy: Double = 0
  @Published var allocationStopped = false

  @Published var trackingText = ""
  @Published var trackingHealthy = true
  @Published var framesIn = 0
  @Published var framesDropped = 0

  @Published var memoryUsedBytes: UInt64 = 0
  @Published var memoryCeilingBytes: UInt64 = 0

  @Published var stages: [StageBar] = []
  @Published var history: [FrameSample] = []
  @Published var groups: [StatGroup] = []
  /// Device, API and capture lines: true prose, so they stay text.
  @Published var deviceLines: [String] = []
  @Published var captureLines: [String] = []

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

  /// Adaptive rather than a fixed column count: the same panel has to work on
  /// a landscape iPad, where six groups fit across, and a portrait phone, where
  /// one does. A hardcoded grid would be right on exactly one of them.
  private let columns = [GridItem(.adaptive(minimum: 290), spacing: 12)]

  var body: some View {
    ScrollView {
      VStack(alignment: .leading, spacing: 10) {
        headline
        if model.allocationStopped { volumeFull }
        if let failure = model.failure { failureBanner(failure) }

        LazyVGrid(columns: columns, alignment: .leading, spacing: 10) {
          Card("Timeline") { timeline }
          Card("Pipeline") { stageBars }
          ForEach(model.groups) { group in
            Card(group.id) { rows(group.items) }
          }
          if !model.captureLines.isEmpty {
            Card("Capture") { lines(model.captureLines) }
          }
          if !model.deviceLines.isEmpty {
            Card("Device") { lines(model.deviceLines) }
          }
        }
      }
      .padding(12)
    }
    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    .overlay(
      RoundedRectangle(cornerRadius: 16)
        .strokeBorder(Color.white.opacity(0.10))
    )
  }

  // The one line that has to be readable without looking: capturing or not,
  // how much surface, how full the volume is.
  private var headline: some View {
    HStack(alignment: .center, spacing: 14) {
      HStack(spacing: 7) {
        Circle()
          .fill(model.scanning ? Color.red : Color.secondary)
          .frame(width: 8, height: 8)
        Text(model.scanning ? "Scanning" : "Paused")
          .font(.title3.weight(.semibold))
        Text(Self.clock(model.elapsed))
          .font(.title3.monospacedDigit())
          .foregroundStyle(.secondary)
      }

      Divider().frame(height: 22)

      HStack(alignment: .firstTextBaseline, spacing: 5) {
        Text(Self.millions(model.triangles))
          .font(.system(size: 34, weight: .bold, design: .rounded))
          .monospacedDigit()
        Text("M tris").font(.footnote).foregroundStyle(.secondary)
      }

      VStack(alignment: .leading, spacing: 3) {
        Meter(fraction: model.occupancy, threshold: 0.85)
        Text("volume \(Int(model.occupancy * 100))% full")
          .font(.footnote).foregroundStyle(.secondary)
      }
      .frame(maxWidth: 150)

      Spacer(minLength: 0)

      HStack(spacing: 6) {
        Chip(model.trackingText, tone: model.trackingHealthy ? .good : .warn)
        Chip("\(Int(model.fps)) fps", tone: .neutral)
        if model.dropFraction > 0.2 {
          Chip("\(Int(model.dropFraction * 100))% dropped", tone: .warn)
        }
      }
    }
    .padding(.horizontal, 4)
  }

  // Stated as an instruction. The meter already says 85%; what a percentage
  // cannot say is that new geometry has stopped going in, or what to do.
  private var volumeFull: some View {
    Banner(tint: .orange, icon: "exclamationmark.triangle.fill") {
      Text("The volume is full.").font(.footnote.weight(.semibold))
      Text(
        "Existing surface keeps refining, but new areas will not be added. "
          + "Finish here, or restart with a coarser voxel size."
      )
      .font(.footnote).foregroundStyle(.secondary)
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

  private var stageBars: some View {
    VStack(alignment: .leading, spacing: 3) {
      ForEach(model.stages) { stage in
        StageRowView(stage: stage, scale: stageScale)
      }
      HStack(spacing: 10) {
        LegendSwatch(color: .secondary, text: "host")
        LegendSwatch(color: .accentColor, text: "device")
      }
      .font(.footnote).foregroundStyle(.secondary).padding(.top, 2)
    }
  }

  /// One scale across the rows, so their lengths compare to each other rather
  /// than each filling its own width.
  private var stageScale: Double {
    max(model.stages.map { max($0.hostMs, $0.deviceMs) }.max() ?? 1, 0.001)
  }

  private func rows(_ items: [StatItem]) -> some View {
    VStack(alignment: .leading, spacing: 3) {
      ForEach(items) { item in
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

  var body: some View {
    Chart(samples) { sample in
      BarMark(x: .value("Frame", sample.id), y: .value("ms", sample.totalMs))
        .foregroundStyle(Color.accentColor.opacity(0.85))
    }
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
  let threshold: Double

  var body: some View {
    GeometryReader { geo in
      ZStack(alignment: .leading) {
        Capsule().fill(.quaternary)
        Capsule()
          .fill(fraction >= threshold ? Color.orange : Color.accentColor)
          .frame(width: geo.size.width * min(max(fraction, 0), 1))
        Rectangle()
          .fill(Color.orange.opacity(0.9))
          .frame(width: 1.5)
          .offset(x: geo.size.width * threshold)
      }
    }
    .frame(height: 6)
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

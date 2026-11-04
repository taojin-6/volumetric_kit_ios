// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// The live link's on-screen entry: the reader's address and port, connect
/// and disconnect, and what the link is doing.
///
/// The fields are remembered between launches; connecting is a tap. The
/// bridge (`VolumetricLink`) reads the fields with the core's checks and says
/// why it refuses one, so the reason under the field is the core's own.

import SwiftUI

/// The bridge's link and what it shows, refreshed twice a second.
final class LinkModel: ObservableObject {
  @Published private(set) var state: VolumetricLinkState = .off
  @Published private(set) var target = ""
  @Published private(set) var dialFailures: UInt64 = 0
  /// Frames the session has sent whole.
  @Published private(set) var frames: UInt64 = 0
  /// Why the last connect was refused, for under the fields; nil after one
  /// that was not.
  @Published private(set) var entryError: String?

  private let link = VolumetricLink()
  /// Whose stream the link dials out: set once the renderer is up, nil
  /// before.
  weak var renderer: VolumetricRenderer?
  private var timer: Timer?

  init() {
    // In the common modes, so it also runs while a scroll view is tracking.
    let timer = Timer(timeInterval: 0.5, repeats: true) { [weak self] _ in
      self?.refresh()
    }
    RunLoop.main.add(timer, forMode: .common)
    self.timer = timer
  }

  deinit { timer?.invalidate() }

  func connect(address: String, port: String) {
    do {
      try link.connect(toAddress: address, port: port, renderer: renderer)
      entryError = nil
    } catch {
      entryError = error.localizedDescription
    }
    refresh()
  }

  func disconnect() {
    link.disconnect()
    entryError = nil
    refresh()
  }

  /// Published only on a change, so a quiet link redraws nothing.
  func refresh() {
    let now = link.status
    if state != now.state { state = now.state }
    if dialFailures != now.dial_failures { dialFailures = now.dial_failures }
    if frames != now.frames { frames = now.frames }
    let named = link.target
    if target != named { target = named }
  }

  /// The headline's chip: a word, nil while off.
  var chip: String? {
    switch state {
    case .off: return nil
    case .dialing: return "dialing"
    case .connected: return "linked"
    case .retrying: return "retrying"
    case .needsLocalNetwork: return "allow Local Network"
    @unknown default: return "link?"
    }
  }

  /// The line under the entry.
  var line: String {
    switch state {
    case .off:
      return "Not connected."
    case .dialing:
      return "Dialing \(target)…"
    case .connected:
      return "Connected to \(target): \(frames) frames sent."
    case .retrying:
      return "Retrying \(target): \(dialFailures) dials failed."
    case .needsLocalNetwork:
      return "No route to \(target). If iOS asked, allow Local Network access "
        + "(Settings › Privacy & Security › Local Network › Scanner); "
        + "otherwise check the address."
    @unknown default:
      return "Link state unknown."
    }
  }
}

/// The entry, for a dashboard card.
struct LinkEntry: View {
  @ObservedObject var link: LinkModel
  @AppStorage("linkAddress") private var address = ""
  @AppStorage("linkPort") private var port = ""
  private enum Field { case address, port }
  @FocusState private var editing: Field?

  var body: some View {
    VStack(alignment: .leading, spacing: 7) {
      HStack(spacing: 6) {
        TextField("Address", text: $address, prompt: Text("10.8.0.1"))
          .keyboardType(.numbersAndPunctuation)
          .submitLabel(.go)
          .onSubmit(connect)
          .focused($editing, equals: .address)
        TextField(
          "Port", text: $port,
          prompt: Text(String(VolumetricLink.defaultPort))
        )
        .keyboardType(.numberPad)
        .focused($editing, equals: .port)
        .frame(width: 76)
      }
      .textFieldStyle(.roundedBorder)
      .textInputAutocapitalization(.never)
      .autocorrectionDisabled()
      .font(.footnote.monospacedDigit())

      HStack(spacing: 8) {
        Button("Connect", action: connect)
          .buttonStyle(.borderedProminent)
        Button("Disconnect") {
          editing = nil
          link.disconnect()
        }
        .buttonStyle(.bordered)
        .disabled(link.state == .off)
        Spacer(minLength: 0)
      }
      .controlSize(.small)

      Text(link.line)
        .font(.footnote)
        .foregroundStyle(tint)
        .fixedSize(horizontal: false, vertical: true)
      if let error = link.entryError {
        Text(error)
          .font(.footnote)
          .foregroundStyle(.red)
          .fixedSize(horizontal: false, vertical: true)
      }
    }
  }

  private func connect() {
    editing = nil
    link.connect(address: address, port: port)
  }

  private var tint: Color {
    switch link.state {
    case .connected: return .green
    case .retrying, .needsLocalNetwork: return .orange
    default: return .secondary
    }
  }
}

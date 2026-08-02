// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import ARKit
import UIKit

/// Drives the renderer from a `CADisplayLink` and shows what it reports.
///
/// Swift owns the view, the display link, and the lifecycle; it never touches
/// Vulkan. Everything below `VolumetricRenderer` is C++ on the other side of the
/// Objective-C++ bridge. When the ARSession lands it joins *this* layer — Swift
/// configures it and hands frames across the same seam.
final class ScannerViewController: UIViewController {
  private var metalView: MetalView!
  private var renderer: VolumetricRenderer?
  private let arSession = ARSessionController()
  private var displayLink: CADisplayLink?
  private let statusLabel = UILabel()

  private var lastFPSUpdate = CFAbsoluteTimeGetCurrent()
  private var framesSinceUpdate = 0
  private var fps = 0.0
  private var shouldLog = false

  override func viewDidLoad() {
    super.viewDidLoad()
    view.backgroundColor = .black

    metalView = MetalView(frame: view.bounds)
    metalView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
    view.addSubview(metalView)

    statusLabel.numberOfLines = 0
    statusLabel.textColor = .white
    statusLabel.font = .monospacedSystemFont(ofSize: 12, weight: .regular)
    statusLabel.translatesAutoresizingMaskIntoConstraints = false
    view.addSubview(statusLabel)
    NSLayoutConstraint.activate([
      statusLabel.topAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 12),
      statusLabel.leadingAnchor.constraint(
        equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 12),
      statusLabel.trailingAnchor.constraint(
        lessThanOrEqualTo: view.safeAreaLayoutGuide.trailingAnchor,
        constant: -12),
    ])
  }

  override func viewDidAppear(_ animated: Bool) {
    super.viewDidAppear(animated)
    // Bring up on first appearance, not in viewDidLoad: the layer has no valid
    // drawableSize until the view has been laid out in a window.
    startRendererIfNeeded()
    arSession.start()
    startDisplayLink()
  }

  override func viewDidDisappear(_ animated: Bool) {
    super.viewDidDisappear(animated)
    stopDisplayLink()
    arSession.pause()
    renderer?.waitIdle()
  }

  private func startRendererIfNeeded() {
    guard renderer == nil else { return }
    guard metalView.metalLayer.drawableSize.width > 0 else { return }

    do {
      renderer = try VolumetricRenderer(layer: metalView.metalLayer)
    } catch {
      statusLabel.text = "Renderer bring-up failed:\n\(error.localizedDescription)"
      statusLabel.textColor = .systemRed
      return
    }
  }

  private func startDisplayLink() {
    guard displayLink == nil else { return }
    let link = CADisplayLink(target: self, selector: #selector(tick))
    link.add(to: .main, forMode: .common)
    displayLink = link
  }

  private func stopDisplayLink() {
    displayLink?.invalidate()
    displayLink = nil
  }

  @objc private func tick() {
    guard let renderer else { return }
    do {
      try renderer.renderFrame(withDrawableSize: metalView.metalLayer.drawableSize)
    } catch {
      statusLabel.text = "Render failed:\n\(error.localizedDescription)"
      statusLabel.textColor = .systemRed
      stopDisplayLink()
      return
    }
    // Poll once per rendered frame -- the same cadence the fuse loop will use,
    // so the capture path is exercised exactly as it will be consumed. The
    // returned frame is not used yet; fusion is the next slice.
    _ = arSession.capture.pollLatest()
    updateStatus(renderer)
  }

  private func updateStatus(_ renderer: VolumetricRenderer) {
    framesSinceUpdate += 1
    let now = CFAbsoluteTimeGetCurrent()
    let elapsed = now - lastFPSUpdate
    if elapsed >= 0.5 {
      fps = Double(framesSinceUpdate) / elapsed
      framesSinceUpdate = 0
      lastFPSUpdate = now
      shouldLog = true
    }
    let size = metalView.metalLayer.drawableSize
    var text = """
      \(renderer.deviceName)
      Vulkan \(renderer.apiVersion) via MoltenVK
      device    \(renderer.sharedDeviceSummary)
      shared    \(renderer.sharesOneDevice ? "yes - recon and gfx hold one VkDevice" : "NO - separate devices")
      drawable  \(Int(size.width)) x \(Int(size.height)) px
      presented \(renderer.framesPresented)
      \(String(format: "%.0f", fps)) fps

      """

    if let reason = arSession.unsupportedReason {
      text += "ARKit: \(reason)"
    } else {
      if let failure = arSession.sessionError {
        // Alongside the counters, not instead of them: a session can fail and
        // recover, and replacing the read-out meant a single transient
        // interruption hid capture for the rest of the run.
        text += "ARKit session failed: \(failure)\n\n"
      }
      let s = arSession.capture.stats
      // Formatted up front rather than inside the literal: a `\`-continuation
      // inside a \(...) interpolation is not something the Swift parser accepts.
      let kept = String(format: "%.0f%%", s.confidence_kept * 100)
      let intrinsics = String(
        format: "fx %.1f  cx %.1f  cy %.1f", s.depth_fx, s.depth_cx, s.depth_cy)
      let position = String(
        format: "%.2f, %.2f, %.2f", s.position_x, s.position_y, s.position_z)
      let convert = String(format: "%.1f", s.convert_ms)
      let counts =
        "\(s.frames_submitted) in / \(s.frames_polled) polled"
        + " / \(s.frames_dropped) dropped / \(s.frames_rejected) rejected"
      text += """
        ARKit capture
          frames    \(counts)
          depth     \(s.depth_width) x \(s.depth_height)  (\(kept) confident)
          colour    \(s.color_width) x \(s.color_height)
          intrinsic \(intrinsics)
          position  \(position) m
          convert   \(convert) ms
        """
    }
    statusLabel.text = text
    if shouldLog {
      shouldLog = false
      // Same text as the on-screen read-out, to stdout, so
      // `devicectl device process launch --console` can verify capture from the
      // build host instead of someone reading the screen.
      print(text)
    }
  }
}

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

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
  private var displayLink: CADisplayLink?
  private let statusLabel = UILabel()

  private var lastFPSUpdate = CFAbsoluteTimeGetCurrent()
  private var framesSinceUpdate = 0
  private var fps = 0.0

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
    startDisplayLink()
  }

  override func viewDidDisappear(_ animated: Bool) {
    super.viewDidDisappear(animated)
    stopDisplayLink()
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
    }
    let size = metalView.metalLayer.drawableSize
    statusLabel.text = """
      \(renderer.deviceName)
      Vulkan \(renderer.apiVersion) via MoltenVK
      drawable  \(Int(size.width)) x \(Int(size.height)) px
      presented \(renderer.framesPresented)
      \(String(format: "%.0f", fps)) fps
      """
  }
}

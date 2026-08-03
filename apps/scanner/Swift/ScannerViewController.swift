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
  private var cameraGestures: CameraGestureController?

  private var lastFPSUpdate = CFAbsoluteTimeGetCurrent()
  private var framesSinceUpdate = 0
  private var fps = 0.0
  private var shouldLog = false

  /// The part of the read-out that is frozen once the renderer is up: the GPU,
  /// the negotiated API version, and how the one shared device was carved up.
  /// Cached rather than re-read per tick -- each of those properties crosses the
  /// bridge to rebuild a C++ string or re-issue
  /// `vkGetPhysicalDeviceProperties`, and the display link asks 60 times a
  /// second for a value that cannot change.
  private var deviceSummary = ""

  override func viewDidLoad() {
    super.viewDidLoad()
    view.backgroundColor = .black

    metalView = MetalView(frame: view.bounds)
    metalView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
    view.addSubview(metalView)

    // Attached here, not after bring-up: the recognizers belong to the view,
    // which exists now, and they hold no camera until `renderer` is handed over
    // below. Installing them late would mean touches during the first frames
    // land on nothing.
    cameraGestures = CameraGestureController(attachingTo: metalView)

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
    // Before the first frame: ARKit poses are in the sensor's frame, which does
    // not turn with the interface, so the renderer has to be told which way the
    // viewport is facing or the scan renders on its side.
    renderer?.viewOrientation = currentViewOrientation()
    arSession.start()
    // Fusion pulls from the capture bridge on its own thread; the render loop
    // only ever picks up whatever mesh it has published.
    renderer?.startFusion(with: arSession.capture)
    startDisplayLink()
  }

  override func viewDidDisappear(_ animated: Bool) {
    super.viewDidDisappear(animated)
    stopDisplayLink()
    // Fusion first, and explicitly. It submits recon work on the same queue the
    // drain below empties, so a fuse thread still running would enqueue behind
    // the teardown -- and waitIdle does not stop it: it waits on the queues and
    // never touches the thread. Until stopFusion returns, the thread is also
    // still dereferencing the capture handle.
    renderer?.stopFusion()
    arSession.pause()
    renderer?.waitIdle()
  }

  override func viewWillTransition(
    to size: CGSize, with coordinator: UIViewControllerTransitionCoordinator
  ) {
    super.viewWillTransition(to: size, with: coordinator)
    // In the completion, not alongside it: `interfaceOrientation` still reports
    // the old value until the transition finishes, so reading it early would
    // pin the camera one quarter turn behind the screen.
    coordinator.animate(alongsideTransition: nil) { [weak self] _ in
      guard let self else { return }
      self.renderer?.viewOrientation = self.currentViewOrientation()
    }
  }

  /// The viewport's turn relative to ARKit's sensor-fixed camera basis.
  ///
  /// From the window scene rather than `UIDevice.orientation`: the device can
  /// be face-up or face-down, and what the render camera needs is where the
  /// *interface* ended up, which is what the app's supported orientations
  /// actually constrain.
  private func currentViewOrientation() -> VolumetricViewOrientation {
    switch view.window?.windowScene?.interfaceOrientation {
    case .portrait: return .portrait
    case .portraitUpsideDown: return .portraitUpsideDown
    case .landscapeLeft: return .landscapeLeft
    case .landscapeRight: return .landscapeRight
    // No window yet, or `.unknown`. Portrait is how the app launches.
    default: return .portrait
    }
  }

  private func startRendererIfNeeded() {
    guard renderer == nil else { return }
    guard metalView.metalLayer.drawableSize.width > 0 else { return }

    let brought: VolumetricRenderer
    do {
      brought = try VolumetricRenderer(layer: metalView.metalLayer)
    } catch {
      let message = "Renderer bring-up failed:\n\(error.localizedDescription)"
      statusLabel.text = message
      statusLabel.textColor = .systemRed
      // To stdout as well: this path never reaches updateStatus, so without it
      // a bring-up failure is visible only on the screen.
      print(message)
      return
    }
    renderer = brought
    cameraGestures?.renderer = brought
    deviceSummary = """
      \(brought.deviceName)
      Vulkan \(brought.apiVersion) via MoltenVK
      device    \(brought.sharedDeviceSummary)
      shared    \(brought.sharesOneDevice ? "yes - recon and gfx hold one VkDevice" : "NO - separate devices")
      """
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
    // No poll here. `ICameraCapture` is single-consumer ("Poll from one
    // thread"), and its CapturedFrame is a non-owning view valid only until the
    // next poll -- so the bring-up placeholder that polled once per rendered
    // frame is now a second consumer racing the fuse thread for the same
    // buffers, and every frame it won was a frame fusion never saw.
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
    // Which camera is driving, and -- once the user is driving -- the way back.
    // A manual camera pointed away from the scan and a scan that stopped
    // producing geometry look identical on screen, so the mode is worth a line.
    let camera =
      renderer.followingDevice
      ? "following device"
      : String(
        format: "manual  %.2f m from pivot  (double-tap to follow)",
        renderer.cameraDistance)
    var text = """
      \(deviceSummary)
      drawable  \(Int(size.width)) x \(Int(size.height)) px
      presented \(renderer.framesPresented)
      camera    \(camera)
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
      // Ahead of the counters, because it is the answer to "why is the frame
      // count rising and nothing appearing": frames arriving while ARKit does
      // not trust its own pose are withheld from fusion on purpose.
      let t = arSession.tracking
      let withheld =
        t.framesWithheld > 0 ? "  (\(t.framesWithheld) withheld)" : ""
      text += "tracking  \(t.description)\(withheld)\n\n"

      let s = arSession.capture.stats
      // Formatted up front rather than inside the literal: a `\`-continuation
      // inside a \(...) interpolation is not something the Swift parser accepts.
      let kept = String(format: "%.0f%%", s.confidence_kept * 100)
      let intrinsics = String(
        format: "fx %.1f  cx %.1f  cy %.1f", s.depth_fx, s.depth_cx, s.depth_cy)
      // What the buffer declared, not what we assumed. The matrix is separate
      // from the transfer and primaries -- it reconstructs chroma, they
      // describe the result -- so all three are shown rather than collapsed.
      let encoding = String(
        format: "%@ matrix -> %@ / %@%@",
        String(cString: s.color_matrix), String(cString: s.color_transfer),
        String(cString: s.color_primaries),
        s.color_was_canonical ? "" : "  (converted)")
      let position = String(
        format: "%.2f, %.2f, %.2f", s.position_x, s.position_y, s.position_z)
      let convert = String(format: "%.1f", s.convert_ms)
      let counts =
        "\(s.frames_submitted) in / \(s.frames_polled) polled"
        + " / \(s.frames_dropped) dropped / \(s.frames_rejected) rejected"
      text += """
        \(renderer.fusionSummary)

        ARKit capture
          frames    \(counts)
          depth     \(s.depth_width) x \(s.depth_height)  (\(kept) confident)
          colour    \(s.color_width) x \(s.color_height)
          encoding  \(encoding)
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

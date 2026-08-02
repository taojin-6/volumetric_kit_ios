// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import ARKit

/// Owns the `ARSession` and feeds its frames to the capture bridge.
///
/// Swift owns the session — configuration, capability checks, lifecycle,
/// and the camera permission prompt UIKit raises on first run — because that is
/// what Swift is good at and what grows as the app grows. It never touches
/// recon: each `ARFrame` crosses to `VolumetricCapture`, and everything below
/// that is C++.
final class ARSessionController: NSObject, ARSessionDelegate {
  let capture = VolumetricCapture()
  private let session = ARSession()

  /// The queue ARKit delivers frames on.
  ///
  /// Set explicitly, because `ARSession.delegateQueue` defaults to nil and nil
  /// means **the main queue** (ARSession.h) — where the conversion in
  /// `submitFrame` would run serialized with the `CADisplayLink` render
  /// callback, spending milliseconds of depth copy and vImage colour conversion
  /// inside every frame's budget. Giving the session a queue of its own is what
  /// lets the bridge's double buffer and lock do the job they were written for,
  /// and what makes "drop rather than queue" describe a slow *consumer* rather
  /// than a saturated main thread.
  private let sessionQueue = DispatchQueue(
    label: "io.taojin.volumetrickit.arsession", qos: .userInitiated)

  /// The frame semantics this app needs: smoothed depth to fuse, plus the raw
  /// map as the fallback for frames the smoother has no history for yet.
  private static let requiredSemantics: ARConfiguration.FrameSemantics = [
    .smoothedSceneDepth, .sceneDepth,
  ]

  /// Set when the device cannot produce scene depth, so the UI can say why
  /// rather than showing a frame counter stuck at zero.
  ///
  /// Read by the render loop, and written only on the main queue: `start()` is
  /// called from the view controller, and the delegate callbacks below hop back
  /// to main before touching any of this state.
  private(set) var unsupportedReason: String?

  /// Most recent session failure, if any. Cleared by a successful `start()`.
  private(set) var sessionError: String?

  /// Whether `session.run` has been called yet, which decides whether the next
  /// `start()` resets tracking or resumes.
  private var hasRun = false

  override init() {
    super.init()
    session.delegateQueue = sessionQueue
    session.delegate = self
  }

  /// Whether this device can produce LiDAR scene depth at all.
  ///
  /// Checked rather than assumed: scene depth needs the LiDAR scanner
  /// (iPhone 12 Pro and later Pro models, iPad Pro 2020 and later). On anything
  /// else `ARWorldTrackingConfiguration` still runs and still tracks — it just
  /// never delivers depth, which would look like a silent hang. Checked against
  /// the exact set `start()` requests, so a device that supported one semantic
  /// and not the other could not slip through.
  static var supportsSceneDepth: Bool {
    ARWorldTrackingConfiguration.supportsFrameSemantics(requiredSemantics)
  }

  func start() {
    guard ARSessionController.supportsSceneDepth else {
      unsupportedReason =
        "This device has no LiDAR scanner, so ARKit cannot produce scene depth."
      return
    }
    let config = ARWorldTrackingConfiguration()
    // smoothedSceneDepth is a *separate* ARFrame property from sceneDepth, not
    // a filter applied to it in place, so requesting the semantic is only half
    // the job — the bridge has to read `smoothedSceneDepth` to actually get the
    // temporal filtering that suppresses the per-frame flicker a TSDF would
    // otherwise average into the volume.
    config.frameSemantics = ARSessionController.requiredSemantics
    config.worldAlignment = .gravity

    if hasRun {
      // Resume, don't reset. Re-running with .resetTracking on every
      // appearance would discard the world origin, and once fusion accumulates
      // into a world-frame volume that would silently invalidate everything
      // already in it on any background/foreground cycle.
      session.run(config)
    } else {
      session.run(config, options: [.resetTracking, .removeExistingAnchors])
      hasRun = true
    }
    sessionError = nil
  }

  func pause() {
    session.pause()
  }

  // MARK: - ARSessionDelegate

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    // Called on `sessionQueue`, not the main thread. The bridge stages the
    // converted frame under its own lock and the render loop polls it, so
    // nothing here touches UI state — the callbacks below, which do, hop back
    // to main first.
    capture.submitFrame(frame)
  }

  func session(_ session: ARSession, didFailWithError error: Error) {
    let message = error.localizedDescription
    DispatchQueue.main.async { [weak self] in
      self?.sessionError = message
    }
  }

  func sessionWasInterrupted(_ session: ARSession) {
    DispatchQueue.main.async { [weak self] in
      self?.sessionError = "Session interrupted; capture is paused."
    }
  }

  func sessionInterruptionEnded(_ session: ARSession) {
    // ARKit does not resume on its own after an interruption — the session has
    // to be re-run — and `start()` clears the message on its way through. Both
    // halves matter: without them one interruption left the session stopped and
    // its error text on screen for the rest of the run.
    DispatchQueue.main.async { [weak self] in
      self?.start()
    }
  }
}

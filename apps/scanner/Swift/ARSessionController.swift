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

  /// Set when the device cannot produce scene depth, so the UI can say why
  /// rather than showing a frame counter stuck at zero.
  private(set) var unsupportedReason: String?

  /// Most recent session failure, if any.
  private(set) var sessionError: String?

  override init() {
    super.init()
    session.delegate = self
  }

  /// Whether this device can produce LiDAR scene depth at all.
  ///
  /// Checked rather than assumed: `sceneDepth` needs the LiDAR scanner
  /// (iPhone 12 Pro and later Pro models, iPad Pro 2020 and later). On anything
  /// else `ARWorldTrackingConfiguration` still runs and still tracks — it just
  /// never delivers depth, which would look like a silent hang.
  static var supportsSceneDepth: Bool {
    ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
  }

  func start() {
    guard ARSessionController.supportsSceneDepth else {
      unsupportedReason =
        "This device has no LiDAR scanner, so ARKit cannot produce scene depth."
      return
    }
    let config = ARWorldTrackingConfiguration()
    // smoothedSceneDepth over sceneDepth: ARKit temporally filters it against
    // previous frames, which suppresses the per-frame flicker a TSDF would
    // otherwise average into the volume. Requesting it also delivers
    // `sceneDepth`, so the bridge reads one property either way.
    config.frameSemantics = [.smoothedSceneDepth, .sceneDepth]
    config.worldAlignment = .gravity
    session.run(config, options: [.resetTracking, .removeExistingAnchors])
  }

  func pause() {
    session.pause()
  }

  // MARK: - ARSessionDelegate

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    // Called on the session queue, not the main thread. The bridge stages the
    // converted frame under its own lock and the render loop polls it, so
    // nothing here touches UI state.
    capture.submitFrame(frame)
  }

  func session(_ session: ARSession, didFailWithError error: Error) {
    sessionError = error.localizedDescription
  }
}

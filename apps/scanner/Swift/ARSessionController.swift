// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import ARKit

/// What ARKit's tracker is doing, and what gating on it has cost.
///
/// A value type so the read-out gets both fields from one lock acquisition,
/// rather than two that could straddle an update.
struct TrackingReport {
  /// Human-readable state for the read-out, e.g. `limited (relocalizing)`.
  var description = "starting"
  /// Frames withheld from fusion because ARKit did not trust the pose.
  var framesWithheld: UInt64 = 0
}

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

  /// Guards @ref trackingReport, which is written on `sessionQueue` (60 Hz) and
  /// read on the main thread by the read-out.
  ///
  /// A lock rather than the `DispatchQueue.main.async` hop the callbacks below
  /// use: those fire on session *events*, this fires on every frame, and 60
  /// main-queue dispatches a second to update a status line is the tail wagging
  /// the dog. Same shape as `VolumetricCapture.stats`, which shares the
  /// bridge's lock for the same reason.
  private let trackingLock = NSLock()
  private var trackingReport = TrackingReport()

  /// The tracker's current state and what gating on it has withheld.
  var tracking: TrackingReport {
    trackingLock.lock()
    defer { trackingLock.unlock() }
    return trackingReport
  }

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

  /// Whether the owner currently wants this session running.
  ///
  /// `sessionInterruptionEnded` used to re-run the session unconditionally,
  /// which put ARKit outside the view controller's state machine entirely: a
  /// background/foreground cycle ran `start()` twice (once from `resume`, once
  /// from the interruption callback, in an order ARKit does not specify), and an
  /// interruption ending after `viewDidDisappear` or after a device loss revived
  /// the camera behind the state machine's back — a live session with
  /// `isCapturing` false, which `suspend()` then declines to wind down because
  /// it guards on exactly that.
  ///
  /// So the callback consults this instead. `start()` and `pause()` are the
  /// only things that move it, which keeps the owner the single decider of
  /// whether capture should be running and leaves this class responsible only
  /// for *how* to resume.
  private var shouldBeRunning = false

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
    shouldBeRunning = true
    sessionError = nil
  }

  func pause() {
    shouldBeRunning = false
    session.pause()
  }

  // MARK: - ARSessionDelegate

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    // Called on `sessionQueue`, not the main thread. The bridge stages the
    // converted frame under its own lock and the render loop polls it, so
    // nothing here touches UI state — the callbacks below, which do, hop back
    // to main first.

    // Gated on tracking state before anything reaches fusion. ARKit keeps
    // delivering frames carrying a `camera.transform` it does not itself trust:
    // `.notAvailable` at session start, before the world origin settles, and
    // `.limited(.relocalizing)` — which `sessionInterruptionEnded` below
    // deliberately induces by re-running the session, so it recurs on every
    // background/foreground cycle.
    //
    // Fusing one is not a dropped frame, it is a permanent one. Each is
    // allocated and integrated at the pose it claims, carving free space along
    // every ray it believes, and `IntegrationMode::Classic` never clears stale
    // geometry — so one relocalization burst writes a ghost surface into the
    // volume that nothing later in the scan removes, however long the user
    // keeps scanning. Withholding costs a few frames; not withholding costs the
    // reconstruction.
    let state = frame.camera.trackingState
    let usable: Bool
    if case .normal = state { usable = true } else { usable = false }

    trackingLock.lock()
    trackingReport.description = ARSessionController.describe(state)
    if !usable { trackingReport.framesWithheld += 1 }
    trackingLock.unlock()

    guard usable else { return }
    capture.submitFrame(frame)
  }

  /// The tracking state as the read-out shows it. Spelled out rather than using
  /// the enum's synthesized description, because "why is nothing fusing" is the
  /// question this line exists to answer.
  private static func describe(_ state: ARCamera.TrackingState) -> String {
    switch state {
    case .normal:
      return "normal"
    case .notAvailable:
      return "not available"
    case .limited(let reason):
      switch reason {
      case .initializing: return "limited (initializing)"
      case .relocalizing: return "limited (relocalizing)"
      case .excessiveMotion: return "limited (excessive motion)"
      case .insufficientFeatures: return "limited (insufficient features)"
      @unknown default: return "limited"
      }
    @unknown default:
      return "unknown"
    }
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
    //
    // Only when the owner still wants it running, though. An interruption can
    // end long after `pause()` — the app is backgrounded, the view is off
    // screen, the render loop has latched a device loss — and re-running here
    // regardless started a camera nothing was consuming and left the view
    // controller's `isCapturing` describing a session that no longer matched
    // it. See `shouldBeRunning`.
    DispatchQueue.main.async { [weak self] in
      guard let self, self.shouldBeRunning else { return }
      self.start()
    }
  }
}

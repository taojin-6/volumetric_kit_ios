// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import UIKit

/// Turns touches on the render view into camera commands.
///
/// The split is the same one the rest of the app keeps: UIKit belongs to Swift,
/// the camera belongs to the renderer. So this recognizes gestures and converts
/// their units — nothing more. It decides that one finger orbits and two
/// translate; it does not decide what orbiting *is*, which is `OrbitCamera`'s
/// job on the other side of the bridge.
///
/// The mapping is the conventional 3D-inspector one — one finger orbits, two
/// pan, pinch zooms — because that is what a hand arriving at an unfamiliar
/// viewer will try first. Double-tap is the way back to the device-following
/// camera; without it the first accidental swipe would strand the user in a
/// manual camera with no route home.
///
/// Deltas cross the bridge as **fractions of the view's height**, in points.
/// The renderer never learns the view's size or scale factor, and a given
/// finger travel means the same rotation on a phone and on an iPad.
final class CameraGestureController: NSObject {
  /// Set once bring-up succeeds. Gestures before that are dropped: there is no
  /// camera to move yet, and nothing on screen to move it against.
  var renderer: VolumetricRenderer?

  // `unowned`, because the view outlives this: the view controller owns both,
  // and the view retains the recognizers, which retain this object as their
  // target. A strong reference back would close that into a cycle.
  private unowned let view: UIView

  private let orbitRecognizer: UIPanGestureRecognizer
  private let panRecognizer: UIPanGestureRecognizer
  private let pinchRecognizer: UIPinchGestureRecognizer
  private let recenterRecognizer: UITapGestureRecognizer

  /// Attach the recognizers to @p view. Safe to call before the renderer
  /// exists.
  init(attachingTo view: UIView) {
    self.view = view
    orbitRecognizer = UIPanGestureRecognizer()
    panRecognizer = UIPanGestureRecognizer()
    pinchRecognizer = UIPinchGestureRecognizer()
    recenterRecognizer = UITapGestureRecognizer()
    super.init()

    // One finger only. Beyond separating this from the two-finger pan, it makes
    // the recognizer *fail* when a second finger lands, which is what lets the
    // pan recognizer pick those touches up instead.
    orbitRecognizer.maximumNumberOfTouches = 1
    orbitRecognizer.addTarget(self, action: #selector(handleOrbit))

    panRecognizer.minimumNumberOfTouches = 2
    panRecognizer.maximumNumberOfTouches = 2
    panRecognizer.addTarget(self, action: #selector(handlePan))
    // Two fingers spreading while also sliding is one motion to a hand, but two
    // recognizers to UIKit, and by default the first to claim the touches wins.
    // Without the delegate below, zoom and pan take turns instead of composing.
    panRecognizer.delegate = self

    pinchRecognizer.addTarget(self, action: #selector(handlePinch))
    pinchRecognizer.delegate = self

    recenterRecognizer.numberOfTapsRequired = 2
    recenterRecognizer.addTarget(self, action: #selector(handleDoubleTap))

    view.addGestureRecognizer(orbitRecognizer)
    view.addGestureRecognizer(panRecognizer)
    view.addGestureRecognizer(pinchRecognizer)
    view.addGestureRecognizer(recenterRecognizer)
  }

  /// One viewport height in points, the unit every delta below is reported in.
  /// Guarded because a zero would turn every delta into an infinity, and this
  /// object is built before the view has been laid out.
  private var heightInPoints: CGFloat? {
    let height = view.bounds.height
    return height > 0 ? height : nil
  }

  @objc private func handleOrbit(_ recognizer: UIPanGestureRecognizer) {
    guard let renderer, let height = heightInPoints else { return }
    guard recognizer.state == .began || recognizer.state == .changed else { return }
    let delta = recognizer.translation(in: view)
    // Consumed, not accumulated: `translation(in:)` reports the total since the
    // gesture began, so resetting it each callback is what turns it into the
    // per-callback increment the camera applies.
    recognizer.setTranslation(.zero, in: view)
    renderer.orbit(dx: Float(delta.x / height), dy: Float(delta.y / height))
  }

  @objc private func handlePan(_ recognizer: UIPanGestureRecognizer) {
    guard let renderer, let height = heightInPoints else { return }
    guard recognizer.state == .began || recognizer.state == .changed else { return }
    let delta = recognizer.translation(in: view)
    recognizer.setTranslation(.zero, in: view)
    renderer.pan(dx: Float(delta.x / height), dy: Float(delta.y / height))
  }

  @objc private func handlePinch(_ recognizer: UIPinchGestureRecognizer) {
    guard let renderer else { return }
    guard recognizer.state == .began || recognizer.state == .changed else { return }
    let scale = recognizer.scale
    // Same reasoning as the pan translation: `scale` is cumulative since the
    // gesture began, so it is reset to the identity to yield an increment.
    recognizer.scale = 1.0
    renderer.zoom(scale: Float(scale))
  }

  @objc private func handleDoubleTap(_ recognizer: UITapGestureRecognizer) {
    guard recognizer.state == .ended else { return }
    renderer?.followDevice()
  }
}

extension CameraGestureController: UIGestureRecognizerDelegate {
  func gestureRecognizer(
    _ recognizer: UIGestureRecognizer,
    shouldRecognizeSimultaneouslyWith other: UIGestureRecognizer
  ) -> Bool {
    // Only the pinch/two-finger-pan pair. Blanket agreement would put the
    // one-finger orbit on the same touches as a pinch, so a two-finger zoom
    // would spin the scene as it scaled.
    let pair = Set([ObjectIdentifier(recognizer), ObjectIdentifier(other)])
    return pair
      == Set([ObjectIdentifier(panRecognizer), ObjectIdentifier(pinchRecognizer)])
  }
}

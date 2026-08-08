// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

import UIKit

/// A `UIView` whose backing layer is a `CAMetalLayer` — the object MoltenVK
/// turns into a `VkSurfaceKHR`.
///
/// Overriding `layerClass` is what makes `self.layer` a `CAMetalLayer`; there is
/// no way to attach one to an ordinary view after the fact. `MTKView` would also
/// provide one, but it brings its own draw loop and drawable management, which
/// would fight the Vulkan swapchain for ownership of the same drawables.
final class MetalView: UIView {
  override class var layerClass: AnyClass { CAMetalLayer.self }

  var metalLayer: CAMetalLayer { layer as! CAMetalLayer }

  /// Called after `drawableSize` changes, so the renderer can rebuild.
  var onDrawableSizeChange: ((CGSize) -> Void)?

  /// Holds every read of and write to the backing `CAMetalLayer` while someone
  /// else owns it.
  ///
  /// Renderer bring-up runs off the main thread and hands *this* layer to
  /// `vkCreateMetalSurfaceEXT`, after which MoltenVK reads and sets `device`,
  /// `pixelFormat`, `framebufferOnly` and `maximumDrawableCount` on it as it
  /// builds the swapchain. A `CALayer` is not thread-safe, and bring-up takes
  /// seconds against a cold shader cache — so any rotation, keyboard, Slide Over
  /// resize or safe-area relayout in that window is an unsynchronized access to
  /// the same object from inside a main-thread `CATransaction` commit.
  /// Snapshotting the *value* of `drawableSize` before dispatching, which is
  /// what the controller does, does not help: the layer object itself is what
  /// crosses.
  ///
  /// Layout still runs while this is set; only the layer touch is deferred, and
  /// it is re-derived from `bounds` and applied once on the way back down.
  var defersLayerUpdates = false {
    didSet {
      guard oldValue, !defersLayerUpdates else { return }
      applyPendingScale()
      updateDrawableSize()
    }
  }
  /// A `contentsScale` that arrived while updates were deferred.
  private var pendingScale: CGFloat?

  override func layoutSubviews() {
    super.layoutSubviews()
    updateDrawableSize()
  }

  override func didMoveToWindow() {
    super.didMoveToWindow()
    // contentScaleFactor is only meaningful once the view has a screen.
    if let scale = window?.screen.nativeScale {
      contentScaleFactor = scale
      pendingScale = scale
      applyPendingScale()
    }
    updateDrawableSize()
  }

  private func applyPendingScale() {
    guard !defersLayerUpdates, let scale = pendingScale else { return }
    metalLayer.contentsScale = scale
    pendingScale = nil
  }

  private func updateDrawableSize() {
    // drawableSize is in PIXELS, while bounds is in POINTS. Skipping this
    // conversion renders at 1/3 resolution on a 3x display and then upscales —
    // it looks merely soft rather than broken, which makes it easy to miss.
    let scale = contentScaleFactor
    let pixels = CGSize(
      width: bounds.width * scale,
      height: bounds.height * scale)
    guard pixels.width > 0, pixels.height > 0 else { return }
    // Before the layer is touched at all — the comparison below reads
    // `drawableSize`, which is as much a data race as writing it. See
    // `defersLayerUpdates`.
    guard !defersLayerUpdates else { return }
    guard metalLayer.drawableSize != pixels else { return }
    metalLayer.drawableSize = pixels
    onDrawableSizeChange?(pixels)
  }
}

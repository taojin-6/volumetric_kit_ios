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

  override func layoutSubviews() {
    super.layoutSubviews()
    updateDrawableSize()
  }

  override func didMoveToWindow() {
    super.didMoveToWindow()
    // contentScaleFactor is only meaningful once the view has a screen.
    if let scale = window?.screen.nativeScale {
      contentScaleFactor = scale
      metalLayer.contentsScale = scale
    }
    updateDrawableSize()
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
    guard metalLayer.drawableSize != pixels else { return }
    metalLayer.drawableSize = pixels
    onDrawableSizeChange?(pixels)
  }
}

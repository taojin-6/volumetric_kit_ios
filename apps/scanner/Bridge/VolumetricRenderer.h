// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file VolumetricRenderer.h
/// @brief The Swift-facing seam onto volumetric_kit_gfx.
///
/// Swift owns the app: lifecycle, the view, and (later) the ARSession. It does
/// not own Vulkan. Everything below this header is C++ — and recon/gfx are
/// move-only types with `Result<T>` returns that Swift's C++ interop handles
/// poorly — so the seam is a narrow Objective-C class that hands Swift plain
/// values and `NSError`s and keeps the C++ entirely on this side.
///
/// This is why the bridge is Objective-C++ rather than Swift: an `.mm` is the
/// one translation unit where a `CAMetalLayer*` and a `vg::app::WindowedApp`
/// are both first-class, so no marshalling layer is needed at all.

#import <Foundation/Foundation.h>
#import <QuartzCore/CAMetalLayer.h>

NS_ASSUME_NONNULL_BEGIN

/// @brief Owns the renderer bring-up chain and draws one frame on demand.
///
/// Construction runs the whole chain — instance → surface (from the layer) →
/// device → allocator → swapchain → frame loop — and builds the triangle
/// pipeline. Failure is reported through @p error rather than an exception, in
/// keeping with the no-exceptions-across-the-boundary rule both libraries hold.
NS_SWIFT_NAME(VolumetricRenderer)
@interface VolumetricRenderer : NSObject

/// @brief Bring up the renderer against @p layer.
/// @param layer  The view's `CAMetalLayer`; becomes the `VkSurfaceKHR`. Its
///               `drawableSize` must already be set in **pixels**.
/// @param error  Populated with the failing step's message on failure.
/// @return The renderer, or `nil`.
- (nullable instancetype)initWithLayer:(CAMetalLayer*)layer
                                 error:(NSError**)error
    NS_DESIGNATED_INITIALIZER;

- (instancetype)init NS_UNAVAILABLE;

/// @brief Draw and present one frame at @p size.
///
/// Pass the layer's current `drawableSize` in pixels; the frame loop rebuilds
/// the swapchain when it differs from the current one, which is how rotation
/// and resize are handled. A frame skipped because the swapchain went stale is
/// reported as success with no error — that is the protocol working, not a
/// failure.
///
/// @param size   The drawable size in pixels.
/// @param error  Populated on a hard failure.
/// @return `YES` if the frame was drawn or legitimately skipped.
- (BOOL)renderFrameWithDrawableSize:(CGSize)size error:(NSError**)error;

/// @brief Block until the renderer's queues are idle. Call before teardown.
- (void)waitIdle;

/// The GPU the renderer selected, e.g. "Apple M5 GPU".
@property(nonatomic, readonly, copy) NSString* deviceName;

/// The negotiated Vulkan API version, e.g. "1.4.357".
@property(nonatomic, readonly, copy) NSString* apiVersion;

/// Frames successfully presented since bring-up.
@property(nonatomic, readonly) uint64_t framesPresented;

@end

NS_ASSUME_NONNULL_END

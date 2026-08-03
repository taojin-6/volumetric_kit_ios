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

#import "ARKitCapture.h"

NS_ASSUME_NONNULL_BEGIN

/// The `NSError` domain every failure below is reported in.
FOUNDATION_EXPORT NSErrorDomain const VolumetricRendererErrorDomain;

/// @brief `NSError.code` values: the *domain* of the library `Status` that
///        failed.
///
/// The domain is the primary discriminator on both sides of the seam — recon
/// and gfx each document their numeric result code as meaningful only for a
/// backend/Vulkan failure and zero everywhere else, so surfacing that code as
/// `NSError.code` would report almost every failure as `0`. The `VkResult`,
/// when there is one, rides in @ref VolumetricRendererVulkanResultKey instead
/// and is named in the localized description.
typedef NS_ERROR_ENUM(VolumetricRendererErrorDomain, VolumetricRendererError){
    VolumetricRendererErrorUnknown = 0,
    VolumetricRendererErrorInvalidArgument = 1,
    VolumetricRendererErrorNotFound = 2,
    VolumetricRendererErrorUnsupported = 3,
    VolumetricRendererErrorOutOfMemory = 4,
    VolumetricRendererErrorIoError = 5,
    /// A Vulkan call failed; the `VkResult` is in
    /// @ref VolumetricRendererVulkanResultKey.
    VolumetricRendererErrorVulkan = 6,
};

/// The failing `VkResult` as an `NSNumber`, present only on a
/// @ref VolumetricRendererErrorVulkan.
FOUNDATION_EXPORT NSErrorUserInfoKey const VolumetricRendererVulkanResultKey;

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

/// @brief Begin fusing from @p capture on a background thread.
///
/// Fusion runs off the render thread so a slow remesh does not stall
/// presentation. They still *serialize on the GPU* — iOS gives one queue, so
/// both libraries submit through one mutex — but the CPU halves overlap, and
/// the render loop never blocks waiting for a mesh: it draws the newest one
/// published, or the previous one when nothing is newer.
///
/// @param capture  The ARKit source to poll. Must outlive the renderer.
- (void)startFusionWithCapture:(VolumetricCapture*)capture;

/// @brief Stop the fuse thread and join it.
- (void)stopFusion;

/// Draw the reconstructed mesh rather than the bring-up triangle. The triangle
/// stays reachable because when the mesh first renders wrong, being able to A/B
/// against a known-good draw is worth more than the code it costs.
@property(nonatomic) BOOL drawMesh;

#pragma mark - Camera

/// @name Camera control
///
/// The camera has two modes. It starts **following the device**: the view sits
/// at the fused pose, which is what shows whether a scan in progress is
/// covering what it is being pointed at. Any of the three gestures below takes
/// it over into a turntable the user drives, seeded from wherever the follow
/// camera was; @ref followDevice hands it back.
///
/// Deltas arrive as **fractions of the viewport height** rather than points or
/// pixels. The view's size and scale factor are Swift's to know; what a drag
/// *means* belongs to the camera. Normalizing here is what keeps a given finger
/// travel producing the same rotation across devices, and keeps UIKit units out
/// of the C++.
///
/// Vertical deltas are positive **downward**, as UIKit reports them.
///
/// @warning Main thread only, alongside `renderFrameWithDrawableSize:error:` —
///          neither locks against the other.
/// @{

/// The Swift spellings are pinned with `NS_SWIFT_NAME` rather than left to the
/// importer. `orbitByFractionX:y:` contains a preposition, so the
/// omit-needless-words rules would relocate everything from `By` onward into
/// the first argument label and import it as `orbit(byFractionX:y:)` — a name
/// that reads worse and, more to the point, would change silently if the
/// selector were ever reworded.

/// @brief Swing the camera around its pivot. Dragging carries the scene with
///        the finger, so the camera travels the opposite way.
- (void)orbitByFractionX:(float)dx y:(float)dy NS_SWIFT_NAME(orbit(dx:dy:));

/// @brief Slide the pivot across the view plane, scaled so the scene keeps pace
///        with the finger at any zoom.
- (void)panByFractionX:(float)dx y:(float)dy NS_SWIFT_NAME(pan(dx:dy:));

/// @brief Pull the camera toward or away from the pivot.
/// @param scale  Relative pinch scale; greater than 1 for fingers spreading,
///               which moves the camera closer.
- (void)zoomByScale:(float)scale NS_SWIFT_NAME(zoom(scale:));

/// @brief Return the camera to the device pose.
- (void)followDevice;

/// Whether the camera is still tracking the device rather than the user.
///
/// Left without a `getter=isFollowingDevice`: Swift imports a boolean property
/// under its *getter's* name, so the custom getter would rename this on the far
/// side of the seam for no gain.
@property(nonatomic, readonly) BOOL followingDevice;

/// Distance from the turntable's pivot in metres. Only meaningful while
/// @ref followingDevice is `NO`; reported so the read-out can show it.
@property(nonatomic, readonly) float cameraDistance;

/// @}

/// Fusion read-out: fused frames, remeshes, mesh size and per-stage timings.
@property(nonatomic, readonly, copy) NSString* fusionSummary;

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

/// How the one shared `VkDevice` was built and its queues carved up, e.g.
/// "Apple M5 GPU, family 0, 2 queues (gfx + recon)".
@property(nonatomic, readonly, copy) NSString* sharedDeviceSummary;

/// Whether recon and gfx both hold the bootstrap's one `VkDevice` *and* neither
/// owns it.
///
/// Handle equality alone would be a post-condition, not evidence: both wrappers
/// were handed the same field, so they cannot differ once bring-up succeeds.
/// What makes this worth reporting is the second half — `owns_device()` is
/// false only because each went through `adopt`, so a library that quietly fell
/// back to creating a device of its own is the one thing this can actually
/// catch.
@property(nonatomic, readonly) BOOL sharesOneDevice;

@end

NS_ASSUME_NONNULL_END

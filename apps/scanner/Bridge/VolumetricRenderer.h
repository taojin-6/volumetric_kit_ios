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

/// @brief How far the viewport is turned from the camera's own basis.
///
/// ARKit fixes `ARCamera.transform` to the **sensor**, not to the interface:
/// its x-axis "always points along the long axis of the device, from the
/// front-facing camera toward the Home button", y along the short axis, z out
/// of the screen. Rotating the phone does not move that basis, so rendering it
/// straight into a portrait drawable puts the scan on its side — which reads as
/// a broken reconstruction rather than a misaligned render camera.
///
/// The values are **quarter turns**: the renderer rotates the device pose about
/// the camera's own +Z by `90° × rawValue`. Landscape-left is the sensor's own
/// basis (`UIDeviceOrientationLandscapeRight`, where ARKit documents +x as
/// pointing viewport-right), so it is the zero.
///
/// Fusion is unaffected — the pose and the intrinsics are mutually consistent
/// in the sensor frame either way — so this is a render-camera concern only.
typedef NS_ENUM(NSInteger, VolumetricViewOrientation) {
  /// The sensor's own basis; no correction.
  VolumetricViewOrientationLandscapeLeft = 0,
  VolumetricViewOrientationPortrait = 1,
  VolumetricViewOrientationLandscapeRight = 2,
  VolumetricViewOrientationPortraitUpsideDown = 3,
};

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
/// both libraries submit through one mutex — but the CPU halves overlap.
///
/// The **handoff** never blocks: the render loop takes the newest published
/// mesh or draws the previous one, and never waits for a remesh in flight. The
/// **upload** that follows it does — `gfx::pipelines::upload_mesh` is a
/// synchronous submit-and-wait, so a fresh mesh still costs the render thread a
/// queue round trip on the frame it arrives. Making that asynchronous needs a
/// transfer path gfx does not expose yet; until then `remesh_every` is the knob
/// that bounds how often it is paid.
///
/// @p capture is **retained** for as long as fusion runs — the fuse thread
/// dereferences the `ICameraCapture` it owns, and a bare "must outlive the
/// renderer" is not something the caller can honour when the two are siblings
/// with no specified destruction order.
///
/// Call @ref stopFusion before tearing the renderer down, or on leaving the
/// screen; a second call while fusion is already running does nothing.
- (void)startFusionWithCapture:(VolumetricCapture*)capture;

/// @brief Ask the fuse thread to stop, without waiting for it.
///
/// Clears the flag the loop tests between iterations and returns immediately.
/// Idempotent, and safe on a renderer that never started.
///
/// Exists so a caller on the main thread can stop new work being submitted
/// *now* and pay for the join somewhere else. @ref stopFusion has to wait out
/// the whole iteration in progress — a resize, a grow loop, an allocate, an
/// integrate and an extract, each ending in a `vkWaitForFences` with no
/// timeout — which is hundreds of milliseconds routinely and seconds on a large
/// scan. Backgrounding runs through a `UIApplication` notification handler on
/// the main thread, and blocking it for that long is a watchdog kill.
///
/// This is not a substitute for @ref stopFusion: until that returns, the thread
/// is still running and still dereferencing the capture handle.
- (void)beginStopFusion;

/// @brief Stop the fuse thread, join it, and release the capture.
///
/// Idempotent, and safe on a renderer that never started. `-dealloc` calls it,
/// so a dropped renderer cannot leave the thread running — but call it
/// explicitly when leaving the screen, because until it returns the thread is
/// still submitting recon work on the shared queue.
///
/// **Blocks** for as long as the iteration in progress takes; see
/// @ref beginStopFusion for the non-blocking half and why the two are split.
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

/// How far the viewport is turned from the sensor basis ARKit poses are in.
///
/// Swift owns this: `UIInterfaceOrientation` is a UIKit value that only the
/// view controller can read, and only on the main thread. Set it at bring-up
/// and again on every rotation; leaving it stale rotates the scan rather than
/// the camera. Affects @ref followingDevice mode, where the view *is* the
/// device pose — the turntable derives its own basis and is already upright.
@property(nonatomic) VolumetricViewOrientation viewOrientation;

/// @}

/// Fusion read-out: fused frames, remeshes, mesh size and per-stage timings.
@property(nonatomic, readonly, copy) NSString* fusionSummary;

/// @brief Record that the OS asked the app to free memory.
///
/// The one piece of this app's memory picture the OS *pushes* rather than
/// waiting to be asked for, and the only warning that arrives before jetsam
/// rather than after. Everything else on the memory row is a poll at whatever
/// rate the view happens to tick, which cannot see a spike shorter than its own
/// interval; this fires on the OS's schedule, at the moment the system decided
/// memory was short.
///
/// Recorded, not acted on. Shrinking the arenas or refusing the next allocation
/// in response is a change to allocation policy and wants its own measured
/// change; what this buys today is that a scan that was warned and then died
/// says so on the read-out and in `log collect`, which is the difference
/// between a diagnosable SIGKILL and a process that simply vanished.
///
/// Safe to call from the main thread at any point in the renderer's life,
/// including before fusion starts.
- (void)noteMemoryWarning;

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

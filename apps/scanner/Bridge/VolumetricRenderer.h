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
/// "the x-axis points to the right when the device is in
/// `UIDeviceOrientation.landscapeLeft` orientation — that is, the x-axis always
/// points along the long axis of the device, from the front-facing camera
/// toward the Home button", y along the short axis, z out of the screen.
/// Rotating the phone does not move that basis, so rendering it straight into a
/// portrait drawable puts the scan on its side — which reads as a broken
/// reconstruction rather than a misaligned render camera.
///
/// The values are **quarter turns**: the renderer rotates the pose about the
/// camera's +Z by `−90° × rawValue`. Note *which* pose — the turn is applied
/// after the CV→GL conversion, where +Z points out of the screen at the viewer.
/// recon's poses are CV (+Z along the view direction), and the same rule
/// applied to one of those comes out with the opposite sign.
///
/// **Which orientation is the zero is unsettled.** The text quoted above puts
/// it at `UIInterfaceOrientationLandscapeRight` (raw 2); the one measurement
/// anyone has taken puts it at landscape-left (raw 0), which is what ships.
/// The two differ by 180° in *every* orientation, and the check that separates
/// them has not been run. The evidence on both sides, and the check, are in
/// `-renderFrameWithDrawableSize:error:` in VolumetricRenderer.mm — read that
/// before changing these values or the sign that consumes them.
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

/// @brief One pipeline stage's host and device time, for the Swift side to
///        chart.
///
/// The counterpart to @ref fusionSummary, and the reason it is not enough: that
/// property is *rendered text*, so Swift can print it and nothing else. A chart
/// needs the numbers. Same rows, same source -- the summary is built from
/// these.
///
/// @note `gpuMs` is meaningful only where `hasGpu` is set, and a reader must
///       never distinguish the cases by inspecting `gpuMs` for a zero. But read
///       that flag as **"a device span was measured for this row"**, which is
///       the only thing it can mean, and *not* as a capability report. It is
///       false for all three of: a stage that is genuinely host-only; a call
///       that returned before dispatching anything (an empty active set, a
///       refused argument), whose host row is still charged deliberately; and
///       every stage on a queue family reporting no timestamps. Rendering "this
///       device does not support GPU timestamps" off a false here is a hardware
///       verdict drawn from a correct measurement of a stage that merely
///       early-returned. Nothing on this object can tell the three apart --
///       recon's own rule is that a caller who must know asks the device.
@interface VolumetricStageRow : NSObject
/// Stage label; a breakdown of the row above it is prefixed with spaces.
///
/// @warning A breakdown's `cpuMs` is **contained in** the `cpuMs` of the row
///          above it, while its `gpuMs` is not contained in that row's `gpuMs`
///          -- the two halves nest differently, because a host scope spans a
///          whole call and a device span covers one dispatch. So summing this
///          column over every row double-counts the host half, and the
///          host-minus-device gap of a row that *has* a breakdown is not idle
///          time. See @ref cpuMs.
@property(nonatomic, readonly, copy) NSString* name;
/// Wall clock around the stage. For a recon compute stage this covers host
/// record, submit, the fence stall AND device execution -- an end-to-end cost,
/// not a host-only one.
///
/// It follows that `cpuMs - gpuMs` is *not* a measure of host overhead wherever
/// the row has a breakdown beneath it: that child's kernel ran inside this
/// host span and reports its device time on its own row, so the difference
/// counts it as stall. See @ref name.
@property(nonatomic, readonly) double cpuMs;
/// Device time from a timestamp span around the dispatch alone.
@property(nonatomic, readonly) double gpuMs;
/// Whether @ref gpuMs holds a real measurement -- see the note on this class,
/// which is not the same claim as "this device can time the GPU".
@property(nonatomic, readonly) BOOL hasGpu;
@end

/// @brief Why a fused frame took no new geometry in, when it took none.
///
/// Mirrors `app::AllocationStop`. A cause rather than a bool because the advice
/// a reader acts on differs per cause and one of them is actively wrong for the
/// others: @ref VolumetricAllocationStopOccupancyUnknown is *not* a full
/// volume, and telling someone to coarsen their voxels there sends them after a
/// limit they have not reached.
typedef NS_ENUM(NSInteger, VolumetricAllocationStop) {
  /// The frame allocated normally.
  VolumetricAllocationStopNone = 0,
  /// Past the refuse-to-allocate guard -- the documented trade working, and the
  /// one cause a user can act on.
  VolumetricAllocationStopVolumeFull,
  /// Occupancy could not be read, so the guard refused on a fabricated figure.
  /// The fault is upstream and named in the summary's error line.
  VolumetricAllocationStopOccupancyUnknown,
  /// The allocate hit a capacity limit and blocks were dropped. Can fire with
  /// occupancy far below the guard, through bucket-local chain exhaustion.
  VolumetricAllocationStopBlocksDropped,
};

/// @brief One fused frame, for charting what a snapshot cannot show.
///
/// @ref VolumetricRenderer.stageRows is the *current* frame; this is the
/// history behind it. The distinction matters because fusion runs on its own
/// thread: polling the snapshot faster does not reveal the frames in between,
/// so a spike is only visible if it was sampled where it happened.
///
/// Totals rather than per-stage rows -- see `FrameSample` for why -- so a chart
/// answers "which frame was slow, and was it host or device", and the current
/// breakdown answers "which stage".
@interface VolumetricFrameSample : NSObject
/// Fused-frame index, so gaps are visible rather than smoothed over.
@property(nonatomic, readonly) uint64_t frame;
/// Summed host milliseconds, breakdown rows excluded.
@property(nonatomic, readonly) double hostMs;
/// Summed device milliseconds, breakdown rows included.
@property(nonatomic, readonly) double deviceMs;
/// Block-table occupancy in `[0, 1]` at this frame.
///
/// Plot this only where @ref occupancyKnown is set: an unreadable figure is
/// published as a fabricated `1.0`, so a series drawn from this column alone
/// climbs to 100% at the moment the reader most needs it not to lie.
@property(nonatomic, readonly) double occupancy;
/// Whether @ref occupancy was read or fabricated.
@property(nonatomic, readonly) BOOL occupancyKnown;
/// Both stamped by the last *successful* remesh rather than by this frame, so
/// they repeat between remeshes -- a staircase, not a per-frame series.
@property(nonatomic, readonly) uint32_t triangles;
@property(nonatomic, readonly) uint32_t activeBlocks;
/// Whether the scan took new geometry in on this frame, and if not, why.
@property(nonatomic, readonly) VolumetricAllocationStop allocationStop;
/// Whether @ref allocationStop is anything but `None`.
///
/// The bare "did it stop" for a caller that only draws a marker. Anything that
/// *names* the stop must read @ref allocationStop instead -- calling every stop
/// a full volume is the misreport that field exists to prevent.
@property(nonatomic, readonly) BOOL allocationStopped;
@end

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
/// the camera. It reaches **both** cameras, not only @ref followingDevice: the
/// turntable's steady state imposes world up, but `OrbitCamera::take_over`
/// seeds its heading from the pose's up column — the column this turn
/// rewrites — whenever the aim is steeper than about 45°. So the correction has
/// to stay where it is, applied to the pose before either camera sees it.
/// Moving it inside the follow branch keeps follow mode looking right and
/// leaves the first drag seeding from a raw sensor pose.
@property(nonatomic) VolumetricViewOrientation viewOrientation;

/// @}

/// Fusion read-out: fused frames, remeshes, mesh size and per-stage timings.
@property(nonatomic, readonly, copy) NSString* fusionSummary;

/// @brief The last fused frame's stages, for charting.
///
/// The counterpart to @ref fusionSummary and the reason it is not enough: that
/// property is *rendered text*, so Swift can print it and nothing else. Same
/// rows, same source -- the summary is built from these.
///
/// Empty when nothing has fused yet, and empty for the whole run when
/// `FusionConfig::measure_stages` is off. It is **not** emptied by a frame that
/// failed: these are the last rows measured, however long ago that was, which
/// is why the summary carries an age beside them.
///
/// Allocated per read, which is affordable because the read-out polls at a few
/// hertz. It would not be at frame rate; a caller sampling faster wants a POD
/// accessor rather than churning this.
///
/// @note This and @ref fusionSummary each take their own snapshot, so reading
///       both gives two instants of a struct the fuse thread is writing. They
///       agree in practice at a few hertz against a 60 Hz producer, but a
///       consumer that needs them to agree *exactly* should render the text
///       from these rows rather than read both.
@property(nonatomic, readonly, copy) NSArray<VolumetricStageRow*>* stageRows;

/// @brief The fused-frame history, **oldest first** -- chart order.
///
/// Sampled per fused frame on the fusion thread, so it carries the frames a
/// poll of @ref stageRows steps over. Bounded; the oldest fall off.
///
/// Allocated per read like @ref stageRows, and for the same reason it is
/// affordable: the display refreshes a few times a second, not per frame. The
/// *sampling* rate and the *display* rate are deliberately different, and this
/// property is the seam between them.
@property(nonatomic, readonly, copy)
    NSArray<VolumetricFrameSample*>* frameHistory;

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

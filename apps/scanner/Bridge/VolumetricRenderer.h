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

/// @brief Which orientation the interface is in, as a count of quarter turns.
///
/// ARKit fixes `ARCamera.transform` to the **sensor**, not to the interface:
/// its x-axis runs along the long axis of the device, y along the short axis, z
/// out of the screen. Rotating the phone does not move that basis, so rendering
/// the pose straight into a portrait drawable puts the scan on its side — which
/// reads as a broken reconstruction rather than a misaligned render camera.
/// This is what the renderer needs in order to correct for that.
///
/// This type says *which way the interface is facing* and nothing else. It
/// deliberately does **not** say which orientation needs no correction, or what
/// angle any of them implies: that is one constant,
/// `kSensorBasisOrientation` in VolumetricRenderer.mm, and it is the only place
/// in the app where an orientation becomes an angle. Read it before changing
/// anything here — the derivation, the two conflicting device sightings, and
/// the pair of measurements that finally settled it are all recorded on it.
///
/// The raw values are consecutive quarter turns, in the order below, and that
/// is load-bearing: the turn is computed by subtracting two of them.
/// VolumetricRenderer.mm static_asserts each one, so reordering this enum is a
/// build failure rather than a silently rotated scan.
///
/// Fusion is unaffected — the pose and the intrinsics are mutually consistent
/// in the sensor frame whatever this says — so it is a render-camera concern
/// only.
typedef NS_ENUM(NSInteger, VolumetricViewOrientation) {
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
/// Unavailable; see @ref VolumetricStatRow.init. A zeroed row would also plot
/// as a real stage that measured 0.00 ms, which is the one reading this file
/// takes trouble elsewhere to keep distinguishable from "not measured".
- (instancetype)init NS_UNAVAILABLE;
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
/// answers "which frame was slow, and was it fuse or mesh, host or device".
///
/// @warning It does **not** answer "which stage". @ref
///          VolumetricRenderer.stageRows holds one frame's rows and is
///          overwritten every fused frame, so the breakdown behind a spike in
///          this history is gone long before the spike is drawn. Only the
///          newest sample has a retrievable breakdown, and that is never the
///          one being investigated.
@interface VolumetricFrameSample : NSObject
/// No sample can be constructed from Swift: a zeroed one plots as a real idle
/// frame at the origin, which is indistinguishable from a measurement.
- (instancetype)init NS_UNAVAILABLE;
/// Fused-frame index. Dense and strictly sequential, so two samples can be
/// aligned by subtraction.
///
/// @warning Not a gap indicator. It advances on exactly the path that appends a
///          sample, so a frame that never fused never took an index -- a
///          tracking dropout draws as adjacent frames at normal spacing. Use
///          @ref timestampNs for elapsed time.
@property(nonatomic, readonly) uint64_t frame;
/// Monotonic nanoseconds at which the sample was taken, from the same clock
/// across the whole history. The only field that can show an outage.
@property(nonatomic, readonly) uint64_t timestampNs;
/// Summed host milliseconds, breakdown rows excluded.
///
/// @warning Not host-only time, and not stackable with @ref deviceMs. recon
///          measures a compute stage as wall clock around a *blocking* submit,
///          so this covers record, submit, fence stall and device execution --
///          it already contains most of `deviceMs`. Stacking the two
///          double-counts; subtracting them reports a device stall as host
///          overhead. Overlay them, or plot this one alone.
@property(nonatomic, readonly) double hostMs;
/// Summed device milliseconds, breakdown rows included.
///
/// Meaningful only where @ref deviceTimingValid is set -- a device that reports
/// no timestamps and a frame that dispatched nothing are both `0.0` here.
@property(nonatomic, readonly) double deviceMs;
/// Whether any stage this frame carried a real device measurement.
@property(nonatomic, readonly) BOOL deviceTimingValid;
/// Milliseconds the mesh extract took, when one ran on this frame.
///
/// Not included in @ref hostMs or @ref deviceMs -- the extract reports through
/// its own timings rather than the stage rows, so neither total can see it.
/// It is also the dominant cost in a remesh frame, and a remesh runs on every
/// fused frame by default. A "which frame was slow" chart that omits this is
/// missing the usual answer.
@property(nonatomic, readonly) double extractMs;
/// Block-table occupancy in `[0, 1]` at this frame.
///
/// Plot this only where @ref occupancyKnown is set: an unreadable figure is
/// published as a fabricated `1.0`, so a series drawn from this column alone
/// climbs to 100% at the moment the reader most needs it not to lie.
@property(nonatomic, readonly) double occupancy;
/// Whether @ref occupancy was read or fabricated.
@property(nonatomic, readonly) BOOL occupancyKnown;
/// Both stamped by the last *successful* remesh: this frame's when one ran and
/// succeeded, an older frame's when it skipped or the extract failed. They
/// flat-line in that case while every neighbouring series keeps moving, so read
/// @ref framesSinceExtract before treating a plateau as a finished surface.
@property(nonatomic, readonly) uint32_t triangles;
@property(nonatomic, readonly) uint32_t activeBlocks;
/// Fused frames since the extract that stamped @ref triangles and
/// @ref activeBlocks; `0` when this frame's own remesh refreshed them.
@property(nonatomic, readonly) uint64_t framesSinceExtract;
/// Whether the scan took new geometry in on this frame, and if not, why.
@property(nonatomic, readonly) VolumetricAllocationStop allocationStop;
/// Whether @ref allocationStop is anything but `None`.
///
/// The bare "did it stop" for a caller that only draws a marker. Anything that
/// *names* the stop must read @ref allocationStop instead -- calling every stop
/// a full volume is the misreport that field exists to prevent.
@property(nonatomic, readonly) BOOL allocationStopped;
@end

/// @brief How a stat should read: plain, healthy, or wanting attention.
typedef NS_ENUM(NSInteger, VolumetricStatTone) {
  VolumetricStatToneNeutral = 0,
  VolumetricStatToneGood,
  VolumetricStatToneWarn,
  VolumetricStatToneCritical,
};

/// @brief One labelled figure.
@interface VolumetricStatRow : NSObject
/// Unavailable for the reason @ref VolumetricRenderer's is: the properties
/// below are `nonnull`, and a zeroed instance built from Swift would hand it a
/// null `String` that traps at the first use rather than at the mistake.
- (instancetype)init NS_UNAVAILABLE;
@property(nonatomic, readonly, copy) NSString* label;
@property(nonatomic, readonly, copy) NSString* value;
/// Semantic, not decorative -- set only where a reader is meant to act.
@property(nonatomic, readonly) VolumetricStatTone tone;
@end

/// @brief A named group of figures.
///
/// The read-out used to be one string, which meant it could only ever be laid
/// out as one block of text: no grouping a UI could act on, no per-figure
/// state, and everything in it competing for the same attention. Published as
/// sections instead, so the same information can be a grid of cards -- and the
/// log line is rendered *from* these, so the screen and the transcript cannot
/// drift apart.
@interface VolumetricStatSection : NSObject
/// Unavailable; see @ref VolumetricStatRow.init.
- (instancetype)init NS_UNAVAILABLE;
@property(nonatomic, readonly, copy) NSString* title;
@property(nonatomic, readonly, copy) NSArray<VolumetricStatRow*>* rows;
@end

/// @brief Everything the dashboard draws, from **one** read of each source.
///
/// The panel used to assemble itself from the individual properties above, and
/// each of those takes its own snapshot: five separate `FusionStats` copies and
/// three `task_info` traps per tick, with the fuse thread writing between them.
/// That is not a theoretical race. The headline meter and the Volume card are
/// the same fraction from two instants, so one can read 84% beside the other
/// reading 86% with allocation stopped; the pipeline bars can belong to a frame
/// the section list does not describe.
///
/// So the whole panel is built here under one `FusionStats` copy and one
/// `MemoryBudget` query, and the figures on it are consistent by construction
/// rather than by being fast enough not to notice. The individual properties
/// remain for callers that want one figure and do not care.
@interface VolumetricDashboardSnapshot : NSObject
/// Unavailable; see @ref VolumetricStatRow.init.
- (instancetype)init NS_UNAVAILABLE;
/// The per-stage rows, end to end.
@property(nonatomic, readonly, copy) NSArray<VolumetricStageRow*>* stages;
/// The grouped figures, in the order they are meant to be read.
@property(nonatomic, readonly, copy) NSArray<VolumetricStatSection*>* sections;
/// The fused-frame history, oldest first.
@property(nonatomic, readonly, copy) NSArray<VolumetricFrameSample*>* history;
/// Live block-table occupancy -- the same figure the Volume section prints,
/// from the same copy, so the headline and the card cannot disagree.
@property(nonatomic, readonly) double occupancy;
/// Whether @ref occupancy was read or fabricated. A fabricated 1.0 must not
/// draw as a full volume.
@property(nonatomic, readonly) BOOL occupancyKnown;
@property(nonatomic, readonly) uint32_t triangles;
/// The other half of the polycount, beside @ref triangles rather than derived
/// from it: the extract emits an indexed mesh, so the ratio is a property of
/// the surface and not a constant a reader can assume.
@property(nonatomic, readonly) uint32_t vertices;
/// Why the frame took no new geometry in, if it did not.
@property(nonatomic, readonly) VolumetricAllocationStop allocationStop;
/// The stop rendered for a reader, or nil when nothing stopped. Carries the
/// *cause*: the advice for a full volume is actively wrong for the others.
@property(nonatomic, readonly, copy, nullable) NSString* allocationStopReason;

/// @name Figures a gauge is drawn from
///
/// Typed rather than pre-formatted, and that is the whole distinction between
/// these and @ref sections. A row is a sentence and a meter is a ratio; a panel
/// that wants to draw the 85% tick, or fill a bar to the fraction it measured,
/// cannot recover either from `"84.6% of 32768 blocks"`.
///
/// Most of these are *also* in a section somewhere as text, because the gauge
/// shows the position and the row shows the quantity and a reader acting on the
/// number needs the second. It is not a rule, and stating it as one was wrong:
/// @ref framesFused, @ref msSinceFuse, @ref stagesTruncated,
/// @ref gpuTimingRetired and @ref extractStale exist precisely because the
/// panel has to say something no row says -- and where a figure *is* in a
/// section, the two must not be drawn on the same card twice, which is what put
/// the same memory fraction on one card as a bar and a sentence.
/// @{

/// Triangles the last extract planned room for, in the one slot it wrote.
///
/// The denominator of the arena fill gauge. Not @ref VolumetricRenderer's arena
/// byte total, which is recon's sum across the whole ring -- see
/// `FusionStats::extract`.
@property(nonatomic, readonly) uint32_t triangleCapacity;
/// The capacity @ref occupancy is a fraction of, sampled in the same breath as
/// it.
///
/// **Not** the capacity that pairs with @ref activeBlocks. The two are stamped
/// at different cadences on purpose (see `FusionStats::table_blocks`), and a
/// panel that divides one by the other builds a ratio out of two instants --
/// which reads as a halved occupancy on exactly the frame after a doubling.
@property(nonatomic, readonly) uint32_t tableBlocks;
/// Active blocks as of the last successful remesh, with @ref extractStale
/// saying whether that is this frame.
@property(nonatomic, readonly) uint32_t activeBlocks;
/// Whether @ref activeBlocks and @ref triangleCapacity come from a remesh older
/// than this frame.
@property(nonatomic, readonly) BOOL extractStale;

/// The fuse thread's per-textured-remesh keyframe copy, which sits inside no
/// stage row and no other total. Zero when no keyframe was published.
///
/// The extract's unaccounted remainder is deliberately **not** here beside it.
/// It looks like the same kind of figure and is not: recon publishes it as the
/// `"  ..other"` stage row, so it is already one of the bars, and a second copy
/// on the snapshot only ever produced a panel that drew it twice and then said
/// it was in neither bar. This one really is outside every span there is.
@property(nonatomic, readonly) double atlasCopyMs;
/// Fused frames so far, as the panel's own answer to "has anything completed
/// yet".
///
/// The distinction @ref msSinceStages cannot make and a zero stage count does
/// not carry: before the first frame fuses all the way through there are no
/// stage rows *and* no measurement switch turned off, and a card that reports
/// the second on the strength of the first sends a reader after a config flag
/// instead of the fault in front of them.
@property(nonatomic, readonly) uint64_t framesFused;
/// Milliseconds since the stage rows were published, or 0 when none ever were.
///
/// The staleness half the bars otherwise lack. A frame count cannot carry it:
/// the rows publish on the same path that increments the fused counter, so the
/// frames that leave them behind are exactly the ones that never reach it.
///
/// Read **against @ref msSinceFuse**, never against a threshold of its own.
@property(nonatomic, readonly) double msSinceStages;
/// Milliseconds since the last fused frame, and the only thing
/// @ref msSinceStages means anything against.
///
/// The pair is the reading; neither half is one alone. An ARKit interruption
/// stops both clocks together, and a panel comparing the first to a fixed
/// second announces stale timings for the duration of a phone call -- blaming
/// the fusion for the camera. The *difference* isolates the case that is
/// actually a fault: frames arriving, none completing.
@property(nonatomic, readonly) double msSinceFuse;
/// Whether recon reported more stages than the snapshot could hold, making the
/// bars an under-report rather than a short pipeline.
@property(nonatomic, readonly) BOOL stagesTruncated;
/// Whether device timing measured once and has since retired itself, which is a
/// fault -- as distinct from a queue family that never reported timestamps,
/// which is a hardware verdict. Both leave every device bar absent.
@property(nonatomic, readonly) BOOL gpuTimingRetired;

@property(nonatomic, readonly) uint64_t memoryFootprintBytes;
@property(nonatomic, readonly) uint64_t gpuWorkingSetBytes;
/// The jetsam ceiling, or 0 when it is not known -- which is not the same as
/// zero headroom.
///
/// **`> 0` is the whole test for drawing a ratio against it**, together with
/// @ref memoryValid for the numerator. The at-limit case is already folded in
/// here: the kernel clamps the remainder there, so the derived ceiling
/// collapses onto the footprint and a bar drawn from the two would read as a
/// tidy 100% in precisely the pre-jetsam window it exists to catch -- which is
/// why this publishes 0 rather than that number. Re-testing @ref memoryAtLimit
/// beside it is a duplicate of that fold, and a consumer that lets it gate more
/// than this one ratio blanks the readings that are still good: the working set
/// is an independent measurement, and the peak is the only figure on the card
/// that survives the gap between polls at all.
@property(nonatomic, readonly) uint64_t memoryLimitBytes;
/// High-water footprint over the process's life, or 0 when the kernel did not
/// supply it. The only figure here that survives the gap between polls, and so
/// the only one that can show a `resize` spike at all.
@property(nonatomic, readonly) uint64_t memoryPeakBytes;
/// Whether the kernel answered. The byte fields above are meaningless if not,
/// and a bar drawn from their zeroes is a fabricated healthy reading.
@property(nonatomic, readonly) BOOL memoryValid;
/// Whether the kernel reports no headroom left: the last state before jetsam.
@property(nonatomic, readonly) BOOL memoryAtLimit;
/// @}
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

/// How far a vertex may sit from the current frame's depth reading and still be
/// textured, in metres. Default 0.02.
///
/// The projective-texturing visibility tolerance, live: turning it while
/// pointing at one surface is how you find the right value, which is the whole
/// reason it is here rather than a constant. Larger textures more of what the
/// camera can see and lets colour bleed through foreground edges; smaller is
/// stricter and makes textured regions go patchy, worst at range where the
/// LiDAR is noisiest.
///
/// It is compared against a *single* frame's depth, not against truth, so it is
/// absorbing sensor noise and pose error as much as real occlusion --
/// `FusionConfig::occlusion_threshold` carries the full argument.
///
/// **Silently ignores a negative or non-finite value**, keeping the previous
/// one. Every comparison with NaN is false, so storing one would texture
/// nothing at all while the read-out went on showing a texture pass that ran.
/// Reading the property back is how a caller confirms a value was taken.
@property(nonatomic) float textureOcclusionThreshold;

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
/// @brief Everything the read-out carries, grouped.
///
/// @ref fusionSummary is these joined into text for the log; this is the same
/// content before it was flattened.
@property(nonatomic, readonly, copy)
    NSArray<VolumetricStatSection*>* statSections;

/// @brief The whole panel, from one read of each source.
///
/// What a UI should call. Assembling the same panel from @ref statSections,
/// @ref stageRows, @ref frameHistory and the memory properties takes five
/// `FusionStats` copies and three `task_info` traps, which the fuse thread
/// writes between -- see @ref VolumetricDashboardSnapshot.
@property(nonatomic, readonly) VolumetricDashboardSnapshot* dashboardSnapshot;

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
/// Allocated per read, and sized to what the ring actually holds rather than to
/// its capacity, so an early scan does not marshal 240 slots to return four.
/// Affordable because the display refreshes a few times a second, not per
/// frame: the *sampling* rate and the *display* rate are deliberately
/// different, and this property is the seam between them. A caller polling at
/// frame rate is using the wrong seam.
@property(nonatomic, readonly, copy)
    NSArray<VolumetricFrameSample*>* frameHistory;

/// @brief The most entries @ref frameHistory can ever return.
///
/// Published because the alternative is every chart hard-coding the same
/// number: one that drifts below the real bound silently plots the newest
/// fraction of the ring against a full-width axis, which reads as a shorter
/// scan rather than as a bug. The C++ constant this mirrors cannot be seen from
/// Swift, so a bridged accessor is the only thing that actually prevents it.
@property(class, nonatomic, readonly) NSUInteger frameHistoryCapacity;

/// @brief Block-table capacity as of the last successful remesh — the partner
///        of @ref VolumetricDashboardSnapshot.activeBlocks, and of nothing
///        else.
///
/// **Not the denominator behind @ref VolumetricFrameSample.occupancy**, which
/// is what this said and what made it a trap. `occupancy` is read from
/// `load_factor` on every fused frame; this is stamped beside the active-block
/// count when a remesh succeeds. Dividing one by the other builds a ratio out
/// of two different instants: it reads `4.3% of 0 blocks` before the first
/// extract, halves on the frame after a doubling whose remesh skips, and then
/// freezes for the rest of the session under a persistent extract failure while
/// the map keeps growing underneath it.
///
/// For a figure to print beside the occupancy meter -- the "223k of 262k" under
/// an "85% full" -- use @ref VolumetricDashboardSnapshot.tableBlocks, which is
/// sampled in the same breath as the fraction it belongs to.
@property(nonatomic, readonly) uint32_t blockCapacity;

/// @brief What the process is charged, and the two ceilings it is charged
///        against.
///
/// Two, because they are different numbers and the **smaller one binds**:
/// @ref memoryLimitBytes is the jetsam ceiling, which on this hardware sits
/// above installed RAM and so is not what runs out first, while
/// @ref gpuWorkingSetBytes is Metal's recommended working set — the figure the
/// voxel grid and the mesh arenas are really sized against. A dashboard showing
/// only the first reports comfortable headroom that does not exist.
@property(nonatomic, readonly) uint64_t memoryFootprintBytes;
@property(nonatomic, readonly) uint64_t memoryLimitBytes;
@property(nonatomic, readonly) uint64_t gpuWorkingSetBytes;

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

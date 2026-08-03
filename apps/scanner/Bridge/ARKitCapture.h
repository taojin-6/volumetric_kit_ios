// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file ARKitCapture.h
/// @brief The ARKit implementation of recon's `sensor::ICameraCapture`.
///
/// This is the driver the 2026-08-02 sensor-tier decision placed *here* rather
/// than in recon: ARKit is iOS-only Objective-C and needs LiDAR hardware to
/// exercise, so recon's CI could neither build nor test it. What recon keeps is
/// the arithmetic — `cv_from_gl_camera` and `depth_from_registered_color`,
/// pinned by host tests — and this file is the platform plumbing that feeds
/// them.
///
/// Swift owns the `ARSession` (configuration, permissions, lifecycle) and hands
/// each `ARFrame` across; the conversion to recon's `CapturedFrame` happens on
/// this side, where an `ARFrame` and a C++ struct are both first-class.

#import <ARKit/ARKit.h>
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// @brief What the last conversion produced, for on-screen diagnostics.
///
/// Plain scalars so Swift reads them without touching a C++ type.
typedef struct {
  /// Frames converted and staged. Not every `ARFrame` Swift hands over: those
  /// arriving before the depth sensor has a result are skipped silently, and
  /// those that fail conversion land in @ref frames_rejected instead.
  uint64_t frames_submitted;
  uint64_t frames_polled;   ///< Frames a consumer actually took.
  uint64_t frames_dropped;  ///< Superseded before anyone polled them.
  /// Frames that carried depth but could not be converted -- an unreadable
  /// pixel buffer, or intrinsics recon refused. Counted rather than dropped on
  /// the floor, so a persistent failure reads as a rising number instead of a
  /// submitted count stuck at zero.
  uint64_t frames_rejected;
  uint32_t depth_width;
  uint32_t depth_height;
  uint32_t color_width;
  uint32_t color_height;
  /// Fraction of depth samples kept after confidence gating, in [0, 1].
  float confidence_kept;
  /// Depth intrinsics after rescaling from the colour camera.
  float depth_fx, depth_fy, depth_cx, depth_cy;
  /// Camera position in ARKit's world frame, after the CV conversion.
  float position_x, position_y, position_z;
  /// Milliseconds spent converting the last frame (depth + colour).
  float convert_ms;
  /// What the last colour buffer declared, read from its CVPixelBuffer
  /// attachments -- the YCbCr matrix used to reconstruct chroma, then the
  /// transfer and primaries those R'G'B' values carry, then whether that
  /// combination needed converting to recon's canonical form.
  ///
  /// Reported because the whole point of declaring an encoding is that it is
  /// observed rather than assumed, and an assumption that stays off-screen is
  /// indistinguishable from a correct reading. Static C strings, so Swift can
  /// hold them without owning them.
  const char* color_matrix;
  const char* color_transfer;
  const char* color_primaries;
  /// `true` when the frame arrived canonical and needed no conversion pass.
  bool color_was_canonical;
} VolumetricCaptureStats;

/// @brief Bridges `ARFrame`s into recon's capture contract.
///
/// Push in, poll out: ARKit delivers frames on the session queue, and a
/// consumer (the render loop, later the fuse thread) polls for the newest one.
/// That is exactly the shape `sensor::ICameraCapture` documents, and the reason
/// it is polled rather than callback-driven — the reverse would put fusion on
/// ARKit's queue.
///
/// Frames are **dropped, not queued**: a consumer slower than the sensor gets
/// the newest frame, never a backlog.
///
/// Thread-safe in exactly one shape, which is the one `ARSessionController`
/// sets up: @ref submitFrame: on the session's delegate queue, everything else
/// on the consumer's thread. Staging, polling and @ref stats share the bridge's
/// lock; @ref submitFrame: additionally owns a scratch buffer of its own and so
/// must not be called from two threads at once.
@interface VolumetricCapture : NSObject

- (instancetype)init NS_DESIGNATED_INITIALIZER;

/// @brief Convert and stage one ARKit frame. Call from the session delegate,
///        and from that queue only.
///
/// Prefers `smoothedSceneDepth` and falls back to `sceneDepth`; frames carrying
/// neither are ignored (the first frames of a session arrive before the depth
/// sensor has a result), so a caller need not filter.
- (void)submitFrame:(ARFrame*)frame;

/// @brief Take the newest staged frame, if a new one has arrived.
///
/// Wraps the C++ `ICameraCapture::poll` so Swift can drive it. The returned
/// pixels stay valid until the next call.
///
/// @return `YES` if a frame was taken; `NO` when nothing new is staged.
- (BOOL)pollLatest;

/// @brief Discard any staged frame and reset the counters.
- (void)reset;

/// Diagnostics from the most recent conversion.
@property(nonatomic, readonly) VolumetricCaptureStats stats;

/// @return The `sensor::ICameraCapture*` this wraps, as an opaque pointer.
///         The fusion slice hands this to recon; nothing else should need it.
- (void*)captureHandle NS_SWIFT_UNAVAILABLE("C++ interop only");

@end

NS_ASSUME_NONNULL_END

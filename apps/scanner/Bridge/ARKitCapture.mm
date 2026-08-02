// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ARKitCapture.h"

#import <Accelerate/Accelerate.h>
#import <CoreVideo/CoreVideo.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// ARKit's near/far working range for scene depth. Outside it the LiDAR return
// is unreliable, and recon skips samples outside the camera's range anyway.
constexpr float kMinDepth = 0.15f;
constexpr float kMaxDepth = 5.0f;

// Keep only ARConfidenceLevelMedium and above. ARKit's low-confidence samples
// are frequently metres wrong at depth discontinuities, and a TSDF carves free
// space along every ray it believes -- so a bad sample does lasting damage to
// the volume, not just to one frame.
constexpr uint8_t kMinConfidence = ARConfidenceLevelMedium;

/// Holds a CVPixelBuffer's base-address lock for a scope.
///
/// The lock can fail, and when it does `CVPixelBufferGetBaseAddress` returns
/// null -- so the result has to be checked before any pixel is touched, and the
/// unlock has to be skipped on the path that never locked. Both are easy to get
/// wrong across the early returns in the converters below, so they are a type
/// rather than a rule.
class PixelBufferLock {
 public:
  explicit PixelBufferLock(CVPixelBufferRef _Nullable buffer)
      : buffer_(buffer) {
    locked_ = buffer != nullptr &&
              CVPixelBufferLockBaseAddress(
                  buffer, kCVPixelBufferLock_ReadOnly) == kCVReturnSuccess;
  }

  ~PixelBufferLock() {
    if (locked_) {
      CVPixelBufferUnlockBaseAddress(buffer_, kCVPixelBufferLock_ReadOnly);
    }
  }

  PixelBufferLock(const PixelBufferLock&) = delete;
  PixelBufferLock& operator=(const PixelBufferLock&) = delete;

  bool locked() const { return locked_; }

 private:
  CVPixelBufferRef _Nullable buffer_;
  bool locked_ = false;
};

/// One frame's converted pixels. Owned, because ARKit recycles its
/// CVPixelBuffers as soon as the delegate returns, while `CapturedFrame` is a
/// non-owning view that must stay valid until the *next* poll.
struct FrameBuffers {
  std::vector<float> depth;
  std::vector<std::uint32_t> color;
  vr::DepthCameraParams depth_camera{};
  vr::ColorCameraParams color_camera{};
  std::uint64_t timestamp_ns = 0;
  bool has_color = false;
  float confidence_kept = 0.0f;
};

/// Copy a `DepthFloat32` plane out row by row.
///
/// A CVPixelBuffer's rows are padded to an alignment, so `bytesPerRow` is
/// generally NOT `width * sizeof(float)`. Treating the base address as tightly
/// packed is the classic mistake here: it does not crash, it shears the depth
/// image diagonally, which reads as a tracking failure rather than a bug.
///
/// Confidence gating happens in the same pass: a sample below the threshold
/// becomes 0, which the fusion kernels already skip -- the mechanism
/// `CapturedFrame::depth` documents for forwarding a confidence mask.
///
/// @return The fraction of samples kept, in [0, 1]; or nullopt if the map could
///         not be read -- an unlockable buffer, or a format that is not
///         `DepthFloat32`, which reinterpreted as `float` would produce
///         plausible garbage rather than an error.
std::optional<float> copy_depth(CVPixelBufferRef depth_buffer,
                                CVPixelBufferRef _Nullable confidence_buffer,
                                std::vector<float>& out) {
  if (CVPixelBufferGetPixelFormatType(depth_buffer) !=
      kCVPixelFormatType_DepthFloat32) {
    return std::nullopt;
  }
  const std::size_t width = CVPixelBufferGetWidth(depth_buffer);
  const std::size_t height = CVPixelBufferGetHeight(depth_buffer);
  if (width == 0 || height == 0) {
    return std::nullopt;
  }

  const PixelBufferLock depth_lock(depth_buffer);
  if (!depth_lock.locked()) {
    return std::nullopt;
  }
  const auto* src = static_cast<const std::uint8_t*>(
      CVPixelBufferGetBaseAddress(depth_buffer));
  if (src == nullptr) {
    return std::nullopt;
  }
  const std::size_t src_stride = CVPixelBufferGetBytesPerRow(depth_buffer);

  // Gate on confidence only when the map covers the same grid. `ARDepthData`
  // documents one confidence value per depth value but does not guarantee the
  // sizes in the type, and a smaller map would be read past its end -- for the
  // sake of a check that costs two comparisons a frame.
  const bool confidence_matches =
      confidence_buffer != nullptr &&
      CVPixelBufferGetWidth(confidence_buffer) == width &&
      CVPixelBufferGetHeight(confidence_buffer) == height;
  const PixelBufferLock confidence_lock(confidence_matches ? confidence_buffer
                                                           : nullptr);

  const std::uint8_t* conf = nullptr;
  std::size_t conf_stride = 0;
  if (confidence_lock.locked()) {
    conf = static_cast<const std::uint8_t*>(
        CVPixelBufferGetBaseAddress(confidence_buffer));
    conf_stride = CVPixelBufferGetBytesPerRow(confidence_buffer);
  }

  out.resize(width * height);
  std::size_t kept = 0;
  for (std::size_t y = 0; y < height; ++y) {
    const auto* row = reinterpret_cast<const float*>(src + y * src_stride);
    const std::uint8_t* conf_row =
        conf != nullptr ? conf + y * conf_stride : nullptr;
    float* dst = out.data() + y * width;
    for (std::size_t x = 0; x < width; ++x) {
      const bool trusted = conf_row == nullptr || conf_row[x] >= kMinConfidence;
      const float d = row[x];
      // NaN and non-finite returns exist in the raw map; reject them here
      // rather than letting them poison a voxel's running average.
      const bool usable = trusted && std::isfinite(d) && d > 0.0f;
      dst[x] = usable ? d : 0.0f;
      kept += usable ? 1u : 0u;
    }
  }

  return static_cast<float>(kept) / static_cast<float>(width * height);
}

/// Convert ARKit's bi-planar full-range YCbCr `capturedImage` to the packed-RGB
/// `uint32` the tsdf/mesh tiers use.
///
/// vImage rather than a hand-rolled loop: this is 2.7 M pixels per frame at
/// 1920x1440, so a scalar YCbCr->RGB would dominate the capture path.
/// Accelerate is a system framework, so it costs no dependency.
///
/// @param expected_width   The size the frame will advertise in its
/// @param expected_height  `ColorCameraParams`, which comes from
///                         `ARCamera.imageResolution` while the buffer is sized
///                         from the luma plane. A consumer reads
///                         `color_camera.width * height` pixels, so if the two
///                         ever disagreed it would read past the end of this
///                         vector. They match on every device this runs on;
///                         checking makes that an assumption the code states
///                         rather than one it silently depends on.
/// @return `true` if @p out holds a full frame of packed RGB.
bool convert_color(CVPixelBufferRef image, std::size_t expected_width,
                   std::size_t expected_height,
                   std::vector<std::uint32_t>& out) {
  if (CVPixelBufferGetPlaneCount(image) < 2) {
    return false;
  }
  // Plane geometry does not need the base-address lock; only the pixels do.
  const std::size_t width = CVPixelBufferGetWidthOfPlane(image, 0);
  const std::size_t height = CVPixelBufferGetHeightOfPlane(image, 0);
  if (width != expected_width || height != expected_height || width == 0 ||
      height == 0) {
    return false;
  }

  // Built once: the matrix depends only on the pixel format, and deriving it
  // per frame would cost more than the conversion. ARKit delivers full-range
  // ("f") 420 bi-planar, so the video-range matrix would clip highlights.
  static vImage_YpCbCrToARGB info;
  static const bool ready = [] {
    vImage_YpCbCrPixelRange range{0, 128, 255, 255, 255, 1, 255, 0};
    return vImageConvert_YpCbCrToARGB_GenerateConversion(
               kvImage_YpCbCrToARGBMatrix_ITU_R_601_4, &range, &info,
               kvImage420Yp8_CbCr8, kvImageARGB8888,
               kvImageNoFlags) == kvImageNoError;
  }();
  if (!ready) {
    return false;
  }

  out.resize(width * height);
  vImage_Buffer argb{out.data(), height, width, width * 4};

  {
    // Scoped: the permute below reads and writes only `out`, and ARKit wants
    // its buffers back, so the read lock is dropped as soon as the read ends.
    const PixelBufferLock lock(image);
    if (!lock.locked()) {
      return false;
    }
    void* luma_base = CVPixelBufferGetBaseAddressOfPlane(image, 0);
    void* chroma_base = CVPixelBufferGetBaseAddressOfPlane(image, 1);
    if (luma_base == nullptr || chroma_base == nullptr) {
      return false;
    }

    vImage_Buffer luma{luma_base, height, width,
                       CVPixelBufferGetBytesPerRowOfPlane(image, 0)};
    vImage_Buffer chroma{chroma_base, CVPixelBufferGetHeightOfPlane(image, 1),
                         CVPixelBufferGetWidthOfPlane(image, 1),
                         CVPixelBufferGetBytesPerRowOfPlane(image, 1)};

    if (vImageConvert_420Yp8_CbCr8ToARGB8888(&luma, &chroma, &argb, &info,
                                             nullptr, 255, kvImageNoFlags) !=
        kvImageNoError) {
      return false;
    }
  }

  // vImage wrote A,R,G,B in memory order; recon reads RGB from the low three
  // bytes of a little-endian uint32, i.e. R,G,B,A in memory order. That is the
  // channel permutation [1,2,3,0] -- done with vImage rather than a scalar loop
  // because this is 2.7 M pixels and the hand-written version was a measurable
  // slice of the per-frame cost on its own.
  const std::uint8_t permute[4] = {1, 2, 3, 0};
  return vImagePermuteChannels_ARGB8888(&argb, &argb, permute,
                                        kvImageNoFlags) == kvImageNoError;
}

/// The `ICameraCapture` recon consumes. Staging rotates three `FrameBuffers`:
/// the session queue converts into the caller's scratch, @ref stage swaps that
/// with `back_`, and a poll swaps `back_` to `front_`. The view handed out
/// points into `front_`, which only a poll ever touches -- so it stays valid
/// exactly as long as the contract promises (until the next poll) even while
/// ARKit keeps delivering into the other two.
class ARKitCapture final : public sensor::ICameraCapture {
 public:
  // Nothing to start: Swift owns the ARSession and pushes frames in. Present so
  // a consumer can drive any ICameraCapture uniformly.
  vr::Status start() override { return vr::Status(); }
  void stop() noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    staged_ = false;
  }

  vr::Result<std::optional<sensor::CapturedFrame>> poll() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!staged_) {
      return no_frame();
    }
    std::swap(front_, back_);
    staged_ = false;
    ++stats_.frames_polled;

    sensor::CapturedFrame frame{};
    frame.depth = front_.depth.data();
    frame.color = front_.has_color ? front_.color.data() : nullptr;
    frame.depth_camera = front_.depth_camera;
    frame.color_camera = front_.color_camera;
    frame.timestamp_ns = front_.timestamp_ns;
    return some_frame(frame);
  }

  /// Hand over a freshly converted frame, superseding any unpolled one, and
  /// record what it held for the read-out.
  ///
  /// Swaps rather than move-assigns: @p buffers comes back owning the vectors
  /// this call retired, and the caller refills *those* next frame. The three
  /// `FrameBuffers` therefore rotate, and the 11 MB colour vector is allocated
  /// once per session instead of once per frame. Move-assigning from a fresh
  /// local -- the obvious shape -- means a fresh mapping every frame, so vImage
  /// paid a first-touch fault on each of its ~690 pages while writing.
  void stage(FrameBuffers& buffers, float convert_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (staged_) {
      ++stats_.frames_dropped;  // nobody took the previous one
    }
    std::swap(back_, buffers);
    staged_ = true;
    ++stats_.frames_submitted;

    const vr::DepthCameraParams& cam = back_.depth_camera;
    stats_.depth_width = cam.width;
    stats_.depth_height = cam.height;
    stats_.color_width = back_.has_color ? back_.color_camera.width : 0;
    stats_.color_height = back_.has_color ? back_.color_camera.height : 0;
    stats_.confidence_kept = back_.confidence_kept;
    stats_.depth_fx = cam.fx;
    stats_.depth_fy = cam.fy;
    stats_.depth_cx = cam.cx;
    stats_.depth_cy = cam.cy;
    stats_.position_x = cam.cam_to_world[3].x;
    stats_.position_y = cam.cam_to_world[3].y;
    stats_.position_z = cam.cam_to_world[3].z;
    stats_.convert_ms = convert_ms;
  }

  /// Count a frame that carried depth but could not be converted.
  ///
  /// Without a counter of its own a persistent conversion failure is invisible:
  /// the submitted count simply stays at zero, which is the same "looks like a
  /// silent hang" symptom the LiDAR capability check exists to avoid.
  void reject() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.frames_rejected;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    staged_ = false;
    stats_ = VolumetricCaptureStats{};
  }

  /// Counters and scalars only, deliberately. An earlier version also returned
  /// the staged `FrameBuffers` for the read-out, which copied ~11 MB of depth +
  /// colour vectors on every submit *and* every poll -- most of a 20 ms
  /// conversion budget, spent to display a handful of scalars.
  ///
  /// Under `mutex_` like the buffers: @ref stage runs on the session queue
  /// while this runs on the render loop, so an unguarded struct would tear.
  VolumetricCaptureStats stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

 private:
  std::mutex mutex_;
  FrameBuffers front_;
  FrameBuffers back_;
  bool staged_ = false;
  VolumetricCaptureStats stats_{};
};

}  // namespace

@implementation VolumetricCapture {
  std::unique_ptr<ARKitCapture> _capture;
  /// Refilled then swapped into the capture every frame, so the frame buffers
  /// rotate rather than being reallocated. Touched only from the session queue,
  /// via `submitFrame:`.
  FrameBuffers _scratch;
}

- (instancetype)init {
  self = [super init];
  if (self != nil) {
    _capture = std::make_unique<ARKitCapture>();
  }
  return self;
}

- (void)submitFrame:(ARFrame*)frame {
  // `smoothedSceneDepth` when the session asked for it. ARKit temporally
  // filters that map against previous frames, which suppresses the per-frame
  // flicker a TSDF would otherwise average into the volume -- but it is a
  // *separate property* from `sceneDepth` (ARFrame.h), not a filter applied in
  // place, so reading the wrong one silently discards smoothing the session is
  // already paying to compute. The fallback covers the frames early in a
  // session where the filter has no history yet.
  ARDepthData* depth_data = frame.smoothedSceneDepth ?: frame.sceneDepth;
  if (depth_data == nil) {
    // Ordinary at session start: tracking begins before the depth sensor has a
    // result. Not an error, so it is not counted as a rejection either.
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();

  const std::optional<float> kept =
      copy_depth(depth_data.depthMap, depth_data.confidenceMap, _scratch.depth);
  if (!kept) {
    _capture->reject();
    return;
  }
  _scratch.confidence_kept = *kept;

  // ARKit reports one set of intrinsics, for capturedImage. The colour camera
  // is described directly by them; the depth camera is *derived* from it,
  // because sceneDepth is registered to colour -- one physical camera, so the
  // two share a pose and differ only by an image-size scale. Letting recon do
  // that derivation is the point: it is the arithmetic its host tests pin.
  const simd_float3x3 k = frame.camera.intrinsics;
  const CGSize image_size = frame.camera.imageResolution;

  vr::ColorCameraParams color{};
  color.fx = k.columns[0][0];
  color.fy = k.columns[1][1];
  color.cx = k.columns[2][0];
  color.cy = k.columns[2][1];
  color.width = static_cast<std::uint32_t>(image_size.width);
  color.height = static_cast<std::uint32_t>(image_size.height);

  // ARKit's camera looks down -Z with +Y up; recon projects +Z forward with +Y
  // down. cv_from_gl_camera is the conversion, and getting it wrong smears the
  // reconstruction rather than raising anything.
  const simd_float4x4 t = frame.camera.transform;
  vr::Mat4f arkit_pose(1.0f);
  for (int c = 0; c < 4; ++c) {
    arkit_pose[c] = vr::Vec4f(t.columns[c][0], t.columns[c][1], t.columns[c][2],
                              t.columns[c][3]);
  }
  color.cam_to_world = sensor::cv_from_gl_camera(arkit_pose);

  vr::Result<vr::DepthCameraParams> derived =
      sensor::depth_from_registered_color(
          color,
          static_cast<std::uint32_t>(
              CVPixelBufferGetWidth(depth_data.depthMap)),
          static_cast<std::uint32_t>(
              CVPixelBufferGetHeight(depth_data.depthMap)),
          kMinDepth, kMaxDepth);
  if (!derived) {
    _capture->reject();
    return;
  }
  _scratch.depth_camera = derived.value();
  _scratch.color_camera = color;
  _scratch.timestamp_ns = static_cast<std::uint64_t>(frame.timestamp * 1e9);
  // A colour failure is not a frame failure: depth alone still fuses, and
  // `CapturedFrame` spells null colour as "no colour this frame".
  _scratch.has_color = convert_color(frame.capturedImage, color.width,
                                     color.height, _scratch.color);

  const float convert_ms = std::chrono::duration<float, std::milli>(
                               std::chrono::steady_clock::now() - t0)
                               .count();
  _capture->stage(_scratch, convert_ms);
}

- (BOOL)pollLatest {
  vr::Result<std::optional<sensor::CapturedFrame>> got = _capture->poll();
  return got && got.value() ? YES : NO;
}

- (void)reset {
  _capture->reset();
}

- (VolumetricCaptureStats)stats {
  return _capture->stats();
}

- (void*)captureHandle {
  return static_cast<sensor::ICameraCapture*>(_capture.get());
}

@end

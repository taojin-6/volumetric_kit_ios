// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ARKitCapture.h"

#import <Accelerate/Accelerate.h>
#import <CoreVideo/CoreVideo.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
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
float copy_depth(CVPixelBufferRef depth_buffer,
                 CVPixelBufferRef _Nullable confidence_buffer,
                 std::vector<float>& out) {
  const std::size_t width = CVPixelBufferGetWidth(depth_buffer);
  const std::size_t height = CVPixelBufferGetHeight(depth_buffer);
  out.resize(width * height);

  CVPixelBufferLockBaseAddress(depth_buffer, kCVPixelBufferLock_ReadOnly);
  const auto* src = static_cast<const std::uint8_t*>(
      CVPixelBufferGetBaseAddress(depth_buffer));
  const std::size_t src_stride = CVPixelBufferGetBytesPerRow(depth_buffer);

  const std::uint8_t* conf = nullptr;
  std::size_t conf_stride = 0;
  if (confidence_buffer != nullptr) {
    CVPixelBufferLockBaseAddress(confidence_buffer,
                                 kCVPixelBufferLock_ReadOnly);
    conf = static_cast<const std::uint8_t*>(
        CVPixelBufferGetBaseAddress(confidence_buffer));
    conf_stride = CVPixelBufferGetBytesPerRow(confidence_buffer);
  }

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

  if (confidence_buffer != nullptr) {
    CVPixelBufferUnlockBaseAddress(confidence_buffer,
                                   kCVPixelBufferLock_ReadOnly);
  }
  CVPixelBufferUnlockBaseAddress(depth_buffer, kCVPixelBufferLock_ReadOnly);

  const std::size_t total = width * height;
  return total == 0 ? 0.0f
                    : static_cast<float>(kept) / static_cast<float>(total);
}

/// Convert ARKit's bi-planar full-range YCbCr `capturedImage` to the packed-RGB
/// `uint32` the tsdf/mesh tiers use.
///
/// vImage rather than a hand-rolled loop: this is 2.7 M pixels per frame at
/// 1920x1440, so a scalar YCbCr->RGB would dominate the capture path.
/// Accelerate is a system framework, so it costs no dependency.
bool convert_color(CVPixelBufferRef image, std::vector<std::uint32_t>& out) {
  if (CVPixelBufferGetPlaneCount(image) < 2) {
    return false;
  }
  CVPixelBufferLockBaseAddress(image, kCVPixelBufferLock_ReadOnly);

  vImage_Buffer luma{CVPixelBufferGetBaseAddressOfPlane(image, 0),
                     CVPixelBufferGetHeightOfPlane(image, 0),
                     CVPixelBufferGetWidthOfPlane(image, 0),
                     CVPixelBufferGetBytesPerRowOfPlane(image, 0)};
  vImage_Buffer chroma{CVPixelBufferGetBaseAddressOfPlane(image, 1),
                       CVPixelBufferGetHeightOfPlane(image, 1),
                       CVPixelBufferGetWidthOfPlane(image, 1),
                       CVPixelBufferGetBytesPerRowOfPlane(image, 1)};

  const std::size_t width = luma.width;
  const std::size_t height = luma.height;
  out.resize(width * height);
  vImage_Buffer argb{out.data(), height, width, width * 4};

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
    CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
    return false;
  }

  const vImage_Error err = vImageConvert_420Yp8_CbCr8ToARGB8888(
      &luma, &chroma, &argb, &info, nullptr, 255, kvImageNoFlags);
  CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
  if (err != kvImageNoError) {
    return false;
  }

  // vImage wrote A,R,G,B in memory order; recon reads RGB from the low three
  // bytes of a little-endian uint32, i.e. R,G,B,A in memory order. That is the
  // channel permutation [1,2,3,0] -- done with vImage rather than a scalar loop
  // because this is 2.7 M pixels and the hand-written version was a measurable
  // slice of the per-frame cost on its own.
  const std::uint8_t permute[4] = {1, 2, 3, 0};
  if (vImagePermuteChannels_ARGB8888(&argb, &argb, permute, kvImageNoFlags) !=
      kvImageNoError) {
    return false;
  }
  return true;
}

/// The `ICameraCapture` recon consumes. Staging is double-buffered: the session
/// queue converts into `back_`, a poll swaps it to `front_`, and the view
/// handed out points into `front_` -- so it stays valid exactly as long as the
/// contract promises (until the next poll) even while ARKit keeps delivering.
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
      return std::optional<sensor::CapturedFrame>{};
    }
    std::swap(front_, back_);
    staged_ = false;
    ++polled_;

    sensor::CapturedFrame frame{};
    frame.depth = front_.depth.data();
    frame.color = front_.has_color ? front_.color.data() : nullptr;
    frame.depth_camera = front_.depth_camera;
    frame.color_camera = front_.color_camera;
    frame.timestamp_ns = front_.timestamp_ns;
    return std::optional<sensor::CapturedFrame>{frame};
  }

  /// Hand over a freshly converted frame, superseding any unpolled one.
  void stage(FrameBuffers&& buffers) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (staged_) {
      ++dropped_;  // nobody took the previous one
    }
    back_ = std::move(buffers);
    staged_ = true;
    ++submitted_;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    staged_ = false;
    submitted_ = polled_ = dropped_ = 0;
  }

  struct Counters {
    std::uint64_t submitted, polled, dropped;
  };

  /// Counters only, deliberately. An earlier version also returned the staged
  /// `FrameBuffers` for the read-out, which copied ~11 MB of depth + colour
  /// vectors on every submit *and* every poll -- most of a 20 ms conversion
  /// budget, spent to display a handful of scalars.
  Counters snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return Counters{submitted_, polled_, dropped_};
  }

 private:
  std::mutex mutex_;
  FrameBuffers front_;
  FrameBuffers back_;
  bool staged_ = false;
  std::uint64_t submitted_ = 0;
  std::uint64_t polled_ = 0;
  std::uint64_t dropped_ = 0;
};

}  // namespace

@implementation VolumetricCapture {
  std::unique_ptr<ARKitCapture> _capture;
  VolumetricCaptureStats _stats;
}

- (instancetype)init {
  self = [super init];
  if (self != nil) {
    _capture = std::make_unique<ARKitCapture>();
    _stats = VolumetricCaptureStats{};
  }
  return self;
}

- (void)submitFrame:(ARFrame*)frame {
  ARDepthData* depth_data = frame.sceneDepth;
  if (depth_data == nil) {
    // Ordinary at session start: tracking begins before the depth sensor has a
    // result. Not an error, so it is silently skipped.
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  FrameBuffers buffers;

  buffers.confidence_kept =
      copy_depth(depth_data.depthMap, depth_data.confidenceMap, buffers.depth);

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
    return;
  }
  buffers.depth_camera = derived.value();
  buffers.color_camera = color;
  buffers.timestamp_ns = static_cast<std::uint64_t>(frame.timestamp * 1e9);
  buffers.has_color = convert_color(frame.capturedImage, buffers.color);

  const float convert_ms = std::chrono::duration<float, std::milli>(
                               std::chrono::steady_clock::now() - t0)
                               .count();

  const auto depth_cam = buffers.depth_camera;
  const float kept = buffers.confidence_kept;
  const bool had_color = buffers.has_color;
  _capture->stage(std::move(buffers));

  const auto counters = _capture->snapshot();
  _stats.frames_submitted = counters.submitted;
  _stats.frames_polled = counters.polled;
  _stats.frames_dropped = counters.dropped;
  _stats.depth_width = depth_cam.width;
  _stats.depth_height = depth_cam.height;
  _stats.color_width = had_color ? color.width : 0;
  _stats.color_height = had_color ? color.height : 0;
  _stats.confidence_kept = kept;
  _stats.depth_fx = depth_cam.fx;
  _stats.depth_fy = depth_cam.fy;
  _stats.depth_cx = depth_cam.cx;
  _stats.depth_cy = depth_cam.cy;
  _stats.position_x = depth_cam.cam_to_world[3].x;
  _stats.position_y = depth_cam.cam_to_world[3].y;
  _stats.position_z = depth_cam.cam_to_world[3].z;
  _stats.convert_ms = convert_ms;
}

- (BOOL)pollLatest {
  vr::Result<std::optional<sensor::CapturedFrame>> got = _capture->poll();
  if (!got || !got.value()) {
    return NO;
  }
  _stats.frames_polled = _capture->snapshot().polled;
  return YES;
}

- (void)reset {
  _capture->reset();
  _stats = VolumetricCaptureStats{};
}

- (VolumetricCaptureStats)stats {
  return _stats;
}

- (void*)captureHandle {
  return static_cast<sensor::ICameraCapture*>(_capture.get());
}

@end

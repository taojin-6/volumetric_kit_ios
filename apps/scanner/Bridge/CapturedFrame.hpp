// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CapturedFrame.hpp
/// @brief The frame the fuse thread polls from the capture, and the source it
///        polls.
///
/// recon's `sensor::CapturedFrame` and `ICameraCapture`, which this app
/// implemented, are gone (recon's 2026-10-07 decision): a source is now
/// volumetric_kit_sensor's `IRgbdSensor` handing out `RgbdFrame`s for its GPU
/// pass. This app still fuses host frames through recon's host-depth
/// overloads, so it keeps the frame it hands them, and the one call the
/// renderer makes on its source, here.

#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <optional>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/frame/camera_params.hpp"
#include "volumetric_kit/sensor/frame/color_space.hpp"
#include "volumetric_kit/sensor/yuv_image.hpp"

namespace volumetric_kit::ios_app {

/// @brief One posed frame, its pixels borrowed until the source's next
///        @ref FrameSource::poll.
struct CapturedFrame {
  /// Row-major depth in metres, `depth_camera.width * height` samples. A
  /// sample of 0, or outside the camera's range, is no return and fusion skips
  /// it, which is how a confidence mask is forwarded.
  const float* depth = nullptr;
  /// Row-major colour, `color_camera.width * height` pixels, R'G'B' in each
  /// word's low three bytes; null when the frame carries no colour.
  const std::uint32_t* color = nullptr;
  /// Poses already in sensor's (OpenCV) convention.
  volumetric_kit::sensor::DepthCameraParams depth_camera{};
  volumetric_kit::sensor::ColorCameraParams color_camera{};
  /// What @ref color carries; meaningful only when it is set.
  volumetric_kit::sensor::ColorEncoding color_encoding{};
  /// The camera picture @ref color was converted from, borrowed like it:
  /// ARKit's `capturedImage`, 8-bit NV12 on an IOSurface, for a renderer to
  /// sample where it is. Null when the frame carries no colour, colour that
  /// had to be brought to the canonical encoding, which the picture is not in,
  /// or chroma sited other than left or centre, which the renderer does not
  /// sample.
  CVPixelBufferRef picture = nullptr;
  /// How @ref picture codes R'G'B' -- matrix weights, range and chroma siting,
  /// as its buffer declared them -- with no planes: sensor's
  /// `PixelBufferImporter` fills those and leaves the coding as it is.
  /// Meaningful only when @ref picture is set.
  volumetric_kit::sensor::YuvImage picture_coding{};
  std::uint64_t timestamp_ns = 0;

  bool has_color() const noexcept { return color != nullptr; }
};

/// @brief What the fuse thread polls: ARKitCapture, behind the opaque handle
///        `VolumetricCapture` hands the renderer.
///
/// Single-consumer: poll from one thread, since a poll ends the borrow of the
/// frame the previous one returned.
class FrameSource {
 public:
  virtual ~FrameSource() = default;

  /// @return The newest frame not yet taken, or none when nothing new has
  ///         arrived.
  virtual volumetric_kit::core::Result<std::optional<CapturedFrame>> poll() = 0;
};

}  // namespace volumetric_kit::ios_app

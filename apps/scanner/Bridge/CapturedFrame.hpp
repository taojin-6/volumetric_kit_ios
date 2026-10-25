// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CapturedFrame.hpp
/// @brief The frame the fuse thread polls from the capture, and the source it
///        polls.
///
/// recon's `sensor::CapturedFrame` and `ICameraCapture`, which this app
/// implemented, are gone from recon (its 2026-10-07 decision): a source there
/// is an `IRgbdSensor` handing out `RgbdFrame`s for its GPU pass. This app
/// still fuses host frames through recon's host-depth overloads, so it keeps
/// the frame it hands them, and the one call the renderer makes on its source,
/// here.

#include <cstdint>
#include <optional>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"

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
  /// Poses already in recon's (OpenCV) convention.
  volumetric_kit::recon::DepthCameraParams depth_camera{};
  volumetric_kit::recon::ColorCameraParams color_camera{};
  /// What @ref color carries; meaningful only when it is set.
  volumetric_kit::recon::ColorEncoding color_encoding{};
  std::uint64_t timestamp_ns = 0;

  bool has_color() const noexcept { return color != nullptr; }
};

/// @brief What the fuse thread polls: ARKitCapture, behind the opaque handle
///        `VolumetricCapture` hands the renderer.
class FrameSource {
 public:
  virtual ~FrameSource() = default;

  /// @return The newest frame not yet taken, or none when nothing new has
  ///         arrived.
  virtual volumetric_kit::core::Result<std::optional<CapturedFrame>> poll() = 0;
};

}  // namespace volumetric_kit::ios_app

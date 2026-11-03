// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fake_rgbd_sensor.hpp
/// @brief An `IRgbdSensor` with no camera, for the session writer's tests:
///        each frame's depth carries its index (`testing::pattern_depth`),
///        and its picture is a few host bytes, as the writer reads neither.

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"

#include <glm/gtc/matrix_transform.hpp>

namespace ios_test {

namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @return A sensor as ARKit's is: tracked, on the host's clock, 60 Hz, a
///         256 x 192 depth camera beside a 1920 x 1440 colour camera.
inline sensor::SensorInfo arkit_like() {
  sensor::SensorInfo info;
  info.id = "ipad-0b";
  info.model = "Apple iPad Pro (M5) LiDAR";
  sensor::camera::CameraModel color;
  color.size = {1920, 1440};
  color.intrinsics = {1588.398, 1588.398, 958.417, 721.026};
  sensor::camera::CameraModel depth;
  depth.size = {256, 192};
  depth.intrinsics = {211.786, 211.786, 127.789, 96.137};
  info.color = color;
  info.depth = depth;
  info.pose = sensor::PoseSource::Tracked;
  info.clock = sensor::ClockDomain::Host;
  info.fps = 60;
  return info;
}

/// @return A sensor as a Femto is: fixed, on its own clock, the rig's
///         primary, 30 Hz, a 640 x 576 depth camera with a lens and an
///         extrinsic, and manual colour controls.
inline sensor::SensorInfo femto_like() {
  sensor::SensorInfo info;
  info.id = "CL8F25300D3";
  info.model = "Orbbec Femto Mega";
  sensor::camera::CameraModel color;
  color.size = {3840, 2160};
  color.intrinsics = {2240.1, 2239.7, 1921.3, 1083.9};
  color.distortion = {0.07, -0.09, 1e-4, -2e-4, 0.03, 0.0, 0.0, 0.0};
  sensor::camera::CameraModel depth;
  depth.size = {640, 576};
  depth.intrinsics = {504.2, 504.3, 321.7, 330.1};
  depth.distortion = {3.1, 1.9, 1e-5, 2e-5, 0.1, 3.4, 2.8, 0.5};
  info.color = color;
  info.depth = depth;
  info.depth_to_color =
      glm::translate(glm::dmat4(1.0), glm::dvec3(-0.032, 0.002, -0.004));
  info.role = sensor::SyncRole::Primary;
  info.clock = sensor::ClockDomain::Device;
  info.pose = sensor::PoseSource::Fixed;
  info.fps = 30;
  info.color_controls = sensor::camera::ColorControls{8330, 60};
  return info;
}

/// @brief Frames of @ref arkit_like or @ref femto_like, one a @ref poll, at
///        a fixed interval.
///
/// Frame i's depth is `pattern_depth(w, h, i)` in millimetres, with its
/// confidence plane at a floor of 2 for a tracked sensor and none for a fixed
/// one; its picture is 16 host bytes; its pose turns and walks with i.
class FakeRgbdSensor final : public sensor::IRgbdSensor {
 public:
  FakeRgbdSensor(sensor::SensorInfo info, std::uint64_t period_ns,
                 std::uint64_t first_ns = 1'000'000'000)
      : info_(std::move(info)), period_ns_(period_ns), first_ns_(first_ns) {}

  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override {
    running_ = true;
    return {};
  }
  void stop() noexcept override { running_ = false; }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    if (!running_) return no_frame();
    ++stats_.received;
    ++stats_.delivered;
    return some_frame(frame(next_++));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, poll());
    if (f) out->push_back(std::move(*f));
    return {};
  }
  sensor::SensorStats stats() const noexcept override { return stats_; }

  /// @return Frame @p i, as @ref poll hands it out.
  sensor::RgbdFrame frame(std::uint32_t i) const {
    struct Pixels {
      sensor::testing::PatternDepth depth;
      std::vector<std::uint8_t> picture;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth = sensor::testing::pattern_depth(info_.depth->size.width,
                                                   info_.depth->size.height, i);
    pixels->picture.assign(16, static_cast<std::uint8_t>(i));

    sensor::RgbdFrame f;
    f.depth = pixels->depth.depth.data();
    if (info_.pose == sensor::PoseSource::Tracked) {
      f.depth_confidence = pixels->depth.confidence.data();
      f.min_confidence = 2;
    }
    f.metres_per_unit = 0.001f;
    f.depth_camera = *info_.depth;
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    f.depth_noise = {0.0f, 0.95e-3f, 0.0f};
    if (info_.color) {
      // A stand-in for the picture: the writer takes the caller's coded one.
      f.color.host = pixels->picture.data();
      f.color.width = info_.color->size.width;
      f.color.height = info_.color->size.height;
      f.color_camera = *info_.color;
      f.color_exposure = {1.0f / 60.0f, 400.0f + i, 0.0088f};
    }
    f.color_to_world = pose(i);
    f.depth_to_color = info_.depth_to_color;
    f.timestamp_ns = first_ns_ + i * period_ns_;
    f.sequence = i;
    f.pixels = std::move(pixels);
    return f;
  }

  /// @return Frame @p i 's pose: walking at 0.5 m/s, turning at 30 deg/s.
  glm::dmat4 pose(std::uint32_t i) const {
    const double t = static_cast<double>(i) * period_ns_ * 1e-9;
    return glm::rotate(
        glm::translate(glm::dmat4(1.0), glm::dvec3(1.0, 1.4, 0.5 * t)),
        0.5236 * t, glm::dvec3(0.0, 1.0, 0.0));
  }

 private:
  sensor::SensorInfo info_;
  std::uint64_t period_ns_ = 0;
  std::uint64_t first_ns_ = 0;
  std::uint32_t next_ = 0;
  bool running_ = false;
  sensor::SensorStats stats_;
};

}  // namespace ios_test

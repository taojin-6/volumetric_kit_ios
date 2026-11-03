// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file MapSteps.hpp
/// @brief ARKit's map steps taken out of a tracked camera's poses before they
///        are sent, and the log of what was taken out.
///
/// ARKit moves its map now and then, and the camera's pose moves with it in
/// one frame or a few: spike 0b's take has three such steps in 61 s, one of
/// 127 mm and 2.5 deg, and its tracking state reads "normal" throughout.
/// A hand does not move like that. So a frame whose camera centre is further
/// from where its last two frames' motion carries it on than an acceleration
/// of @ref MapSteps::kMaxAccel (36 m/s^2) explains is taken as a step: its
/// pose becomes that prediction, the rigid step from the prediction to the
/// pose ARKit gave is taken out of every later pose, and the step is logged.
/// At 60 Hz the bound is 10 mm, twice the 5 mm off the midpoint of its
/// neighbours by which 0b's three steps, and nothing else of its frames,
/// stand out; a 30 m/s^2 hand motion is kept as it is.
///
/// The rule needs every frame the camera gives, so it runs on each pose
/// before the stream's rate gate (RateGate.hpp), and a frame sent carries the
/// pose with every step so far taken out. Until the first step, a pose comes
/// back as it was given, bit for bit.
///
/// A run of steps lasting longer than @ref MapSteps::kMaxStepNs is not one of
/// ARKit's, whose steps last a few frames: the rule then takes the next pose
/// as it is (less the steps already taken out) and starts its prediction
/// again from it. Without that, a prediction that carried on a wrong velocity
/// would take every later frame for a step and drift away.
///
/// @code
/// app::MapSteps steps;
/// app::RateGate gate(10);
/// // ... each frame ARKit gives, 60 Hz:
/// frame.color_to_world = steps.take(frame.timestamp_ns, frame.color_to_world);
/// if (!gate.take(frame.timestamp_ns)) continue;
/// // ... send the frame; at the end, write steps.steps() beside the session.
/// @endcode
///
/// @warning Not thread-safe: call it from one thread.

#include <cstdint>
#include <vector>

#include <glm/mat4x4.hpp>

namespace volumetric_kit::ios_app {

/// @brief A map step taken out: from @ref pts_ns on, a pose ARKit gave is
///        `raw_from_sent * sent`, the steps so far all in it.
struct MapStep {
  std::uint64_t pts_ns = 0;  ///< The capture time of the first frame it moved.
  glm::dmat4 raw_from_sent = glm::dmat4(1.0);  ///< Rigid.
};

class MapSteps {
 public:
  /// The acceleration of the camera centre past which a frame is a map step
  /// (m/s^2): 5 mm off the midpoint of its neighbours at 60 Hz.
  static constexpr double kMaxAccel = 36.0;
  /// How long a run of steps may last: 0b's longest is 3 frames, 33 ms.
  static constexpr std::uint64_t kMaxStepNs = 50'000'000;

  /// @param max_accel    The bound, m/s^2.
  /// @param max_step_ns  The longest run of steps.
  explicit MapSteps(double max_accel = kMaxAccel,
                    std::uint64_t max_step_ns = kMaxStepNs) noexcept;

  /// @brief The pose to send for the frame captured at @p timestamp_ns,
  ///        which ARKit placed at @p camera_to_world.
  ///
  /// The test needs the two frames before, each before it in time; the
  /// first two frames, and a frame at or before the one before it, are only
  /// corrected by the steps so far. A frame after a gap is held to the
  /// bound over the gap, which a step within it does not pass.
  /// @param timestamp_ns     Its capture time, ns, on one clock.
  /// @param camera_to_world  Its pose as ARKit gave it: rigid, metres.
  /// @return The pose with every map step so far taken out.
  glm::dmat4 take(std::uint64_t timestamp_ns,
                  const glm::dmat4& camera_to_world);

  /// @return Each step taken out, in order: the log a session's reader
  ///         rebuilds ARKit's own poses from.
  const std::vector<MapStep>& steps() const noexcept { return steps_; }

 private:
  struct Pose {
    std::uint64_t ns = 0;
    glm::dmat4 sent = glm::dmat4(1.0);
  };

  double max_accel_ = kMaxAccel;
  std::uint64_t max_step_ns_ = kMaxStepNs;
  std::vector<MapStep> steps_;
  // The inverse of the last step's raw_from_sent.
  glm::dmat4 sent_from_raw_ = glm::dmat4(1.0);
  Pose before_[2];            // the last two poses sent, the older first
  int known_ = 0;             // how many of them there are
  bool in_run_ = false;       // the last frame was a step
  std::uint64_t run_ns_ = 0;  // when that run began
};

}  // namespace volumetric_kit::ios_app

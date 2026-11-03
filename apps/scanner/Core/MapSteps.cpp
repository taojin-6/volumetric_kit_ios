// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "MapSteps.hpp"

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

namespace volumetric_kit::ios_app {

namespace {

// A rigid transform's inverse, exact for the identity.
glm::dmat4 rigid_inverse(const glm::dmat4& m) {
  const glm::dmat3 rt = glm::transpose(glm::dmat3(m));
  glm::dmat4 out(rt);
  out[3] = glm::dvec4(-(rt * glm::dvec3(m[3])), 1.0);
  return out;
}

// The rotation @p w carried on for @p ratio of its own angle.
glm::dmat3 scaled_rotation(const glm::dmat3& w, double ratio) {
  glm::dquat q = glm::quat_cast(w);
  if (q.w < 0.0) q = -q;
  const glm::dvec3 v(q.x, q.y, q.z);
  const double s = glm::length(v);
  if (s == 0.0) return glm::dmat3(1.0);
  const double half = std::atan2(s, q.w) * ratio;
  const glm::dvec3 axis = v / s;
  return glm::mat3_cast(glm::dquat(std::cos(half), axis * std::sin(half)));
}

}  // namespace

MapSteps::MapSteps(double max_accel, std::uint64_t max_step_ns) noexcept
    : max_accel_(max_accel), max_step_ns_(max_step_ns) {}

glm::dmat4 MapSteps::take(std::uint64_t timestamp_ns,
                          const glm::dmat4& camera_to_world) {
  // The pose less the steps so far; before the first, exactly as given.
  glm::dmat4 sent =
      steps_.empty() ? camera_to_world : sent_from_raw_ * camera_to_world;

  const auto remember = [&](const glm::dmat4& pose) {
    if (known_ == 2) before_[0] = before_[1];
    before_[known_ == 2 ? 1 : known_] = {timestamp_ns, pose};
    if (known_ < 2) ++known_;
  };

  if (known_ < 2 || timestamp_ns <= before_[1].ns ||
      before_[1].ns <= before_[0].ns) {
    in_run_ = false;
    remember(sent);
    return sent;
  }

  // Where the last two frames' motion carries the camera on to: constant
  // velocity, and constant angular velocity, over this frame's interval.
  const Pose& older = before_[0];
  const Pose& last = before_[1];
  const double d1 = static_cast<double>(last.ns - older.ns) * 1e-9;
  const double d2 = static_cast<double>(timestamp_ns - last.ns) * 1e-9;
  const double ratio = d2 / d1;
  const glm::dvec3 p1(last.sent[3]);
  const glm::dvec3 p0(older.sent[3]);
  const glm::dvec3 predicted = p1 + (p1 - p0) * ratio;
  // Constant acceleration a leaves the centre a * d2 * (d1 + d2) / 2 off that.
  const double accel =
      glm::length(glm::dvec3(sent[3]) - predicted) / (d2 * (d1 + d2) * 0.5);
  if (!(accel > max_accel_)) {
    in_run_ = false;
    remember(sent);
    return sent;
  }

  if (in_run_ && timestamp_ns - run_ns_ > max_step_ns_) {
    // Too long for a step: start again from this pose.
    in_run_ = false;
    known_ = 0;
    remember(sent);
    return sent;
  }
  if (!in_run_) {
    in_run_ = true;
    run_ns_ = timestamp_ns;
  }

  const glm::dmat3 r1(last.sent);
  const glm::dmat3 r0(older.sent);
  glm::dmat4 expected(scaled_rotation(r1 * glm::transpose(r0), ratio) * r1);
  expected[3] = glm::dvec4(predicted, 1.0);
  // sent = step * expected: the step is on the world's side, so it moves
  // every later pose alike, and raw = (the steps before) * step * expected.
  const glm::dmat4 step = sent * rigid_inverse(expected);
  const glm::dmat4 raw_from_sent =
      (steps_.empty() ? glm::dmat4(1.0) : steps_.back().raw_from_sent) * step;
  steps_.push_back({timestamp_ns, raw_from_sent});
  sent_from_raw_ = rigid_inverse(raw_from_sent);
  remember(expected);
  return expected;
}

}  // namespace volumetric_kit::ios_app

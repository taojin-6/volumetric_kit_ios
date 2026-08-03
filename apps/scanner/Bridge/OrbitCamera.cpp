// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "OrbitCamera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace volumetric_kit::ios_app {

namespace {

/// ARKit's world frame is gravity-aligned with +Y up, and `cv_from_gl_camera`
/// only rewrites the camera's own basis columns — so the world the poses live
/// in reaches us +Y up regardless of how the phone is held.
constexpr glm::vec3 kWorldUp{0.0f, 1.0f, 0.0f};

/// A full viewport height of drag sweeps half a turn. Enough to get behind a
/// scan in one gesture without the scene skidding away under small corrections.
constexpr float kOrbitRadiansPerHeight = 3.1415927f;

/// How far the pivot is placed ahead of the device when the user takes over.
/// Roughly arm's length to the far side of a desk: near enough that a
/// tabletop-sized scan fills the view, far enough not to start inside it.
constexpr float kTakeoverPivotDistance = 1.5f;

/// Distance limits, in metres. The ceiling is the load-bearing one: an
/// unbounded zoom-out would put the pivot past the far plane, clipping the scan
/// away entirely and reading as "the render broke".
///
/// Derived from the far plane rather than written next to it, so the two cannot
/// drift: half of it, which leaves the same again behind the pivot for the
/// scene to occupy. The first value here was a flat 10 m against a 20 m plane,
/// and a room-scale scan sat pinned against it -- the clamp had become the
/// limit the user was fighting rather than a backstop.
constexpr float kMinDistance = 0.05f;
constexpr float kMaxDistance = kFarClip * 0.5f;

/// Keep the eye off the poles. At exactly ±90° the pivot-to-eye direction is
/// parallel to the world up vector, `glm::lookAt`'s cross product collapses,
/// and the view matrix comes out full of NaNs.
constexpr float kMaxPitch = 1.5533431f;  // 89 degrees.

}  // namespace

void OrbitCamera::set_device_pose(const glm::mat4& camera_to_world) {
  device_pose_ = camera_to_world;
}

glm::vec3 OrbitCamera::direction() const {
  const float cos_pitch = std::cos(pitch_);
  return glm::vec3(cos_pitch * std::sin(yaw_), std::sin(pitch_),
                   cos_pitch * std::cos(yaw_));
}

glm::mat4 OrbitCamera::view() const {
  if (following_) {
    return glm::inverse(device_pose_);
  }
  return glm::lookAt(target_ + direction() * distance_, target_, kWorldUp);
}

void OrbitCamera::take_over() {
  if (!following_) {
    return;
  }
  following_ = false;

  // Start from what is already on screen, so the first drag moves the view
  // instead of teleporting it. In the GL convention the camera looks down its
  // own −Z, so world-space forward is the negated third basis column.
  const glm::vec3 eye(device_pose_[3]);
  const glm::vec3 forward = -glm::vec3(device_pose_[2]);

  distance_ = kTakeoverPivotDistance;
  target_ = eye + forward * distance_;

  // Position and view direction carry over; *roll* does not. A turntable has
  // none by construction, so a phone held tilted snaps level at the moment of
  // takeover. That is the constraint doing its job — an inspection camera that
  // inherited a 20° tilt would have no gesture to undo it — but it is a visible
  // snap, not a seamless handover.
  //
  // Heading carries over too, but only because of the fallback below; the
  // obvious spelling of it does not. See there.
  const glm::vec3 dir = -forward;
  // Clamped like every other write to pitch_, so no path can leave the class
  // holding a pitch the rest of it assumes is impossible. A phone aimed
  // straight down at a tabletop -- the ordinary way to scan a small object --
  // reaches this line at ±90°, which is the pole `kMaxPitch` exists to keep the
  // eye off.
  //
  // Measured, not assumed: the pole does *not* produce a NaN. `asin(1.0f)`
  // lands on 1.57079625, a hair below π/2, so `cos` of it is 7.5e-8 rather than
  // zero and every basis downstream stays finite and unit-length. But the
  // camera's whole orientation there rests on the direction that last bit
  // happened to round -- had `asin` returned the float *above* π/2 instead, the
  // residue would have been negative. Depending on that is not worth the line
  // it saves.
  pitch_ = std::clamp(std::asin(std::clamp(dir.y, -1.0f, 1.0f)), -kMaxPitch,
                      kMaxPitch);

  // Heading is the same pole problem, and it does *not* resolve the same way.
  // `atan2(dir.x, dir.z)` reads the horizontal part of the view direction,
  // whose length is cos(pitch) -- so in the posture named above, a phone aimed
  // straight down at a tabletop, it is reading two numbers that are both ~0.
  // The result is whatever a hand tremor dictates, or exactly 0 at
  // `atan2(0, 0)`: a heading unrelated to where the phone was pointed. The
  // first drag would not nudge the view, it would snap the whole scene to a new
  // heading under the finger, differently each time from the same phone pose.
  //
  // The camera's own up vector is the complement: its horizontal length is
  // sin(pitch), so it is longest exactly where the view direction's is
  // shortest, and at the pole it is fully horizontal and still says where the
  // phone was aimed. The two agree wherever both are defined -- for a turntable
  // eye at (yaw, pitch), lookAt's up vector comes out with horizontal part
  // -sin(pitch) * (sin yaw, cos yaw), so negating it recovers the same yaw --
  // which is what makes picking the longer of the two continuous rather than a
  // seam.
  const glm::vec3 cam_up(device_pose_[1]);
  glm::vec2 heading(dir.x, dir.z);
  const glm::vec2 from_up(-cam_up.x, -cam_up.z);
  if (glm::dot(from_up, from_up) > glm::dot(heading, heading)) {
    heading = from_up;
  }
  // The two lengths are cos(pitch) and sin(pitch), so one of them is always at
  // least 1/sqrt(2) and this guard is unreachable for any orthonormal pose. It
  // is here because `device_pose_` arrives from outside the class.
  yaw_ = glm::dot(heading, heading) > 0.0f ? std::atan2(heading.x, heading.y)
                                           : 0.0f;
}

void OrbitCamera::orbit(float dx, float dy) {
  take_over();
  // Away from the drag: the finger is notionally on the scene, not the camera,
  // so dragging right must carry the scene right and the camera the other way.
  yaw_ -= dx * kOrbitRadiansPerHeight;
  // With, though, because dy is positive downward: a downward drag pulls the
  // near side of the scene down, which lifts the camera over the top of it.
  pitch_ =
      std::clamp(pitch_ + dy * kOrbitRadiansPerHeight, -kMaxPitch, kMaxPitch);
}

void OrbitCamera::pan(float dx, float dy) {
  take_over();

  // What one viewport height of drag spans, in world units, at the pivot's
  // depth. This is the whole reason pan takes the FOV: it is what makes the
  // surface stay under the finger instead of sliding relative to it.
  const float world_per_height =
      2.0f * distance_ * std::tan(kVerticalFov * 0.5f);

  const glm::vec3 dir = direction();
  const glm::vec3 right = glm::normalize(glm::cross(kWorldUp, dir));
  const glm::vec3 up = glm::cross(dir, right);

  // The pivot moves opposite the finger for the same reason orbit does; the
  // +dy term is the downward-positive convention, again.
  target_ -= right * (dx * world_per_height);
  target_ += up * (dy * world_per_height);
}

void OrbitCamera::zoom(float scale) {
  // A pinch that reports 0 or a NaN would otherwise poison the distance
  // permanently — every later frame divides by it, and there is no gesture that
  // recovers from a NaN pivot.
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return;
  }
  take_over();
  distance_ = std::clamp(distance_ / scale, kMinDistance, kMaxDistance);
}

}  // namespace volumetric_kit::ios_app

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

/// Distance limits, in metres. The ceiling is the load-bearing one: the
/// projection's far plane is at 20 m, so an unbounded zoom-out would clip the
/// scan away entirely and read as "the render broke". Ten metres leaves the
/// same again behind the pivot for the scene to occupy.
constexpr float kMinDistance = 0.05f;
constexpr float kMaxDistance = 10.0f;

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
  yaw_ = std::atan2(dir.x, dir.z);
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

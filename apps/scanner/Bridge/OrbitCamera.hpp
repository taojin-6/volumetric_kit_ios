// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file OrbitCamera.hpp
/// @brief The view camera: either following the device, or under the user's
///        fingers.
///
/// Scanning and inspecting want opposite cameras. While the scan runs, the view
/// should sit at the device — that is the frame the depth is being fused in, so
/// it is the one that shows whether coverage is landing where it is being
/// pointed. Once there is geometry to look at, a camera bolted to the phone is
/// useless: putting the far side of a scan on screen would mean physically
/// walking around it.
///
/// So this holds both, and the gesture is the switch. It starts in
/// @ref following mode, reproducing the device pose exactly; the first orbit,
/// pan or zoom takes it over, seeded from wherever the follow camera was
/// looking, and @ref follow_device hands it back.
///
/// The manual half is a **turntable**, not a free-flying camera: a pivot, a
/// distance, and two angles, with world +Y locked as up. That constraint is the
/// point — six degrees of freedom under three gestures is how a hand-driven
/// camera ends up rolled onto its side with no obvious way back.
///
/// Pure math and glm, deliberately: no Vulkan, no recon, no Objective-C. It
/// takes a camera-to-world pose and returns a view matrix, and holds no opinion
/// about who produced either.
///
/// @warning **Main thread only.** The gesture callbacks and the `CADisplayLink`
///          that renders both run on the main thread, so nothing here locks.
///          The *device pose* crosses a thread boundary — but that read happens
///          in `Fusion::last_pose`, under its mutex, before the result is
///          handed to @ref set_device_pose.

#include <glm/glm.hpp>

namespace volumetric_kit::ios_app {

/// Vertical field of view for the render camera.
///
/// Shared, not duplicated: the projection matrix and this file's pan scaling
/// both need it, and a pan that assumes a different FOV than the one being
/// projected with drifts out from under the finger the further it is dragged.
///
/// Still the 60° guess the fusion slice introduced. When it becomes what it
/// should be — derived from ARKit's `fx` and the image height — this is the one
/// place that changes, and the gesture feel follows for free.
inline constexpr float kVerticalFov = 1.0471976f;  // 60 degrees.

/// @brief A turntable camera with a device-following mode.
///
/// Gesture deltas arrive in **fractions of the viewport height**, not points or
/// pixels. The caller owns the view and its scale factor; this owns what a drag
/// means. Normalizing at the seam is what keeps "half a screen of drag" the
/// same rotation on every device, and keeps this file free of UIKit units.
class OrbitCamera {
 public:
  /// @brief Record the newest device pose.
  ///
  /// Call once per frame, in the **OpenGL camera convention** (−Z forward, +Y
  /// up) — the same convention `glm::perspective` and `glm::lookAt` assume.
  /// recon's poses are CV, so they need `cv_from_gl_camera` on the way in.
  ///
  /// Kept even while the user is driving, because it is what a later takeover
  /// is seeded from.
  void set_device_pose(const glm::mat4& camera_to_world);

  /// @return World-to-view. Multiply the projection by this.
  glm::mat4 view() const;

  /// @return Whether the camera is still tracking the device.
  bool following() const noexcept { return following_; }

  /// @return Distance from the pivot in metres; meaningless while following.
  float distance() const noexcept { return distance_; }

  /// @brief Hand the camera back to the device pose.
  void follow_device() noexcept { following_ = true; }

  /// @brief Swing the camera around the pivot.
  /// @param dx  Horizontal drag, in fractions of viewport height. Positive is
  ///            rightward, and turns the scene rightward with the finger — so
  ///            the camera itself travels left.
  /// @param dy  Vertical drag, positive **downward** (UIKit's sense), which
  ///            tips the camera up and over the scene.
  void orbit(float dx, float dy);

  /// @brief Slide the pivot across the view plane.
  ///
  /// Scaled by distance and FOV so the scene keeps pace with the finger at any
  /// zoom, rather than crawling when close and leaping when far.
  ///
  /// @param dx  Horizontal drag, in fractions of viewport height.
  /// @param dy  Vertical drag, positive downward.
  void pan(float dx, float dy);

  /// @brief Pull the camera toward or away from the pivot.
  /// @param scale  Relative pinch scale: >1 for fingers spreading (closer).
  ///               Non-finite and non-positive values are ignored rather than
  ///               turning the distance into a NaN the view never recovers
  ///               from.
  void zoom(float scale);

 private:
  /// Seed the turntable from the device pose, once, on the first gesture.
  void take_over();

  /// The pivot-to-eye direction the two angles describe.
  glm::vec3 direction() const;

  bool following_ = true;
  glm::mat4 device_pose_{1.0f};

  glm::vec3 target_{0.0f};
  float distance_ = 1.5f;
  float yaw_ = 0.0f;
  float pitch_ = 0.0f;
};

}  // namespace volumetric_kit::ios_app

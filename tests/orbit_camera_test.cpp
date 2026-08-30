// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file orbit_camera_test.cpp
/// @brief The turntable camera's math, and the takeover seed in particular.
///
/// `OrbitCamera::take_over` is where this class has been wrong: it reconstructs
/// a turntable from an arbitrary device pose, and two of its branches exist
/// only because the obvious spelling failed on real postures -- a phone aimed
/// straight down at a tabletop, and a phone aimed above the horizon. Neither is
/// exotic, and neither is visible in a code reading. They are pinned here.

#include "OrbitCamera.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <glm/glm.hpp>

namespace {

namespace app = volumetric_kit::ios_app;
using app::OrbitCamera;

/// A camera-to-world pose in the **GL** convention: -Z forward, +Y up, which is
/// what `set_device_pose` documents itself as taking.
glm::mat4 pose_looking(const glm::vec3& eye, const glm::vec3& forward) {
  const glm::vec3 f = glm::normalize(forward);
  const glm::vec3 right = glm::normalize(glm::cross(f, glm::vec3(0, 1, 0)));
  const glm::vec3 up = glm::cross(right, f);
  glm::mat4 m(1.0f);
  m[0] = glm::vec4(right, 0.0f);
  m[1] = glm::vec4(up, 0.0f);
  // GL cameras look down their own -Z, so the third basis column is *backward*.
  m[2] = glm::vec4(-f, 0.0f);
  m[3] = glm::vec4(eye, 1.0f);
  return m;
}

glm::vec3 view_eye(const glm::mat4& view) {
  return glm::vec3(glm::inverse(view)[3]);
}

glm::vec3 view_forward(const glm::mat4& view) {
  return -glm::vec3(glm::inverse(view)[2]);
}

bool all_finite(const glm::mat4& m) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

TEST(OrbitCamera, StartsFollowingTheDevice) {
  OrbitCamera cam;
  EXPECT_TRUE(cam.following());

  const glm::mat4 pose = pose_looking({1.0f, 2.0f, 3.0f}, {0.0f, 0.0f, -1.0f});
  cam.set_device_pose(pose);
  // While following, the view is exactly the device pose inverted -- no
  // turntable involved.
  const glm::mat4 expected = glm::inverse(pose);
  const glm::mat4 got = cam.view();
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      EXPECT_NEAR(got[c][r], expected[c][r], 1e-5f)
          << "[" << c << "][" << r << "]";
    }
  }
}

TEST(OrbitCamera, FirstGestureTakesOverAndFollowDeviceHandsItBack) {
  OrbitCamera cam;
  cam.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}));
  ASSERT_TRUE(cam.following());

  cam.orbit(0.1f, 0.0f);
  EXPECT_FALSE(cam.following());

  cam.follow_device();
  EXPECT_TRUE(cam.following());
}

/// Takeover seeds "from what is already on screen", and for an unclamped pitch
/// that is exact: the pivot is placed `kTakeoverPivotDistance` straight ahead,
/// and the eye then sits back down on the device's own position looking the
/// same way. So the first drag moves the view rather than teleporting it.
TEST(OrbitCamera, TakeoverPreservesPositionAndDirection) {
  OrbitCamera cam;
  const glm::vec3 eye{1.0f, 0.5f, -2.0f};
  const glm::vec3 fwd = glm::normalize(glm::vec3{0.3f, 0.0f, -1.0f});
  cam.set_device_pose(pose_looking(eye, fwd));

  cam.orbit(0.0f, 0.0f);  // Takes over without moving anything.

  EXPECT_NEAR(view_eye(cam.view()).x, eye.x, 1e-4f);
  EXPECT_NEAR(view_eye(cam.view()).y, eye.y, 1e-4f);
  EXPECT_NEAR(view_eye(cam.view()).z, eye.z, 1e-4f);

  const glm::vec3 got = view_forward(cam.view());
  EXPECT_NEAR(got.x, fwd.x, 1e-4f);
  EXPECT_NEAR(got.y, fwd.y, 1e-4f);
  EXPECT_NEAR(got.z, fwd.z, 1e-4f);
}

/// **The signed-factor regression.**
///
/// Heading is recovered from the camera's own up vector whenever that is the
/// longer of the two horizontal projections, which is every pose past 45
/// degrees of pitch. Negating the up vector alone recovers the yaw only while
/// `sin(pitch)` is positive -- and pitch is negative exactly when the phone is
/// aimed *above* the horizon. Without `copysign` cancelling that factor, this
/// pose seeds a yaw half a turn out, and the first drag snaps the whole scene
/// behind the camera.
///
/// Not a pole-only edge case: scanning a ceiling, a high shelf, or a held-up
/// object lands here. 53 degrees up, heading +X.
TEST(OrbitCamera, TakeoverKeepsHeadingWhenAimedAboveTheHorizon) {
  OrbitCamera cam;
  const glm::vec3 fwd = glm::normalize(glm::vec3{0.6f, 0.8f, 0.0f});
  cam.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, fwd));

  cam.orbit(0.0f, 0.0f);

  const glm::vec3 got = view_forward(cam.view());
  // The half-turn error shows up here as a sign flip on the horizontal part.
  EXPECT_GT(got.x, 0.0f) << "heading flipped: seeded half a turn out";
  EXPECT_NEAR(got.x, fwd.x, 1e-4f);
  EXPECT_NEAR(got.y, fwd.y, 1e-4f);
  EXPECT_NEAR(got.z, fwd.z, 1e-4f);
}

/// The pole. A phone aimed straight down at a tabletop is the ordinary way to
/// scan a small object, and it is where `atan2(dir.x, dir.z)` reads two numbers
/// that are both ~0 and returns a heading unrelated to where the phone was
/// pointed. The pitch clamp and the up-vector fallback together keep the view
/// finite and the seed stable.
///
/// Built column-wise rather than through `pose_looking`, which cannot express
/// it: `cross(forward, worldUp)` is degenerate when the two are parallel.
TEST(OrbitCamera, TakeoverSurvivesPhoneAimedStraightDown) {
  glm::mat4 pose(1.0f);
  pose[0] = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);   // right
  pose[1] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);  // up
  pose[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);   // backward: forward is -Y
  pose[3] = glm::vec4(0.0f, 2.0f, 0.0f, 1.0f);

  OrbitCamera cam;
  cam.set_device_pose(pose);
  cam.orbit(0.0f, 0.0f);

  EXPECT_TRUE(all_finite(cam.view())) << "lookAt collapsed at the pole";
  EXPECT_FLOAT_EQ(cam.distance(), 1.5f);
}

/// Positive dx is a rightward drag, which carries the scene right and so the
/// camera left; positive dy is downward (UIKit's sense), which lifts the camera
/// over the top of the scene. Both are conventions a refactor can silently
/// invert, and neither is checkable by reading.
TEST(OrbitCamera, OrbitMovesTheCameraOppositeTheFinger) {
  const glm::mat4 pose = pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});

  OrbitCamera right_drag;
  right_drag.set_device_pose(pose);
  right_drag.orbit(0.25f, 0.0f);
  EXPECT_LT(view_eye(right_drag.view()).x, -0.1f)
      << "a rightward drag must send the camera left";

  OrbitCamera down_drag;
  down_drag.set_device_pose(pose);
  down_drag.orbit(0.0f, 0.25f);
  EXPECT_GT(view_eye(down_drag.view()).y, 0.1f)
      << "a downward drag must lift the camera over the scene";
}

/// At exactly +-90 degrees the pivot-to-eye direction is parallel to world up,
/// `glm::lookAt`'s cross product collapses, and the view comes out full of
/// NaNs. `kMaxPitch` exists to make that unreachable however hard the drag.
TEST(OrbitCamera, PitchClampKeepsTheViewFiniteUnderAnyDrag) {
  OrbitCamera cam;
  cam.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}));

  for (int i = 0; i < 50; ++i) {
    cam.orbit(0.0f, 1.0f);
    ASSERT_TRUE(all_finite(cam.view())) << "NaN after " << (i + 1) << " drags";
  }
  for (int i = 0; i < 100; ++i) {
    cam.orbit(0.0f, -1.0f);
    ASSERT_TRUE(all_finite(cam.view()))
        << "NaN after " << (i + 1) << " reverse drags";
  }
}

/// A pinch reporting 0 or a NaN would poison the distance permanently -- every
/// later frame divides by it, and no gesture recovers from a NaN pivot. The
/// guard sits *before* the takeover, so a rejected pinch also leaves the camera
/// following rather than stranding it in a manual mode it never entered.
TEST(OrbitCamera, ZoomRejectsNonFiniteAndNonPositiveScales) {
  for (const float bad : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity()}) {
    OrbitCamera cam;
    cam.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}));
    cam.zoom(bad);
    EXPECT_TRUE(cam.following()) << "scale " << bad << " took the camera over";
    EXPECT_TRUE(std::isfinite(cam.distance()));
  }
}

/// The ceiling is the load-bearing one: an unbounded zoom-out would put the
/// pivot past the far plane, clipping the scan away entirely and reading as
/// "the render broke". It is derived from `kFarClip` rather than written beside
/// it, so this asserts the derivation and not a literal.
TEST(OrbitCamera, ZoomClampsToTheDerivedDistanceLimits) {
  OrbitCamera out;
  out.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}));
  for (int i = 0; i < 200; ++i) out.zoom(0.5f);
  EXPECT_FLOAT_EQ(out.distance(), app::kFarClip * 0.5f);

  OrbitCamera in;
  in.set_device_pose(pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}));
  for (int i = 0; i < 200; ++i) in.zoom(2.0f);
  EXPECT_GT(in.distance(), 0.0f);
  EXPECT_FLOAT_EQ(in.distance(), 0.05f);
}

/// Pan is scaled by distance and FOV so the scene keeps pace with the finger at
/// any zoom, rather than crawling when close and leaping when far. The same
/// drag must therefore move the pivot further the further out the camera is.
TEST(OrbitCamera, PanScalesWithDistance) {
  const glm::mat4 pose = pose_looking({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});

  OrbitCamera near_cam;
  near_cam.set_device_pose(pose);
  near_cam.zoom(2.0f);  // Closer.
  const glm::vec3 near_before = view_eye(near_cam.view());
  near_cam.pan(0.25f, 0.0f);
  const float near_moved = glm::length(view_eye(near_cam.view()) - near_before);

  OrbitCamera far_cam;
  far_cam.set_device_pose(pose);
  far_cam.zoom(0.25f);  // Further away.
  const glm::vec3 far_before = view_eye(far_cam.view());
  far_cam.pan(0.25f, 0.0f);
  const float far_moved = glm::length(view_eye(far_cam.view()) - far_before);

  EXPECT_GT(far_moved, near_moved);
}

}  // namespace

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file map_steps_test.cpp
/// @brief ARKit's map steps taken out of a 60 Hz pose chain, and a hand's
///        motion left as it is.
///
/// The chains are synthetic: a hand at 60 Hz, and a step as ARKit makes one,
/// a rigid move of the whole world in one frame or a few. Recorded poses are
/// not part of the repository, so they are checked outside CI.

#include "MapSteps.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

namespace {

namespace app = volumetric_kit::ios_app;

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kStart = 97'546'447'978'750;  // any clock's origin
constexpr std::uint64_t kFrameNs = 16'669'375;        // ARKit's 60 Hz
constexpr double kFrameS = kFrameNs * 1e-9;

double deg(double rad) { return rad * 180.0 / kPi; }
double rad(double deg) { return deg * kPi / 180.0; }

std::uint64_t time_of(std::size_t i) { return kStart + i * kFrameNs; }

// A hand held 3 m from the world's origin, walking at 0.5 m/s, turning at
// 60 deg/s and swaying; with @p shake, it also shakes at 30 m/s^2 at most,
// 0.05 m at 3.9 Hz.
glm::dmat4 hand(std::size_t i, bool shake) {
  const double t = static_cast<double>(i) * kFrameS;
  const double w = std::sqrt(30.0 / 0.05);
  const glm::dvec3 position(3.0 + (shake ? 0.05 * std::sin(w * t) : 0.0),
                            1.2 + 0.02 * std::sin(1.3 * t), 0.5 * t);
  glm::dmat4 pose = glm::translate(glm::dmat4(1.0), position);
  pose = glm::rotate(pose, rad(60.0) * t, glm::dvec3(0.0, 1.0, 0.0));
  pose = glm::rotate(pose, rad(10.0) * std::sin(0.7 * t),
                     glm::dvec3(1.0, 0.0, 0.0));
  return pose;
}

// A move of the whole world: @p mm along a fixed direction, then a turn of
// @p degrees about a fixed axis through the origin.
glm::dmat4 world_step(double mm, double degrees) {
  const glm::dmat4 turn =
      glm::rotate(glm::dmat4(1.0), rad(degrees),
                  glm::normalize(glm::dvec3(0.3, 1.0, -0.2)));
  return glm::translate(
             glm::dmat4(1.0),
             glm::normalize(glm::dvec3(1.0, -0.4, 0.6)) * mm * 1e-3) *
         turn;
}

double distance_mm(const glm::dmat4& a, const glm::dmat4& b) {
  return glm::length(glm::dvec3(a[3]) - glm::dvec3(b[3])) * 1e3;
}

double angle_deg(const glm::dmat4& a, const glm::dmat4& b) {
  const glm::dmat3 r = glm::transpose(glm::dmat3(a)) * glm::dmat3(b);
  const double c =
      std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0, -1.0, 1.0);
  return deg(std::acos(c));
}

// Takes @p n frames of the hand, ARKit's pose for frame i being
// `raw_from_true(i) * hand(i)`, and returns the poses sent.
template <class RawFromTrue>
std::vector<glm::dmat4> run(app::MapSteps& steps, std::size_t n, bool shake,
                            RawFromTrue raw_from_true) {
  std::vector<glm::dmat4> sent;
  for (std::size_t i = 0; i < n; ++i) {
    sent.push_back(steps.take(time_of(i), raw_from_true(i) * hand(i, shake)));
  }
  return sent;
}

/// 30 m/s^2 is a hard shake, not a map step: with no step, every pose goes
/// out as ARKit gave it, bit for bit.
TEST(MapSteps, AHandShakingAt30IsKeptBitForBit) {
  app::MapSteps steps;
  const std::vector<glm::dmat4> sent =
      run(steps, 3600, true, [](std::size_t) { return glm::dmat4(1.0); });
  EXPECT_TRUE(steps.steps().empty());
  for (std::size_t i = 0; i < sent.size(); ++i) {
    ASSERT_EQ(sent[i], hand(i, true)) << "frame " << i;
  }
}

/// The largest recorded: 127 mm and 2.5 deg in one frame. It is taken out of
/// that frame and every later one, and logged where it happened.
TEST(MapSteps, AStepIsTakenOutOfEveryLaterPose) {
  const glm::dmat4 step = world_step(127.0, 2.5);
  app::MapSteps steps;
  const std::vector<glm::dmat4> sent =
      run(steps, 1200, false,
          [&](std::size_t i) { return i >= 600 ? step : glm::dmat4(1.0); });
  ASSERT_EQ(steps.steps().size(), 1u);
  EXPECT_EQ(steps.steps()[0].pts_ns, time_of(600));
  EXPECT_LT(distance_mm(steps.steps()[0].raw_from_sent, step), 1.0);
  EXPECT_LT(angle_deg(steps.steps()[0].raw_from_sent, step), 0.05);
  for (std::size_t i = 0; i < sent.size(); ++i) {
    if (i < 600) {
      ASSERT_EQ(sent[i], hand(i, false)) << "frame " << i;
      continue;
    }
    ASSERT_LT(distance_mm(sent[i], hand(i, false)), 1.0) << "frame " << i;
    ASSERT_LT(angle_deg(sent[i], hand(i, false)), 0.05) << "frame " << i;
  }
}

/// ARKit spreads some steps over a few frames, as one recorded. Each part
/// is a step of its own, and the log's last says the whole.
TEST(MapSteps, AStepSpreadOverThreeFramesIsTakenOut) {
  const glm::dmat4 a = world_step(56.0, 0.8);
  const glm::dmat4 b = world_step(45.0, 0.6) * a;
  const glm::dmat4 c = world_step(14.0, 0.2) * b;
  app::MapSteps steps;
  const std::vector<glm::dmat4> sent =
      run(steps, 900, false, [&](std::size_t i) {
        if (i >= 602) return c;
        if (i == 601) return b;
        if (i == 600) return a;
        return glm::dmat4(1.0);
      });
  ASSERT_EQ(steps.steps().size(), 3u);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_EQ(steps.steps()[k].pts_ns, time_of(600 + k));
  }
  EXPECT_LT(distance_mm(steps.steps().back().raw_from_sent, c), 1.0);
  EXPECT_LT(angle_deg(steps.steps().back().raw_from_sent, c), 0.05);
  for (std::size_t i = 600; i < sent.size(); ++i) {
    ASSERT_LT(distance_mm(sent[i], hand(i, false)), 1.0) << "frame " << i;
  }
}

/// Steps compose: the log's raw_from_sent holds every step so far.
TEST(MapSteps, TwoStepsCompose) {
  const glm::dmat4 first = world_step(80.0, 1.5);
  const glm::dmat4 second = world_step(-60.0, -2.0);
  app::MapSteps steps;
  const std::vector<glm::dmat4> sent =
      run(steps, 2400, false, [&](std::size_t i) {
        if (i >= 1800) return second * first;
        if (i >= 600) return first;
        return glm::dmat4(1.0);
      });
  ASSERT_EQ(steps.steps().size(), 2u);
  EXPECT_EQ(steps.steps()[1].pts_ns, time_of(1800));
  EXPECT_LT(distance_mm(steps.steps()[1].raw_from_sent, second * first), 1.0);
  EXPECT_LT(distance_mm(sent.back(), hand(2399, false)), 1.0);
}

/// An acceleration past the bound held for 0.3 s is no step of ARKit's. Each
/// run of steps stops at 50 ms and is undone, and the prediction starts again
/// from the pose given, so once the hand is smooth again every pose goes out
/// as given, and the log still rebuilds every pose. Without the stop, the
/// prediction would carry the old velocity on and take every later frame for
/// a step; without the undo, the runs' steps would stay in every later pose.
TEST(MapSteps, ARunLongerThanAStepIsUndone) {
  constexpr std::size_t kBurst = 18;  // 0.3 s at 60 m/s^2
  app::MapSteps steps;
  std::vector<glm::dmat4> raw;
  std::vector<glm::dmat4> sent;
  glm::dvec3 p(3.0, 1.2, 0.0);
  glm::dvec3 v(0.0, 0.0, 0.5);
  for (std::size_t i = 0; i < 600; ++i) {
    if (i >= 300 && i < 300 + kBurst) v.x += 60.0 * kFrameS;
    p += v * kFrameS;
    raw.push_back(glm::translate(glm::dmat4(1.0), p));
    sent.push_back(steps.take(time_of(i), raw.back()));
  }
  // The frames taken for steps, sent other than as given.
  std::vector<std::size_t> stepped;
  for (std::size_t i = 0; i < sent.size(); ++i) {
    if (distance_mm(sent[i], raw[i]) > 1e-3) stepped.push_back(i);
  }
  ASSERT_FALSE(stepped.empty());
  EXPECT_GE(stepped.front(), 300u);
  EXPECT_LE(stepped.back(), 300 + kBurst + 2);
  // No run is longer than 50 ms: 3 frames at 60 Hz, the 4th 50.008 ms on.
  std::size_t run_frames = 1;
  for (std::size_t k = 1; k < stepped.size(); ++k) {
    run_frames = stepped[k] - stepped[k - 1] == 1 ? run_frames + 1 : 1;
    EXPECT_LE(run_frames, 3u) << "at frame " << stepped[k];
  }
  // After the burst, every pose as given: nothing of the runs stays.
  for (std::size_t i = stepped.back() + 1; i < sent.size(); ++i) {
    ASSERT_LT(distance_mm(sent[i], raw[i]), 1e-3) << "frame " << i;
  }
  // Each pose given is the log's raw_from_sent at its time, times the sent.
  glm::dmat4 raw_from_sent(1.0);
  std::size_t next = 0;
  for (std::size_t i = 0; i < sent.size(); ++i) {
    while (next < steps.steps().size() &&
           steps.steps()[next].pts_ns <= time_of(i)) {
      raw_from_sent = steps.steps()[next++].raw_from_sent;
    }
    ASSERT_LT(distance_mm(raw_from_sent * sent[i], raw[i]), 1e-3)
        << "frame " << i;
  }
}

/// The camera set down, from 1 m/s to rest in one frame, is motion: the run
/// of steps it starts is undone at 50 ms, and from then on every pose goes
/// out as given.
TEST(MapSteps, ASuddenStopIsMotion) {
  constexpr std::size_t kStop = 120;  // the first frame at rest is kStop + 1
  app::MapSteps steps;
  for (std::size_t i = 0; i < 300; ++i) {
    const double t = static_cast<double>(std::min(i, kStop)) * kFrameS;
    const glm::dmat4 raw =
        glm::translate(glm::dmat4(1.0), glm::dvec3(1.0, 1.0, 1.0 * t));
    const glm::dmat4 sent = steps.take(time_of(i), raw);
    if (i <= kStop || i > kStop + 3) {
      ASSERT_LT(distance_mm(sent, raw), 1e-3) << "frame " << i;
    }
  }
  ASSERT_FALSE(steps.steps().empty());
  EXPECT_EQ(steps.steps().back().raw_from_sent, glm::dmat4(1.0));
}

/// The test needs two frames before, each earlier: the first two frames, and
/// a frame no later than the one before it, go out with only the steps so
/// far taken out, however far they jump.
TEST(MapSteps, FramesItCannotPredictGoOutAsGiven) {
  app::MapSteps steps;
  const glm::dmat4 far = glm::translate(glm::dmat4(1.0), glm::dvec3(5.0));
  EXPECT_EQ(steps.take(time_of(0), hand(0, false)), hand(0, false));
  EXPECT_EQ(steps.take(time_of(1), far), far);
  EXPECT_EQ(steps.take(time_of(1), hand(2, false)), hand(2, false));
  EXPECT_EQ(steps.take(time_of(1), far), far);
  EXPECT_TRUE(steps.steps().empty());
}

// How far @p m's rotation is from orthonormal: the largest entry of
// R^T R - I, as a recorded take's log was measured.
double off_orthonormal(const glm::dmat4& m) {
  const glm::dmat3 r(m);
  const glm::dmat3 rtr = glm::transpose(r) * r;
  double worst = 0.0;
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) {
      worst = std::max(worst, std::abs(rtr[c][k] - (c == k ? 1.0 : 0.0)));
    }
  }
  return worst;
}

// The largest difference of two rotations' entries: an angle's acos reads
// off-orthonormal noise of 1e-7 as a turn of 0.05 deg.
double rotation_entry(const glm::dmat4& a, const glm::dmat4& b) {
  double worst = 0.0;
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      worst = std::max(worst, std::abs(a[c][r] - b[c][r]));
    }
  }
  return worst;
}

// @p pose as ARKit hands one over: each rotation entry off by up to 4.4e-7
// (a fixed pseudo-random pattern per frame), as its float rotations are.
glm::dmat4 as_arkit_gives(const glm::dmat4& pose, std::uint32_t frame) {
  std::uint32_t state = frame * 2654435761u + 1u;
  glm::dmat4 out = pose;
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      state = state * 1664525u + 1013904223u;
      const double unit = static_cast<double>(state >> 8) / 16777216.0;
      out[c][r] += (unit * 2.0 - 1.0) * 4.4e-7;
    }
  }
  return out;
}

/// Many steps on ARKit's slightly non-orthonormal rotations: each logged
/// raw_from_sent stays rigid, within 1e-6 of orthonormal (it grew about
/// threefold a step, 9.1e-6 after six on a recorded take, when the steps
/// were composed as given), and the step math holds: every step found where
/// it happened, every pose sent within 1 mm and 0.05 deg of the hand, and
/// the log rebuilding every pose ARKit gave.
TEST(MapSteps, ManyStepsStayRigid) {
  constexpr std::size_t kSteps = 40;
  constexpr std::size_t kEvery = 90;  // a step each 1.5 s
  constexpr std::size_t kFrames = (kSteps + 1) * kEvery;
  // The world as ARKit has it after each step: steps of 40-80 mm and
  // 0.6-1.4 deg, turning one way then the other.
  std::vector<glm::dmat4> world(kSteps + 1, glm::dmat4(1.0));
  for (std::size_t k = 1; k <= kSteps; ++k) {
    const double sign = k % 2 == 0 ? 1.0 : -1.0;
    world[k] = world_step(40.0 + 20.0 * static_cast<double>(k % 3),
                          sign * (0.6 + 0.4 * static_cast<double>(k % 3))) *
               world[k - 1];
  }
  app::MapSteps steps;
  std::vector<glm::dmat4> raw;
  std::vector<glm::dmat4> sent;
  for (std::size_t i = 0; i < kFrames; ++i) {
    raw.push_back(as_arkit_gives(world[i / kEvery] * hand(i, false),
                                 static_cast<std::uint32_t>(i)));
    sent.push_back(steps.take(time_of(i), raw.back()));
  }

  ASSERT_EQ(steps.steps().size(), kSteps);
  double worst = 0.0;
  for (std::size_t k = 0; k < kSteps; ++k) {
    EXPECT_EQ(steps.steps()[k].pts_ns, time_of((k + 1) * kEvery)) << k;
    worst = std::max(worst, off_orthonormal(steps.steps()[k].raw_from_sent));
    ASSERT_LT(off_orthonormal(steps.steps()[k].raw_from_sent), 1e-6)
        << "step " << k;
  }
  EXPECT_LT(distance_mm(steps.steps().back().raw_from_sent, world[kSteps]),
            1.0);
  EXPECT_LT(angle_deg(steps.steps().back().raw_from_sent, world[kSteps]), 0.05);
  // 0.05 deg is 8.7e-4 rad: a rotation entry off by no more than that.
  double worst_mm = 0.0;
  double worst_entry = 0.0;
  double worst_rebuilt_mm = 0.0;
  double worst_rebuilt_entry = 0.0;
  glm::dmat4 raw_from_sent(1.0);
  std::size_t next = 0;
  for (std::size_t i = 0; i < kFrames; ++i) {
    worst_mm = std::max(worst_mm, distance_mm(sent[i], hand(i, false)));
    worst_entry =
        std::max(worst_entry, rotation_entry(sent[i], hand(i, false)));
    while (next < steps.steps().size() &&
           steps.steps()[next].pts_ns <= time_of(i)) {
      raw_from_sent = steps.steps()[next++].raw_from_sent;
    }
    worst_rebuilt_mm = std::max(worst_rebuilt_mm,
                                distance_mm(raw_from_sent * sent[i], raw[i]));
    worst_rebuilt_entry = std::max(
        worst_rebuilt_entry, rotation_entry(raw_from_sent * sent[i], raw[i]));
  }
  EXPECT_LT(worst_mm, 1.0);
  EXPECT_LT(worst_entry, 8.7e-4);
  // A step's own frame is rebuilt only as well as ARKit's rotations are
  // rigid: 4.4e-7 off, 30 m from the world's origin, is 0.013 mm.
  EXPECT_LT(worst_rebuilt_mm, 0.1);
  EXPECT_LT(worst_rebuilt_entry, 1e-5);
  char margins[160];
  std::snprintf(margins, sizeof margins,
                "off orthonormal %.2e, sent %.3f mm %.2e, rebuilt %.4f mm "
                "%.2e",
                worst, worst_mm, worst_entry, worst_rebuilt_mm,
                worst_rebuilt_entry);
  RecordProperty("worst", margins);
}

/// Over a gap the bound grows with the gap, as a hand's reach does: 200 ms of
/// frames lost while it walks and turns is no step.
TEST(MapSteps, AGapIsNoStep) {
  app::MapSteps steps;
  for (std::size_t i = 0; i < 120; ++i) {
    if (i >= 60 && i < 72) continue;  // 200 ms lost
    EXPECT_EQ(steps.take(time_of(i), hand(i, true)), hand(i, true));
  }
  EXPECT_TRUE(steps.steps().empty());
}

}  // namespace

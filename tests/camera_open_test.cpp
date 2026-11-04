// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file camera_open_test.cpp
/// @brief When the scanner opens its camera again: one open at a time, again
///        after `NotFound` at a wake or past the backoff, never after a
///        refusal, and no more once open.

#include "CameraOpen.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace {

namespace app = volumetric_kit::ios_app;
using app::CameraOpenCause;
using app::CameraOpenEnd;
using app::CameraOpenPolicy;
using app::CameraOpenState;

constexpr std::uint64_t kS = 1'000'000'000;
constexpr std::uint64_t kT0 = 97'546'447'978'750;  // any steady clock's now

const std::string kNotFound =
    "ArkitSensor: no frame with depth from ARKit within 5000 ms: its session "
    "is interrupted (the device locked, the app in the background, or the "
    "camera in use elsewhere)";

/// The first open begins at any cause; while it is in flight nothing else
/// begins, wake or tick, however late: it may be waiting for the camera
/// prompt's answer, which has no bound.
TEST(CameraOpen, OneOpenAtATime) {
  CameraOpenPolicy policy;
  EXPECT_EQ(policy.state(), CameraOpenState::Idle);
  EXPECT_EQ(policy.opens(), 0u);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  EXPECT_EQ(policy.state(), CameraOpenState::Opening);
  EXPECT_EQ(policy.opens(), 1u);
  for (std::uint64_t t = kT0; t < kT0 + 3600 * kS; t += 60 * kS) {
    EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, t));
    EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, t));
  }
  EXPECT_EQ(policy.opens(), 1u);
  EXPECT_EQ(policy.state(), CameraOpenState::Opening);
}

/// Opened: no open follows, by any cause, and nothing of a failure remains.
TEST(CameraOpen, OnceOpenNoOpenFollows) {
  CameraOpenPolicy policy;
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  policy.end(CameraOpenEnd::Opened, "ignored", kT0 + kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Open);
  EXPECT_TRUE(policy.error().empty());
  EXPECT_EQ(policy.retry_at_ns(), 0u);
  EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, kT0 + 2 * kS));
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, kT0 + 3600 * kS));
  policy.refuse("too late");
  EXPECT_EQ(policy.state(), CameraOpenState::Open);
  EXPECT_EQ(camera_open_text(policy, kT0), "open");
}

/// NotFound: a wake opens again at once; a tick only once the backoff from
/// the open's end has passed.
TEST(CameraOpen, NotFoundOpensAgainAtAWakeOrPastTheBackoff) {
  CameraOpenPolicy policy;
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  const std::uint64_t ended = kT0 + 5 * kS;
  policy.end(CameraOpenEnd::NotFound, kNotFound, ended);
  EXPECT_EQ(policy.state(), CameraOpenState::Retrying);
  EXPECT_EQ(policy.error(), kNotFound);
  EXPECT_EQ(policy.retry_at_ns(), ended + CameraOpenPolicy::kFirstBackoffNs);

  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, ended));
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, policy.retry_at_ns() - 1));
  EXPECT_EQ(policy.state(), CameraOpenState::Retrying);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, policy.retry_at_ns()));
  EXPECT_EQ(policy.retries(), 1u);
  EXPECT_EQ(policy.error(), kNotFound) << "kept for the read-out";

  policy.end(CameraOpenEnd::NotFound, kNotFound, ended + 10 * kS);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, ended + 10 * kS))
      << "a wake waits for no backoff";
  EXPECT_EQ(policy.retries(), 2u);
  policy.end(CameraOpenEnd::Opened, "", ended + 11 * kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Open);
  EXPECT_TRUE(policy.error().empty());
  EXPECT_EQ(camera_open_text(policy, ended), "open after 2 retries");
}

/// Each open that finds no camera doubles the wait for the next tick, from
/// 2 s to 30 s, and it stays at 30 s however many fail.
TEST(CameraOpen, TheBackoffDoublesToThirtySeconds) {
  const std::uint64_t expected_s[] = {2, 4, 8, 16, 30, 30, 30};
  CameraOpenPolicy policy;
  std::uint64_t now = kT0;
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, now));
  for (const std::uint64_t wait_s : expected_s) {
    now += 5 * kS;  // the open's own wait
    policy.end(CameraOpenEnd::NotFound, kNotFound, now);
    EXPECT_EQ(policy.retry_at_ns() - now, wait_s * kS);
    now = policy.retry_at_ns();
    ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, now));
  }
  for (int i = 0; i < 100; ++i) {
    now += 5 * kS;
    policy.end(CameraOpenEnd::NotFound, kNotFound, now);
    ASSERT_EQ(policy.retry_at_ns() - now, CameraOpenPolicy::kMaxBackoffNs);
    now = policy.retry_at_ns();
    ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, now));
  }
  EXPECT_EQ(policy.retries(), 107u);
}

/// A wake while an open is in flight begins nothing, but is kept: if that
/// open finds no camera, the next tick opens at once, without the backoff.
/// The kept wake serves that one open; a tick in flight is never kept.
TEST(CameraOpen, AWakeDuringAnOpenIsKept) {
  CameraOpenPolicy policy;
  std::uint64_t now = kT0;
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, now));
  for (int i = 0; i < 5; ++i) {  // five opens found no camera: 30 s next
    now += 5 * kS;
    policy.end(CameraOpenEnd::NotFound, kNotFound, now);
    now = policy.retry_at_ns();
    ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, now));
  }
  // The sixth is in flight. 3 s in, the app is in front again: both wakes
  // (the foreground's -startFusion, then active again) begin nothing.
  EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, now + 3 * kS));
  EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, now + 3 * kS));
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, now + 3 * kS));
  EXPECT_EQ(policy.state(), CameraOpenState::Opening);
  now += 5 * kS;
  policy.end(CameraOpenEnd::NotFound, kNotFound, now);
  EXPECT_EQ(policy.retry_at_ns(), now);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, now));
  EXPECT_EQ(policy.retries(), 6u);

  // No wake during this one: the backoff again, as it stood.
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, now + 2 * kS));
  now += 5 * kS;
  policy.end(CameraOpenEnd::NotFound, kNotFound, now);
  EXPECT_EQ(policy.retry_at_ns(), now + CameraOpenPolicy::kMaxBackoffNs);
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, now + kS));

  // A wake kept during an open that opens leaves nothing behind.
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, now + kS));
  EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, now + 2 * kS));
  policy.end(CameraOpenEnd::Opened, "", now + 3 * kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Open);
  EXPECT_FALSE(policy.begin(CameraOpenCause::Tick, now + 3 * kS));
}

/// IoError, Unsupported, InvalidArgument: final. The reason stays for the
/// read-out and nothing opens again.
TEST(CameraOpen, ARefusalIsFinal) {
  const std::string denied =
      "ArkitSensor: the camera is not allowed (Settings, Privacy & Security, "
      "Camera)";
  CameraOpenPolicy policy;
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  policy.end(CameraOpenEnd::NotFound, kNotFound, kT0 + 5 * kS);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0 + 6 * kS));
  policy.end(CameraOpenEnd::Refused, denied, kT0 + 7 * kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Failed);
  EXPECT_EQ(policy.error(), denied);
  EXPECT_EQ(policy.retry_at_ns(), 0u);
  for (std::uint64_t t = kT0 + 7 * kS; t < kT0 + 600 * kS; t += kS) {
    ASSERT_FALSE(policy.begin(CameraOpenCause::Wake, t));
    ASSERT_FALSE(policy.begin(CameraOpenCause::Tick, t));
  }
  EXPECT_EQ(camera_open_text(policy, kT0), denied);
}

/// A launch that names no source opens nothing, ever.
TEST(CameraOpen, NoSourceIsRefusedWithoutAnOpen) {
  CameraOpenPolicy policy;
  policy.refuse("no camera opened: the launch's arguments were refused");
  EXPECT_EQ(policy.state(), CameraOpenState::Failed);
  EXPECT_FALSE(policy.begin(CameraOpenCause::Wake, kT0));
  EXPECT_EQ(policy.opens(), 0u);
  EXPECT_EQ(camera_open_text(policy, kT0),
            "no camera opened: the launch's arguments were refused");
}

/// An end with no open in flight changes nothing.
TEST(CameraOpen, AnEndWithNoOpenIsIgnored) {
  CameraOpenPolicy policy;
  policy.end(CameraOpenEnd::Refused, "stray", kT0);
  EXPECT_EQ(policy.state(), CameraOpenState::Idle);
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  policy.end(CameraOpenEnd::NotFound, kNotFound, kT0 + 5 * kS);
  policy.end(CameraOpenEnd::Opened, "", kT0 + 6 * kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Retrying);
  policy.end(CameraOpenEnd::Refused, "stray", kT0 + 6 * kS);
  EXPECT_EQ(policy.state(), CameraOpenState::Retrying);
  EXPECT_EQ(policy.error(), kNotFound);
}

/// The read-out says what the open is doing, the retry count, and when the
/// next tick opens.
TEST(CameraOpen, TheReadOutSaysTheStateAndTheRetries) {
  CameraOpenPolicy policy;
  EXPECT_EQ(camera_open_text(policy, kT0), "not opened yet");
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0));
  EXPECT_EQ(camera_open_text(policy, kT0), "opening");
  policy.end(CameraOpenEnd::NotFound, "no frame", kT0 + 5 * kS);
  EXPECT_EQ(camera_open_text(policy, kT0 + 5 * kS),
            "no frame; retry 1 at the next wake, or in 2 s");
  EXPECT_EQ(camera_open_text(policy, kT0 + 5 * kS + kS / 2),
            "no frame; retry 1 at the next wake, or in 2 s")
      << "rounded up";
  ASSERT_TRUE(policy.begin(CameraOpenCause::Tick, kT0 + 7 * kS));
  EXPECT_EQ(camera_open_text(policy, kT0 + 7 * kS),
            "opening again (retry 1) after: no frame");
  policy.end(CameraOpenEnd::NotFound, "no frame", kT0 + 12 * kS);
  EXPECT_EQ(camera_open_text(policy, kT0 + 13 * kS),
            "no frame; retry 2 at the next wake, or in 3 s");
  ASSERT_TRUE(policy.begin(CameraOpenCause::Wake, kT0 + 13 * kS));
  policy.end(CameraOpenEnd::Opened, "", kT0 + 14 * kS);
  EXPECT_EQ(camera_open_text(policy, kT0 + 14 * kS), "open after 2 retries");
}

}  // namespace

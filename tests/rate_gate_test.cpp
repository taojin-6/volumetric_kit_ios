// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file rate_gate_test.cpp
/// @brief Which frames a stream sends from a faster camera: one an interval,
///        the gate's rate on average, whatever the camera's timing.

#include "RateGate.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

namespace app = volumetric_kit::ios_app;

constexpr std::uint64_t kStart = 1'000'000'000'000;  // any clock's origin
constexpr std::uint64_t k60HzNs = 16'666'667;        // 60 Hz, rounded
constexpr std::uint64_t kArkitNs = 16'669'375;       // ARKit's, as measured

// The indices @p gate takes of frames at @p times.
std::vector<std::size_t> taken(app::RateGate& gate,
                               const std::vector<std::uint64_t>& times) {
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (gate.take(times[i])) out.push_back(i);
  }
  return out;
}

std::vector<std::uint64_t> steady(std::size_t n, std::uint64_t period_ns) {
  std::vector<std::uint64_t> t(n);
  for (std::size_t i = 0; i < n; ++i) t[i] = kStart + i * period_ns;
  return t;
}

// The gaps between the frames @p got takes of @p times are within
// [@p lo_ns, @p hi_ns].
void expect_gaps(const std::vector<std::uint64_t>& times,
                 const std::vector<std::size_t>& got, std::uint64_t lo_ns,
                 std::uint64_t hi_ns) {
  for (std::size_t k = 1; k < got.size(); ++k) {
    const std::uint64_t gap = times[got[k]] - times[got[k - 1]];
    EXPECT_GE(gap, lo_ns) << "frame " << got[k];
    EXPECT_LE(gap, hi_ns) << "frame " << got[k];
  }
}

TEST(RateGate, SixtyHertzAtTenTakesEverySixthFrame) {
  app::RateGate gate(10);
  const std::vector<std::size_t> got = taken(gate, steady(3600, k60HzNs));
  ASSERT_EQ(got.size(), 600u);  // a minute, one each 100 ms
  for (std::size_t k = 0; k < got.size(); ++k) EXPECT_EQ(got[k], 6 * k);
}

/// ARKit's frames come a little slower than 60 Hz, so the frame of each
/// interval drifts from every sixth; the rate is the intervals', exactly.
TEST(RateGate, ArkitsCadenceKeepsTheIntervalsRate) {
  const std::vector<std::uint64_t> t = steady(3600, kArkitNs);
  app::RateGate gate(10);
  const std::vector<std::size_t> got = taken(gate, t);
  // The frames span 59.993 s: intervals 0 to 600, each with one frame.
  EXPECT_EQ(got.size(), 601u);
  expect_gaps(t, got, 5 * kArkitNs, 6 * kArkitNs);
}

/// A camera's timestamps wander; the intervals do not, so the rate holds and
/// no interval sends two.
TEST(RateGate, JitterKeepsTheRateAndOneFrameAnInterval) {
  std::vector<std::uint64_t> t = steady(3600, kArkitNs);
  for (std::size_t i = 1; i < t.size(); ++i) {
    t[i] += (i * 7919 % 9) * 1'000'000 - 4'000'000;  // -4 .. +4 ms
  }
  app::RateGate gate(10);
  const std::vector<std::size_t> got = taken(gate, t);
  EXPECT_GE(got.size(), 600u);
  EXPECT_LE(got.size(), 601u);
  // An interval's frame is the first at or past its start less 10 ms, so it is
  // within one frame interval, jitter included, of that.
  expect_gaps(t, got, 100'000'000 - kArkitNs - 8'000'000,
              100'000'000 + kArkitNs + 8'000'000);
}

/// A camera at the gate's own rate keeps every frame, early ones included.
TEST(RateGate, ASourceAtTheGatesRateKeepsEveryFrame) {
  std::vector<std::uint64_t> t = steady(100, 100'000'000);
  for (std::size_t i = 1; i < t.size(); i += 2)
    t[i] -= 8'000'000;  // 8 ms early
  app::RateGate gate(10);
  EXPECT_EQ(taken(gate, t).size(), t.size());
}

TEST(RateGate, ASlowerSourceKeepsEveryFrame) {
  app::RateGate gate(10);
  EXPECT_EQ(taken(gate, steady(50, 200'000'000)).size(), 50u);
}

/// After a pause the next frame goes at once, and the cadence after it is the
/// gate's, not a burst making up the intervals missed.
TEST(RateGate, APauseTakesTheNextFrameWithNoBurst) {
  std::vector<std::uint64_t> t = steady(60, kArkitNs);
  // A pause of 1.04 s, so the next frame falls mid-interval in the old
  // cadence, whose next interval would start 67 ms after it.
  const std::uint64_t resume = t.back() + 1'040'000'000;
  for (std::size_t i = 0; i < 60; ++i) t.push_back(resume + i * kArkitNs);
  app::RateGate gate(10);
  const std::vector<std::size_t> got = taken(gate, t);
  std::vector<std::size_t> after;
  for (const std::size_t i : got) {
    if (i >= 60) after.push_back(i);
  }
  ASSERT_FALSE(after.empty());
  EXPECT_EQ(after.front(), 60u);  // the first frame after the pause
  EXPECT_EQ(after.size(), 10u);   // then the cadence from it
  expect_gaps(t, after, 5 * kArkitNs, 6 * kArkitNs);
}

TEST(RateGate, ATimeAtOrBeforeTheLastTakenIsNotTaken) {
  app::RateGate gate(10);
  EXPECT_TRUE(gate.take(kStart));
  EXPECT_FALSE(gate.take(kStart));
  EXPECT_FALSE(gate.take(kStart - 1));
  EXPECT_TRUE(gate.take(kStart + 100'000'000));
  EXPECT_FALSE(gate.take(kStart + 50'000'000));
}

TEST(RateGate, ZeroTakesEveryFrame) {
  app::RateGate gate(0);
  EXPECT_EQ(taken(gate, steady(120, k60HzNs)).size(), 120u);
  EXPECT_EQ(gate.hz(), 0u);
}

/// `due` says what `take` would and takes nothing, so a frame left while the
/// sender is busy keeps the interval for the next one.
TEST(RateGate, DueSaysWhatTakeWouldAndTakesNothing) {
  const std::vector<std::uint64_t> t = steady(240, k60HzNs);
  app::RateGate asked(10);
  app::RateGate plain(10);
  for (const std::uint64_t ns : t) {
    const bool due = asked.due(ns);
    EXPECT_EQ(asked.due(ns), due);  // asking twice changes nothing
    EXPECT_EQ(asked.take(ns), due);
    EXPECT_EQ(plain.take(ns), due);
  }
  // Frame 0 is due and left; frame 1 then takes the interval.
  app::RateGate left(10);
  EXPECT_TRUE(left.due(t[0]));
  EXPECT_TRUE(left.take(t[1]));
  EXPECT_FALSE(left.due(t[2]));
  EXPECT_TRUE(left.due(t[7]));
}

}  // namespace

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file stream_rate_test.cpp
/// @brief The panel's stream rate reads steadily at a whole window's rate.

#include "StreamRate.hpp"

#include <gtest/gtest.h>

namespace {

using volumetric_kit::ios_app::StreamRate;

TEST(StreamRate, ZeroBeforeAWholeWindow) {
  StreamRate rate;
  EXPECT_EQ(rate.take(100, 1.0), 0.0);
  EXPECT_EQ(rate.take(110, 2.5), 0.0);
}

TEST(StreamRate, IsTheCountOverTheWindow) {
  StreamRate rate;
  (void)rate.take(100, 1.0);
  EXPECT_DOUBLE_EQ(rate.take(120, 3.0), 10.0);
  // Held between windows, whatever the count did meanwhile.
  EXPECT_DOUBLE_EQ(rate.take(121, 3.5), 10.0);
  EXPECT_DOUBLE_EQ(rate.take(130, 6.0), 10.0 / 3.0);
}

TEST(StreamRate, ACountThatStoppedReadsZero) {
  StreamRate rate;
  (void)rate.take(100, 0.0);
  EXPECT_DOUBLE_EQ(rate.take(100, 2.0), 0.0);
}

/// A count that went backwards is a new count, not a huge rate.
TEST(StreamRate, ACountThatRestartedReadsZero) {
  StreamRate rate;
  (void)rate.take(100, 0.0);
  EXPECT_DOUBLE_EQ(rate.take(5, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(rate.take(25, 4.0), 10.0);
}

}  // namespace

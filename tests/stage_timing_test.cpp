// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file stage_timing_test.cpp
/// @brief How the read-out shows a stage row: busy beside wait, and a device
///        cell that is a time, n/t, n/a or blank.

#include "StageTiming.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::DeviceTime;
using app::StageTiming;

// The core's values, which Readout.mm casts across: a renumbering here would
// relabel every device cell rather than fail, so the numbers are the contract.
static_assert(static_cast<std::uint8_t>(DeviceTime::None) == 0);
static_assert(static_cast<std::uint8_t>(DeviceTime::Measured) == 1);
static_assert(static_cast<std::uint8_t>(DeviceTime::NotTimed) == 2);
static_assert(static_cast<std::uint8_t>(DeviceTime::Unavailable) == 3);

StageTiming row(double cpu, double wait, double gpu, DeviceTime device) {
  StageTiming t;
  t.cpu_ms = cpu;
  t.wait_ms = wait;
  t.gpu_ms = gpu;
  t.device = device;
  return t;
}

/// A stage that blocks on its fence: the stall is wait, not host work.
TEST(StageTiming, BusyIsTheSpanLessItsWait) {
  EXPECT_DOUBLE_EQ(app::busy_ms(row(12.5, 10.0, 9.0, DeviceTime::Measured)),
                   2.5);
  EXPECT_DOUBLE_EQ(app::busy_ms(row(3.0, 0.0, 0.0, DeviceTime::None)), 3.0);
}

/// A wait-only row whose rounding leaves its wait just above its span draws no
/// negative busy bar.
TEST(StageTiming, BusyNeverGoesNegative) {
  EXPECT_DOUBLE_EQ(app::busy_ms(row(4.0, 4.0000001, 0.0, DeviceTime::None)),
                   0.0);
  EXPECT_DOUBLE_EQ(app::busy_ms(row(0.0, 0.0, 0.0, DeviceTime::None)), 0.0);
}

TEST(StageTiming, MeasuredCellIsTheTime) {
  EXPECT_EQ(app::device_cell(row(5.0, 4.0, 3.26, DeviceTime::Measured), 2),
            "3.26");
  EXPECT_EQ(app::device_cell(row(5.0, 4.0, 3.26, DeviceTime::Measured), 1),
            "3.3");
}

/// A measured zero is a measurement: it prints, unlike a host-only blank.
TEST(StageTiming, MeasuredZeroPrints) {
  EXPECT_EQ(app::device_cell(row(0.0, 0.0, 0.0, DeviceTime::Measured), 1),
            "0.0");
}

/// Each unmeasured state says why, whatever gpu_ms happens to hold.
TEST(StageTiming, UnmeasuredCellsSayWhy) {
  EXPECT_EQ(app::device_cell(row(5.0, 0.0, 7.0, DeviceTime::NotTimed), 2),
            "n/t");
  EXPECT_EQ(app::device_cell(row(5.0, 0.0, 7.0, DeviceTime::Unavailable), 2),
            "n/a");
}

TEST(StageTiming, HostOnlyCellIsBlank) {
  EXPECT_EQ(app::device_cell(row(5.0, 0.0, 7.0, DeviceTime::None), 2), "");
}

}  // namespace

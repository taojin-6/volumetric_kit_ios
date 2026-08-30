// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file memory_budget_test.cpp
/// @brief What a kernel memory reading is allowed to claim.
///
/// This app's worst failure returns no `Status`: jetsam sends `SIGKILL` and the
/// process disappears with nothing in the log. The read-out that is supposed to
/// give warning is only as good as its refusal to report a figure it cannot
/// support -- and the case that matters most, a kernel-clamped headroom of 0,
/// is unreachable through the live call without first persuading a real process
/// to run out of memory. Splitting the derivation out from the `task_info` call
/// is what makes it reachable here at all.

#include "MemoryBudget.hpp"

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

/// A healthy reading: the kernel answered in full, with headroom left.
app::TaskMemoryReading healthy() {
  app::TaskMemoryReading r;
  r.ok = true;
  r.task_info_status = 0;
  r.has_peak = true;
  r.has_remaining = true;
  r.phys_footprint = 3ULL * 1024 * 1024 * 1024;
  r.ledger_phys_footprint_peak = 4ULL * 1024 * 1024 * 1024;
  r.limit_bytes_remaining = 2ULL * 1024 * 1024 * 1024;
  r.device_ram_bytes = 16ULL * 1024 * 1024 * 1024;
  return r;
}

TEST(MemoryBudget, DerivesTheCeilingFromFootprintPlusHeadroom) {
  const app::MemoryBudget b = app::interpret_memory_budget(healthy());

  EXPECT_TRUE(b.valid);
  EXPECT_TRUE(b.limit_known);
  EXPECT_FALSE(b.at_limit);
  EXPECT_EQ(b.footprint_bytes, 3ULL * 1024 * 1024 * 1024);
  EXPECT_EQ(b.available_bytes, 2ULL * 1024 * 1024 * 1024);
  EXPECT_EQ(b.limit_bytes, 5ULL * 1024 * 1024 * 1024);
  EXPECT_EQ(b.peak_footprint_bytes, 4ULL * 1024 * 1024 * 1024);
  EXPECT_EQ(b.device_ram_bytes, 16ULL * 1024 * 1024 * 1024);
}

/// **The dangerous case.** The kernel clamps `limit_bytes_remaining` to 0 once
/// the process is at or past its limit rather than reporting a negative
/// remainder. Deriving a ceiling from that would produce `limit == footprint`:
/// a read-out showing the limit *rising to meet* the footprint and calling it a
/// tidy 100%, in exactly the pre-jetsam window the instrument exists to catch.
///
/// So the ceiling must be left unknown and the alarm raised instead.
TEST(MemoryBudget, ClampedHeadroomIsAnAlarmNotACeiling) {
  app::TaskMemoryReading r = healthy();
  r.limit_bytes_remaining = 0;

  const app::MemoryBudget b = app::interpret_memory_budget(r);

  EXPECT_TRUE(b.valid);
  EXPECT_TRUE(b.at_limit);
  EXPECT_FALSE(b.limit_known);
  EXPECT_EQ(b.limit_bytes, 0ULL) << "a ceiling was fabricated from a clamp";
  EXPECT_NE(b.limit_bytes, b.footprint_bytes)
      << "limit collapsed onto the footprint: this reads as 100% headroom used "
         "with a ceiling that is not real";
  // The footprint itself is still a true reading and stays.
  EXPECT_EQ(b.footprint_bytes, 3ULL * 1024 * 1024 * 1024);
}

/// A reply too short to carry `limit_bytes_remaining` (pre-REV4) degrades to
/// "not known" rather than to a stack reading printed as a measurement.
TEST(MemoryBudget, ShortReplyWithoutHeadroomLeavesTheCeilingUnknown) {
  app::TaskMemoryReading r = healthy();
  r.has_remaining = false;
  r.limit_bytes_remaining = 999;  // Whatever happened to be in the struct.

  const app::MemoryBudget b = app::interpret_memory_budget(r);

  EXPECT_TRUE(b.valid);
  EXPECT_FALSE(b.limit_known);
  EXPECT_FALSE(b.at_limit) << "an unread field must not raise the alarm";
  EXPECT_EQ(b.limit_bytes, 0ULL);
  EXPECT_EQ(b.available_bytes, 0ULL) << "unread field leaked into the read-out";
}

/// Same rule one revision earlier, for the peak (pre-REV3).
TEST(MemoryBudget, ShortReplyWithoutPeakReportsNoPeak) {
  app::TaskMemoryReading r = healthy();
  r.has_peak = false;
  r.ledger_phys_footprint_peak = 777;

  EXPECT_EQ(app::interpret_memory_budget(r).peak_footprint_bytes, 0ULL);
}

/// The kernel may carry the field and still have nothing in it; 0 means "not
/// supplied", not "the process has never held anything".
TEST(MemoryBudget, ZeroPeakIsReportedAsNotSupplied) {
  app::TaskMemoryReading r = healthy();
  r.ledger_phys_footprint_peak = 0;

  EXPECT_EQ(app::interpret_memory_budget(r).peak_footprint_bytes, 0ULL);
}

/// When the task port refuses, every per-process figure is meaningless and is
/// reported as such -- but the RAM figure came from a different call that did
/// not fail, and the kernel's own code is kept so the failure can say why
/// rather than printing an unactionable placeholder.
TEST(MemoryBudget, FailedTaskInfoInvalidatesFiguresButKeepsContext) {
  app::TaskMemoryReading r = healthy();
  r.ok = false;
  r.task_info_status = 5;  // KERN_FAILURE.

  const app::MemoryBudget b = app::interpret_memory_budget(r);

  EXPECT_FALSE(b.valid);
  EXPECT_FALSE(b.limit_known);
  EXPECT_FALSE(b.at_limit);
  EXPECT_EQ(b.footprint_bytes, 0ULL);
  EXPECT_EQ(b.limit_bytes, 0ULL);
  EXPECT_EQ(b.peak_footprint_bytes, 0ULL);
  EXPECT_EQ(b.task_info_status, 5);
  EXPECT_EQ(b.device_ram_bytes, 16ULL * 1024 * 1024 * 1024)
      << "a reading that did not fail was dropped along with one that did";
}

/// `device_ram_bytes` is context, never the budget. A sysctl that failed
/// reports 0 rather than a guess, and that does not invalidate the rest.
TEST(MemoryBudget, MissingDeviceRamDoesNotInvalidateTheReading) {
  app::TaskMemoryReading r = healthy();
  r.device_ram_bytes = 0;

  const app::MemoryBudget b = app::interpret_memory_budget(r);

  EXPECT_TRUE(b.valid);
  EXPECT_TRUE(b.limit_known);
  EXPECT_EQ(b.device_ram_bytes, 0ULL);
}

}  // namespace

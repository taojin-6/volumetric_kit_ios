// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file growth_policy_test.cpp
/// @brief The two guards that decide how much scene a scan can hold.
///
/// Both of these were branches inside `Fusion::fuse`, which submits GPU work --
/// so reaching them meant a device, a LiDAR frame, and a block table driven
/// into its pathological regime on purpose. The states worth pinning are
/// exactly the ones hardest to reach that way: a doubling refused because the
/// jetsam headroom will not cover it, a resize that already failed once, and a
/// fabricated occupancy tripping the allocate guard on a table with room left.
///
/// Neither number these enforce has been re-measured on device since the recon
/// kernel they guard was repaired (see `kRefuseAllocateAtOccupancy`). That is a
/// separate debt. What these tests pin is that the guards *behave as
/// documented* -- which was previously not checkable at all.

#include "GrowthPolicy.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

/// recon's `VoxelHashMap::kGrowThreshold`. Restated here, in a test, on
/// purpose: the point of `GrowthInputs::grow_threshold` being a field is that
/// the app never hardcodes it, and a test that took it from the app could not
/// tell whether the app was still asking recon.
constexpr float kReconGrowThreshold = 0.7f;

/// A scan sitting just past the grow threshold, with a healthy budget and room
/// below the ceiling: the ordinary "time to double" case.
app::GrowthInputs due_to_grow() {
  app::GrowthInputs in;
  in.occupancy = 0.75f;
  in.occupancy_known = true;
  in.grow_threshold = kReconGrowThreshold;
  in.num_buckets = 1024;
  in.max_buckets = 32768;
  in.declined_at = 0;
  in.budget.valid = true;
  in.budget.limit_known = true;
  in.budget.available_bytes = 8ULL * 1024 * 1024 * 1024;
  return in;
}

// --- The sizing arithmetic ---------------------------------------------------

/// The figure `scanner.entitlements` records for the grid at 16384 buckets is
/// 805 MB. This is the arithmetic that has to agree with that measurement, and
/// the input to every headroom decision below.
///
/// 16384 * 8 blocks * 512 voxels * 12 B is 805,306,368 bytes exactly -- which
/// is the recorded 805 MB decimal and 768 MiB, the same quantity under the two
/// conventions. Asserted in bytes so neither reading has to be trusted.
TEST(GrowthPolicy, GridBytesMatchTheRecordedMeasurement) {
  EXPECT_EQ(app::grid_bytes_for(16384), 805306368ULL);
  // And the ceiling this app configures, quoted as ~1.5 GiB in FusionConfig.
  EXPECT_EQ(app::grid_bytes_for(32768) / (1024 * 1024), 1536u);
}

TEST(GrowthPolicy, TableBlocksScaleByTheBucketSize) {
  EXPECT_EQ(app::table_blocks_for(1024), 8192);
  EXPECT_EQ(app::table_blocks_for(32768), 262144);
}

/// The doubling is capped rather than allowed to overshoot: a table two thirds
/// of the way to the ceiling grows *to* the ceiling, not past it.
TEST(GrowthPolicy, GrowTargetDoublesButNeverPassesTheCeiling) {
  EXPECT_EQ(app::grow_target(1024, 32768), 2048);
  EXPECT_EQ(app::grow_target(16384, 32768), 32768);
  EXPECT_EQ(app::grow_target(32768, 32768), 32768);
  // Already past it -- a configuration change could leave a table here.
  EXPECT_EQ(app::grow_target(65536, 32768), 32768);
}

// --- When to grow ------------------------------------------------------------

TEST(GrowthPolicy, GrowsWhenOccupancyPassesReconsThreshold) {
  const app::GrowthPlan plan = app::plan_growth(due_to_grow());

  EXPECT_EQ(plan.action, app::GrowthAction::Resize);
  EXPECT_EQ(plan.grow_to, 2048);
  EXPECT_EQ(plan.needed_bytes, app::grid_bytes_for(2048));
}

TEST(GrowthPolicy, LeavesAHealthyTableAlone) {
  app::GrowthInputs in = due_to_grow();
  in.occupancy = 0.5f;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);
}

/// The threshold is recon's, and it is compared strictly: sitting *exactly* on
/// it is not yet past it. Pinned because the allocate guard below is inclusive
/// in the other direction, and having the two disagree silently is the kind of
/// off-by-one that only shows up as a table that grew a frame early.
TEST(GrowthPolicy, TheThresholdItselfIsNotYetPastIt) {
  app::GrowthInputs in = due_to_grow();
  in.occupancy = kReconGrowThreshold;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);
}

/// A fabricated occupancy must not read as a standing instruction to double.
///
/// `Fusion` sets occupancy to 1.0 when `load_factor` cannot be read, so it
/// fails *safe* for the allocate guard -- and would fail maximally *unsafe*
/// here, asking for the largest allocation this app makes, on every fused
/// frame, on the strength of a number nobody read.
TEST(GrowthPolicy, AnUnreadableOccupancyNeverTriggersAGrow) {
  app::GrowthInputs in = due_to_grow();
  in.occupancy = 1.0f;
  in.occupancy_known = false;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);
}

TEST(GrowthPolicy, StopsAtTheConfiguredCeiling) {
  app::GrowthInputs in = due_to_grow();
  in.num_buckets = 32768;
  in.occupancy = 0.8f;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);
}

/// `num_buckets` advances only on a *successful* resize, so without this the
/// same condition is satisfied again on the very next fused frame -- and the
/// app asks the allocator for its largest block at capture rate, forever.
TEST(GrowthPolicy, DoesNotReAskAtASizeThatAlreadyFailed) {
  app::GrowthInputs in = due_to_grow();
  in.declined_at = in.num_buckets;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);

  // ...but a *different* size is still fair game: the refusal is pinned to the
  // size that failed, not latched for the run.
  in.num_buckets = 2048;
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);
}

// --- The cheap half ----------------------------------------------------------

/// `growth_due` exists so the fuse loop can skip a `task_info` trap on every
/// frame that is nowhere near a doubling. It must therefore agree with
/// `plan_growth` about every case where no grow happens -- if it ever said
/// "due" where the full plan says None, the syscall comes back per frame; if it
/// said "not due" where the plan would have grown, the table never grows at
/// all.
TEST(GrowthPolicy, TheCheapHalfAgreesWithTheFullPlan) {
  const app::GrowthInputs base = due_to_grow();

  struct Case {
    const char* what;
    app::GrowthInputs in;
  };
  std::vector<Case> cases;
  cases.push_back({"due", base});
  {
    app::GrowthInputs in = base;
    in.occupancy = 0.5f;
    cases.push_back({"below threshold", in});
  }
  {
    app::GrowthInputs in = base;
    in.occupancy_known = false;
    cases.push_back({"unreadable occupancy", in});
  }
  {
    app::GrowthInputs in = base;
    in.num_buckets = in.max_buckets;
    cases.push_back({"at the ceiling", in});
  }
  {
    app::GrowthInputs in = base;
    in.declined_at = in.num_buckets;
    cases.push_back({"already refused", in});
  }
  {
    app::GrowthInputs in = base;
    in.budget.available_bytes = 0;
    cases.push_back({"no headroom", in});
  }

  for (const Case& c : cases) {
    const bool due = app::growth_due(c.in);
    const bool planned =
        app::plan_growth(c.in).action != app::GrowthAction::None;
    EXPECT_EQ(due, planned) << c.what;
  }
}

/// The budget is not read by the cheap half, so a caller may ask before paying
/// for one -- which is the entire point of the split.
TEST(GrowthPolicy, TheCheapHalfIgnoresTheBudgetEntirely) {
  app::GrowthInputs in = due_to_grow();
  in.budget = app::MemoryBudget{};  // never read

  EXPECT_TRUE(app::growth_due(in));
}

// --- The headroom check ------------------------------------------------------

/// The allocation that gets a scan SIGKILLed rather than failed. `resize`
/// builds the grown arrays beside the live ones, so the transient is the whole
/// new size on top of what is resident -- and jetsam returns no `Status`.
TEST(GrowthPolicy, DeclinesADoublingTheHeadroomCannotCover) {
  app::GrowthInputs in = due_to_grow();
  in.num_buckets = 16384;  // doubling to 32768 wants 1536 MB
  in.budget.available_bytes = 512ULL * 1024 * 1024;

  const app::GrowthPlan plan = app::plan_growth(in);

  EXPECT_EQ(plan.action, app::GrowthAction::DeclinedForMemory);
  // The target and its cost are still reported: the declining message names
  // both, and a plan that withheld them would leave the reader with "not
  // growing" and no figures to judge it by.
  EXPECT_EQ(plan.grow_to, 32768);
  EXPECT_EQ(plan.needed_bytes, app::grid_bytes_for(32768));
}

TEST(GrowthPolicy, HeadroomExactlyCoveringTheGrowIsEnough) {
  app::GrowthInputs in = due_to_grow();
  in.budget.available_bytes = app::grid_bytes_for(2048);

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);
}

/// An unreadable budget is not evidence of a full one. Refusing on it would
/// stop every scan on a device whose kernel answered short -- turning a missing
/// diagnostic into a hard cap on scene size.
TEST(GrowthPolicy, AnUnreadableBudgetDoesNotDeclineTheGrow) {
  app::GrowthInputs in = due_to_grow();
  in.num_buckets = 16384;
  in.budget.available_bytes = 0;

  in.budget.valid = false;
  in.budget.limit_known = true;
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);

  // Same for a valid reading the kernel gave no ceiling for -- including the
  // at-limit case, where `available_bytes` is a clamped 0 rather than a
  // measurement. See MemoryBudget::limit_known.
  in.budget.valid = true;
  in.budget.limit_known = false;
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);
}

// --- The allocate guard ------------------------------------------------------

TEST(AllocationGuard, AllocatesWhileThereIsRoom) {
  const app::AllocationGuard g = app::guard_allocation(0.5f, true);

  EXPECT_TRUE(g.allocate);
  EXPECT_EQ(g.stop, app::AllocationStop::None);
}

TEST(AllocationGuard, RefusesPastTheOccupancyCliff) {
  const app::AllocationGuard g = app::guard_allocation(0.9f, true);

  EXPECT_FALSE(g.allocate);
  EXPECT_EQ(g.stop, app::AllocationStop::VolumeFull);
}

/// The guard is strict, so a table sitting *exactly* on the cliff still
/// allocates. Deliberately the opposite of `StatTone`'s inclusive boundaries:
/// that one gates an alarm, where sitting on the limit is the case worth
/// catching, and this one gates the app's ability to take new geometry at all.
TEST(AllocationGuard, TheCliffItselfStillAllocates) {
  EXPECT_TRUE(
      app::guard_allocation(app::kRefuseAllocateAtOccupancy, true).allocate);
}

/// **The distinction the read-out's advice depends on.** A fabricated occupancy
/// trips this guard exactly as a genuinely full table does, and only this scope
/// knows which happened -- so the two causes are separated here rather than at
/// the read-out, where "coarsen your voxels" is the right advice for one and
/// actively wrong for the other.
TEST(AllocationGuard, AFabricatedOccupancyIsNotAFullVolume) {
  const app::AllocationGuard g = app::guard_allocation(1.0f, false);

  EXPECT_FALSE(g.allocate);
  EXPECT_EQ(g.stop, app::AllocationStop::OccupancyUnknown);
  // And that cause points the reader at the errors row rather than at a limit
  // they have not reached.
  EXPECT_TRUE(app::allocation_stop_text(g.stop).on_errors_row);
}

/// The band between recon's grow threshold and this app's refusal is the room a
/// doubling needs to land in. Collapsing them would refuse allocation at the
/// moment growth begins, on a table with plenty of room -- so the ordering is
/// pinned rather than left to two constants that read as interchangeable.
TEST(AllocationGuard, LeavesRoomForADoublingToLandIn) {
  EXPECT_GT(app::kRefuseAllocateAtOccupancy, kReconGrowThreshold);

  app::GrowthInputs in = due_to_grow();
  in.occupancy = (kReconGrowThreshold + app::kRefuseAllocateAtOccupancy) / 2.0f;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);
  EXPECT_TRUE(app::guard_allocation(in.occupancy, true).allocate);
}

TEST(AllocationGuard, IsConstexpr) {
  static_assert(app::guard_allocation(0.5f, true).allocate,
                "the guard is used in constant expressions");
  static_assert(app::guard_allocation(0.99f, true).stop ==
                    app::AllocationStop::VolumeFull,
                "the guard is used in constant expressions");
}

}  // namespace

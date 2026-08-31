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

#include <limits>
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

/// Every factor named, and the arithmetic the product of the names.
///
/// `grid_bytes_for` carried 512 and 12 as literals in its body while
/// `Fusion::start` configured the grid from its own copies of both -- in a
/// different directory and a different library, since this one cannot include
/// recon. `GridBytesMatchTheRecordedMeasurement` above pins the *output*, so it
/// stays green through a fourth attribute or a 16^3 block while the headroom
/// check silently under-reports the commit by a third or by 8x. This pins the
/// relationship; the static_assert beside the AttributeSpec table in Fusion.mm
/// pins the half that has to be checked where recon is visible.
TEST(GrowthPolicy, GridBytesIsTheProductOfItsNamedFactors) {
  EXPECT_EQ(
      app::kVoxelsPerBlock,
      app::kBlockEdgeVoxels * app::kBlockEdgeVoxels * app::kBlockEdgeVoxels);
  EXPECT_EQ(app::grid_bytes_for(1024),
            1024ULL * static_cast<std::uint64_t>(app::kBlocksPerBucket) *
                static_cast<std::uint64_t>(app::kVoxelsPerBlock) *
                static_cast<std::uint64_t>(app::kAttributeBytesPerVoxel));
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

/// An omitted threshold means "never grow", not "grow on every fused frame".
///
/// The deleted code named recon's constant inside the condition, so there was
/// nothing to omit; a field can be. Defaulted to 0.0 it was satisfied at 2%
/// occupancy -- so a second call site, or a partial copy of the assignment
/// block in `Fusion::fuse`, would put the `task_info` trap back on the
/// per-frame path and ask the allocator for this app's largest block at capture
/// rate. That is the runaway `declined_at` exists to stop, arriving through the
/// door beside it. 1.0 is unsatisfiable: no load factor exceeds it, and the
/// fabricated reading `Fusion` substitutes when it cannot take one is
/// exactly 1.0.
TEST(GrowthPolicy, AnOmittedGrowThresholdNeverGrows) {
  app::GrowthInputs in;
  in.occupancy = 0.99f;
  in.occupancy_known = true;
  in.num_buckets = 1024;
  in.max_buckets = 32768;
  // `grow_threshold` deliberately left at its default.

  EXPECT_FALSE(app::growth_due(in));
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
///
/// Each case names the action it expects, rather than the two halves being
/// compared to each other. Compared to each other the test could not fail:
/// `plan_growth` opens with `if (!growth_due(in)) return plan;` and every path
/// past it sets a non-None action, so `plan_growth(in).action != None` *is*
/// `growth_due(in)` for every possible input, and the six rows stayed green
/// through exactly the divergence the paragraph above describes -- someone
/// open-coding the conditions in `plan_growth` and dropping one. Both halves
/// would have moved together.
TEST(GrowthPolicy, TheCheapHalfAgreesWithTheFullPlan) {
  const app::GrowthInputs base = due_to_grow();

  struct Case {
    const char* what;
    app::GrowthInputs in;
    app::GrowthAction expected;
  };
  std::vector<Case> cases;
  cases.push_back({"due", base, app::GrowthAction::Resize});
  {
    app::GrowthInputs in = base;
    in.occupancy = 0.5f;
    cases.push_back({"below threshold", in, app::GrowthAction::None});
  }
  {
    app::GrowthInputs in = base;
    in.occupancy_known = false;
    cases.push_back({"unreadable occupancy", in, app::GrowthAction::None});
  }
  {
    app::GrowthInputs in = base;
    in.num_buckets = in.max_buckets;
    cases.push_back({"at the ceiling", in, app::GrowthAction::None});
  }
  {
    app::GrowthInputs in = base;
    in.declined_at = in.num_buckets;
    cases.push_back({"already refused", in, app::GrowthAction::None});
  }
  {
    app::GrowthInputs in = base;
    in.memory_declined = true;
    in.frames_since_memory_decline = 1;
    cases.push_back({"waiting out a decline", in, app::GrowthAction::None});
  }
  {
    app::GrowthInputs in = base;
    in.budget.available_bytes = 0;
    cases.push_back({"no headroom", in, app::GrowthAction::DeclinedForMemory});
  }

  for (const Case& c : cases) {
    EXPECT_EQ(app::plan_growth(c.in).action, c.expected) << c.what;
    EXPECT_EQ(app::growth_due(c.in), c.expected != app::GrowthAction::None)
        << c.what;
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

/// **A headroom refusal expires. Pinned to a size it caps the scan for good.**
///
/// `num_buckets` advances only on a successful resize, so a size-pinned decline
/// never goes stale on its own. Growth is the only thing that lowers occupancy,
/// so occupancy climbs; once it passes `kRefuseAllocateAtOccupancy` the
/// allocate guard short-circuits the overflow count the reactive backstop grows
/// on, and that route shuts too. One momentary shortfall at a doubling boundary
/// -- some other app briefly holding the headroom -- then held the table at its
/// current size for the rest of the scan, reporting a volume full short of a
/// ceiling it never reached.
TEST(GrowthPolicy, AHeadroomDeclineExpiresRatherThanCappingTheScan) {
  app::GrowthInputs in = due_to_grow();
  in.num_buckets = 16384;  // doubling to 32768 wants 1536 MB
  in.budget.available_bytes = 512ULL * 1024 * 1024;

  ASSERT_EQ(app::plan_growth(in).action, app::GrowthAction::DeclinedForMemory);

  // The frames just after it do not re-ask, which is what the backoff buys: the
  // `task_info` trap stays off the per-frame path.
  in.memory_declined = true;
  in.frames_since_memory_decline = 1;
  EXPECT_FALSE(app::growth_due(in));
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);

  // ...and then it lapses, at the same size, with nothing about the table
  // having changed. That is the whole difference from `declined_at`.
  in.frames_since_memory_decline = app::kMemoryDeclineRetryFrames;
  EXPECT_TRUE(app::growth_due(in));
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::DeclinedForMemory);

  // Once the pressure lifts, the doubling the scan was capped at goes through.
  in.budget.available_bytes = 8ULL * 1024 * 1024 * 1024;
  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::Resize);
}

/// The two refusals are separate inputs because they expire differently, and a
/// lapsed headroom decline must not lift a pin the allocator put there.
TEST(GrowthPolicy, ALapsedDeclineDoesNotClearARefusedSize) {
  app::GrowthInputs in = due_to_grow();
  in.memory_declined = true;
  in.frames_since_memory_decline = app::kMemoryDeclineRetryFrames;
  in.declined_at = in.num_buckets;

  EXPECT_EQ(app::plan_growth(in).action, app::GrowthAction::None);
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

/// A non-finite occupancy is *not* past the cliff.
///
/// Written as `occupancy <= kRefuseAllocateAtOccupancy` the guard reads as the
/// inverse of the `occupancy > k` it replaced, and is -- for ordered values. A
/// NaN compares false against both, so that form falls through to the refusal:
/// a frame that used to allocate silently instead stops the scan, latches an
/// AllocationStop `Fusion` never lowers, and prints "volume full: 0%" from a UB
/// cast of the NaN. `guard_allocation` is host-testable API now, and its only
/// documented contract is "past kRefuseAllocateAtOccupancy".
TEST(AllocationGuard, ANonFiniteOccupancyIsNotPastTheCliff) {
  const app::AllocationGuard g =
      app::guard_allocation(std::numeric_limits<float>::quiet_NaN(), true);

  EXPECT_TRUE(g.allocate);
  EXPECT_EQ(g.stop, app::AllocationStop::None);
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

/// The tiers the occupancy figure is *reported* against are the two numbers
/// that decide, not a second pair beside them.
///
/// They were a `{0.7, 0.85}` literal in StatTone.hpp -- restating recon's grow
/// threshold, which GrowthPolicy.hpp forbids categorically, and the allocate
/// guard, which it invites re-measuring on the next device pass. They were not
/// even the same quantity: `(double)0.85f` is 0.85000002384 against a literal
/// 0.84999999999. Move `kRefuseAllocateAtOccupancy` to 0.92 and the meter, its
/// tick and the `occupied` row would all have gone on reddening at 0.85 while
/// the table allocated normally for another seven points.
TEST(AllocationGuard, TheReportedTiersAreTheNumbersThatDecide) {
  const app::ToneThresholds t = app::occupancy_thresholds(kReconGrowThreshold);

  EXPECT_EQ(t.warn, static_cast<double>(kReconGrowThreshold));
  EXPECT_EQ(t.critical, static_cast<double>(app::kRefuseAllocateAtOccupancy));

  // Warn arrives where recon starts growing; red arrives *at* the cliff rather
  // than after it -- the guard is strict and the tone inclusive, so the table
  // sitting exactly on it still allocates and already reads Critical. That is
  // the direction an alarm should err, and it is now one number, not two.
  EXPECT_EQ(app::tone_for(static_cast<double>(kReconGrowThreshold), t),
            app::StatTone::Warn);
  const double at_cliff = static_cast<double>(app::kRefuseAllocateAtOccupancy);
  EXPECT_EQ(app::tone_for(at_cliff, t), app::StatTone::Critical);
  EXPECT_TRUE(
      app::guard_allocation(app::kRefuseAllocateAtOccupancy, true).allocate);
}

TEST(AllocationGuard, IsConstexpr) {
  static_assert(app::guard_allocation(0.5f, true).allocate,
                "the guard is used in constant expressions");
  static_assert(app::guard_allocation(0.99f, true).stop ==
                    app::AllocationStop::VolumeFull,
                "the guard is used in constant expressions");
}

}  // namespace

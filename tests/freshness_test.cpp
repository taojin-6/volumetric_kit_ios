// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file freshness_test.cpp
/// @brief When a published figure stops describing the present.
///
/// Both of these guard a read-out that cannot notice on its own. The extract's
/// whole breakdown is written only by a fully successful remesh, and the dirty
/// rows are gated on a latch that never lowers -- so a stage that stops
/// publishing leaves its last numbers on screen, presented as this frame's,
/// indefinitely. These are what say otherwise.

#include "Freshness.hpp"

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// --- The survey --------------------------------------------------------------

/// A scan that has never surveyed is early, not stale. The read-out says the
/// two differently -- "no sample yet" against "failing" -- and getting this
/// wrong reports a fault on every scan for its first second.
TEST(Freshness, ASurveyThatHasNeverRunIsNotStale) {
  EXPECT_FALSE(app::survey_stale(false, 0));
  EXPECT_FALSE(app::survey_stale(false, 10 * app::kSurveyEveryFrames));
}

TEST(Freshness, AnOnCadenceSurveyIsCurrent) {
  EXPECT_FALSE(app::survey_stale(true, 0));
  EXPECT_FALSE(app::survey_stale(true, app::kSurveyEveryFrames));
}

/// The margin is a window and a half, and it is not inclusive: sitting exactly
/// on it is the last frame that still counts as current.
TEST(Freshness, TheMarginItselfIsStillCurrent) {
  EXPECT_FALSE(app::survey_stale(true, app::kSurveyStaleAfter));
  EXPECT_TRUE(app::survey_stale(true, app::kSurveyStaleAfter + 1));
}

/// The ordinary cadence must never trip the marker, or it is noise. The sample
/// publishes exactly every `kSurveyEveryFrames` frames when it is working, so
/// the margin has to sit strictly above that with room to spare.
TEST(Freshness, TheOrdinaryCadenceNeverTripsIt) {
  EXPECT_GT(app::kSurveyStaleAfter, app::kSurveyEveryFrames);
  for (std::uint64_t f = 0; f <= app::kSurveyEveryFrames; ++f) {
    EXPECT_FALSE(app::survey_stale(true, f)) << "at " << f << " frames";
  }
}

// --- The extract -------------------------------------------------------------

TEST(Freshness, AnExtractThatHasNeverRunIsNotStale) {
  EXPECT_FALSE(app::extract_stale(false, 1000, 1));
}

/// Two cadences of slack rather than none: a remesh that skips because the
/// renderer has not collected the last mesh is the ordinary steady state, and a
/// marker that flickered on it would be noise rather than signal.
TEST(Freshness, AllowsTwoRemeshCadencesOfSlack) {
  EXPECT_FALSE(app::extract_stale(true, 2, 1));
  EXPECT_TRUE(app::extract_stale(true, 3, 1));

  EXPECT_FALSE(app::extract_stale(true, 10, 5));
  EXPECT_TRUE(app::extract_stale(true, 11, 5));
}

/// The staleness window tracks the configured cadence rather than a fixed frame
/// count: a scan re-meshing every 5th frame is not stale at 4 frames behind,
/// and one re-meshing every frame is.
TEST(Freshness, TheWindowTracksTheConfiguredCadence) {
  EXPECT_TRUE(app::extract_stale(true, 4, 1));
  EXPECT_FALSE(app::extract_stale(true, 4, 5));
}

/// A zero cadence would otherwise make every frame past the first read as
/// stale. `FusionConfig::remesh_every` is documented as "1 = every frame" and
/// nothing validates it, so the clamp is what stops a 0 there turning the whole
/// read-out amber.
TEST(Freshness, AZeroCadenceIsTreatedAsEveryFrame) {
  EXPECT_FALSE(app::extract_stale(true, 2, 0));
  EXPECT_TRUE(app::extract_stale(true, 3, 0));
}

// --- The stage rows ----------------------------------------------------------

/// Measured against the fuse age, never against the clock. An ARKit
/// interruption -- a call, Control Centre, the app switcher -- stops both
/// clocks together, and a panel comparing the first to a fixed second announces
/// stale timings for the length of a phone call, blaming the fusion for the
/// camera.
TEST(Freshness, APausedScanHasStaleClocksButFreshStages) {
  EXPECT_FALSE(app::stages_stale(30000.0f, 30000.0f));
  EXPECT_FALSE(app::stages_stale(30100.0f, 30000.0f));
}

/// The case that *is* a fault: frames arriving, none completing. The stage rows
/// publish only on frames `measure_stages` seeds, so they can fall behind a
/// loop that is otherwise running normally.
TEST(Freshness, StagesFallingBehindARunningLoopAreStale) {
  EXPECT_TRUE(app::stages_stale(5000.0f, 16.0f));
}

TEST(Freshness, TheStageMarginIsTheFuseStalenessSpan) {
  const float fuse = 16.0f;
  EXPECT_FALSE(app::stages_stale(fuse + app::kFuseStaleAfterMs, fuse));
  EXPECT_TRUE(app::stages_stale(fuse + app::kFuseStaleAfterMs + 1.0f, fuse));
}

TEST(Freshness, IsConstexpr) {
  static_assert(!app::survey_stale(true, app::kSurveyStaleAfter),
                "the freshness rules are used in constant expressions");
  static_assert(app::extract_stale(true, 3, 1),
                "the freshness rules are used in constant expressions");
}

}  // namespace

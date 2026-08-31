// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file allocation_stop_test.cpp
/// @brief Every way the app says a scan stopped taking geometry, and the rule
///        that stops it saying so about a scan that is merely not running.

#include "AllocationStop.hpp"

#include <string>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::AllocationStop;
using app::AllocationStopText;

constexpr AllocationStop kCauses[] = {
    AllocationStop::VolumeFull,
    AllocationStop::OccupancyUnknown,
    AllocationStop::BlocksDropped,
};

TEST(AllocationStop, NoneRendersAsNothing) {
  const AllocationStopText text =
      app::allocation_stop_text(AllocationStop::None);
  EXPECT_STREQ(text.headline, "");
  EXPECT_STREQ(text.advice, "");
  EXPECT_EQ(app::allocation_stop_row(AllocationStop::None), "");
}

TEST(AllocationStop, EveryCauseHasItsOwnWords) {
  for (const AllocationStop stop : kCauses) {
    const AllocationStopText text = app::allocation_stop_text(stop);
    EXPECT_STRNE(text.headline, "") << app::allocation_stop_tag(stop);
    EXPECT_STRNE(text.advice, "") << app::allocation_stop_tag(stop);
    EXPECT_STRNE(app::allocation_stop_tag(stop), "ok");
  }
}

/// The tag is a column value, so two causes sharing one would make a dump
/// unreadable in exactly the regime it is collected for.
TEST(AllocationStop, TagsAreDistinct) {
  EXPECT_STRNE(app::allocation_stop_tag(AllocationStop::VolumeFull),
               app::allocation_stop_tag(AllocationStop::OccupancyUnknown));
  EXPECT_STRNE(app::allocation_stop_tag(AllocationStop::VolumeFull),
               app::allocation_stop_tag(AllocationStop::BlocksDropped));
  EXPECT_STRNE(app::allocation_stop_tag(AllocationStop::OccupancyUnknown),
               app::allocation_stop_tag(AllocationStop::BlocksDropped));
}

/// The regression this table was written for: the read-out's row appended a
/// hard-coded "(volume full)" to a flag that meant only "not allocating this
/// frame", so a failed `load_factor` -- which fabricates a full table to fail
/// safe -- reported a full volume beneath the banner naming the real fault.
TEST(AllocationStop, OnlyAFullVolumeSaysFullVolume) {
  EXPECT_NE(
      app::allocation_stop_row(AllocationStop::VolumeFull).find("volume full"),
      std::string::npos);
  EXPECT_EQ(app::allocation_stop_row(AllocationStop::OccupancyUnknown)
                .find("volume full"),
            std::string::npos);
  EXPECT_EQ(app::allocation_stop_row(AllocationStop::BlocksDropped)
                .find("volume full"),
            std::string::npos);
}

/// The row and the banner's headline are one phrase, composed rather than
/// written twice: rewording a cause in one place must not leave a collected log
/// and a screenshot of the panel naming it differently. This is the assertion
/// that makes the composer worth having -- the panel's `state` row and the
/// banner beside it are the two renderings that would drift.
TEST(AllocationStop, RowCarriesTheHeadlineVerbatim) {
  for (const AllocationStop stop : kCauses) {
    const std::string row = app::allocation_stop_row(stop);
    EXPECT_NE(row.find(app::allocation_stop_text(stop).headline),
              std::string::npos)
        << row;
  }
}

/// Every cause says the scan has stopped taking geometry, in the same words.
/// The row leads with the claim rather than the cause because that is what a
/// reader glancing at the card is deciding on; a cause without it reads as a
/// note about the volume rather than as a stopped scan.
TEST(AllocationStop, EveryCauseLeadsWithTheClaim) {
  for (const AllocationStop stop : kCauses) {
    EXPECT_EQ(app::allocation_stop_row(stop).rfind("ALLOCATION STOPPED", 0), 0u)
        << app::allocation_stop_row(stop);
  }
}

/// Only the two causes whose fault is upstream point at the errors row; a full
/// volume is the documented trade working and posts no error to look at.
///
/// The same words `AllocationStopText::advice` uses for it, because a row that
/// sends a reader to a differently-named place than the banner does is the
/// drift this composer exists to stop.
TEST(AllocationStop, OnlyUpstreamFaultsPointAtTheErrorsRow) {
  EXPECT_EQ(
      app::allocation_stop_row(AllocationStop::VolumeFull).find("errors row"),
      std::string::npos);
  EXPECT_NE(app::allocation_stop_row(AllocationStop::OccupancyUnknown)
                .find("errors row"),
            std::string::npos);
  EXPECT_NE(app::allocation_stop_row(AllocationStop::BlocksDropped)
                .find("errors row"),
            std::string::npos);
}

// --- The freshness rule ------------------------------------------------------

TEST(FuseStaleness, ARunningLoopReportsTheLatchedCause) {
  for (const AllocationStop stop : kCauses) {
    EXPECT_EQ(app::reportable_allocation_stop(stop, 0.0f), stop);
    EXPECT_EQ(app::reportable_allocation_stop(stop, 16.7f), stop);
  }
}

/// The failure this exists for: an ARKit interruption stops fuse frames without
/// stopping the display link, and `allocation_stop` is a latch Fusion never
/// clears -- so a full volume was announced in the present tense for the length
/// of a phone call.
TEST(FuseStaleness, AStalledLoopReportsNoCause) {
  for (const AllocationStop stop : kCauses) {
    EXPECT_EQ(app::reportable_allocation_stop(stop, 12'400.0f),
              AllocationStop::None);
  }
}

/// Exactly on the threshold is still running: this is meant to fire on an
/// interruption, not on a slow frame, so the boundary rounds towards keeping
/// the claim rather than dropping it.
TEST(FuseStaleness, TheThresholdItselfIsStillRunning) {
  EXPECT_TRUE(app::fuse_loop_running(app::kFuseStaleAfterMs));
  EXPECT_FALSE(app::fuse_loop_running(app::kFuseStaleAfterMs + 1.0f));
  EXPECT_EQ(app::reportable_allocation_stop(AllocationStop::VolumeFull,
                                            app::kFuseStaleAfterMs),
            AllocationStop::VolumeFull);
}

/// A stalled loop that had nothing to report still reports nothing -- the guard
/// must not invent a cause on the way to clearing one.
TEST(FuseStaleness, NoneStaysNoneEitherWay) {
  EXPECT_EQ(app::reportable_allocation_stop(AllocationStop::None, 0.0f),
            AllocationStop::None);
  EXPECT_EQ(app::reportable_allocation_stop(AllocationStop::None, 12'400.0f),
            AllocationStop::None);
}

}  // namespace

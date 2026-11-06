// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file tracking_line_test.cpp
/// @brief The tracking chip's words and tone, and the stream's paused state.

#include "TrackingLine.hpp"

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::StatTone;

/// Every state but normal withholds its frames, so it warns and the stream is
/// paused, whichever words say why.
TEST(TrackingLine, EachLostStateHasItsWordsAtWarnAndPausesTheStream) {
  struct Lost {
    const char* name;
    const char* words;
  };
  for (const Lost& lost : {Lost{"relocalizing", "Relocalizing"},
                           Lost{"initializing", "Initializing"},
                           Lost{"excessive motion", "Moving too fast"},
                           Lost{"insufficient features", "Too little in view"},
                           Lost{"not available", "No tracking"}}) {
    const app::TrackingLine line = app::tracking_line(lost.name, false, true);
    EXPECT_EQ(line.text, lost.words);
    EXPECT_EQ(line.tone, StatTone::Warn) << lost.name;
    EXPECT_TRUE(line.stream_paused) << lost.name;
  }
}

TEST(TrackingLine, NormalIsGoodAndTheStreamRuns) {
  const app::TrackingLine line = app::tracking_line("normal", false, true);
  EXPECT_EQ(line.text, "normal");
  EXPECT_EQ(line.tone, StatTone::Good);
  EXPECT_FALSE(line.stream_paused);
}

/// The last frame's tracking may read normal, but no frame is coming.
TEST(TrackingLine, AnInterruptionWinsOverTheLastFramesTracking) {
  for (const char* last : {"normal", "relocalizing"}) {
    const app::TrackingLine line = app::tracking_line(last, true, true);
    EXPECT_EQ(line.text, "Camera interrupted") << last;
    EXPECT_EQ(line.tone, StatTone::Warn) << last;
    EXPECT_TRUE(line.stream_paused) << last;
  }
}

/// A sender that never opened has nothing to pause, whatever the camera does.
TEST(TrackingLine, NoStreamIsNotAPausedOne) {
  EXPECT_FALSE(app::tracking_line("relocalizing", false, false).stream_paused);
  EXPECT_FALSE(app::tracking_line("normal", true, false).stream_paused);
  EXPECT_FALSE(app::tracking_line("normal", false, false).stream_paused);
}

/// A later sensor's new state still warns, in its own name.
TEST(TrackingLine, ANameItDoesNotKnowIsShownAsItCameAtWarn) {
  const app::TrackingLine line = app::tracking_line("unknown", false, true);
  EXPECT_EQ(line.text, "unknown");
  EXPECT_EQ(line.tone, StatTone::Warn);
  EXPECT_TRUE(line.stream_paused);
}

}  // namespace

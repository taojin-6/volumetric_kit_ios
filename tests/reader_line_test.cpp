// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file reader_line_test.cpp
/// @brief The top bar's first chip: the reader's line until it stops talking.

#include "ReaderLine.hpp"

#include <chrono>
#include <string>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::StatTone;
using namespace std::chrono_literals;

/// The reader's words pass as they came, in its tone: the scanner parses none.
TEST(ReaderLine, UnderTheLimitItIsTheReadersOwnLineAndTone) {
  struct Said {
    const char* line;
    StatTone tone;
  };
  for (const Said& said : {Said{"Looking for the tag", StatTone::Neutral},
                           Said{"Tag seen: 3 of 6", StatTone::Good},
                           Said{"Tag refused, trying again", StatTone::Warn},
                           Said{"Tag found, registered", StatTone::Good},
                           Said{"", StatTone::Neutral},
                           Said{"words of a later server", StatTone::Good}}) {
    const app::ReaderLine shown = app::reader_line(said.line, said.tone, 0ms);
    EXPECT_EQ(shown.text, said.line);
    EXPECT_EQ(shown.tone, said.tone) << said.line;
  }
}

/// Exactly 3 s is quiet, as "3 s or more" says, and the line and its tone
/// both go: a green tone must not outlive the green words.
TEST(ReaderLine, ThreeSecondsIsQuietAndOneMillisecondLessIsNot) {
  const app::ReaderLine fresh =
      app::reader_line("Tag seen: 3 of 6", StatTone::Good, 2'999ms);
  EXPECT_EQ(fresh.text, "Tag seen: 3 of 6");
  EXPECT_EQ(fresh.tone, StatTone::Good);

  const app::ReaderLine quiet =
      app::reader_line("Tag seen: 3 of 6", StatTone::Good, 3'000ms);
  EXPECT_EQ(quiet.text, "Server quiet for 3 s");
  EXPECT_EQ(quiet.tone, StatTone::Warn);
}

TEST(ReaderLine, QuietCountsWholeSeconds) {
  const auto quiet = [](std::chrono::milliseconds age) {
    return app::reader_line("Looking for the tag", StatTone::Neutral, age).text;
  };
  EXPECT_EQ(quiet(3'999ms), "Server quiet for 3 s");
  EXPECT_EQ(quiet(5'000ms), "Server quiet for 5 s");
  EXPECT_EQ(quiet(5'999ms), "Server quiet for 5 s");
  EXPECT_EQ(quiet(125s), "Server quiet for 125 s");
}

/// A note that reads as newer than the clock, or just arrived, is not old.
TEST(ReaderLine, ANoteNoOlderThanNowIsTheLine) {
  EXPECT_EQ(app::reader_line("Tag seen: 1 of 6", StatTone::Good, -5'000ms).text,
            "Tag seen: 1 of 6");
}

/// As grid numbers them. rgbd_stream_link_test.cpp checks them against grid's
/// own enum.
TEST(NoteTone, GoodIsGreenCautionIsOrangeAndTheRestIsGrey) {
  EXPECT_EQ(app::note_tone(0), StatTone::Neutral);
  EXPECT_EQ(app::note_tone(1), StatTone::Good);
  EXPECT_EQ(app::note_tone(2), StatTone::Warn);
  EXPECT_EQ(app::note_tone(3), StatTone::Neutral);
  EXPECT_EQ(app::note_tone(255), StatTone::Neutral);
}

}  // namespace

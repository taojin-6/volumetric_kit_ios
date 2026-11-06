// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file ReaderLine.hpp
/// @brief What the top bar's first chip says: the reader's own last line, or
///        the scanner's words for a reader that stopped talking.
///
/// The reader says one line with a tone up the live link, at least once a
/// second (`RgbdStreamWriter::reader_note`), and the scanner parses no word of
/// it: new words need no new app. Only the scanner can tell that the words have
/// stopped, so that one line is decided here. No note at all is no chip, which
/// is the bridge's nil.

#include <chrono>
#include <cstdint>
#include <string>

#include "StatTone.hpp"

namespace volumetric_kit::ios_app {

/// A note this old means a reader that stopped talking.
inline constexpr std::chrono::seconds kReaderQuiet{3};

/// @brief The chip's words and how they read.
struct ReaderLine {
  std::string text;
  StatTone tone = StatTone::Neutral;
};

/// @brief A note's tone as the screen reads it.
/// @param tone  As grid's `ViewerNote::Tone` numbers it, which this library
///              links nothing to name: 0 neutral, 1 good, 2 caution.
/// @return Neutral, Good or Warn; Neutral for a value it does not know, as
///         grid reads one.
constexpr StatTone note_tone(std::uint8_t tone) noexcept {
  switch (tone) {
    case 1:
      return StatTone::Good;
    case 2:
      return StatTone::Warn;
    default:
      return StatTone::Neutral;
  }
}

/// @param line  The reader's last line.
/// @param tone  Its tone.
/// @param age   How long ago it came.
/// @return @p line and @p tone; at @ref kReaderQuiet or older,
///         "Server quiet for N s" in whole seconds at Warn.
ReaderLine reader_line(std::string line, StatTone tone,
                       std::chrono::milliseconds age);

}  // namespace volumetric_kit::ios_app

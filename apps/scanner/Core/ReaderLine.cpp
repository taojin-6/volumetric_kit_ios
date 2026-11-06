// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "ReaderLine.hpp"

#include <utility>

namespace volumetric_kit::ios_app {

ReaderLine reader_line(std::string line, StatTone tone,
                       std::chrono::milliseconds age) {
  if (age < kReaderQuiet) return {std::move(line), tone};
  const auto quiet = std::chrono::duration_cast<std::chrono::seconds>(age);
  return {"Server quiet for " + std::to_string(quiet.count()) + " s",
          StatTone::Warn};
}

}  // namespace volumetric_kit::ios_app

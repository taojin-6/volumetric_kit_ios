// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "FailureHandling.hpp"

namespace volumetric_kit::ios_app {

void record_error(std::uint64_t& count, std::string& stored,
                  std::string_view message, std::string_view context) noexcept {
  ++count;
  try {
    std::string next(context);
    next.append(message);
    stored.swap(next);
  } catch (...) {
    stored.clear();
  }
}

}  // namespace volumetric_kit::ios_app

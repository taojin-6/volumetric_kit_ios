// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file StreamRate.hpp
/// @brief The stream's frames per second for the panel, over a window long
///        enough to read steadily at its 10 Hz.

#include <cstdint>

namespace volumetric_kit::ios_app {

/// @brief A running count's rate, new at most every @ref kWindowSeconds.
class StreamRate {
 public:
  /// A half-second window at 10 Hz reads 8, 10 or 12 by where it falls.
  static constexpr double kWindowSeconds = 2.0;

  /// @brief The count now.
  /// @param count    The running count.
  /// @param seconds  A steady clock, in seconds.
  /// @return The rate over the last whole window; 0 before the first.
  double take(std::uint64_t count, double seconds) noexcept {
    if (!started_) {
      started_ = true;
    } else if (seconds - since_ < kWindowSeconds) {
      return rate_;
    } else {
      rate_ = count >= from_
                  ? static_cast<double>(count - from_) / (seconds - since_)
                  : 0.0;
    }
    since_ = seconds;
    from_ = count;
    return rate_;
  }

 private:
  bool started_ = false;
  double since_ = 0.0;
  std::uint64_t from_ = 0;
  double rate_ = 0.0;
};

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "RateGate.hpp"

namespace volumetric_kit::ios_app {

namespace {

constexpr std::uint64_t kNsPerSecond = 1'000'000'000;

}  // namespace

RateGate::RateGate(std::uint32_t hz) noexcept
    : hz_(hz), period_ns_(hz == 0 ? 0 : kNsPerSecond / hz) {}

bool RateGate::take(std::uint64_t timestamp_ns) noexcept {
  if (started_ && timestamp_ns <= last_ns_) return false;
  std::uint64_t interval = 0;
  if (started_ && period_ns_ != 0) {
    // The interval whose start, less a tenth of a period, the frame is at or
    // past. No overflow: frames are nanoseconds on one clock, far below
    // 2^64 apart.
    const std::uint64_t elapsed = timestamp_ns - first_ns_;
    interval = (elapsed + period_ns_ / 10) / period_ns_;
    if (interval <= last_interval_) return false;
  }
  if (!started_ || interval > last_interval_ + 1) {
    // The first frame, or the first after an interval with none: the
    // intervals start from it.
    started_ = true;
    first_ns_ = timestamp_ns;
    interval = 0;
  }
  last_interval_ = interval;
  last_ns_ = timestamp_ns;
  return true;
}

bool RateGate::due(std::uint64_t timestamp_ns) const noexcept {
  RateGate trial = *this;
  return trial.take(timestamp_ns);
}

}  // namespace volumetric_kit::ios_app

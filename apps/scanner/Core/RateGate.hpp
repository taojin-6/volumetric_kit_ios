// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file RateGate.hpp
/// @brief Which of a camera's frames a stream sends, at a rate below the
///        camera's: ARKit's 60 Hz cut to the stream's 10 Hz.
///
/// Time from the first frame taken is cut into intervals of one period each.
/// A frame is taken when it is the first of an interval no frame has taken
/// yet, so at most one frame goes out an interval, and a source faster than
/// the gate sends exactly the gate's rate on average, however its frames fall:
/// from 60 Hz, every sixth. A frame up to a tenth of a period before its
/// interval's start counts for the interval, so a source at the gate's own
/// rate keeps every frame through the jitter of its timestamps. A source
/// slower than the gate keeps every frame. After a pause, which leaves an
/// interval with no frame, the next frame is taken at once and the intervals
/// start again from it, so no burst follows it.
///
/// A frame the gate does not take is not a lost frame: the stream numbers only
/// what it sends (`RgbdFrame::sequence`'s rate-gate rule).
///
/// @code
/// app::RateGate gate(10);
/// // ... each frame, after its pose is final (MapSteps.hpp):
/// if (!gate.take(frame.timestamp_ns)) continue;  // not this frame
/// @endcode

#include <cstdint>

namespace volumetric_kit::ios_app {

class RateGate {
 public:
  /// @param hz  Frames a second it takes at most; 0 takes every frame.
  explicit RateGate(std::uint32_t hz) noexcept;

  /// @brief Decide whether the frame captured at @p timestamp_ns goes out.
  /// @param timestamp_ns  Its capture time, on one clock for every frame.
  /// @return `true` for a frame taken: the first frame, and after it the
  ///         first frame of each interval. `false` for a frame at or before
  ///         the last frame taken, which a gate cannot place.
  bool take(std::uint64_t timestamp_ns) noexcept;

  /// @return The rate it was made with.
  std::uint32_t hz() const noexcept { return hz_; }

 private:
  std::uint32_t hz_ = 0;
  std::uint64_t period_ns_ = 0;      // 0: every frame
  std::uint64_t first_ns_ = 0;       // where the intervals start
  std::uint64_t last_ns_ = 0;        // the last frame taken
  std::uint64_t last_interval_ = 0;  // its interval, from 0
  bool started_ = false;
};

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file StageTiming.hpp
/// @brief How the read-out shows one stage row: busy beside wait on the host,
///        and a device cell that says why it holds no time.
///
/// The rule gfx's stage table draws, kept here over a mirror of the core's row
/// so it is host tested without linking the core into vi_core. `Readout.mm`
/// copies each `core::StageRow` into a @ref StageTiming and static_asserts
/// @ref DeviceTime against the core's enum and the Objective-C
/// `VolumetricDeviceTime`, value for value, as `StatTone` is pinned.

#include <cstdint>
#include <cstdio>
#include <string>

namespace volumetric_kit::ios_app {

/// @brief What a row's device half holds: the core's `DeviceTime`.
enum class DeviceTime : std::uint8_t {
  /// A host-only stage: no device half.
  None = 0,
  /// @ref StageTiming::gpu_ms is the stage's device time.
  Measured = 1,
  /// Its device work ran untimed: no query room, its batch failed, or its
  /// timestamps were not yet readable. The next frame may time again.
  NotTimed = 2,
  /// The queue cannot time, or the timer has no queries.
  Unavailable = 3,
};

/// @brief One stage row's figures, copied from the core's `StageRow`.
struct StageTiming {
  /// The host span, busy and blocked together, in milliseconds.
  double cpu_ms = 0.0;
  /// The part of @ref cpu_ms blocked on the device.
  double wait_ms = 0.0;
  /// Device time; meaningful only when @ref device is Measured.
  double gpu_ms = 0.0;
  DeviceTime device = DeviceTime::None;
};

/// @brief The host span less its wait, never negative: a mean's rounding can
///        leave a wait-only row's wait a few ulps above its span.
constexpr double busy_ms(const StageTiming& row) noexcept {
  return row.cpu_ms > row.wait_ms ? row.cpu_ms - row.wait_ms : 0.0;
}

/// @brief The device cell's text: the time at @p decimals places when
///        measured, `n/t` when the work ran untimed, `n/a` when the queue
///        cannot time, and empty for a host-only stage.
inline std::string device_cell(const StageTiming& row, int decimals) {
  switch (row.device) {
    case DeviceTime::Measured: {
      char text[32];
      std::snprintf(text, sizeof(text), "%.*f", decimals, row.gpu_ms);
      return text;
    }
    case DeviceTime::NotTimed:
      return "n/t";
    case DeviceTime::Unavailable:
      return "n/a";
    case DeviceTime::None:
      break;
  }
  return "";
}

}  // namespace volumetric_kit::ios_app

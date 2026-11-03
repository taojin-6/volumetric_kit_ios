// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CaptureReport.hpp
/// @brief Which camera a scanner launch asks for (`-source`, `-minutes`), and
///        the console line a run prints every @ref kReportEverySeconds:
///        counts, times, states and colour codes, never a pixel, a depth
///        value, a pose or a path.
///
/// Pure, so the argument rule and the line are host tested. The bridge
/// (Bridge/FuseLoop.cpp) fills a @ref CaptureReport from `ArkitSensor`'s
/// status and counters and the fuse thread's own timings, and prints it.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace volumetric_kit::ios_app {

/// @brief Where the scanner's frames come from (`ArkitSource`, mirrored so
///        this library includes no sibling).
enum class CaptureSource : std::uint8_t {
  Camera,   ///< ARKit's camera.
  Pattern,  ///< sensor's test pattern: no camera opens.
};

/// @return "camera" or "pattern".
const char* to_string(CaptureSource source) noexcept;

/// @brief What a launch's arguments ask of the camera.
struct LaunchOptions {
  /// The source; empty when an argument was refused, so a mistyped launch
  /// opens no camera rather than falling back to it.
  std::optional<CaptureSource> source = CaptureSource::Camera;
  /// Minutes the scanner runs before it stops and exits; 0 runs on.
  std::uint32_t minutes = 0;
  /// Why an argument was refused, for the screen and the console; empty when
  /// none was.
  std::string refusal;
};

/// @brief The most minutes `-minutes` takes: a day.
inline constexpr std::uint32_t kMaxRunMinutes = 24 * 60;

/// @return What @p arguments (the process's, the program first) ask for:
///         `-source camera` or `-source pattern` (the camera when absent), and
///         `-minutes N` with N a whole number from 1 to @ref kMaxRunMinutes
///         (run on when absent). Another value, or a flag with none after it,
///         is refused: `source` empty and `refusal` naming it.
LaunchOptions launch_options(const std::vector<std::string>& arguments);

/// @brief How often a run prints its @ref capture_report_line.
inline constexpr std::uint32_t kReportEverySeconds = 10;

/// @brief One window of a run, as its console line says it.
///
/// The counters are the camera's since the start (`SensorStats` and
/// `ArkitStatus` begin again at each start, a resume after a lock or
/// backgrounding too); the times are this window's.
struct CaptureReport {
  CaptureSource source = CaptureSource::Camera;
  double seconds = 0.0;  ///< Since the start.
  bool stop = false;     ///< The line at a stop: each stop prints one.
  /// Seconds from the start to its first frame; empty before one came.
  std::optional<double> first_frame_s;
  std::string tracking;         ///< ARKit's tracking at the last frame, named.
  bool interrupted = false;     ///< ARKit has stopped the camera for a while.
  std::string session_error;    ///< ARKit's failure; empty for none.
  std::uint64_t received = 0;   ///< `SensorStats::received`.
  std::uint64_t delivered = 0;  ///< `SensorStats::delivered`.
  std::uint64_t dropped = 0;    ///< `SensorStats::dropped`.
  std::uint64_t failed = 0;     ///< `SensorStats::failed`.
  std::uint64_t withheld = 0;   ///< `ArkitStatus::withheld`, part of failed.
  std::uint64_t refused = 0;    ///< `ArkitStatus::refused`, part of failed.
  std::uint64_t missed = 0;     ///< `ArkitStatus::missed`.
  std::uint64_t fused = 0;      ///< Frames fused since the start.
  /// This window's polls that took a frame, ms each.
  std::vector<double> poll_ms;
  /// This window's fused frames, ms each: prep + allocate + integrate.
  std::vector<double> fuse_ms;
  /// Of those, the frame prep alone, ms each.
  std::vector<double> prep_ms;
  /// The last checked picture's colour tags as H.273 code points
  /// (`sensor::ColorCodes`); @ref codes_known false before one was read.
  bool codes_known = false;
  int matrix = 2;
  int transfer = 2;
  int primaries = 2;
  bool full_range = false;
  int chroma_location = 0;
  /// The first frame-contract rule broken since the start
  /// (`testing::RgbdContract`); empty when kept.
  std::string breach;
  /// The pattern's frames checked for their index, and those whose depth did
  /// not carry their sequence's; unused for the camera.
  std::uint64_t index_checked = 0;
  std::uint64_t index_mismatches = 0;
};

/// @return @p report as one line, "sa <source> <seconds> s: ..." ("sa stop"
///         at a stop): tracking; the counters; frames fused; when the first
///         frame came; poll, fuse and prep ms at p50 and p95 over the window,
///         with their counts; the colour codes; the contract; and, for the
///         pattern, the index check.
std::string capture_report_line(const CaptureReport& report);

}  // namespace volumetric_kit::ios_app

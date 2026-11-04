// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CaptureReport.hpp
/// @brief What a scanner launch asks for (`-source`, `-minutes`, `-record`,
///        `-connect`), and the console lines a run prints every
///        @ref kReportEverySeconds: counts, times, states and colour codes,
///        never a pixel, a depth value, a pose or a path.
///
/// Pure, so the argument rule and the lines are host tested. The bridge
/// (Bridge/FuseLoop.cpp) fills a @ref CaptureReport from `ArkitSensor`'s
/// status and counters and the fuse thread's own timings, and a
/// @ref StreamReport from the stream sender's, and prints them.

#include <cstddef>
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
  /// The session file the stream writes, a plain name the app's Documents
  /// holds it under; empty writes none.
  std::string record;
  /// The reader the stream dials once the scanner is up, as the on-screen
  /// entry's address names one ("a.b.c.d:port", or the address alone for the
  /// default port), which the link checks then; empty dials none.
  std::string connect;
  /// A dry run for the launch's life: the stream codes and counts, writes no
  /// file and refuses every connect, the on-screen one too, so nothing the
  /// camera sees leaves the device.
  bool dry = false;
  /// A test of the camera's retry (CameraOpen.hpp), with the pattern alone:
  /// the first this many opens wait 1 s and come back `NotFound`, as a
  /// camera that gave no frame in time does; 0 when the flag is absent.
  std::uint32_t fail_opens = 0;
  /// Why an argument was refused, for the screen and the console; empty when
  /// none was.
  std::string refusal;
};

/// @brief The most minutes `-minutes` takes: a day.
inline constexpr std::uint32_t kMaxRunMinutes = 24 * 60;

/// @brief The most opens `-fail-opens` fails.
inline constexpr std::uint32_t kMaxFailOpens = 9;

/// @brief The longest `-record` name.
inline constexpr std::size_t kMaxRecordName = 64;

/// @return What @p arguments (the process's, the program first) ask for:
///         `-source camera` or `-source pattern` (the camera when absent);
///         `-minutes N` with N a whole number from 1 to @ref kMaxRunMinutes
///         (run on when absent); `-record <name>`, a name of letters, digits,
///         `.`, `_` and `-`, not starting with `.`, ending `.vrst`, at most
///         @ref kMaxRecordName long (no file when absent);
///         `-connect <address>`, digits, `.` and `:` (no dial when absent);
///         `-dry`, which takes no value and neither of the last two; and
///         `-fail-opens N`, N from 1 to @ref kMaxFailOpens, with
///         `-source pattern` alone. Another value, a flag with none after
///         it, `-dry` with `-record` or `-connect`, or `-fail-opens` without
///         the pattern is refused: `source` empty and `refusal` naming it.
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

/// @brief The stream's part of a window, as its console line says it: the
///        sender's counts since it opened
///        (Session/rgbd_stream_sender.hpp's `RgbdStreamSenderStats`, mirrored
///        so this library includes no sibling) and this window's rates and
///        times.
struct StreamReport {
  double seconds = 0.0;  ///< Since the run's start, as its `sa` line's.
  bool stop = false;     ///< The line at a stop.
  /// A sender is open; when not, @ref error says why and nothing else is
  /// read.
  bool on = false;
  /// A session is open, a file, a link or both; otherwise the sender codes
  /// and counts only (the dry run).
  bool session = false;
  std::uint64_t taken = 0;   ///< Frames the rate gate took.
  std::uint64_t busy = 0;    ///< Due frames left while the sender was busy.
  std::uint64_t coded = 0;   ///< Pictures coded.
  std::uint64_t keys = 0;    ///< Of them, keys.
  std::uint64_t failed = 0;  ///< Pictures not coded: their frames went nowhere.
  std::uint64_t written = 0;    ///< Frames written whole to the session.
  std::uint64_t refused = 0;    ///< Coded frames the session refused.
  std::uint64_t map_steps = 0;  ///< Map steps taken out of the poses.
  /// This window's access units and depth records, bits a second.
  double video_bits_per_s = 0.0;
  double depth_bits_per_s = 0.0;
  /// This window's pictures, ms each: the send's return, and send to the
  /// access unit back.
  std::vector<double> send_ms;
  std::vector<double> coded_ms;
  std::size_t viewers = 0;          ///< Live readers connected now.
  std::uint64_t dial_failures = 0;  ///< Dials failed in a row.
  /// The sender's last failure, or why none is open; empty for none.
  std::string error;
};

/// @return @p report as one line, "st <seconds> s: ..." ("st stop" at a
///         stop): "dry" or "session"; the counts; video and depth Mbit/s;
///         send and coded ms at p50 and p95 with their counts; the live
///         readers and failed dials; the last failure. "st <seconds> s: off,
///         <error>" with no sender.
std::string stream_report_line(const StreamReport& report);

/// @return @p text with each @p path in it, the session file's place on the
///         device, said as "the session file", so a message the console or
///         the screen shows never holds a path; @p text as it is for an empty
///         @p path.
std::string without_path(std::string text, const std::string& path);

}  // namespace volumetric_kit::ios_app

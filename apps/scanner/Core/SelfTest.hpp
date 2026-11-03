// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file SelfTest.hpp
/// @brief The scanner's launch self-tests: which one a launch asks for, and
///        what `-self-test hevc` reports and whether it passed.
///
/// Pure, so the argument rule, the report's text and its verdict are host
/// tested. The run itself, on the device's encoder, is
/// `Bridge/HevcSelfTest.cpp`'s.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace volumetric_kit::ios_app {

/// @brief What a launch's arguments ask for.
enum class SelfTestKind : std::uint8_t {
  None,     ///< No `-self-test`: the scanner runs.
  Hevc,     ///< `-self-test hevc`.
  Unknown,  ///< `-self-test` with no name after it, or a name no test has.
};

/// @return What @p arguments (the process's, the program first) ask for:
///         the word after the first `-self-test`.
SelfTestKind self_test_kind(const std::vector<std::string>& arguments);

/// @brief One run of sensor's encoder rules (`hevc_encoder_breach`).
struct RuleCase {
  enum class Result : std::uint8_t {
    Kept,    ///< Every rule held.
    Broken,  ///< A rule broke; @ref detail names it ("C5: ...").
    /// The run gave a status instead, such as the session's refusal to open;
    /// @ref detail is its text.
    NotRun,
  };
  std::string name;  ///< The case: size, colour, QP.
  Result result = Result::NotRun;
  std::string detail;    ///< Empty when kept.
  double seconds = 0.0;  ///< The run's wall time.
};

/// @brief The encoder's timing on one session, one picture in flight: each
///        picture sent, then received before the next.
struct EncodeTiming {
  std::string what;             ///< The session: size, colour, QP.
  std::uint32_t sent = 0;       ///< Pictures sent and timed.
  std::uint32_t coded = 0;      ///< Of those, access units received.
  std::uint32_t keys = 0;       ///< Of those coded, IDRs.
  std::string failure;          ///< The first failure's text; empty for none.
  std::vector<double> send_ms;  ///< Each send's return time.
  std::vector<double> to_unit_ms;  ///< Each send to its access unit.
  std::vector<std::size_t> bytes;  ///< Each access unit's size.
};

/// @return The @p p-th percentile (0 to 100) of @p values by nearest rank;
///         0 for none.
double percentile(std::vector<double> values, double p);

/// @return @p c as one report line.
std::string rule_line(const RuleCase& c);

/// @return @p t as report lines: counts, then the send return time, send to
///         access unit and access unit bytes at p50, p95 and max.
std::vector<std::string> timing_lines(const EncodeTiming& t);

/// @return Whether the run passed: at least one case, every case kept, and
///         the timing coded every picture it sent, with no failure.
bool self_test_passed(const std::vector<RuleCase>& cases,
                      const EncodeTiming& timing);

/// @return The report's last line: PASS or FAIL, with the counts it rests
///         on.
std::string verdict_line(const std::vector<RuleCase>& cases,
                         const EncodeTiming& timing);

}  // namespace volumetric_kit::ios_app

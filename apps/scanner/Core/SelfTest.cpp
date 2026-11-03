// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "SelfTest.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace volumetric_kit::ios_app {
namespace {

std::string seconds_text(double seconds) {
  char text[32];
  std::snprintf(text, sizeof text, "%.1f s", seconds);
  return text;
}

// "p50 a  p95 b  max c" of @p values, each with @p decimals.
std::string spread(const std::vector<double>& values, int decimals) {
  char text[96];
  std::snprintf(text, sizeof text, "p50 %.*f  p95 %.*f  max %.*f", decimals,
                percentile(values, 50), decimals, percentile(values, 95),
                decimals, percentile(values, 100));
  return text;
}

std::size_t kept(const std::vector<RuleCase>& cases) {
  return static_cast<std::size_t>(std::count_if(
      cases.begin(), cases.end(),
      [](const RuleCase& c) { return c.result == RuleCase::Result::Kept; }));
}

}  // namespace

SelfTestKind self_test_kind(const std::vector<std::string>& arguments) {
  const auto flag = std::find(arguments.begin(), arguments.end(), "-self-test");
  if (flag == arguments.end()) return SelfTestKind::None;
  if (flag + 1 != arguments.end() && flag[1] == "hevc") {
    return SelfTestKind::Hevc;
  }
  return SelfTestKind::Unknown;
}

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double rank =
      std::ceil(std::clamp(p, 0.0, 100.0) / 100.0 * values.size());
  const std::size_t at = rank < 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
  return values[std::min(at, values.size() - 1)];
}

std::string rule_line(const RuleCase& c) {
  switch (c.result) {
    case RuleCase::Result::Kept:
      return "kept     " + c.name + " (" + seconds_text(c.seconds) + ")";
    case RuleCase::Result::Broken:
      return "BROKEN   " + c.name + " (" + seconds_text(c.seconds) +
             "): " + c.detail;
    case RuleCase::Result::NotRun:
      return "NOT RUN  " + c.name + ": " + c.detail;
  }
  return "NOT RUN  " + c.name + ": " + c.detail;
}

std::vector<std::string> timing_lines(const EncodeTiming& t) {
  if (t.sent == 0) {
    return {"timing " + t.what + ": not run: " + t.failure};
  }
  std::string head = "timing " + t.what + ": " + std::to_string(t.sent) +
                     " sent, " + std::to_string(t.coded) + " coded, " +
                     std::to_string(t.keys) + " keys";
  if (!t.failure.empty()) head += "; first failure: " + t.failure;
  const std::vector<double> bytes(t.bytes.begin(), t.bytes.end());
  return {head, "  send returns  " + spread(t.send_ms, 3) + " ms",
          "  send to AU    " + spread(t.to_unit_ms, 3) + " ms",
          "  AU bytes      " + spread(bytes, 0)};
}

bool self_test_passed(const std::vector<RuleCase>& cases,
                      const EncodeTiming& timing) {
  return !cases.empty() && kept(cases) == cases.size() && timing.sent > 0 &&
         timing.coded == timing.sent && timing.failure.empty();
}

std::string verdict_line(const std::vector<RuleCase>& cases,
                         const EncodeTiming& timing) {
  return std::string("self-test hevc: ") +
         (self_test_passed(cases, timing) ? "PASS" : "FAIL") + ": " +
         std::to_string(kept(cases)) + " of " + std::to_string(cases.size()) +
         " cases kept the rules; " + std::to_string(timing.coded) + " of " +
         std::to_string(timing.sent) + " timed pictures coded";
}

}  // namespace volumetric_kit::ios_app

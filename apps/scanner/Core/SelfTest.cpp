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

// The tail of a case's line for its run without the decode.
std::string without_decode_text(RuleCase::Result result,
                                const std::string& detail) {
  switch (result) {
    case RuleCase::Result::Kept:
      return "kept";
    case RuleCase::Result::Broken:
      return detail;  // the rule's own name leads it ("C3: ...")
    case RuleCase::Result::Failed:
      return "failed: " + detail;
    case RuleCase::Result::NotRun:
      return "not run: " + detail;
  }
  return "not run: " + detail;
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

bool rerun_without_decode(const RuleCase& c) {
  return c.result == RuleCase::Result::Failed ||
         (c.result == RuleCase::Result::Broken &&
          c.detail.rfind("C6:", 0) == 0);
}

std::string rule_line(const RuleCase& c) {
  const std::string timed = c.name + " (" + seconds_text(c.seconds) + ")";
  std::string line;
  switch (c.result) {
    case RuleCase::Result::Kept:
      line = "kept     " + timed;
      break;
    case RuleCase::Result::Broken:
      line = "BROKEN   " + timed + ": " + c.detail;
      break;
    case RuleCase::Result::Failed:
      line = "FAILED   " + timed + ": " + c.detail;
      break;
    case RuleCase::Result::NotRun:
      line = "NOT RUN  " + c.name + ": " + c.detail;
      break;
  }
  if (c.without_decode) {
    line += "; without C6: " +
            without_decode_text(*c.without_decode, c.without_decode_detail);
  }
  return line;
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
                      const EncodeTiming& timing, bool decoded) {
  return decoded && !cases.empty() && kept(cases) == cases.size() &&
         timing.sent > 0 && timing.coded == timing.sent &&
         timing.failure.empty();
}

std::string verdict_line(const std::vector<RuleCase>& cases,
                         const EncodeTiming& timing, bool decoded) {
  return std::string("self-test hevc: ") +
         (self_test_passed(cases, timing, decoded) ? "PASS" : "FAIL") + ": " +
         std::to_string(kept(cases)) + " of " + std::to_string(cases.size()) +
         " cases kept the rules" + (decoded ? "" : ", none decoded (no GPU)") +
         "; " + std::to_string(timing.coded) + " of " +
         std::to_string(timing.sent) + " timed pictures coded";
}

}  // namespace volumetric_kit::ios_app

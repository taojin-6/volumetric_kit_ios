// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file self_test_test.cpp
/// @brief Which self-test a launch asks for, and what `-self-test hevc`
///        reports and when it passes.

#include "SelfTest.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::EncodeTiming;
using app::RuleCase;
using app::SelfTestKind;

bool has(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

RuleCase kept_case(const std::string& name) {
  RuleCase c;
  c.name = name;
  c.result = RuleCase::Result::Kept;
  c.seconds = 2.44;
  return c;
}

EncodeTiming full_timing() {
  EncodeTiming t;
  t.what = "1920x1440 BT.601 full, QP 22";
  t.sent = 4;
  t.coded = 4;
  t.keys = 1;
  t.send_ms = {0.010, 0.008, 0.021, 0.009};
  t.to_unit_ms = {5.6, 5.5, 5.7, 5.4};
  t.bytes = {90000, 12000, 13000, 11000};
  return t;
}

TEST(SelfTestKind, NoFlagRunsTheScanner) {
  EXPECT_EQ(app::self_test_kind({}), SelfTestKind::None);
  EXPECT_EQ(app::self_test_kind({"scanner"}), SelfTestKind::None);
  EXPECT_EQ(app::self_test_kind({"scanner", "hevc"}), SelfTestKind::None);
}

TEST(SelfTestKind, TheWordAfterTheFlagNamesTheTest) {
  EXPECT_EQ(app::self_test_kind({"scanner", "-self-test", "hevc"}),
            SelfTestKind::Hevc);
  EXPECT_EQ(
      app::self_test_kind({"scanner", "-x", "1", "-self-test", "hevc", "-y"}),
      SelfTestKind::Hevc);
}

TEST(SelfTestKind, NoNameOrAnUnknownOneIsUnknown) {
  EXPECT_EQ(app::self_test_kind({"scanner", "-self-test"}),
            SelfTestKind::Unknown);
  EXPECT_EQ(app::self_test_kind({"scanner", "-self-test", "jpeg"}),
            SelfTestKind::Unknown);
  EXPECT_EQ(app::self_test_kind({"scanner", "-self-test", "HEVC"}),
            SelfTestKind::Unknown);
  // The first flag decides.
  EXPECT_EQ(app::self_test_kind(
                {"scanner", "-self-test", "jpeg", "-self-test", "hevc"}),
            SelfTestKind::Unknown);
}

TEST(SelfTestPercentile, NearestRank) {
  std::vector<double> values;
  for (int i = 100; i >= 1; --i) values.push_back(i);  // unsorted
  EXPECT_EQ(app::percentile(values, 0), 1.0);
  EXPECT_EQ(app::percentile(values, 50), 50.0);
  EXPECT_EQ(app::percentile(values, 95), 95.0);
  EXPECT_EQ(app::percentile(values, 100), 100.0);
  EXPECT_EQ(app::percentile({7.0}, 95), 7.0);
  EXPECT_EQ(app::percentile({}, 50), 0.0);
}

TEST(SelfTestReport, AKeptCaseSaysSo) {
  const std::string line = app::rule_line(kept_case("1920x1440 BT.601 full"));
  EXPECT_TRUE(has(line, "kept"));
  EXPECT_TRUE(has(line, "1920x1440 BT.601 full"));
  EXPECT_TRUE(has(line, "2.4 s"));
}

TEST(SelfTestReport, ABrokenRuleIsReportedByName) {
  RuleCase c = kept_case("1920x1440 BT.601 video");
  c.result = RuleCase::Result::Broken;
  c.detail = "C5: picture 3's slice QP is 23, not 22";
  const std::string line = app::rule_line(c);
  EXPECT_TRUE(has(line, "BROKEN"));
  EXPECT_TRUE(has(line, "1920x1440 BT.601 video"));
  EXPECT_TRUE(has(line, "C5: picture 3's slice QP is 23, not 22"));
}

TEST(SelfTestReport, ACaseThatDidNotRunCarriesItsStatus) {
  RuleCase c;
  c.name = "1920x1440 BT.709 full";
  c.detail = "Unsupported: takes no QP a picture (SupportsBaseFrameQP)";
  const std::string line = app::rule_line(c);
  EXPECT_TRUE(has(line, "NOT RUN"));
  EXPECT_TRUE(has(line, c.detail));
}

TEST(SelfTestReport, TimingGivesCountsAndSpreads) {
  const auto lines = app::timing_lines(full_timing());
  ASSERT_EQ(lines.size(), 4U);
  EXPECT_TRUE(has(lines[0], "1920x1440 BT.601 full, QP 22"));
  EXPECT_TRUE(has(lines[0], "4 sent, 4 coded, 1 keys"));
  EXPECT_TRUE(has(lines[1], "send returns"));
  EXPECT_TRUE(has(lines[1], "p50 0.009  p95 0.021  max 0.021 ms"));
  EXPECT_TRUE(has(lines[2], "send to AU"));
  EXPECT_TRUE(has(lines[2], "p50 5.500  p95 5.700  max 5.700 ms"));
  EXPECT_TRUE(has(lines[3], "p50 12000  p95 90000  max 90000"));
}

TEST(SelfTestReport, TimingNamesItsFailure) {
  EncodeTiming t = full_timing();
  t.coded = 3;
  t.failure = "IoError: dropped";
  EXPECT_TRUE(has(app::timing_lines(t)[0], "first failure: IoError: dropped"));

  EncodeTiming none;
  none.what = "1920x1440";
  none.failure = "Unsupported: busy";
  const auto lines = app::timing_lines(none);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_TRUE(has(lines[0], "not run: Unsupported: busy"));
}

TEST(SelfTestVerdict, PassesOnlyWhenEveryCaseKeptAndEveryPictureCoded) {
  const std::vector<RuleCase> cases = {kept_case("a"), kept_case("b")};
  EXPECT_TRUE(app::self_test_passed(cases, full_timing()));
  EXPECT_TRUE(has(app::verdict_line(cases, full_timing()),
                  "PASS: 2 of 2 cases kept the rules; 4 of 4 timed pictures"));

  std::vector<RuleCase> broken = cases;
  broken[1].result = RuleCase::Result::Broken;
  broken[1].detail = "C7: the key after a drop";
  EXPECT_FALSE(app::self_test_passed(broken, full_timing()));
  EXPECT_TRUE(
      has(app::verdict_line(broken, full_timing()), "FAIL: 1 of 2 cases kept"));

  std::vector<RuleCase> not_run = cases;
  not_run[0].result = RuleCase::Result::NotRun;
  EXPECT_FALSE(app::self_test_passed(not_run, full_timing()));

  EXPECT_FALSE(app::self_test_passed({}, full_timing()));
}

TEST(SelfTestVerdict, FailsOnATimingShortfall) {
  const std::vector<RuleCase> cases = {kept_case("a")};
  EncodeTiming lost = full_timing();
  lost.coded = 3;
  EXPECT_FALSE(app::self_test_passed(cases, lost));

  EncodeTiming failed = full_timing();
  failed.failure = "Backend: lost";
  EXPECT_FALSE(app::self_test_passed(cases, failed));

  EXPECT_FALSE(app::self_test_passed(cases, EncodeTiming{}));
}

}  // namespace

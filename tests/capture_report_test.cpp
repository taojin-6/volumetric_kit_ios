// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file capture_report_test.cpp
/// @brief Which camera a launch asks for, and the console line a run prints.

#include "CaptureReport.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::CaptureReport;
using app::CaptureSource;
using app::LaunchOptions;

bool has(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

LaunchOptions parse(std::vector<std::string> arguments) {
  arguments.insert(arguments.begin(), "scanner");
  return app::launch_options(arguments);
}

TEST(LaunchOptions, NoArgumentsIsTheCameraRunningOn) {
  const LaunchOptions o = parse({});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Camera);
  EXPECT_EQ(o.minutes, 0u);
  EXPECT_TRUE(o.refusal.empty());
}

TEST(LaunchOptions, SourceAndMinutesAreRead) {
  const LaunchOptions o = parse({"-source", "pattern", "-minutes", "2"});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Pattern);
  EXPECT_EQ(o.minutes, 2u);
  EXPECT_TRUE(o.refusal.empty());

  const LaunchOptions camera = parse({"-minutes", "1440", "-source", "camera"});
  ASSERT_TRUE(camera.source.has_value());
  EXPECT_EQ(*camera.source, CaptureSource::Camera);
  EXPECT_EQ(camera.minutes, 1440u);
}

TEST(LaunchOptions, OtherArgumentsAreLeftAlone) {
  // The program itself is never read as a flag, nor are other flags' words.
  const LaunchOptions o = app::launch_options(
      {"-source", "-self-test", "hevc", "-dry", "-source", "pattern"});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Pattern);
}

/// A mistyped source opens no camera: it is refused, never read as the
/// camera, so a launch meant for the pattern cannot turn the camera on.
TEST(LaunchOptions, AMistypedSourceOpensNoCamera) {
  const LaunchOptions o = parse({"-source", "patern"});
  EXPECT_FALSE(o.source.has_value());
  EXPECT_TRUE(has(o.refusal, "-source takes camera or pattern"));
  EXPECT_TRUE(has(o.refusal, "\"patern\""));

  // A refusal holds against a valid source after it, either one.
  for (const char* valid : {"camera", "pattern"}) {
    const LaunchOptions later = parse({"-source", "x", "-source", valid});
    EXPECT_FALSE(later.source.has_value()) << valid;
  }
}

TEST(LaunchOptions, AFlagWithNoValueIsRefused) {
  for (const char* flag : {"-source", "-minutes"}) {
    const LaunchOptions o = parse({flag});
    EXPECT_FALSE(o.source.has_value()) << flag;
    EXPECT_TRUE(has(o.refusal, "needs a value")) << flag;
  }
}

TEST(LaunchOptions, MinutesOutsideOneToADayAreRefused) {
  for (const char* bad : {"0", "1441", "-1", "2.5", "two", "", "99999"}) {
    const LaunchOptions o = parse({"-minutes", bad});
    EXPECT_FALSE(o.source.has_value()) << bad;
    EXPECT_TRUE(has(o.refusal, "-minutes takes a whole number from 1 to 1440"))
        << bad;
  }
}

CaptureReport pattern_report() {
  CaptureReport r;
  r.source = CaptureSource::Pattern;
  r.seconds = 10.04;
  r.tracking = "normal";
  r.received = 601;
  r.delivered = 600;
  r.dropped = 1;
  r.fused = 600;
  r.first_frame_s = 0.024;
  r.poll_ms = {0.10, 0.20, 0.30, 0.40};
  r.fuse_ms = {2.0, 4.0, 3.0, 1.0};
  r.prep_ms = {0.5, 0.7};
  r.codes_known = true;
  r.matrix = 6;
  r.transfer = 1;
  r.primaries = 1;
  r.full_range = true;
  r.index_checked = 600;
  r.index_mismatches = 0;
  return r;
}

TEST(CaptureReportLine, CarriesCountsTimesStatesAndCodes) {
  const std::string line = app::capture_report_line(pattern_report());
  EXPECT_EQ(line.rfind("sa pattern 10.0 s: tracking normal | ", 0), 0u) << line;
  EXPECT_TRUE(has(line,
                  "received 601 delivered 600 dropped 1 failed 0 (withheld 0 "
                  "refused 0) missed 0 | fused 600 | first frame 0.02 s | "))
      << line;
  // Nearest rank: p50 of four is the second, p95 the fourth.
  EXPECT_TRUE(has(line, "poll ms p50 0.200 p95 0.400 (4)")) << line;
  EXPECT_TRUE(has(line,
                  "fuse ms p50 2.00 p95 4.00 (4), prep p50 0.50 p95 "
                  "0.70 (2)"))
      << line;
  EXPECT_TRUE(has(line, "codes 6/1/1 full loc 0")) << line;
  EXPECT_TRUE(has(line, "contract kept")) << line;
  EXPECT_TRUE(has(line, "index 600/600")) << line;
  EXPECT_FALSE(has(line, "session failed")) << line;
}

TEST(CaptureReportLine, SaysWhatIsMissingOrWrong) {
  CaptureReport r = pattern_report();
  r.stop = true;
  r.first_frame_s.reset();
  r.interrupted = true;
  r.poll_ms.clear();
  r.fuse_ms.clear();
  r.prep_ms.clear();
  r.codes_known = false;
  r.breach = "frame 12: sequence 5 after 7";
  r.index_mismatches = 3;
  r.session_error = "camera in use";
  const std::string line = app::capture_report_line(r);
  EXPECT_EQ(
      line.rfind("sa stop pattern 10.0 s: tracking normal, interrupted", 0), 0u)
      << line;
  EXPECT_TRUE(has(line, "| first frame - |")) << line;
  EXPECT_TRUE(has(line, "poll ms - (0) | fuse ms - (0), prep - (0)")) << line;
  EXPECT_TRUE(has(line, "codes -")) << line;
  EXPECT_TRUE(has(line, "contract: frame 12: sequence 5 after 7")) << line;
  EXPECT_TRUE(has(line, "index 597/600")) << line;
  EXPECT_TRUE(has(line, "session failed: camera in use")) << line;
}

TEST(CaptureReportLine, TheCameraHasNoIndexCheck) {
  CaptureReport r = pattern_report();
  r.source = CaptureSource::Camera;
  r.tracking.clear();
  r.full_range = false;
  const std::string line = app::capture_report_line(r);
  EXPECT_EQ(line.rfind("sa camera 10.0 s: tracking - | ", 0), 0u) << line;
  EXPECT_TRUE(has(line, "codes 6/1/1 limited loc 0")) << line;
  EXPECT_FALSE(has(line, "index")) << line;
}

}  // namespace

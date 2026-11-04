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
      {"-dry", "-self-test", "hevc", "-verbose", "-source", "pattern"});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Pattern);
  EXPECT_FALSE(o.dry);
}

/// `-dry` takes no value, and no file or reader: a dry launch sends nothing.
TEST(LaunchOptions, DryIsReadAndTakesNoRecordOrConnect) {
  const LaunchOptions o = parse({"-dry", "-minutes", "10"});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Camera);
  EXPECT_TRUE(o.dry);
  EXPECT_EQ(o.minutes, 10u);
  EXPECT_TRUE(o.refusal.empty());
  EXPECT_FALSE(parse({"-source", "pattern"}).dry);

  for (const std::vector<std::string>& with :
       {std::vector<std::string>{"-dry", "-record", "a.vrst"},
        std::vector<std::string>{"-connect", "10.8.0.1:7300", "-dry"}}) {
    const LaunchOptions refused = parse(with);
    EXPECT_FALSE(refused.source.has_value()) << with[0];
    EXPECT_TRUE(has(refused.refusal, "-dry records and sends nothing"))
        << refused.refusal;
  }
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

TEST(LaunchOptions, RecordAndConnectAreRead) {
  const LaunchOptions o =
      parse({"-source", "pattern", "-record", "i1v_pattern-1.vrst", "-connect",
             "192.168.1.20:7300"});
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Pattern);
  EXPECT_EQ(o.record, "i1v_pattern-1.vrst");
  EXPECT_EQ(o.connect, "192.168.1.20:7300");
  EXPECT_TRUE(o.refusal.empty());

  const LaunchOptions none = parse({"-source", "camera"});
  EXPECT_TRUE(none.record.empty());
  EXPECT_TRUE(none.connect.empty());
}

/// A record name is a plain file in the app's own folder: no directory, no
/// hidden file, nothing but a session's extension.
TEST(LaunchOptions, ARecordNameThatIsNotAPlainSessionFileIsRefused) {
  const std::string long_name = std::string(60, 'a') + ".vrst";  // 65
  for (const std::string& bad :
       {std::string("take"), std::string("take.mp4"), std::string(".vrst"),
        std::string(".hidden.vrst"), std::string("takes/a.vrst"),
        std::string("../a.vrst"), std::string("a b.vrst"), std::string(""),
        long_name}) {
    const LaunchOptions o = parse({"-record", bad});
    EXPECT_FALSE(o.source.has_value()) << bad;
    EXPECT_TRUE(o.record.empty()) << bad;
    EXPECT_TRUE(has(o.refusal, "-record takes a file name ending .vrst"))
        << bad;
  }
  EXPECT_TRUE(
      parse({"-record", std::string(59, 'a') + ".vrst"}).source.has_value());
}

TEST(LaunchOptions, AConnectThatCannotBeAnAddressIsRefused) {
  for (const char* bad : {"lab.local:7300", "-source", "", "10.0.0.1:7300 ",
                          "1.2.3.4;80", "255.255.255.255:655350"}) {
    const LaunchOptions o = parse({"-connect", bad});
    EXPECT_FALSE(o.source.has_value()) << bad;
    EXPECT_TRUE(o.connect.empty()) << bad;
    EXPECT_TRUE(has(o.refusal, "-connect takes a numeric address")) << bad;
  }
  // The link checks the rest: the default port, and the parts' ranges.
  EXPECT_EQ(parse({"-connect", "10.8.0.1"}).connect, "10.8.0.1");
}

TEST(LaunchOptions, AFlagWithNoValueIsRefused) {
  for (const char* flag : {"-source", "-minutes", "-record", "-connect"}) {
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

app::StreamReport session_stream() {
  app::StreamReport r;
  r.seconds = 20.02;
  r.on = true;
  r.session = true;
  r.taken = 200;
  r.busy = 1;
  r.coded = 199;
  r.keys = 2;
  r.failed = 1;
  r.written = 198;
  r.refused = 1;
  r.map_steps = 3;
  r.video_bits_per_s = 4.25e6;
  r.depth_bits_per_s = 11.8e6;
  r.send_ms = {0.1, 0.3, 0.2, 0.4};
  r.coded_ms = {5.0, 7.0, 6.0, 9.0};
  r.viewers = 1;
  r.dial_failures = 0;
  return r;
}

TEST(StreamReportLine, CarriesCountsRatesTimesAndTheLink) {
  const std::string line = app::stream_report_line(session_stream());
  EXPECT_EQ(line,
            "st 20.0 s: session | taken 200 busy 1 coded 199 (keys 2) failed 1 "
            "written 198 refused 1 | map steps 3 | video 4.25 depth 11.80 "
            "Mbit/s | send ms p50 0.20 p95 0.40 (4) | coded ms p50 6.00 p95 "
            "9.00 (4) | viewers 1 dial failures 0");
}

TEST(StreamReportLine, SaysDryOffAndTheLastFailure) {
  app::StreamReport dry = session_stream();
  dry.session = false;
  dry.stop = true;
  dry.send_ms.clear();
  dry.coded_ms.clear();
  dry.error = "HevcEncoder::receive: the picture was dropped";
  const std::string line = app::stream_report_line(dry);
  EXPECT_EQ(line.rfind("st stop 20.0 s: dry | taken 200", 0), 0u) << line;
  EXPECT_TRUE(has(line, "send ms - (0) | coded ms - (0)")) << line;
  EXPECT_FALSE(has(line, "viewers")) << line;  // no link in a dry run
  EXPECT_TRUE(has(line,
                  "| last failure: HevcEncoder::receive: the picture was "
                  "dropped"))
      << line;

  app::StreamReport off;
  off.seconds = 10.0;
  // The file's place on the device is never said.
  const std::string path = "/var/mobile/Containers/Data/X/Documents/a.vrst";
  EXPECT_EQ(
      app::without_path(
          "StreamWriter::create: " + path + " exists; open " + path, path),
      "StreamWriter::create: the session file exists; open the session "
      "file");
  EXPECT_EQ(app::without_path("no path here", ""), "no path here");
  off.error = "the camera did not open";
  EXPECT_EQ(app::stream_report_line(off),
            "st 10.0 s: off, the camera did not open");
}

}  // namespace

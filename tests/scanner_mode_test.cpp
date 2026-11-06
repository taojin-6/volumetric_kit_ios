// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file scanner_mode_test.cpp
/// @brief The client is the default and runs no model; `-model` asks for the
///        model, alone and beside every other argument.

#include "ScannerMode.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "CaptureReport.hpp"

namespace {

namespace app = volumetric_kit::ios_app;
using app::CaptureSource;
using app::LaunchOptions;
using app::ModePlan;
using app::ScannerMode;

LaunchOptions parse(std::vector<std::string> arguments) {
  arguments.insert(arguments.begin(), "scanner");
  return app::launch_options(arguments);
}

TEST(ScannerMode, TheDefaultIsTheClient) {
  EXPECT_EQ(parse({}).mode, ScannerMode::Client);
  for (const std::vector<std::string>& arguments :
       {std::vector<std::string>{"-source", "pattern"},
        std::vector<std::string>{"-dry"},
        std::vector<std::string>{"-record", "a.vrst"},
        std::vector<std::string>{"-connect", "10.8.0.1:7300"},
        std::vector<std::string>{"-minutes", "2"}}) {
    const LaunchOptions o = parse(arguments);
    EXPECT_TRUE(o.refusal.empty()) << arguments[0];
    EXPECT_EQ(o.mode, ScannerMode::Client) << arguments[0];
  }
}

TEST(ScannerMode, ModelSelectsTheModelAlone) {
  const LaunchOptions o = parse({"-model"});
  EXPECT_EQ(o.mode, ScannerMode::Model);
  ASSERT_TRUE(o.source.has_value());
  EXPECT_EQ(*o.source, CaptureSource::Camera);
  EXPECT_TRUE(o.refusal.empty());
}

/// It takes no value, so the argument after it is read as its own.
TEST(ScannerMode, ModelGoesWithConnectRecordAndDry) {
  const LaunchOptions connected =
      parse({"-model", "-connect", "10.8.0.1:7300"});
  EXPECT_EQ(connected.mode, ScannerMode::Model);
  EXPECT_EQ(connected.connect, "10.8.0.1:7300");
  EXPECT_TRUE(connected.refusal.empty());

  const LaunchOptions recorded = parse({"-record", "a.vrst", "-model"});
  EXPECT_EQ(recorded.mode, ScannerMode::Model);
  EXPECT_EQ(recorded.record, "a.vrst");
  EXPECT_TRUE(recorded.refusal.empty());

  const LaunchOptions dry = parse({"-source", "pattern", "-model", "-dry"});
  EXPECT_EQ(dry.mode, ScannerMode::Model);
  EXPECT_TRUE(dry.dry);
  ASSERT_TRUE(dry.source.has_value());
  EXPECT_EQ(*dry.source, CaptureSource::Pattern);
  EXPECT_TRUE(dry.refusal.empty());
}

/// `-model` excuses nothing: what was refused without it is refused with it.
TEST(ScannerMode, RefusalsStandBesideModel) {
  for (const std::vector<std::string>& arguments :
       {std::vector<std::string>{"-model", "-source", "patern"},
        std::vector<std::string>{"-dry", "-model", "-record", "a.vrst"},
        std::vector<std::string>{"-model", "-connect", "10.8.0.1:7300", "-dry"},
        std::vector<std::string>{"-model", "-fail-opens", "2"},
        std::vector<std::string>{"-model", "-minutes"},
        std::vector<std::string>{"-record", "-model"}}) {
    const LaunchOptions o = parse(arguments);
    EXPECT_FALSE(o.source.has_value()) << arguments[0];
    EXPECT_FALSE(o.refusal.empty()) << arguments[0];
  }
}

/// The owner's rule for the client: no TSDF grid, no meshing, no texturing, no
/// mesh arenas and no fusion worker, and a camera view to aim with.
TEST(ModePlan, TheClientRunsNoModelAndShowsTheCamera) {
  const ModePlan plan = app::mode_plan(parse({}).mode);
  EXPECT_FALSE(plan.grid);
  EXPECT_FALSE(plan.fusion);
  EXPECT_TRUE(plan.camera_view);
}

TEST(ModePlan, TheModelRunsItAllAndDrawsIt) {
  const ModePlan plan = app::mode_plan(parse({"-model"}).mode);
  EXPECT_TRUE(plan.grid);
  EXPECT_TRUE(plan.fusion);
  EXPECT_FALSE(plan.camera_view);
}

TEST(SectionShown, TheClientKeepsOnlyWhatIsNotAboutFusion) {
  for (const char* kept : {"Alerts", "Memory"}) {
    EXPECT_TRUE(app::section_shown(ScannerMode::Client, kept)) << kept;
    EXPECT_TRUE(app::section_shown(ScannerMode::Model, kept)) << kept;
  }
  for (const char* hidden :
       {"Scene", "Fuse timing", "Extract phases", "Block table", "Dirty"}) {
    EXPECT_FALSE(app::section_shown(ScannerMode::Client, hidden)) << hidden;
    EXPECT_TRUE(app::section_shown(ScannerMode::Model, hidden)) << hidden;
  }
}

TEST(CaptureReportLine, SaysWhichModeRan) {
  app::CaptureReport r;
  r.seconds = 10.0;
  const std::string client = app::capture_report_line(r);
  EXPECT_NE(client.find("| fused 0 skipped 0 |"), std::string::npos) << client;
  EXPECT_NE(client.find("| mode client"), std::string::npos) << client;

  r.mode = ScannerMode::Model;
  const std::string model = app::capture_report_line(r);
  EXPECT_NE(model.find("| mode model"), std::string::npos) << model;
}

}  // namespace

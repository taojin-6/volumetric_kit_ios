// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <gtest/gtest.h>

#include "volumetric_kit/core/config/config.hpp"

namespace config = volumetric_kit::core::config;

TEST(ScannerConfig, BundledScenarioPreservesScannerDefaults) {
  const auto loaded =
      config::load(VI_SCANNER_CONFIG, config::Application::Scanner);
  ASSERT_TRUE(loaded) << loaded.status().message();
  const auto& settings = loaded.value();
  EXPECT_EQ(settings.operation, config::Operation::Fuse);
  EXPECT_EQ(settings.source.driver, "arkit");
  EXPECT_FLOAT_EQ(settings.fusion.voxel_size_m, 0.01F);
  EXPECT_FLOAT_EQ(settings.fusion.truncation_m, 0.03F);
  EXPECT_EQ(settings.fusion.buckets, 1024);
  EXPECT_EQ(settings.fusion.max_buckets, 32768);
  EXPECT_EQ(settings.fusion.fuse_every, 1);
  EXPECT_EQ(settings.fusion.remesh_every, 1);
  EXPECT_FLOAT_EQ(settings.fusion.occlusion_m, 0.02F);
  EXPECT_TRUE(settings.render.texture);
  EXPECT_TRUE(settings.diagnostics.measure_stages);
}

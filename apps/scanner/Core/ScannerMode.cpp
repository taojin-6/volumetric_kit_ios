// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "ScannerMode.hpp"

namespace volumetric_kit::ios_app {

const char* to_string(ScannerMode mode) noexcept {
  switch (mode) {
    case ScannerMode::Client:
      return "client";
    case ScannerMode::Model:
      return "model";
  }
  return "client";
}

ModePlan mode_plan(ScannerMode mode) noexcept {
  ModePlan plan;
  switch (mode) {
    case ScannerMode::Client:
      plan.camera_view = true;
      break;
    case ScannerMode::Model:
      plan.grid = true;
      plan.fusion = true;
      break;
  }
  return plan;
}

bool section_shown(ScannerMode mode, std::string_view title) noexcept {
  return mode == ScannerMode::Model || title == "Alerts" || title == "Memory";
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ThermalState.hpp"

#import <Foundation/Foundation.h>

namespace volumetric_kit::ios_app {

const char* thermal_state() noexcept {
  switch (NSProcessInfo.processInfo.thermalState) {
    case NSProcessInfoThermalStateNominal:
      return "nominal";
    case NSProcessInfoThermalStateFair:
      return "fair";
    case NSProcessInfoThermalStateSerious:
      return "serious";
    case NSProcessInfoThermalStateCritical:
      return "critical";
  }
  return "unknown";
}

}  // namespace volumetric_kit::ios_app

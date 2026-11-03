// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ScannerSelfTest.h"

#import "BridgeStrings.hpp"

#include <string>
#include <vector>

#include "HevcSelfTest.hpp"
#include "SelfTest.hpp"

namespace app = volumetric_kit::ios_app;

static_assert(static_cast<NSInteger>(ScannerSelfTestKindNone) ==
              static_cast<NSInteger>(app::SelfTestKind::None));
static_assert(static_cast<NSInteger>(ScannerSelfTestKindHevc) ==
              static_cast<NSInteger>(app::SelfTestKind::Hevc));
static_assert(static_cast<NSInteger>(ScannerSelfTestKindUnknown) ==
              static_cast<NSInteger>(app::SelfTestKind::Unknown));

namespace {

NSString* thermal_state() {
  switch (NSProcessInfo.processInfo.thermalState) {
    case NSProcessInfoThermalStateNominal:
      return @"nominal";
    case NSProcessInfoThermalStateFair:
      return @"fair";
    case NSProcessInfoThermalStateSerious:
      return @"serious";
    case NSProcessInfoThermalStateCritical:
      return @"critical";
  }
  return @"unknown";
}

}  // namespace

@implementation ScannerSelfTest

+ (ScannerSelfTestKind)kindForArguments:(NSArray<NSString*>*)arguments {
  std::vector<std::string> args;
  args.reserve(arguments.count);
  for (NSString* argument in arguments) {
    const char* text = argument.UTF8String;
    args.emplace_back(text != nullptr ? text : "");
  }
  return static_cast<ScannerSelfTestKind>(app::self_test_kind(args));
}

+ (BOOL)runHevcWithLine:(NS_NOESCAPE void (^)(NSString* line))line {
  line([@"thermal: " stringByAppendingString:thermal_state()]);
  const bool passed = app::run_hevc_self_test(
      [line](const std::string& text) { line(app::to_ns_string(text)); });
  line([@"thermal at the end: " stringByAppendingString:thermal_state()]);
  return passed ? YES : NO;
}

@end

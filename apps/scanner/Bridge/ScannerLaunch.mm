// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ScannerLaunch.h"

#import "BridgeStrings.hpp"

#include <string>
#include <vector>

#include "CaptureReport.hpp"

namespace app = volumetric_kit::ios_app;

@implementation ScannerLaunch

- (instancetype)initWithArguments:(NSArray<NSString*>*)arguments {
  self = [super init];
  if (self == nil) {
    return nil;
  }
  std::vector<std::string> args;
  args.reserve(arguments.count);
  for (NSString* argument in arguments) {
    const char* text = argument.UTF8String;
    args.emplace_back(text != nullptr ? text : "");
  }
  const app::LaunchOptions options = app::launch_options(args);
  if (!options.source) {
    _source = VolumetricCameraSourceNone;
  } else {
    switch (*options.source) {
      case app::CaptureSource::Camera:
        _source = VolumetricCameraSourceCamera;
        break;
      case app::CaptureSource::Pattern:
        _source = VolumetricCameraSourcePattern;
        break;
    }
  }
  _minutes = options.minutes;
  _record = app::to_ns_string(options.record);
  _connect = app::to_ns_string(options.connect);
  _dry = options.dry ? YES : NO;
  _failOpens = options.fail_opens;
  _refusal = app::to_ns_string(options.refusal);
  return self;
}

@end

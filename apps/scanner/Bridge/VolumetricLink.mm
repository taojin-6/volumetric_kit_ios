// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "VolumetricLink.h"

#import "BridgeStrings.hpp"
#import "VolumetricRenderer.h"

#include <stdlib.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "LinkState.hpp"
#include "dial_entry.hpp"
#include "rgbd_stream_writer.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace app = volumetric_kit::ios_app;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

NSErrorDomain const VolumetricLinkErrorDomain = @"VolumetricLinkErrorDomain";

static_assert(static_cast<NSInteger>(VolumetricLinkStateOff) ==
              static_cast<NSInteger>(app::LinkState::Off));
static_assert(static_cast<NSInteger>(VolumetricLinkStateDialing) ==
              static_cast<NSInteger>(app::LinkState::Dialing));
static_assert(static_cast<NSInteger>(VolumetricLinkStateConnected) ==
              static_cast<NSInteger>(app::LinkState::Connected));
static_assert(static_cast<NSInteger>(VolumetricLinkStateRetrying) ==
              static_cast<NSInteger>(app::LinkState::Retrying));
static_assert(static_cast<NSInteger>(VolumetricLinkStateNeedsLocalNetwork) ==
              static_cast<NSInteger>(app::LinkState::NeedsLocalNetwork));

namespace {

// The rate the scanner's stream runs at (`RateGate`), the tracks' rate.
constexpr std::uint32_t kStreamHz = 10;

void set_error(NSError* _Nullable* _Nullable error, const vkc::Status& status) {
  if (error == nullptr) return;
  *error = [NSError
      errorWithDomain:VolumetricLinkErrorDomain
                 code:static_cast<NSInteger>(status.domain())
             userInfo:@{
               NSLocalizedDescriptionKey : app::to_ns_string(status.message())
             }];
}

std::string utf8(NSString* text) {
  const char* bytes = text.UTF8String;
  return bytes != nullptr ? std::string(bytes) : std::string();
}

}  // namespace

@implementation VolumetricLink {
  std::unique_ptr<app::RgbdStreamWriter> _writer;
  app::DialTarget _target;  // the reader named; an empty host names none
}

- (instancetype)init {
  return [super init];
}

+ (uint16_t)defaultPort {
  return volumetric_kit::grid::networking::kDefaultPort;
}

- (BOOL)connectToAddress:(NSString*)address
                    port:(NSString*)port
                renderer:(nullable VolumetricRenderer*)renderer
                   error:(NSError* _Nullable*)error {
  vkc::Result<app::DialTarget> target =
      app::parse_dial_entry(utf8(address), utf8(port));
  if (!target.ok()) {
    set_error(error, target.status());
    return NO;
  }
  if (target.value().host.empty()) {
    [self disconnect];
    return YES;
  }
  if (!_writer) {
    // The camera as it opened, fixed for the renderer's life.
    const auto* info = static_cast<const sensor::SensorInfo*>(
        renderer != nil ? [renderer sensorInfoHandle] : nullptr);
    if (info == nullptr) {
      set_error(error,
                vkc::Status::invalid_argument(
                    renderer == nil
                        ? "the scanner is still starting; connect once it "
                          "shows frames"
                        : "the camera did not open, so there is no session "
                          "to send"));
      return NO;
    }
    app::RgbdStreamOptions options;
    options.dial_host = target.value().host;
    options.dial_port = target.value().port;
    options.fps = kStreamHz;
    options.color = false;
    arc4random_buf(options.session_id.data(), options.session_id.size());
    options.session_id[0] |= 1;  // never all zero
    options.created_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    auto writer = app::RgbdStreamWriter::open(options, *info);
    if (!writer.ok()) {
      set_error(error, writer.status());
      return NO;
    }
    _writer = std::move(writer).value();
  } else {
    const vkc::Status moved =
        _writer->dial_to(target.value().host, target.value().port);
    if (!moved.ok()) {
      set_error(error, moved);
      return NO;
    }
  }
  _target = std::move(target).value();
  return YES;
}

- (void)disconnect {
  if (_writer) (void)_writer->dial_to("", 0);
  _target = app::DialTarget{};
}

- (NSString*)target {
  if (_target.host.empty()) return @"";
  return app::to_ns_string(_target.host + ":" + std::to_string(_target.port));
}

- (VolumetricLinkStatus)status {
  const app::RgbdStreamStats s =
      _writer ? _writer->stats() : app::RgbdStreamStats{};
  VolumetricLinkStatus out{};
  out.state = static_cast<VolumetricLinkState>(app::link_state(
      !_target.host.empty(), s.viewers, s.dial_failures, s.no_route));
  out.dial_failures = s.dial_failures;
  out.key_wanted = s.key_wanted;
  return out;
}

@end

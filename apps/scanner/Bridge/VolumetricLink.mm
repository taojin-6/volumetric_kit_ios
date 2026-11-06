// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "VolumetricLink.h"

#import "BridgeStrings.hpp"
#import "VolumetricRenderer.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "LinkState.hpp"
#include "ReaderLine.hpp"
#include "dial_entry.hpp"
#include "rgbd_stream_sender.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace app = volumetric_kit::ios_app;
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
  // Whose stream the link drives: weak, as the session is the renderer's.
  __weak VolumetricRenderer* _renderer;
  app::DialTarget _target;  // the reader named; an empty host names none
}

// @p renderer's stream sender; null without either. Valid while the caller
// holds @p renderer strongly.
static app::RgbdStreamSender* sender_of(
    VolumetricRenderer* _Nullable renderer) {
  return renderer != nil
             ? static_cast<app::RgbdStreamSender*>([renderer streamHandle])
             : nullptr;
}

// What the screen says of @p renderer's reader now; none with no stream or no
// note. Valid as sender_of is.
static std::optional<app::ReaderLine> reader_line_of(
    VolumetricRenderer* _Nullable renderer) {
  const app::RgbdStreamSender* sender = sender_of(renderer);
  if (sender == nullptr) return std::nullopt;
  auto note = sender->reader_note();
  if (!note) return std::nullopt;
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - note->received);
  return app::reader_line(std::move(note->line),
                          app::note_tone(static_cast<std::uint8_t>(note->tone)),
                          age);
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
  if (renderer == nil) {
    set_error(error, vkc::Status::invalid_argument(
                         "the scanner is still starting; connect once it "
                         "shows frames"));
    return NO;
  }
  app::RgbdStreamSender* sender = sender_of(renderer);
  if (sender == nullptr) {
    set_error(error,
              vkc::Status::invalid_argument("there is no stream to send: " +
                                            utf8(renderer.streamError)));
    return NO;
  }
  // The session opens here if no file opened it, and dials from now on.
  const vkc::Status dialled =
      sender->dial_to(target.value().host, target.value().port);
  if (!dialled.ok()) {
    set_error(error, dialled);
    return NO;
  }
  _renderer = renderer;
  _target = std::move(target).value();
  return YES;
}

- (void)disconnect {
  VolumetricRenderer* renderer = _renderer;  // held through the call
  if (app::RgbdStreamSender* sender = sender_of(renderer)) {
    (void)sender->dial_to("", 0);
  }
  _target = app::DialTarget{};
}

- (NSString*)target {
  if (_target.host.empty()) return @"";
  return app::to_ns_string(_target.host + ":" + std::to_string(_target.port));
}

- (VolumetricLinkStatus)status {
  VolumetricRenderer* renderer = _renderer;  // held through the call
  app::RgbdStreamSender* sender = sender_of(renderer);
  const app::RgbdStreamStats s =
      sender != nullptr ? sender->stats().session : app::RgbdStreamStats{};
  VolumetricLinkStatus out{};
  // A target with no stream left to dial it is off, not dialing.
  out.state = static_cast<VolumetricLinkState>(
      app::link_state(!_target.host.empty() && sender != nullptr, s.viewers,
                      s.dial_failures, s.no_route));
  out.dial_failures = s.dial_failures;
  out.key_wanted = s.key_wanted;
  out.frames = s.frames;
  return out;
}

- (nullable NSString*)readerLine {
  VolumetricRenderer* renderer = _renderer;  // held through the call
  const std::optional<app::ReaderLine> line = reader_line_of(renderer);
  return line ? app::to_ns_string(line->text) : nil;
}

- (VolumetricStatTone)readerTone {
  VolumetricRenderer* renderer = _renderer;  // held through the call
  const std::optional<app::ReaderLine> line = reader_line_of(renderer);
  // Value for value: Readout.mm checks.
  return line ? static_cast<VolumetricStatTone>(line->tone)
              : VolumetricStatToneNeutral;
}

@end

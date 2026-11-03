// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file VolumetricLink.h
/// @brief The scanner's live link to a reader, as the on-screen entry drives
///        it: connect to the address and port typed, disconnect, and what the
///        link is doing now.
///
/// The phone dials and listens on nothing: the session writer
/// (`Session/rgbd_stream_writer.hpp`) serves its session to the reader it
/// dials, and dials again while none is served. The session is opened at the
/// first connect and kept until the link goes, so a reconnect, here or to
/// another address, is the same session.
///
/// **It sends the session alone, with no frames,** until the scanner reads
/// ARKit through sensor's `IRgbdSensor`, whose `SensorInfo` then opens it.
/// Until then it describes the sensor from the capture's last converted
/// frame: ARKit's depth camera, tracked, on the host's clock, depth alone.
///
/// @warning Main thread only. Releasing the link ends its session there,
///          which waits for the dial under way and the reader's last records
///          (up to about 3 s); the scanner's view controller keeps it for
///          the app's life, so that wait does not arise today.

#import <Foundation/Foundation.h>

#import "ARKitCapture.h"

NS_ASSUME_NONNULL_BEGIN

/// The `NSError` domain of a refused connect; the code is the library
/// status's domain, the localized description its message.
FOUNDATION_EXPORT NSErrorDomain const VolumetricLinkErrorDomain;

/// @brief What the link shows: `Core/LinkState.hpp`'s `LinkState`, value for
///        value (VolumetricLink.mm checks).
typedef NS_ENUM(NSInteger, VolumetricLinkState) {
  /// No reader named: nothing is dialled.
  VolumetricLinkStateOff = 0,
  /// Named, not connected, no dial failed yet.
  VolumetricLinkStateDialing,
  /// A reader is being served.
  VolumetricLinkStateConnected,
  /// Dials failed, or readers reached left at once; one more each second.
  VolumetricLinkStateRetrying,
  /// The last dial found no route to the host: iOS's answer until the app is
  /// allowed Local Network access, and a router's for a host it cannot
  /// reach. Dialling goes on.
  VolumetricLinkStateNeedsLocalNetwork,
};

/// @brief The link now, as plain values.
typedef struct {
  VolumetricLinkState state;
  /// Dials failed in a row, since the address was last set or a reader last
  /// stayed connected a second.
  uint64_t dial_failures;
  /// Readers that joined or fell back and so asked for a key.
  uint64_t key_wanted;
} VolumetricLinkStatus;

@interface VolumetricLink : NSObject

- (instancetype)init NS_DESIGNATED_INITIALIZER;

/// The port a reader listens on when the entry names none.
@property(class, nonatomic, readonly) uint16_t defaultPort;

/// @brief Dial the reader the entry names, from now on: the address field
///        (a numeric IPv4 address, or one with ":port") and the port field
///        (empty for @ref defaultPort). An empty address disconnects.
/// @param capture  The capture whose depth camera the session describes, at
///                 the first connect.
/// @return `YES`; `NO` with @p error for an entry the core's checks refuse,
///         its description the core's sentence for the screen, a capture
///         with no converted frame yet at the first connect, or a session
///         that could not start. The last target is kept on a refusal.
- (BOOL)connectToAddress:(NSString*)address
                    port:(NSString*)port
                 capture:(VolumetricCapture*)capture
                   error:(NSError* _Nullable*)error;

/// @brief Stop dialling, and let a connected reader go.
- (void)disconnect;

/// "a.b.c.d:port" of the reader named, or empty when none is.
@property(nonatomic, readonly, copy) NSString* target;

/// The link now.
@property(nonatomic, readonly) VolumetricLinkStatus status;

@end

NS_ASSUME_NONNULL_END

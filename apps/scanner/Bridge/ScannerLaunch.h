// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file ScannerLaunch.h
/// @brief What a scanner launch's arguments ask of the camera and its
///        stream: `-source camera|pattern`, `-minutes N`, `-record <name>`
///        and `-connect <address>` (`Core/CaptureReport.hpp`'s
///        `launch_options`).

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// @brief Where the scanner's frames come from.
typedef NS_ENUM(NSInteger, VolumetricCameraSource) {
  /// ARKit's camera, the default.
  VolumetricCameraSourceCamera = 0,
  /// sensor's test pattern (`ArkitSource::Pattern`): no camera opens.
  VolumetricCameraSourcePattern,
  /// The launch's arguments were refused: no camera opens, and
  /// `ScannerLaunch.refusal` says why.
  VolumetricCameraSourceNone,
};

@interface ScannerLaunch : NSObject

/// @param arguments  The process's, the program first.
- (instancetype)initWithArguments:(NSArray<NSString*>*)arguments
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;

/// The source the arguments name.
@property(nonatomic, readonly) VolumetricCameraSource source;
/// Minutes the scanner runs before it stops and exits; 0 runs on.
@property(nonatomic, readonly) uint32_t minutes;
/// The session file the stream writes, a plain name in the app's Documents;
/// empty for none.
@property(nonatomic, readonly, copy) NSString* record;
/// The reader the stream dials once the scanner is up, as the on-screen
/// address field takes it ("a.b.c.d:port"); empty for none.
@property(nonatomic, readonly, copy) NSString* connect;
/// Why an argument was refused; empty when none was.
@property(nonatomic, readonly, copy) NSString* refusal;

@end

NS_ASSUME_NONNULL_END

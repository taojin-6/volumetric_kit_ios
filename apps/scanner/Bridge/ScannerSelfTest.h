// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file ScannerSelfTest.h
/// @brief The scanner's launch self-tests (`-self-test <name>`), which run
///        instead of the scanner and never open the camera.

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/// @brief What a launch's arguments ask for: `Core/SelfTest.hpp`'s
///        `SelfTestKind`, value for value (ScannerSelfTest.mm checks).
typedef NS_ENUM(NSInteger, ScannerSelfTestKind) {
  ScannerSelfTestKindNone = 0,
  ScannerSelfTestKindHevc,
  ScannerSelfTestKindUnknown,
};

@interface ScannerSelfTest : NSObject

/// @return What @p arguments (the process's, the program first) ask for.
+ (ScannerSelfTestKind)kindForArguments:(NSArray<NSString*>*)arguments;

/// @brief Run `-self-test hevc` (`HevcSelfTest.hpp`) on the calling thread,
///        which must not be the main one, with the device's thermal state.
/// @param line  Handed each line of the report as it is made, on the calling
///              thread.
/// @return Whether it passed.
+ (BOOL)runHevcWithLine:(NS_NOESCAPE void (^)(NSString* line))line;

- (instancetype)init NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file BridgeStrings.hpp
/// @brief Turning C++ strings into ones Swift can hold.
///
/// One function, in its own header because three unrelated units need it -- the
/// error translation, the read-out's value types, and the read-out itself --
/// and none of them is the natural owner of the other two.

#import <Foundation/Foundation.h>

#include <string>

namespace volumetric_kit::ios_app {

/// @brief @p text as an `NSString`, never nil.
///
/// Never nil, so a `nonnull` property cannot hand Swift a null it traps on:
/// `stringWithUTF8String:` returns nil for invalid UTF-8, and Vulkan promises
/// only that VkPhysicalDeviceProperties::deviceName is a NUL-terminated
/// char[256] -- a driver may put any bytes in it, and Swift imports the
/// property as a non-optional String.
inline NSString* to_ns_string(const std::string& text) {
  if (NSString* utf8 = [NSString stringWithUTF8String:text.c_str()]) {
    return utf8;
  }
  // Latin-1 maps every byte to a code point, so this cannot fail in turn.
  NSString* latin1 = [[NSString alloc] initWithBytes:text.data()
                                              length:text.size()
                                            encoding:NSISOLatin1StringEncoding];
  return latin1 != nil ? latin1 : @"(unprintable)";
}

}  // namespace volumetric_kit::ios_app

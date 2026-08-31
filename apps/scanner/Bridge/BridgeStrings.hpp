// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file BridgeStrings.hpp
/// @brief Turning C++ strings into ones Swift can hold.
///
/// One function, in its own header because two unrelated units need it -- the
/// error translation and the read-out -- and neither is the natural owner of
/// the other.

#import <Foundation/Foundation.h>

#include <string>

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

/// @brief @p text as an `NSString`, never nil.
///
/// Never nil, so a `nonnull` property cannot hand Swift a null it traps on:
/// `stringWithUTF8String:` returns nil for invalid UTF-8, and Vulkan promises
/// only that VkPhysicalDeviceProperties::deviceName is a NUL-terminated
/// char[256] -- a driver may put any bytes in it, and Swift imports the
/// property as a non-optional String.
///
/// Declared here and defined once in BridgeStrings.mm rather than `inline`.
/// An inline definition on a PUBLIC include path is emitted as a weak external
/// in every includer, and this signature is ARC-invariant -- unlike
/// `set_error`, whose `NSError**` picks up an `__autoreleasing` qualifier that
/// *is* mangled, so a mismatch there is a link error rather than a silent one.
/// An ARC body and a manual-retain body would collide under one symbol with
/// nothing diagnosed, and the linker would keep whichever it saw first.
///
/// The nullability region is the other half of the guarantee: the never-nil
/// promise above is prose the compiler cannot check outside one, and Swift
/// imports an unannotated `NSString*` as an implicitly-unwrapped optional
/// rather than trapping at the boundary.
NSString* to_ns_string(const std::string& text);

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

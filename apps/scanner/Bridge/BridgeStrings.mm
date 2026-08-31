// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "BridgeStrings.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

NSString* to_ns_string(const std::string& text) {
  if (NSString* utf8 = [NSString stringWithUTF8String:text.c_str()]) {
    return utf8;
  }
  // Latin-1 maps every byte to a code point, so this cannot fail in turn.
  //
  // ARC balances the +1 this returns. Under manual retain/release it leaked
  // once per conversion, while the `stringWithUTF8String:` path above is
  // autoreleased and did not -- two branches of one function with different
  // ownership, on the path reached only by a driver or a library message this
  // app does not choose the bytes of. That is invisible on a well-behaved
  // device, which is why the target's ARC setting is asserted in the build
  // file rather than assumed here.
  NSString* latin1 = [[NSString alloc] initWithBytes:text.data()
                                              length:text.size()
                                            encoding:NSISOLatin1StringEncoding];
  return latin1 != nil ? latin1 : @"(unprintable)";
}

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

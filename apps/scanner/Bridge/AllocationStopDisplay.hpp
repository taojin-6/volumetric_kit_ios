// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file AllocationStopDisplay.hpp
/// @brief The one rendering of an @ref AllocationStop that needs Objective-C.
///
/// The other three -- the log's suffix, the trace's column and the panel's
/// headline and advice -- are plain C++ over a four-value enum and live in
/// Core/AllocationStop.hpp, where a host test can pin them. This is the half
/// that genuinely needs the bridge, and it is here for that reason rather than
/// because it started next to the others.
///
/// Splitting them does not weaken the guarantee they were collected for: both
/// files switch exhaustively over the same enum, and both are compiled with
/// `-Werror=switch`, so a cause added upstream stops the compile on every
/// rendering rather than reporting a stopped scan as a healthy one.

#import "VolumetricRenderer.h"

#include "AllocationStop.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

/// @brief The cause as the Swift-facing enum.
///
/// Switched rather than cast, though the two enumerations are declared in the
/// same order: a cast makes that order load-bearing across two files that no
/// build step compares, and the failure is a sample reporting the
/// *neighbouring* cause -- a wrong answer that looks exactly like a right one.
/// This way a value added on one side stops the compile.
VolumetricAllocationStop allocation_stop_value(AllocationStop stop) noexcept;

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

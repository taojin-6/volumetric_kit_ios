// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file ThermalState.hpp
/// @brief The device's thermal state, for the read-out and the console.
///
/// Plain C++ so the `.cpp` units can call it; the query is Objective-C, in
/// ThermalState.mm.

namespace volumetric_kit::ios_app {

/// @return `NSProcessInfo.thermalState` named: "nominal", "fair", "serious"
///         or "critical", or "unknown" for a state newer than these. Any
///         thread.
const char* thermal_state() noexcept;

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file ScannerMode.hpp
/// @brief The scanner's two ways to run, and what each one runs.
///
/// Pure, so the default and what the client leaves out are host tested. A
/// launch's `-model` (`launch_options`) or the panel's switch chooses the mode;
/// the bridge builds, runs and draws what @ref mode_plan says. The camera and
/// its stream run in both, the same.

#include <cstdint>
#include <string_view>

namespace volumetric_kit::ios_app {

/// @brief How the scanner runs.
enum class ScannerMode : std::uint8_t {
  /// The default: a client of the server's scene. The camera's frames go to
  /// the stream and its picture to the screen; nothing is fused or meshed.
  Client,
  /// The stream as above, and on the device a TSDF grid, its mesh and the
  /// textured model on screen.
  Model,
};

/// @return "client" or "model".
const char* to_string(ScannerMode mode) noexcept;

/// @brief What a mode runs beside the camera and its stream.
struct ModePlan {
  /// Fusion's state exists: the TSDF grid, meshing and texturing, and their
  /// mesh arenas (`Fusion::start`).
  bool grid = false;
  /// A worker fuses the camera's newest frame (`FrameFanout`).
  bool fusion = false;
  /// The camera's picture fills the screen, instead of the model.
  bool camera_view = false;
};

/// @return What @p mode runs.
ModePlan mode_plan(ScannerMode mode) noexcept;

/// @return Whether @p mode shows the read-out section named @p title
///         (`stat_sections`): every section in the model; in the client only
///         those not about fusion, `Alerts` and `Memory`.
bool section_shown(ScannerMode mode, std::string_view title) noexcept;

}  // namespace volumetric_kit::ios_app

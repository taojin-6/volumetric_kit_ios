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

#include <chrono>
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
  /// With no frame to show, the view asks for the picture of one the camera
  /// withheld (`ArkitSensor::poll_withheld_picture`). A camera never asked
  /// holds none for it.
  bool withheld_pictures = false;
};

/// @return What @p mode runs.
ModePlan mode_plan(ScannerMode mode) noexcept;

/// How long the fuse loop waits for a frame before it looks at its stop flag
/// again: a stop is seen within this, as is a frame the signal missed.
inline constexpr std::chrono::milliseconds kIdleWait{50};

/// The wait after a withheld picture came. No signal fires for one, so this
/// sets their rate: about 30 a second.
inline constexpr std::chrono::milliseconds kWithheldPictureWait{33};

/// @brief The fuse loop's turn when a poll gave no frame: @p ask, if @p plan
///        says, and @p offer what comes.
///
/// Only a picture shortens the wait. Normal tracking gives none and a stopped
/// camera neither, so each waits as it always did.
/// @tparam Ask    `std::optional<Picture>()`.
/// @tparam Offer  `void(const Picture&)`.
/// @return How long to wait for the next frame.
template <typename Ask, typename Offer>
std::chrono::milliseconds no_frame_turn(const ModePlan& plan, Ask&& ask,
                                        Offer&& offer) {
  if (!plan.withheld_pictures) return kIdleWait;
  const auto picture = ask();
  if (!picture) return kIdleWait;
  offer(*picture);
  return kWithheldPictureWait;
}

/// @return Whether @p mode shows the read-out section named @p title
///         (`stat_sections`): every section in the model; in the client only
///         those not about fusion, `Alerts` and `Memory`.
bool section_shown(ScannerMode mode, std::string_view title) noexcept;

}  // namespace volumetric_kit::ios_app

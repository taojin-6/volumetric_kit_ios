// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CameraOpen.hpp
/// @brief When the scanner opens its camera: at bring-up, and again after an
///        open that found no camera in time, never two at once.
///
/// sensor's `ArkitSensor::open` comes back `NotFound` when no frame with depth
/// arrived in time: the iPad was locked or the app sent to the background
/// during the open, or the camera was in use elsewhere (Split View, Stage
/// Manager). A later open of the same options may find it, so the scanner
/// opens again:
/// - at a wake, at once: bring-up, the app in front or active again (a
///   foreground, an unlock) and `-startFusion`;
/// - at the read-out's tick while the scan runs, once @ref
///   CameraOpenPolicy::kFirstBackoffNs has passed since the last open ended,
///   doubling with each open that failed, to @ref
///   CameraOpenPolicy::kMaxBackoffNs. A camera let go elsewhere wakes
///   nothing, so this is what finds it again, within about half a minute;
/// - never while an open is in flight. An open waits without bound for the
///   user's answer to the camera prompt, so none starts beside it.
///
/// Any other failure is final, since it comes back for the same options on
/// the same device: `IoError` (the camera not allowed, ARKit's session
/// failed, a first frame of the wrong size), `Unsupported` (no LiDAR) and
/// `InvalidArgument`. The read-out says why and nothing opens again. Once
/// open, the camera is the renderer's for its life, and no open follows.
///
/// Pure, so the rule is host tested; the bridge (Bridge/VolumetricRenderer.mm)
/// runs the opens and says how each ended. The Objective-C
/// `VolumetricCameraOpenState` mirrors @ref CameraOpenState value for value,
/// and VolumetricRenderer.mm static_asserts the two agree.
///
/// @code
/// app::CameraOpenPolicy policy;
/// if (policy.begin(app::CameraOpenCause::Wake, now_ns())) {
///   // ... off the main thread: ArkitSensor::open(options), then on it:
///   policy.end(opened ? app::CameraOpenEnd::Opened
///              : code == NotFound ? app::CameraOpenEnd::NotFound
///                                 : app::CameraOpenEnd::Refused,
///              message, now_ns());
/// }
/// // ... each read-out tick while the camera is not open:
/// if (policy.begin(app::CameraOpenCause::Tick, now_ns())) { /* open */ }
/// @endcode
///
/// @warning Not thread-safe: call it from one thread.

#include <cstdint>
#include <string>

namespace volumetric_kit::ios_app {

/// @brief What the camera's open is doing.
enum class CameraOpenState : std::uint8_t {
  /// No open yet.
  Idle = 0,
  /// An open is in flight: no other begins.
  Opening,
  /// The camera opened: no open follows.
  Open,
  /// The last open found no camera in time: a wake opens again at once, a
  /// tick once the backoff has passed.
  Retrying,
  /// An open was refused, or the launch named no source: no open follows,
  /// and @ref CameraOpenPolicy::error says why.
  Failed,
};

/// @brief How an open ended.
enum class CameraOpenEnd : std::uint8_t {
  /// The camera is open.
  Opened,
  /// `Status::Code::NotFound`: no frame with depth in time. Open again.
  NotFound,
  /// Any other failure: final.
  Refused,
};

/// @brief What asks for an open.
enum class CameraOpenCause : std::uint8_t {
  /// Bring-up, the app in front or active again, `-startFusion`: at once.
  Wake,
  /// The read-out's tick while the scan runs: once the backoff has passed.
  Tick,
};

/// @brief Which open may begin now, and what the opens so far came to.
class CameraOpenPolicy {
 public:
  /// The backoff after the first open that failed, ns.
  static constexpr std::uint64_t kFirstBackoffNs = 2'000'000'000;
  /// The longest backoff, ns: each failed open doubles it to this.
  static constexpr std::uint64_t kMaxBackoffNs = 30'000'000'000;

  /// @brief Whether to open the camera now.
  /// @param cause   What asks.
  /// @param now_ns  Now, on one steady clock.
  /// @return true when @ref state is Idle, or Retrying and @p cause a wake or
  ///         a tick at or after @ref retry_at_ns: the state is then Opening,
  ///         and the caller opens and says how it ended through @ref end.
  ///         false otherwise, the state as it was: never while Opening.
  bool begin(CameraOpenCause cause, std::uint64_t now_ns) noexcept;

  /// @brief The open begun last ended. Nothing unless Opening.
  /// @param end     How.
  /// @param error   Why it failed: the open's message; ignored when opened.
  /// @param now_ns  Now, on @ref begin's clock: the backoff counts from it.
  void end(CameraOpenEnd end, std::string error, std::uint64_t now_ns);

  /// @brief No camera is to open, as when the launch's arguments were
  ///        refused: Failed, @ref error saying @p why. Nothing once open.
  void refuse(std::string why);

  /// @return What the open is doing.
  CameraOpenState state() const noexcept { return state_; }
  /// @return Opens begun, the first included.
  std::uint32_t opens() const noexcept { return opens_; }
  /// @return Opens begun after the first.
  std::uint32_t retries() const noexcept { return opens_ > 0 ? opens_ - 1 : 0; }
  /// @return The last failed open's message; empty before one, and once open.
  const std::string& error() const noexcept { return error_; }
  /// @return When a tick opens again, on @ref begin's clock; 0 unless
  ///         Retrying.
  std::uint64_t retry_at_ns() const noexcept {
    return state_ == CameraOpenState::Retrying ? retry_at_ns_ : 0;
  }

 private:
  CameraOpenState state_ = CameraOpenState::Idle;
  std::uint32_t opens_ = 0;
  std::uint32_t failed_ = 0;  // opens in a row that found no camera
  std::uint64_t retry_at_ns_ = 0;
  std::string error_;
};

/// @return The read-out's words for @p policy at @p now_ns: "not opened yet";
///         "opening"; "opening again (retry 2) after: <error>"; "<error>;
///         retry 3 when the app is in front again, or in 8 s"; "<error>" when
///         refused; "open", or "open after 2 retries".
std::string camera_open_text(const CameraOpenPolicy& policy,
                             std::uint64_t now_ns);

}  // namespace volumetric_kit::ios_app

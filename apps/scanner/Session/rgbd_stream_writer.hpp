// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rgbd_stream_writer.hpp
/// @brief Any RGB-D sensor's frames as one session: a `.vrst` file.
///
/// The session holds one sensor. Track 0 is its colour (`RgbdColor`) and
/// track 1 its depth (`RgbdDepth`); without colour, track 0 is its depth.
/// Both tracks name the sensor's id, its clock and the rate frames are
/// appended at. The depth track's params say what the sensor is
/// (`sensor::wire::encode_sensor`). The clock is the session's 0 for a sensor
/// on the host's clock (`ClockDomain::Host`), and 1 for one on its own
/// (`ClockDomain::Device`), shared by both tracks.
///
/// A frame is written whole or not at all: its colour record, then its depth
/// record, both at its capture time and in its set (its count among the
/// frames written, from 0). Each track's seq is one up per record written.
/// Both records are checked before either is written. A frame the caller
/// does not append, such as one its rate gate did not take or whose picture
/// the encoder dropped, leaves no gap.
///
/// The records are the frame's wire form (`sensor/wire/rgbd_wire.hpp`): the
/// depth record's prefix, then the frame's own rows and confidence plane,
/// written from where the frame holds them, not copied; the colour record's
/// prefix, then the caller's coded picture as given. The writer decodes and
/// encodes nothing.
///
/// @code
/// namespace app = volumetric_kit::ios_app;
/// app::RgbdStreamOptions out;
/// out.path = path;
/// out.fps = 10;
/// out.session_id = new_session_id();
/// VKC_ASSIGN(auto writer, app::RgbdStreamWriter::open(out, sensor.info()));
/// // ... each frame taken, with its picture as the encoder coded it:
/// const app::CodedColor color{au.data(), au.size(), key, facts.color};
/// VKC_TRY(writer->append(frame, &color));  // the colour, then the depth
/// // ...
/// VKC_TRY(writer->finish());
/// @endcode
///
/// @warning Not thread-safe: open, append, finish and stats from one thread.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"

namespace volumetric_kit::ios_app {

/// @brief A frame's colour as the caller coded it.
struct CodedColor {
  /// One HEVC access unit, Annex B, a key's parameter sets first. Borrowed
  /// for the call.
  const std::uint8_t* data = nullptr;
  std::size_t size = 0;  ///< Its bytes; above 0.
  bool key = false;      ///< An IRAP picture.
  /// The colour its stream states (`HevcEncoder::Facts::color`).
  sensor::ColorCodes codes;
};

/// @brief Where and how a @ref RgbdStreamWriter writes.
struct RgbdStreamOptions {
  std::string path;  ///< A new session file; empty writes none.
  /// Frames a second the caller appends, the tracks' rate; 0 for the
  /// sensor's own (`SensorInfo::fps`).
  std::uint32_t fps = 0;
  /// Write the pictures (`RgbdColor`), for a sensor with a colour camera; false
  /// writes the depth alone.
  bool color = true;
  /// The session's id, never all zero: a tracked sensor's registration
  /// names it.
  std::array<std::uint8_t, 16> session_id{};
  std::uint64_t created_ns = 0;    ///< `Session::created_ns`.
  std::string writer = "scanner";  ///< `Session::writer`.
  std::string calibration;         ///< `Session::calibration`, or empty.
};

/// @brief What a @ref RgbdStreamWriter has written since it opened.
struct RgbdStreamStats {
  std::uint64_t frames = 0;       ///< Frames written whole.
  std::uint64_t keys = 0;         ///< Of them, with a key picture.
  std::uint64_t color_bytes = 0;  ///< Colour records' payloads.
  std::uint64_t depth_bytes = 0;  ///< Depth records' payloads.
};

/// @brief Writes one sensor's frames as a session (above).
class RgbdStreamWriter {
 public:
  /// @brief Write the session header.
  /// @param info  The sensor's, from when it opened.
  /// @return The writer; `Status::Code::InvalidArgument` for no path or an
  ///         all-zero session id, with no file made; a sensor
  ///         `sensor::wire::encode_sensor` refuses, as its code;
  ///         `StreamWriter::create`'s errors otherwise.
  static core::Result<std::unique_ptr<RgbdStreamWriter>> open(
      const RgbdStreamOptions& options, const sensor::SensorInfo& info);

  /// Finishes, as @ref finish.
  ~RgbdStreamWriter();
  RgbdStreamWriter(const RgbdStreamWriter&) = delete;
  RgbdStreamWriter& operator=(const RgbdStreamWriter&) = delete;

  /// @brief Write one frame whole: its colour record (from @p color), then
  ///        its depth record.
  /// @param frame  The frame, as its sensor handed it out, its pose final.
  /// @param color  Its coded picture; null in a session without colour.
  /// @return OK; with nothing written, `sensor::wire::encode_depth`'s or
  ///         `encode_color`'s refusal of the frame, as its code (depth on
  ///         the device only is `Unsupported`), and
  ///         `Status::Code::InvalidArgument` for a null @p color in a
  ///         session with colour or one in a session without, an empty
  ///         picture, the first picture not a key, no capture time (0) or
  ///         one not after the last frame's, or a finished writer; the
  ///         file's write error, or
  ///         a failure to append the depth record after the colour record,
  ///         after which the file may end inside the frame and later appends
  ///         fail.
  core::Status append(const sensor::RgbdFrame& frame, const CodedColor* color);

  /// @brief End the session: the file's footer. Idempotent.
  /// @return OK; the footer's write error, or the file's earlier one; a
  ///         later call returns the first's. `Status::Code::OutOfMemory`
  ///         leaves the session open, and a later call tries again.
  core::Status finish();

  /// @return The counts so far.
  RgbdStreamStats stats() const noexcept;

 private:
  struct Impl;
  explicit RgbdStreamWriter(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::ios_app

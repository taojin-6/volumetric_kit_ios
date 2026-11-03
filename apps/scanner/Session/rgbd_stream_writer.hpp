// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rgbd_stream_writer.hpp
/// @brief Any RGB-D sensor's frames as one session: a `.vrst` file, a live
///        link to a reader that listens, or both, with the same records.
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
/// **Live, the writer dials and listens on nothing.** Grid's `StreamServer`
/// serves the session with no listener (`ServerOptions::listen` false), so no
/// host that reaches the phone can connect to it. A thread of the writer's
/// dials the reader (`core::net::dial`) and hands the socket to the server
/// (`StreamServer::add_viewer`); while no reader is served it dials again,
/// once a second after a failed dial. One reader at a time. A reader that
/// joins waits for a key, and asks for one (@ref key_wanted); one that falls
/// more than @ref RgbdStreamOptions::max_gop_bytes behind loses whole frames
/// up to the next key, which the file keeps. The on-screen connect and
/// disconnect move the target (@ref dial_to) while the session and the file
/// go on, so the reader's next connection is the same session, which it
/// accepts. The link serves a frame once the file holds it whole.
///
/// @code
/// namespace app = volumetric_kit::ios_app;
/// app::RgbdStreamOptions out;
/// out.path = path;
/// out.dial_host = "10.8.0.1";  // the address entered on screen
/// out.dial_port = 7101;        // later connects and disconnects: dial_to
/// out.fps = 10;
/// out.session_id = new_session_id();
/// VKC_ASSIGN(auto writer, app::RgbdStreamWriter::open(out, sensor.info()));
/// // ... each frame taken, with its picture as the encoder coded it:
/// const bool key = encoder_owes_key || writer->key_wanted();
/// const app::CodedColor color{au.data(), au.size(), key, facts.color};
/// VKC_TRY(writer->append(frame, &color));  // the colour, then the depth
/// // ...
/// VKC_TRY(writer->finish());
/// @endcode
///
/// @warning Not thread-safe: open, append and finish from one thread;
///          @ref dial_to, @ref key_wanted and @ref stats from any.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/video/color_codes.hpp"

namespace volumetric_kit::ios_app {

/// The longest one dial waits for the reader to answer.
inline constexpr std::chrono::milliseconds kDialLimit{2'000};
/// How long the link waits after a failed dial before the next.
inline constexpr std::chrono::milliseconds kRedialWait{1'000};

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
  /// The reader to dial, a numeric IPv4 address; empty dials none.
  std::string dial_host;
  std::uint16_t dial_port = grid::networking::kDefaultPort;  ///< Its port.
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
  /// How far behind the newest record a live reader may fall before it waits
  /// for the next key (`ServerOptions::max_gop_bytes`).
  std::uint64_t max_gop_bytes = std::uint64_t{4} << 20;
};

/// @brief What a @ref RgbdStreamWriter has written since it opened.
struct RgbdStreamStats {
  std::uint64_t frames = 0;       ///< Frames written whole.
  std::uint64_t keys = 0;         ///< Of them, with a key picture.
  std::uint64_t color_bytes = 0;  ///< Colour records' payloads.
  std::uint64_t depth_bytes = 0;  ///< Depth records' payloads.
  /// Live readers that joined or fell back to waiting for a key
  /// (`StreamServer::key_wanted`): what the caller's rate rule reads. Never
  /// falls.
  std::uint64_t key_wanted = 0;
  std::size_t viewers = 0;  ///< Live readers connected now: 0 or 1.
  /// Dials that failed since the target was last set (@ref
  /// RgbdStreamWriter::dial_to): the screen's "retrying" while above 0 and no
  /// reader is connected.
  std::uint64_t dial_failures = 0;
  /// The last dial failed with no route to the host (`is_no_route`,
  /// dial_entry.hpp): on iOS, the app's Local Network access not granted
  /// yet. Cleared by a dial that connects and by @ref
  /// RgbdStreamWriter::dial_to.
  bool no_route = false;
};

/// @brief Writes one sensor's frames as a session (above).
class RgbdStreamWriter {
 public:
  /// @brief Write the session header, and start dialling with a dial host.
  /// @param info  The sensor's, from when it opened.
  /// @return The writer; with no file made, `Status::Code::InvalidArgument`
  ///         for neither a path nor a dial host, an all-zero session id, or a
  ///         dial host that is not one host's numeric IPv4 address or a dial
  ///         port of 0 (`core::net::parse_endpoint`'s reason); a sensor
  ///         `sensor::wire::encode_sensor` refuses, as its code;
  ///         `StreamWriter::create`'s and `StreamServer::start`'s errors
  ///         otherwise, and `Status::Code::IoError` for a dial thread that
  ///         cannot start.
  static core::Result<std::unique_ptr<RgbdStreamWriter>> open(
      const RgbdStreamOptions& options, const sensor::SensorInfo& info);

  /// Finishes, as @ref finish, and stops the link however that went.
  ~RgbdStreamWriter();
  RgbdStreamWriter(const RgbdStreamWriter&) = delete;
  RgbdStreamWriter& operator=(const RgbdStreamWriter&) = delete;

  /// @brief Write one frame whole: its colour record (from @p color), then
  ///        its depth record; then serve both to a live reader.
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
  ///         fail. A live reader's failure is never returned.
  core::Status append(const sensor::RgbdFrame& frame, const CodedColor* color);

  /// @return Whether the next picture should be a key: a live reader joined
  ///         or fell back since the last key was appended, and none is still
  ///         sending the last key's records (`StreamServer::draining() ==
  ///         0`). Always false with no link.
  bool key_wanted() const noexcept;

  /// @brief Dial @p host : @p port from now on, in place of the last target
  ///        (the on-screen connect); an empty @p host stops dialling and lets
  ///        a connected reader go (disconnect). It starts the link on a
  ///        writer opened without one.
  ///
  /// The session and the file go on: a reader dialled again, here or at the
  /// new target, gets the same session, from its next key. Another target
  /// lets the connected reader go first; the same one keeps it. Either way
  /// @ref RgbdStreamStats::dial_failures starts again from 0, and a dial
  /// starts at once. Returns without waiting on a socket.
  /// @param host  A numeric IPv4 address, or empty.
  /// @param port  Its port; not read with an empty @p host.
  /// @return OK; with the last target kept, `Status::Code::InvalidArgument`
  ///         for a host that is not one host's numeric IPv4 address, port 0
  ///         (`core::net::parse_endpoint`'s reason) or a finished writer;
  ///         `StreamServer::start`'s errors, or `Status::Code::IoError` for a
  ///         dial thread that cannot start, when it starts the link.
  core::Status dial_to(const std::string& host, std::uint16_t port);

  /// @brief End the session: the file's footer, then the link's end. The
  ///        dialling stops, within the dial under way (@ref kDialLimit), and
  ///        a live reader gets the records queued for it before its end
  ///        (`StreamServer::finish`, within `kFinishLimit`). Idempotent.
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

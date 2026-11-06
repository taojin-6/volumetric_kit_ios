// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file rgbd_stream_sender.hpp
/// @brief A camera's frames as an RGBD stream: each frame's picture copied on
///        the device into the hardware encoder's own and coded, then the
///        frame written whole to the session (rgbd_stream_writer.hpp) -- a
///        `.vrst` file, a live link, both, or, with neither, nowhere (the dry
///        run: it codes and counts).
///
/// Every frame offered has ARKit's map steps taken out of its pose
/// (`MapSteps`), then the rate gate (`RateGate`, at the session's `fps`)
/// picks the frames that go. A frame taken goes to a thread of the sender's
/// own, which copies its picture's planes on the device into the next of
/// sensor's `HevcEncoder` pictures (`HevcEncoder::target`, one copy of two
/// planes, waited for), codes that (`send`), takes its access unit back
/// (`receive`), and appends the frame: its colour record, then its depth
/// record (`RgbdStreamWriter::append`). One picture is in flight, which is
/// enough at the stream's 10 Hz, and the thread that offers never waits on
/// the device, the encoder, the file or the link.
///
/// - **The copy keeps the camera's pictures free.** VideoToolbox's
///   low-latency session keeps the last picture it coded until it codes the
///   next. Handed the camera's own, that is one of the camera's few pictures
///   for the whole 100 ms between codings, beside those the scanner holds
///   (its keyframe ring): on the pattern's 4 that stops the camera for good.
///   With the copy the encoder keeps its own, and the frame's picture goes
///   back once its frame is sent.
/// - **The encoder opens at the first picture**, at its size and from its
///   pixel buffer's tags (`pixel_buffer_of`, `color_codes_of`), an untagged
///   matrix being BT.601, ARKit's, and codes every picture at
///   @ref RgbdStreamSenderOptions::qp. The records state the colour the
///   stream states (`HevcEncoder::Facts`); the copy moves the codes as they
///   are.
/// - **A key goes** when the encoder owes one (its first picture, and the
///   next after one it dropped or failed), as the session's first frame, and
///   when a live reader joined or fell back (`RgbdStreamWriter::key_wanted`).
/// - **A picture the encoder does not code** -- dropped, failed or refused,
///   which `receive` or the send reports -- leaves its frame out whole:
///   neither record is written, and the session numbers only what it writes,
///   so it leaves no gap there. It is counted
///   (@ref RgbdStreamSenderStats::failed).
/// - **A frame due while one taken still waits** is left, and the gate keeps
///   its interval for the next frame (`RateGate::due`), so a busy sender
///   sends late rather than queueing.
/// - **The session** opens with the sender when its options name a path or a
///   reader to dial, and otherwise at the first @ref RgbdStreamSender::dial_to
///   (the on-screen connect); until then the sender is a dry run. A sender
///   opened @ref RgbdStreamSenderOptions::dry stays one: it refuses every
///   dial. A frame's exposure, noise and pose go as the frame states them,
///   its pose with the map steps out.
/// - **Map steps are found in the frames offered.** A frame the camera's
///   poller let go unseen (`IRgbdSensor::poll` keeps the newest) is a gap to
///   `MapSteps`, which finds no step across a gap of 100 ms or more.
///
/// @code
/// namespace app = volumetric_kit::ios_app;
/// app::RgbdStreamSenderOptions o;
/// o.session.path = path;  // or empty: a dry run until a connect
/// o.session.fps = 10;
/// o.session.session_id = new_session_id();
/// o.device = &device;  // the camera's (hevc_encoder_device_requirements())
/// VKC_ASSIGN(auto sender, app::RgbdStreamSender::open(o, camera.info()));
/// // The camera's poller, each frame:
/// sender->offer(frame);  // at once; the picture codes on the sender's thread
/// // The on-screen connect, from any thread:
/// VKC_TRY(sender->dial_to("10.8.0.1", 7101));
/// // ...
/// VKC_TRY(sender->finish());
/// @endcode
///
/// @warning @ref offer from one thread, the camera's poller; @ref dial_to,
///          @ref stats, @ref take_times, @ref flush and @ref inject from any;
///          @ref finish once, not beside @ref offer, and destroy the sender
///          only once no call is under way.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rgbd_stream_writer.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"

namespace volumetric_kit::ios_app {

/// @brief How a @ref RgbdStreamSender sends, fixed when it opens.
struct RgbdStreamSenderOptions {
  /// The session, as `RgbdStreamWriter::open` takes it: its file (`path`),
  /// the reader it dials (`dial_host`, `dial_port`), its id and header.
  /// `fps` is also the rate frames are taken at (`RateGate`; 0 takes every
  /// frame); `color` is not read, since every frame goes with its picture.
  RgbdStreamOptions session;
  /// The stream's slice QP (`HevcEncoder::Options::qp`).
  int qp = 22;
  /// The device the camera's pictures are on (`ArkitSensor::Options::
  /// device`), made with `sensor::hevc_encoder_device_requirements()`: the
  /// encoder's pictures are written there. Required. Borrowed: it must
  /// outlive the sender.
  const core::Device* device = nullptr;
  /// A dry run for the sender's life: no file, no reader, and every
  /// @ref RgbdStreamSender::dial_to that would open the session refused, so
  /// nothing the camera sees leaves the device. It still codes and counts.
  bool dry = false;
};

/// @brief One picture's host times, ms, both from the start of its send.
struct PictureTimes {
  /// The picture copied into the encoder's own (`HevcEncoder::target`) and
  /// sent (`HevcEncoder::send`).
  double send_ms = 0.0;
  double coded_ms = 0.0;  ///< Its access unit back from `receive`.
};

/// @brief What a @ref RgbdStreamSender has done since it opened.
struct RgbdStreamSenderStats {
  std::uint64_t offered =
      0;                    ///< Frames offered (@ref RgbdStreamSender::offer).
  std::uint64_t taken = 0;  ///< Of them, taken by the rate gate.
  /// Frames due at the gate while the frame taken before still waited: left,
  /// the gate's interval kept for the next.
  std::uint64_t busy = 0;
  std::uint64_t coded = 0;  ///< Pictures coded.
  std::uint64_t keys = 0;   ///< Of them, keys.
  /// Frames taken whose picture was not coded -- no NV12 planes or no pixel
  /// buffer, an encoder that did not open, a copy or a send that failed, a
  /// picture dropped or failed -- and that therefore went nowhere.
  std::uint64_t failed = 0;
  std::uint64_t written = 0;        ///< Frames written whole to the session.
  std::uint64_t refused = 0;        ///< Coded frames the session refused.
  std::uint64_t picture_bytes = 0;  ///< The access units coded.
  std::uint64_t map_steps = 0;      ///< `MapSteps::steps()`'s length.
  /// A session is open: a file, a link or both. False is the dry run.
  bool session_open = false;
  RgbdStreamStats session;  ///< The session's counts; zero before it opens.
  /// The last failure (a picture or a frame), named; empty for none.
  std::string error;
};

/// @brief Sends a camera's frames as an RGBD stream (above).
class RgbdStreamSender {
 public:
  /// @brief Ready the sender, and open the session when @p options name a
  ///        path or a reader to dial.
  /// @param info  The camera's, from when it opened: its colour camera is
  ///              the pictures'.
  /// @return The sender; `Status::Code::InvalidArgument` for a sensor without
  ///         a colour camera, a QP outside 0 to 51, no
  ///         @ref RgbdStreamSenderOptions::device, or a dry run with a path or
  ///         a reader; `RgbdStreamWriter::open`'s errors for the session, with
  ///         nothing made; `Status::Code::IoError` for a thread that cannot
  ///         start. The encoder's own errors come at the first frame
  ///         (@ref RgbdStreamSenderStats::error).
  static core::Result<std::unique_ptr<RgbdStreamSender>> open(
      const RgbdStreamSenderOptions& options, const sensor::SensorInfo& info);

  /// Finishes, as @ref finish.
  ~RgbdStreamSender();
  RgbdStreamSender(const RgbdStreamSender&) = delete;
  RgbdStreamSender& operator=(const RgbdStreamSender&) = delete;

  /// @brief The camera's next frame, as its sensor handed it out: its map
  ///        steps out, the gate, and, if taken, handed to the sender's
  ///        thread. Returns at once; nothing after @ref finish.
  void offer(const sensor::RgbdFrame& frame);

  /// @brief The on-screen connect and disconnect: dial @p host : @p port
  ///        from now on (`RgbdStreamWriter::dial_to`), opening the session
  ///        first if none is open; an empty @p host lets a reader go and
  ///        opens nothing.
  /// @return OK; `RgbdStreamWriter::open`'s or `dial_to`'s refusal, with the
  ///         last target kept; `Status::Code::InvalidArgument` once
  ///         finished, or for a host on a dry run.
  core::Status dial_to(const std::string& host, std::uint16_t port);

  /// @return The counts so far.
  RgbdStreamSenderStats stats() const;

  /// @return What the reader last said (`RgbdStreamWriter::reader_note`); none
  ///         with no session. From any thread.
  std::optional<grid::networking::ViewerNote> reader_note() const;

  /// @return The pictures' times since the last call, oldest first; at most
  ///         the last 1024 coded.
  std::vector<PictureTimes> take_times();

  /// @brief Send the frame handed over, end the thread, and finish the
  ///        session (`RgbdStreamWriter::finish`). Idempotent.
  /// @return OK; the session's finish error, which a later call returns.
  core::Status finish();

  /// @brief For tests: wait until the frames handed over are sent.
  void flush();

  /// @brief For tests: the next picture sent meets @p fault, through the
  ///        encoder's own path (`HevcEncoder::inject`).
  void inject(sensor::HevcEncoder::Fault fault);

 private:
  struct Impl;
  explicit RgbdStreamSender(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::ios_app

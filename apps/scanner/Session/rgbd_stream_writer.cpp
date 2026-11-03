// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "rgbd_stream_writer.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <utility>

#include "volumetric_kit/core/base/exception_status.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace volumetric_kit::ios_app {

namespace stream = grid::stream;
namespace wire = sensor::wire;

namespace {

// A sensor on its own clock takes the session's clock 1; one sensor a session.
constexpr std::uint16_t kDeviceClock = 1;

core::Status refuse(const char* api, const std::string& why) {
  return core::Status::invalid_argument(std::string("RgbdStreamWriter::") +
                                        api + ": " + why);
}

}  // namespace

struct RgbdStreamWriter::Impl {
  sensor::SensorInfo info;
  std::unique_ptr<stream::StreamWriter> file;
  bool color = false;             // the session has RgbdColor, as track 0
  std::uint16_t depth_track = 0;  // 1 with colour
  RgbdStreamStats stats;
  std::uint64_t last_pts_ns = 0;  // the last frame's, once one is written
  // A frame's depth record failed after its colour record went out: every
  // later append fails with it, as the file's own write failures do.
  core::Status split;
  bool finished = false;
  core::Status finish_status;  // what the first finish returned
};

RgbdStreamWriter::RgbdStreamWriter(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

RgbdStreamWriter::~RgbdStreamWriter() {
  if (impl_) (void)finish();
}

core::Result<std::unique_ptr<RgbdStreamWriter>> RgbdStreamWriter::open(
    const RgbdStreamOptions& options, const sensor::SensorInfo& info) try {
  constexpr const char* kApi = "open";
  if (options.path.empty()) return refuse(kApi, "no path, so nothing to write");
  if (std::all_of(options.session_id.begin(), options.session_id.end(),
                  [](std::uint8_t b) { return b == 0; })) {
    return refuse(kApi, "an all-zero session id, which names no session");
  }
  VKC_ASSIGN(std::string depth_params, wire::encode_sensor(info));

  auto impl = std::make_unique<Impl>();
  impl->info = info;
  impl->color = options.color && info.color.has_value();
  impl->depth_track = impl->color ? 1 : 0;

  const std::uint16_t clock = info.clock == sensor::ClockDomain::Host
                                  ? stream::kHostClock
                                  : kDeviceClock;
  const stream::Rate rate{options.fps != 0 ? options.fps : info.fps, 1};

  stream::Session session;
  session.id = options.session_id;
  session.created_ns = options.created_ns;
  session.writer = options.writer;
  session.calibration = options.calibration;
  if (impl->color) {
    stream::Track color;
    color.kind = stream::TrackKind::RgbdColor;
    color.clock = clock;
    color.rate = rate;
    color.width = info.color->size.width;
    color.height = info.color->size.height;
    color.sensor = info.id;
    color.params = wire::encode_color_params(wire::ColorCodec::HevcMain8);
    session.tracks.push_back(std::move(color));
  }
  stream::Track depth;
  depth.kind = stream::TrackKind::RgbdDepth;
  depth.clock = clock;
  depth.rate = rate;
  depth.width = info.depth->size.width;
  depth.height = info.depth->size.height;
  depth.sensor = info.id;
  depth.params = std::move(depth_params);
  session.tracks.push_back(std::move(depth));

  VKC_ASSIGN(impl->file, stream::StreamWriter::create(options.path, session));
  return std::unique_ptr<RgbdStreamWriter>(
      new RgbdStreamWriter(std::move(impl)));
} catch (...) {
  return core::exception_status("RgbdStreamWriter::open",
                                core::Status::Code::InvalidArgument);
}

core::Status RgbdStreamWriter::append(const sensor::RgbdFrame& frame,
                                      const CodedColor* color) try {
  constexpr const char* kApi = "append";
  Impl& s = *impl_;
  if (s.finished) return refuse(kApi, "the session is finished");
  if (!s.split.ok()) return s.split;

  // Every check, then the two records: nothing is written for a refusal.
  if (s.color && color == nullptr) {
    return refuse(kApi, "no picture, in a session with colour");
  }
  if (!s.color && color != nullptr) {
    return refuse(kApi, "a picture, in a session without colour");
  }
  if (color != nullptr && (color->data == nullptr || color->size == 0)) {
    return refuse(kApi, "an empty picture");
  }
  if (color != nullptr && s.stats.frames == 0 && !color->key) {
    return refuse(kApi, "a first picture that is not a key");
  }
  if (s.stats.frames > 0 && frame.timestamp_ns < s.last_pts_ns) {
    return refuse(kApi, "a capture time of " +
                            std::to_string(frame.timestamp_ns) +
                            " ns, below the last frame's " +
                            std::to_string(s.last_pts_ns));
  }
  VKC_ASSIGN(const wire::DepthRecord depth, wire::encode_depth(s.info, frame));
  std::array<std::uint8_t, wire::kColorPrefixBytes> color_prefix{};
  if (color != nullptr) {
    VKC_ASSIGN(color_prefix, wire::encode_color(s.info, frame, color->codes));
  }

  // One set, one pts, and seq = set on both tracks: a frame is written whole
  // or the session stops, so each track has had one record a frame.
  stream::RecordInfo record;
  record.pts_ns = frame.timestamp_ns;
  record.seq = s.stats.frames;
  record.set = s.stats.frames;
  const stream::RecordPayload depth_payload = stream::RecordPayload::of(
      depth.prefix.data(), depth.prefix.size(), depth.depth, depth.depth_bytes,
      depth.confidence, depth.confidence_bytes);
  if (color != nullptr) {
    const stream::RecordPayload color_payload = stream::RecordPayload::of(
        color_prefix.data(), color_prefix.size(), color->data, color->size);
    record.track = 0;
    record.key = color->key;
    VKC_TRY(s.file->append(record, color_payload));
    record.track = s.depth_track;
    record.key = true;
    const core::Status written = s.file->append(record, depth_payload);
    if (!written.ok()) {
      s.split = written.with_context(
          "RgbdStreamWriter::append: the depth record after its colour");
      return s.split;
    }
    s.stats.color_bytes += color_payload.size();
    if (color->key) ++s.stats.keys;
  } else {
    record.track = s.depth_track;
    record.key = true;
    VKC_TRY(s.file->append(record, depth_payload));
  }
  s.stats.depth_bytes += depth_payload.size();
  ++s.stats.frames;
  s.last_pts_ns = frame.timestamp_ns;
  return {};
} catch (...) {
  return core::exception_status("RgbdStreamWriter::append",
                                core::Status::Code::InvalidArgument);
}

core::Status RgbdStreamWriter::finish() {
  Impl& s = *impl_;
  if (s.finished) return s.finish_status;
  s.finished = true;
  s.finish_status = s.file->finish();
  return s.finish_status;
}

RgbdStreamStats RgbdStreamWriter::stats() const noexcept {
  return impl_->stats;
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "rgbd_stream_writer.hpp"

#include <fcntl.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "dial_entry.hpp"
#include "volumetric_kit/core/base/exception_status.hpp"
#include "volumetric_kit/core/base/unique_fd.hpp"
#include "volumetric_kit/core/net/address.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace volumetric_kit::ios_app {

namespace net = core::net;
namespace networking = grid::networking;
namespace stream = grid::stream;
namespace wire = sensor::wire;

namespace {

using Clock = std::chrono::steady_clock;

// A sensor on its own clock takes the session's clock 1; one sensor a session.
constexpr std::uint16_t kDeviceClock = 1;
// The longest the dial thread goes without looking whether the served reader
// is still there.
constexpr std::chrono::milliseconds kWatch{100};

core::Status refuse(const char* api, const std::string& why) {
  return core::Status::invalid_argument(std::string("RgbdStreamWriter::") +
                                        api + ": " + why);
}

// One host's numeric IPv4 address and a port to dial: InvalidArgument
// "RgbdStreamWriter::<api>: <the core's reason>".
core::Status check_target(const char* api, const std::string& host,
                          std::uint16_t port) {
  const std::string where = std::string("RgbdStreamWriter::") + api;
  const core::Status address = net::parse_endpoint(host, port).status();
  if (!address.ok()) return address.with_context(where);
  if (port == 0) return refuse(api, "port 0 is not a port to dial");
  return {};
}

// Shut both directions of the socket `fd` shares with the server, then close
// it: the server's thread reads the end and lets the reader go, and the
// reader reads its end now rather than when the server's descriptor closes.
void let_go(core::UniqueFd* fd) noexcept {
  if (!fd->valid()) return;
  (void)::shutdown(fd->get(), SHUT_RDWR);
  fd->reset();
}

}  // namespace

struct RgbdStreamWriter::Impl {
  // --- The writer's thread: open, append, finish -----------------------------
  sensor::SensorInfo info;
  std::unique_ptr<stream::StreamWriter> file;  // null without a path
  bool color = false;             // the session has RgbdColor, as track 0
  std::uint16_t depth_track = 0;  // 1 with colour
  std::uint64_t frames = 0;       // written whole: the next frame's set and seq
  std::uint64_t last_pts_ns = 0;  // the last frame's, once one is written
  // A frame's depth record failed after its colour record went out: every
  // later append fails with it, as the file's own write failures do.
  core::Status split;
  bool finished = false;
  core::Status finish_status;  // what the first finish returned

  // The counts stats() reads from any thread.
  mutable std::mutex counts_mutex;
  RgbdStreamStats counts;

  // --- The link --------------------------------------------------------------
  stream::Session session;  // what a reader gets first
  networking::ServerOptions server_options;
  std::chrono::milliseconds redial_wait = kRedialWait;  // the options'
  std::chrono::milliseconds dial_limit = kDialLimit;
  std::chrono::milliseconds watch = kWatch;  // from redial_wait
  // Set once, under link_mutex, and kept until the writer goes: append and
  // key_wanted read it without the lock.
  std::unique_ptr<networking::StreamServer> server;
  std::atomic<networking::StreamServer*> live{nullptr};
  // The server's key_wanted() just before the last key was published: a
  // count above it is a reader the key did not serve.
  std::atomic<std::uint64_t> wanted_at_key{0};

  mutable std::mutex link_mutex;
  std::condition_variable link_wake;
  std::string host;  // the target; empty dials none
  std::uint16_t port = 0;
  std::uint64_t generation = 0;  // one up each time the target moves
  // A connect (open or dial_to) wants a dial now, not at the redial time; the
  // dial under way, or the next, answers it.
  bool dial_now = false;
  Clock::time_point last_dial{};  // when the last dial started
  std::uint64_t dial_failures = 0;
  bool no_route = false;
  // A second descriptor of the served reader's socket, to let it go: the
  // server owns the first, and closes it only as the reader leaves or fails.
  core::UniqueFd reader;
  Clock::time_point served_at{};  // when `reader` was served
  bool stopping = false;
  std::thread dialer;

  // However open or finish went, nothing outlives the writer.
  ~Impl() { stop_link(); }

  // Under link_mutex: make the server and the dial thread, once.
  core::Status start_link();
  // The dial thread.
  void dial_loop() noexcept;
  // Under link_mutex: serve `connected`, keeping a second descriptor of it.
  core::Status serve(core::UniqueFd connected);
  // Stop dialling, end the server's readers after what is queued for them,
  // and let the last one go. Idempotent.
  void stop_link() noexcept;
};

core::Status RgbdStreamWriter::Impl::start_link() {
  if (server) return {};
  VKC_ASSIGN(server, networking::StreamServer::start(session, server_options));
  try {
    dialer = std::thread([this] { dial_loop(); });  // waits for link_mutex
  } catch (...) {
    server.reset();  // so the next start tries again
    return core::exception_status("RgbdStreamWriter: the dial thread",
                                  core::Status::Code::IoError);
  }
  live.store(server.get(), std::memory_order_release);
  return {};
}

core::Status RgbdStreamWriter::Impl::serve(core::UniqueFd connected) try {
  core::UniqueFd copy(::fcntl(connected.get(), F_DUPFD_CLOEXEC, 0));
  if (!copy.valid()) {
    return core::Status::io_error(
        "RgbdStreamWriter: cannot keep the reader's socket: " +
        std::generic_category().message(errno));
  }
  VKC_TRY(server->add_viewer(std::move(connected)));
  reader = std::move(copy);
  served_at = Clock::now();
  return {};
} catch (...) {
  return core::exception_status("RgbdStreamWriter: serve",
                                core::Status::Code::IoError);
}

void RgbdStreamWriter::Impl::dial_loop() noexcept try {
  std::unique_lock<std::mutex> lock(link_mutex);
  while (!stopping) {
    const Clock::time_point now = Clock::now();
    if (reader.valid()) {
      if (server->viewers() == 0) {
        // The served reader has ended (left, refused the session, failed or
        // stalled): its end goes now. One served for less than redial_wait
        // counts as a failed dial, so a reader that refuses every connection
        // reads as retrying.
        let_go(&reader);
        if (now - served_at < redial_wait) ++dial_failures;
      } else if (now - served_at >= redial_wait) {
        dial_failures = 0;  // a reader that stays ends the failures' run
      }
    }
    if (host.empty() && server->viewers() == 0) {
      link_wake.wait(lock);  // until a target, or the stop
      continue;
    }
    if (host.empty() || reader.valid() || server->viewers() > 0) {
      link_wake.wait_for(lock, watch);  // a reader served, or going
      continue;
    }
    // At most one dial a redial_wait, however the last ended; a connect
    // dials at once.
    if (!dial_now && now < last_dial + redial_wait) {
      link_wake.wait_until(lock, last_dial + redial_wait,
                           [&] { return stopping || dial_now; });
      continue;
    }
    const std::uint64_t dialled = generation;
    const std::string to = host;
    const std::uint16_t at = port;
    last_dial = now;
    lock.unlock();
    core::Result<core::UniqueFd> fd = net::dial(to, at, dial_limit);
    lock.lock();
    // Stopped, or the target moved meanwhile: the socket closes unserved,
    // and the new target is dialled at once.
    if (stopping || dialled != generation) continue;
    dial_now = false;  // a connect to this target while it ran: answered
    const core::Status served =
        fd.ok() ? serve(std::move(fd).value()) : fd.status();
    if (served.ok()) {
      no_route = false;
      continue;
    }
    ++dial_failures;
    no_route = is_no_route(served);
  }
} catch (...) {
  // Only copying the target allocates: out of memory, the dialling ends, and
  // a served reader stays until it leaves.
}

void RgbdStreamWriter::Impl::stop_link() noexcept {
  {
    const std::lock_guard<std::mutex> lock(link_mutex);
    stopping = true;
  }
  link_wake.notify_all();
  if (dialer.joinable()) dialer.join();
  if (server) server->finish();
  const std::lock_guard<std::mutex> lock(link_mutex);
  let_go(&reader);
}

RgbdStreamWriter::RgbdStreamWriter(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

RgbdStreamWriter::~RgbdStreamWriter() {
  if (impl_) (void)finish();  // the link stops with impl_, however it went
}

core::Result<std::unique_ptr<RgbdStreamWriter>> RgbdStreamWriter::open(
    const RgbdStreamOptions& options, const sensor::SensorInfo& info) try {
  constexpr const char* kApi = "open";
  if (options.path.empty() && options.dial_host.empty()) {
    return refuse(kApi, "neither a path nor a dial host, so nothing to write");
  }
  if (std::all_of(options.session_id.begin(), options.session_id.end(),
                  [](std::uint8_t b) { return b == 0; })) {
    return refuse(kApi, "an all-zero session id, which names no session");
  }
  // Read by a link, which dial_to can start later: checked with or without.
  if (options.redial_wait.count() <= 0 || options.dial_limit.count() <= 0) {
    return refuse(kApi, "a redial wait or dial limit of 0 or less");
  }
  if (!options.dial_host.empty()) {
    VKC_TRY(check_target(kApi, options.dial_host, options.dial_port));
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

  stream::Session& session = impl->session;
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

  // Dials out and listens on nothing; one reader, which asks for keys and
  // says how it is doing.
  networking::ServerOptions& serving = impl->server_options;
  serving.listen = false;
  serving.replay_gop = false;
  serving.max_viewers = 1;
  serving.max_gop_bytes = options.max_gop_bytes;
  serving.read_notes = true;
  impl->redial_wait = options.redial_wait;
  impl->dial_limit = options.dial_limit;
  // A tenth of the redial wait, so a redial trails a leave by the same share
  // of the wait; at most kWatch, and at least 1 ms, so a redial wait under
  // 10 ms never spins.
  impl->watch = std::clamp<std::chrono::milliseconds>(
      options.redial_wait / 10, std::chrono::milliseconds{1}, kWatch);

  if (!options.path.empty()) {
    VKC_ASSIGN(impl->file, stream::StreamWriter::create(options.path, session));
  }
  if (!options.dial_host.empty()) {
    const std::lock_guard<std::mutex> lock(impl->link_mutex);
    impl->host = options.dial_host;
    impl->port = options.dial_port;
    impl->dial_now = true;
    VKC_TRY(impl->start_link());
  }
  return std::unique_ptr<RgbdStreamWriter>(
      new RgbdStreamWriter(std::move(impl)));
} catch (...) {
  return core::exception_status("RgbdStreamWriter::open",
                                core::Status::Code::InvalidArgument);
}

core::Status RgbdStreamWriter::append(const sensor::RgbdFrame& frame,
                                      const CodedColor* color) try {
  constexpr const char* kApi = "append";
  constexpr const char* kWhere = "RgbdStreamWriter::append";
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
  if (color != nullptr && s.frames == 0 && !color->key) {
    return refuse(kApi, "a first picture that is not a key");
  }
  // Times strictly increase, as the frame contract's: 0 is no time.
  if (frame.timestamp_ns == 0) return refuse(kApi, "no capture time (0)");
  if (s.frames > 0 && frame.timestamp_ns <= s.last_pts_ns) {
    return refuse(kApi, "a capture time of " +
                            std::to_string(frame.timestamp_ns) +
                            " ns, not after the last frame's " +
                            std::to_string(s.last_pts_ns));
  }
  auto encoded = wire::encode_depth(s.info, frame);
  if (!encoded.ok()) return encoded.status().with_context(kWhere);
  const wire::DepthRecord depth = std::move(encoded).value();
  std::array<std::uint8_t, wire::kColorPrefixBytes> color_prefix{};
  if (color != nullptr) {
    auto prefix = wire::encode_color(s.info, frame, color->codes);
    if (!prefix.ok()) return prefix.status().with_context(kWhere);
    color_prefix = prefix.value();
  }

  // One set, one pts, and seq = set on both tracks: a frame is written whole
  // or the session stops, so each track has had one record a frame.
  stream::RecordInfo colour_info;
  colour_info.pts_ns = frame.timestamp_ns;
  colour_info.seq = s.frames;
  colour_info.set = s.frames;
  colour_info.track = 0;
  colour_info.key = color != nullptr && color->key;
  stream::RecordInfo depth_info = colour_info;
  depth_info.track = s.depth_track;
  depth_info.key = true;
  const stream::RecordPayload depth_payload = stream::RecordPayload::of(
      depth.prefix.data(), depth.prefix.size(), depth.depth, depth.depth_bytes,
      depth.confidence, depth.confidence_bytes);
  std::optional<stream::RecordPayload> color_payload;
  if (color != nullptr) {
    color_payload = stream::RecordPayload::of(
        color_prefix.data(), color_prefix.size(), color->data, color->size);
  }

  if (s.file) {
    if (color_payload) {
      const core::Status colour = s.file->append(colour_info, *color_payload);
      if (!colour.ok()) return colour.with_context(kWhere);
      const core::Status written = s.file->append(depth_info, depth_payload);
      if (!written.ok()) {
        s.split = written.with_context(std::string(kWhere) +
                                       ": the depth record after its colour");
        return s.split;
      }
    } else {
      const core::Status written = s.file->append(depth_info, depth_payload);
      if (!written.ok()) return written.with_context(kWhere);
    }
  }

  // The file holds the frame whole: serve it. A reader's failure, or a record
  // the server drops, is the link's alone.
  if (networking::StreamServer* live = s.live.load(std::memory_order_acquire)) {
    // Track 0's key serves every reader that asked before it is published.
    if (colour_info.key || !color_payload) {
      s.wanted_at_key.store(live->key_wanted(), std::memory_order_relaxed);
    }
    if (color_payload) (void)live->publish(colour_info, *color_payload);
    (void)live->publish(depth_info, depth_payload);
  }

  ++s.frames;
  s.last_pts_ns = frame.timestamp_ns;
  const std::lock_guard<std::mutex> lock(s.counts_mutex);
  s.counts.frames = s.frames;
  s.counts.depth_bytes += depth_payload.size();
  if (color_payload) {
    s.counts.color_bytes += color_payload->size();
    if (color->key) ++s.counts.keys;
  }
  return {};
} catch (...) {
  return core::exception_status("RgbdStreamWriter::append",
                                core::Status::Code::InvalidArgument);
}

bool RgbdStreamWriter::key_wanted() const noexcept {
  const networking::StreamServer* live =
      impl_->live.load(std::memory_order_acquire);
  if (live == nullptr) return false;
  return live->key_wanted() >
             impl_->wanted_at_key.load(std::memory_order_relaxed) &&
         live->draining() == 0;
}

core::Status RgbdStreamWriter::dial_to(const std::string& host,
                                       std::uint16_t port) try {
  constexpr const char* kApi = "dial_to";
  if (!host.empty()) VKC_TRY(check_target(kApi, host, port));
  Impl& s = *impl_;
  {
    const std::lock_guard<std::mutex> lock(s.link_mutex);
    if (s.stopping) return refuse(kApi, "the session is finished");
    if (host.empty() && !s.server) return {};  // no link, and none wanted
    if (!host.empty()) VKC_TRY(s.start_link());
    const bool moved = host != s.host || (!host.empty() && port != s.port);
    if (moved) {
      // A dial under way to the last target is let go as it ends.
      let_go(&s.reader);
      ++s.generation;
    }
    s.host = host;
    s.port = host.empty() ? 0 : port;
    // The same target keeps a served reader, and a dial under way to it.
    s.dial_now = !host.empty() && !s.reader.valid();
    s.dial_failures = 0;
    s.no_route = false;
  }
  s.link_wake.notify_all();
  return {};
} catch (...) {
  return core::exception_status("RgbdStreamWriter::dial_to",
                                core::Status::Code::InvalidArgument);
}

core::Status RgbdStreamWriter::finish() {
  Impl& s = *impl_;
  if (s.finished) return s.finish_status;
  core::Status done = s.file ? s.file->finish() : core::Status{};
  // Out of memory leaves grid's writer as it was, open: a later call retries.
  if (done.domain() == core::Status::Code::OutOfMemory) return done;
  s.finished = true;
  s.finish_status = std::move(done);
  s.stop_link();
  return s.finish_status;
}

RgbdStreamStats RgbdStreamWriter::stats() const noexcept {
  const Impl& s = *impl_;
  RgbdStreamStats out;
  {
    const std::lock_guard<std::mutex> lock(s.counts_mutex);
    out = s.counts;
  }
  if (const networking::StreamServer* live =
          s.live.load(std::memory_order_acquire)) {
    out.key_wanted = live->key_wanted();
    out.viewers = live->viewers();
  }
  const std::lock_guard<std::mutex> lock(s.link_mutex);
  out.dial_failures = s.dial_failures;
  out.no_route = s.no_route;
  return out;
}

std::optional<networking::ViewerNote> RgbdStreamWriter::reader_note()
    const noexcept {
  const networking::StreamServer* live =
      impl_->live.load(std::memory_order_acquire);
  if (live == nullptr) return std::nullopt;
  return live->newest_note();
}

}  // namespace volumetric_kit::ios_app

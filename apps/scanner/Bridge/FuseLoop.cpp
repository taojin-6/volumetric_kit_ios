// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "FuseLoop.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "CaptureReport.hpp"
#include "FailureHandling.hpp"
#include "RendererImpl.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/sensor/apple/core_video_color.hpp"
#include "volumetric_kit/sensor/apple/pixel_buffer_importer.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/testing/index_band.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"

namespace volumetric_kit::ios_app {
namespace {

using Clock = std::chrono::steady_clock;

// How long the loop waits for a frame before it looks at its stop flag
// again: a stop is seen within this, as is a frame the signal missed.
constexpr std::chrono::milliseconds kIdleWait{50};

// A picture's colour tags are read again every this many frames, so a change
// mid-run shows in the next line without reading four attachments a frame.
constexpr std::uint64_t kCodesEveryFrames = 60;

float ms_since(Clock::time_point t0) {
  return std::chrono::duration<float, std::milli>(Clock::now() - t0).count();
}

// The window a line reports, and the run's own counts since its start.
struct Window {
  std::vector<double> poll_ms;
  std::vector<double> fuse_ms;
  std::vector<double> prep_ms;
  std::uint64_t fused = 0;  // since the start
  // When the first frame came, since the start: after a lock, how soon the
  // camera is back.
  std::optional<double> first_frame_s;
  // The stream's bytes at the window's start, and when it started: the
  // window's rates.
  std::uint64_t video_bytes = 0;
  std::uint64_t depth_bytes = 0;
  Clock::time_point since = Clock::now();
  void clear() {
    poll_ms.clear();
    fuse_ms.clear();
    prep_ms.clear();
  }
};

// One frame taken: checked, its codes read now and then, and fused.
void take(RendererImpl& impl, sensor::testing::RgbdContract& contract,
          Window& window, const sensor::RgbdFrame& frame, float poll_ms) {
  contract.take(frame);
  window.poll_ms.push_back(poll_ms);

  // The pattern's own check: its depth names the frame's sequence. On the
  // driver's host copy of the synthetic depth; no camera frame is read.
  std::optional<bool> index_kept;
  if (impl.camera_pattern && frame.depth != nullptr) {
    const std::optional<std::uint32_t> index =
        sensor::testing::depth_index(frame.depth, frame.depth_camera.size.width,
                                     frame.depth_camera.size.height);
    constexpr std::uint64_t kModulus = std::uint64_t{1}
                                       << sensor::testing::kIndexBits;
    index_kept = index && *index == frame.sequence % kModulus;
  }
  std::optional<sensor::ColorCodes> codes;
  bool codes_known = false;
  {
    std::lock_guard<std::mutex> lock(impl.camera_readout.mutex);
    codes_known = impl.camera_readout.codes_known;
  }
  if (!codes_known || frame.sequence % kCodesEveryFrames == 0) {
    if (CVPixelBufferRef picture = sensor::pixel_buffer_of(frame.color)) {
      vkc::Result<sensor::ColorCodes> read = sensor::color_codes_of(picture);
      if (read) codes = read.value();
    }
  }
  {
    std::lock_guard<std::mutex> lock(impl.camera_readout.mutex);
    if (index_kept) {
      ++impl.camera_readout.index_checked;
      if (!*index_kept) ++impl.camera_readout.index_mismatches;
    }
    if (codes) {
      impl.camera_readout.codes = *codes;
      impl.camera_readout.codes_known = true;
    }
  }

  // Before the fuse, so the picture codes on the sender's thread meanwhile.
  if (impl.stream) impl.stream->offer(frame);

  if (const std::optional<FuseTimes> fused = impl.fusion.fuse(frame, poll_ms)) {
    ++window.fused;
    window.fuse_ms.push_back(fused->fuse_ms());
    window.prep_ms.push_back(fused->prep_ms);
  }
}

// The window as its line, printed to stdout, where the device console reads
// it, with the contract's verdict published for the read-out.
void report(RendererImpl& impl, const sensor::testing::RgbdContract& contract,
            Window& window, Clock::time_point started, bool stop) {
  const sensor::ArkitSensor& camera = *impl.arkit;
  const sensor::SensorStats stats = camera.stats();
  const sensor::ArkitStatus status = camera.status();
  CaptureReport r;
  r.source =
      impl.camera_pattern ? CaptureSource::Pattern : CaptureSource::Camera;
  r.seconds = std::chrono::duration<double>(Clock::now() - started).count();
  r.stop = stop;
  r.first_frame_s = window.first_frame_s;
  r.tracking = sensor::to_string(status.tracking);
  r.interrupted = status.interrupted;
  r.session_error = status.session_error;
  r.received = stats.received;
  r.delivered = stats.delivered;
  r.dropped = stats.dropped;
  r.failed = stats.failed;
  r.withheld = status.withheld;
  r.refused = status.refused;
  r.missed = status.missed;
  r.fused = window.fused;
  // Copied, so the window keeps its capacity for the next one.
  r.poll_ms = window.poll_ms;
  r.fuse_ms = window.fuse_ms;
  r.prep_ms = window.prep_ms;
  r.breach = contract.breach(stats);
  {
    std::lock_guard<std::mutex> lock(impl.camera_readout.mutex);
    impl.camera_readout.breach = r.breach;
    const CameraReadout& o = impl.camera_readout;
    r.codes_known = o.codes_known;
    r.matrix = o.codes.matrix;
    r.transfer = o.codes.transfer;
    r.primaries = o.codes.primaries;
    r.full_range = o.codes.full_range;
    r.chroma_location = o.codes.chroma_location;
    r.index_checked = o.index_checked;
    r.index_mismatches = o.index_mismatches;
  }
  const std::string line = capture_report_line(r);
  std::fputs(line.c_str(), stdout);
  std::fputc('\n', stdout);

  // The stream's line beside it: counts since the sender opened, this
  // window's rates and picture times.
  StreamReport st;
  st.seconds = r.seconds;
  st.stop = stop;
  const Clock::time_point now = Clock::now();
  if (impl.stream) {
    const RgbdStreamSenderStats s = impl.stream->stats();
    st.on = true;
    st.session = s.session_open;
    st.taken = s.taken;
    st.busy = s.busy;
    st.coded = s.coded;
    st.keys = s.keys;
    st.failed = s.failed;
    st.written = s.written;
    st.refused = s.refused;
    st.map_steps = s.map_steps;
    const double window_s =
        std::chrono::duration<double>(now - window.since).count();
    if (window_s > 0.0) {
      st.video_bits_per_s =
          8.0 * static_cast<double>(s.picture_bytes - window.video_bytes) /
          window_s;
      st.depth_bits_per_s =
          8.0 *
          static_cast<double>(s.session.depth_bytes - window.depth_bytes) /
          window_s;
    }
    window.video_bytes = s.picture_bytes;
    window.depth_bytes = s.session.depth_bytes;
    for (const PictureTimes& t : impl.stream->take_times()) {
      st.send_ms.push_back(t.send_ms);
      st.coded_ms.push_back(t.coded_ms);
    }
    st.viewers = s.session.viewers;
    st.dial_failures = s.session.dial_failures;
    st.error = without_path(s.error, impl.record_path);
  } else {
    st.error = impl.stream_error;
  }
  window.since = now;
  const std::string stream_line = stream_report_line(st);
  std::fputs(stream_line.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
  window.clear();
}

}  // namespace

void run_fuse_loop(RendererImpl& impl) {
  sensor::ArkitSensor& camera = *impl.arkit;
  // One checker a start: the camera's sequences and counters begin again at
  // each.
  sensor::testing::RgbdContract contract(camera.info());
  {
    std::lock_guard<std::mutex> lock(impl.camera_readout.mutex);
    impl.camera_readout.breach.clear();
    impl.camera_readout.index_checked = 0;
    impl.camera_readout.index_mismatches = 0;
  }
  Window window;
  if (impl.stream) {
    // The sender lives across starts: this run's rates count from here, and
    // its times from this run's first frame.
    const RgbdStreamSenderStats s = impl.stream->stats();
    window.video_bytes = s.picture_bytes;
    window.depth_bytes = s.session.depth_bytes;
    (void)impl.stream->take_times();
  }
  // A line's worth of samples at the camera's 60 Hz, so the hot path does not
  // grow them.
  window.poll_ms.reserve(1024);
  window.fuse_ms.reserve(1024);
  window.prep_ms.reserve(1024);
  const Clock::time_point started = Clock::now();
  const auto every = std::chrono::seconds(kReportEverySeconds);
  Clock::time_point next_line = started + every;

  while (impl.fusing.load()) {
    // Keep the boundary inside the loop: one failed poll or fuse is reported
    // and skipped without ending the scan. Reporting must itself remain safe
    // when host allocation is failing.
    const bool completed = contain_frame_failure(
        [&] {
          if (camera.exhausted()) {
            // ARKit's session failed: every poll would say so again. The
            // status carries its message to the read-out and the line.
            impl.frame_signal.wait_for(kIdleWait);
            return;
          }
          const auto t0 = Clock::now();
          vkc::Result<std::optional<sensor::RgbdFrame>> got = camera.poll();
          const float poll_ms = ms_since(t0);
          if (!got) {
            impl.fusion.note_error(got.status().message(), "camera: ");
            impl.frame_signal.wait_for(kIdleWait);
            return;
          }
          if (!got.value()) {
            impl.frame_signal.wait_for(kIdleWait);
            return;
          }
          if (!window.first_frame_s) {
            window.first_frame_s =
                std::chrono::duration<double>(Clock::now() - started).count();
          }
          take(impl, contract, window, *got.value(), poll_ms);
        },
        [&impl](std::string_view message) noexcept {
          impl.fusion.note_error(message, "fuse thread: ");
        });
    if (!completed) {
      impl.frame_signal.wait_for(kIdleWait);
    }
    if (Clock::now() >= next_line) {
      (void)contain_frame_failure(
          [&] { report(impl, contract, window, started, false); },
          [&impl](std::string_view message) noexcept {
            impl.fusion.note_error(message, "fuse thread report: ");
          });
      next_line += every;
    }
  }
  (void)contain_frame_failure(
      [&] { report(impl, contract, window, started, true); },
      [&impl](std::string_view message) noexcept {
        impl.fusion.note_error(message, "fuse thread report: ");
      });
}

}  // namespace volumetric_kit::ios_app

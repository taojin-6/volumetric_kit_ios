// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "rgbd_stream_sender.hpp"

#include <CoreVideo/CoreVideo.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "MapSteps.hpp"
#include "RateGate.hpp"
#include "volumetric_kit/core/base/exception_status.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/sensor/apple/core_video_color.hpp"
#include "volumetric_kit/sensor/apple/pixel_buffer_importer.hpp"

namespace volumetric_kit::ios_app {

namespace {

using Clock = std::chrono::steady_clock;

// The most picture times kept for take_times.
constexpr std::size_t kMaxTimes = 1024;
// H.273's "unspecified" matrix, and the one ARKit documents for an untagged
// picture: BT.601.
constexpr int kUntagged = 2;
constexpr int kBt601 = 6;

core::Status refuse(const char* api, const std::string& why) {
  return core::Status::invalid_argument(std::string("RgbdStreamSender::") +
                                        api + ": " + why);
}

double ms(Clock::duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

// Whether @p plane can be copied from (@p from) or into, as a plane of
// @p format at least @p width x @p height.
bool copyable(const std::shared_ptr<const core::Image>& plane, VkFormat format,
              std::uint32_t width, std::uint32_t height, bool from) {
  if (plane == nullptr || !plane->valid() || plane->format() != format ||
      plane->width() < width || plane->height() < height) {
    return false;
  }
  const VkImageUsageFlags usage =
      from ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  const VkImageLayout optimal = from ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
                                     : VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  return (plane->usage() & usage) != 0 &&
         (plane->layout() == VK_IMAGE_LAYOUT_GENERAL ||
          plane->layout() == optimal);
}

// Records the copy of @p from's NV12 planes into @p to's, @p width x
// @p height from their corners, and a barrier that makes the writes visible
// to the host, as a CommandBatch's last one does.
void record_copy(VkCommandBuffer cmd, const sensor::YuvImage& from,
                 const sensor::YuvImage& to, std::uint32_t width,
                 std::uint32_t height) {
  VkMemoryBarrier before{};
  before.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
  before.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
  before.dstAccessMask =
      VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
  vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                       VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &before, 0,
                       nullptr, 0, nullptr);
  const VkExtent3D extent[2] = {{width, height, 1},
                                {(width + 1) / 2, (height + 1) / 2, 1}};
  for (int p = 0; p < 2; ++p) {
    VkImageCopy region{};
    region.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.dstSubresource = region.srcSubresource;
    region.extent = extent[p];
    vkCmdCopyImage(cmd, from.image[p]->handle(), from.image[p]->layout(),
                   to.image[p]->handle(), to.image[p]->layout(), 1, &region);
  }
  VkMemoryBarrier after{};
  after.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
  after.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
  after.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_HOST_READ_BIT;
  vkCmdPipelineBarrier(
      cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
      VK_PIPELINE_STAGE_ALL_COMMANDS_BIT | VK_PIPELINE_STAGE_HOST_BIT, 0, 1,
      &after, 0, nullptr, 0, nullptr);
}

}  // namespace

struct RgbdStreamSender::Impl {
  RgbdStreamSenderOptions options;
  sensor::SensorInfo info;

  // --- The thread that offers ------------------------------------------------
  MapSteps steps;
  RateGate gate{0};

  // --- Shared, under `mutex` ------------------------------------------------
  mutable std::mutex mutex;
  std::condition_variable wake;           // a frame handed over, or the stop
  std::condition_variable idle;           // nothing handed over is left
  std::optional<sensor::RgbdFrame> slot;  // a frame taken, waiting
  bool working = false;                   // the thread holds a frame
  bool stopping = false;
  bool finished = false;
  core::Status finish_status;
  std::optional<sensor::HevcEncoder::Fault> fault;
  // Set once, then kept until the sender goes: appended on the thread alone,
  // which reads it here at each frame.
  std::unique_ptr<RgbdStreamWriter> writer;
  RgbdStreamSenderStats counts;  // less `session`, read from the writer
  std::vector<PictureTimes> times;

  // One dial_to at a time, and none once the finish has begun: a session
  // opened by one is never made twice, nor after the finish. A finish holds
  // it only to begin, so a dial_to never waits out a finish.
  std::mutex dial_mutex;
  // One finish at a time: a second waits for the first's result.
  std::mutex finish_mutex;

  // --- The sender's thread --------------------------------------------------
  std::optional<sensor::HevcEncoder> encoder;
  std::uint32_t width = 0;  // the encoder's pictures', from the first
  std::uint32_t height = 0;
  // A coded frame the session refused is not in its stream, so the next
  // picture, which would predict from it, must be a key.
  bool key_next = false;
  sensor::HevcEncoder::CodedPicture coded;
  std::thread thread;

  // The session, opened with the options' path and target.
  core::Result<std::unique_ptr<RgbdStreamWriter>> open_session(
      const std::string& host, std::uint16_t port) const {
    RgbdStreamOptions o = options.session;
    o.dial_host = host;
    o.dial_port = port;
    o.color = true;
    return RgbdStreamWriter::open(o, info);
  }

  void run() noexcept;
  // One frame: its picture coded, then the frame written to `out`, if any.
  void send(const sensor::RgbdFrame& frame, RgbdStreamWriter* out,
            std::optional<sensor::HevcEncoder::Fault> injected);
  // The encoder, at @p picture's size and its pixel buffer's tags.
  core::Status open_encoder(const sensor::YuvImage& picture);
  // @p picture copied on the device into the encoder's next target, its
  // copy done when this returns.
  core::Status copy_to_target(const sensor::YuvImage& picture);
  // A frame taken that went nowhere.
  void fail(const core::Status& why) {
    const std::lock_guard<std::mutex> lock(mutex);
    ++counts.failed;
    counts.error = why.message();
  }
};

core::Status RgbdStreamSender::Impl::open_encoder(
    const sensor::YuvImage& picture) {
  CVPixelBufferRef pixels = sensor::pixel_buffer_of(picture);
  if (pixels == nullptr) {
    return refuse("send",
                  "the first picture has no pixel buffer to read its tags "
                  "from (pixel_buffer_of)");
  }
  core::Result<sensor::ColorCodes> tags = sensor::color_codes_of(pixels);
  if (!tags.ok()) {
    return tags.status().with_context("RgbdStreamSender: the first picture");
  }
  sensor::HevcEncoder::Options o;
  o.width = picture.width;
  o.height = picture.height;
  o.qp = options.qp;
  o.color = tags.value();
  if (o.color.matrix == kUntagged) o.color.matrix = kBt601;
  // The camera's device, which writes each target (copy_to_target).
  o.device = options.device;
  core::Result<sensor::HevcEncoder> made = sensor::HevcEncoder::create(o);
  if (!made.ok()) return made.status().with_context("RgbdStreamSender");
  encoder.emplace(std::move(made).value());
  width = o.width;
  height = o.height;
  return {};
}

core::Status RgbdStreamSender::Impl::copy_to_target(
    const sensor::YuvImage& picture) {
  if (picture.layout != sensor::YuvLayout::Nv12 || picture.width != width ||
      picture.height != height ||
      !copyable(picture.image[0], VK_FORMAT_R8_UNORM, width, height, true) ||
      !copyable(picture.image[1], VK_FORMAT_R8G8_UNORM, (width + 1) / 2,
                (height + 1) / 2, true)) {
    return refuse("send",
                  "the frame's picture is not NV12 plane images at the "
                  "stream's " +
                      std::to_string(width) + "x" + std::to_string(height) +
                      " that a copy reads");
  }
  core::CommandBatch batch(*options.device);
  VKC_ASSIGN(const sensor::YuvImage target, encoder->target(batch));
  if (!copyable(target.image[0], VK_FORMAT_R8_UNORM, width, height, false) ||
      !copyable(target.image[1], VK_FORMAT_R8G8_UNORM, (width + 1) / 2,
                (height + 1) / 2, false)) {
    return refuse("send", "the encoder's target is not NV12 plane images");
  }
  // What the target's planes need before their writes: on VideoToolbox,
  // nothing, so nothing is submitted.
  VKC_TRY(batch.submit());
  return options.device->submit_single_time([&](VkCommandBuffer cmd) {
    record_copy(cmd, picture, target, width, height);
  });
}

void RgbdStreamSender::Impl::send(
    const sensor::RgbdFrame& frame, RgbdStreamWriter* out,
    std::optional<sensor::HevcEncoder::Fault> injected) {
  if (!encoder) {
    const core::Status opened = open_encoder(frame.color);
    if (!opened.ok()) return fail(opened);
  }
  if (injected) encoder->inject(*injected);
  const auto pts = static_cast<std::int64_t>(frame.timestamp_ns);
  const Clock::time_point t0 = Clock::now();
  // The camera's picture is copied into one of the encoder's own, which is
  // what the encoder codes and keeps: it holds the last picture it coded
  // until it codes the next, and the camera's few pictures must go back.
  const core::Status copied = copy_to_target(frame.color);
  if (!copied.ok()) return fail(copied);
  // A session's first frame is a key, as is the first after a reader asked;
  // decided after the target, which opens a lost session again.
  bool key = encoder->key_owed() || key_next;
  if (!key && out != nullptr) {
    key = out->stats().frames == 0 || out->key_wanted();
  }
  // The copy is done: nothing for the encoder to wait for.
  const core::Status sent = encoder->send(key, pts, {});
  const Clock::time_point t1 = Clock::now();
  if (!sent.ok()) return fail(sent);
  const core::Result<bool> got = encoder->receive(coded);
  const Clock::time_point t2 = Clock::now();
  // Dropped or failed: neither record. The encoder owes a key from here.
  if (!got.ok()) return fail(got.status());
  if (!got.value()) {
    return fail(refuse("send", "no access unit came back for the picture"));
  }
  {
    const std::lock_guard<std::mutex> lock(mutex);
    ++counts.coded;
    if (coded.key) ++counts.keys;
    counts.picture_bytes += coded.access_unit.size();
    if (times.size() == kMaxTimes) times.erase(times.begin());  // the last
    times.push_back({ms(t1 - t0), ms(t2 - t0)});
  }
  if (coded.key) key_next = false;
  if (out == nullptr) return;  // the dry run: coded and counted
  const CodedColor color{coded.access_unit.data(), coded.access_unit.size(),
                         coded.key, encoder->facts().color};
  const core::Status written = out->append(frame, &color);
  // A frame the session refused is not in its stream: the next picture
  // cannot predict from it.
  if (!written.ok()) key_next = true;
  const std::lock_guard<std::mutex> lock(mutex);
  if (written.ok()) {
    ++counts.written;
  } else {
    ++counts.refused;
    counts.error = written.message();
  }
}

void RgbdStreamSender::Impl::run() noexcept {
  for (;;) {
    std::optional<sensor::RgbdFrame> frame;
    RgbdStreamWriter* out = nullptr;
    std::optional<sensor::HevcEncoder::Fault> injected;
    {
      std::unique_lock<std::mutex> lock(mutex);
      wake.wait(lock, [this] { return slot.has_value() || stopping; });
      if (!slot) break;  // stopped, with nothing left to send
      frame = std::move(slot);
      slot.reset();
      working = true;
      out = writer.get();
      injected = std::exchange(fault, std::nullopt);
    }
    try {
      send(*frame, out, injected);
    } catch (...) {
      fail(core::exception_status("RgbdStreamSender::send",
                                  core::Status::Code::OutOfMemory));
    }
    frame.reset();  // the camera's picture goes back before the thread waits
    {
      const std::lock_guard<std::mutex> lock(mutex);
      working = false;
    }
    idle.notify_all();
  }
}

RgbdStreamSender::RgbdStreamSender(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

RgbdStreamSender::~RgbdStreamSender() {
  if (impl_) (void)finish();
}

core::Result<std::unique_ptr<RgbdStreamSender>> RgbdStreamSender::open(
    const RgbdStreamSenderOptions& options,
    const sensor::SensorInfo& info) try {
  if (!info.color) return refuse("open", "a sensor without a colour camera");
  if (options.qp < 0 || options.qp > 51) {
    return refuse("open",
                  "a QP of " + std::to_string(options.qp) + ", not 0 to 51");
  }
  if (options.device == nullptr) {
    return refuse("open", "no device to copy the pictures on");
  }
  if (options.dry &&
      (!options.session.path.empty() || !options.session.dial_host.empty())) {
    return refuse("open", "a dry run with a file or a reader to send to");
  }
  auto impl = std::make_unique<Impl>();
  impl->options = options;
  impl->info = info;
  impl->gate = RateGate(options.session.fps);
  if (!options.session.path.empty() || !options.session.dial_host.empty()) {
    VKC_ASSIGN(impl->writer, impl->open_session(options.session.dial_host,
                                                options.session.dial_port));
  }
  Impl* s = impl.get();
  try {
    impl->thread = std::thread([s] { s->run(); });
  } catch (...) {
    return core::Status::io_error(
        "RgbdStreamSender::open: the sender's thread did not start");
  }
  return std::unique_ptr<RgbdStreamSender>(
      new RgbdStreamSender(std::move(impl)));
} catch (...) {
  return core::exception_status("RgbdStreamSender::open",
                                core::Status::Code::OutOfMemory);
}

void RgbdStreamSender::offer(const sensor::RgbdFrame& frame) {
  Impl& s = *impl_;
  // Every pose, before the gate: a step is found from the frames around it.
  const glm::dmat4 pose =
      s.steps.take(frame.timestamp_ns, frame.color_to_world);
  bool handed = false;
  {
    const std::lock_guard<std::mutex> lock(s.mutex);
    ++s.counts.offered;
    s.counts.map_steps = s.steps.steps().size();
    if (s.stopping || !s.gate.due(frame.timestamp_ns)) return;
    if (s.slot) {
      ++s.counts.busy;  // the gate keeps the interval for the next frame
      return;
    }
    (void)s.gate.take(frame.timestamp_ns);
    ++s.counts.taken;
    s.slot = frame;
    s.slot->color_to_world = pose;
    handed = true;
  }
  if (handed) s.wake.notify_one();
}

core::Status RgbdStreamSender::dial_to(const std::string& host,
                                       std::uint16_t port) try {
  Impl& s = *impl_;
  const std::lock_guard<std::mutex> dialing(s.dial_mutex);
  RgbdStreamWriter* current = nullptr;
  {
    const std::lock_guard<std::mutex> lock(s.mutex);
    if (s.stopping) return refuse("dial_to", "the sender is finished");
    current = s.writer.get();
  }
  if (current != nullptr) return current->dial_to(host, port);
  if (host.empty()) return {};  // no session, and none wanted
  if (s.options.dry) {
    return refuse("dial_to",
                  "this launch is a dry run (-dry): it records and sends "
                  "nothing");
  }
  // Opened outside the lock the camera's thread takes; finish waits for it.
  VKC_ASSIGN(std::unique_ptr<RgbdStreamWriter> opened,
             s.open_session(host, port));
  const std::lock_guard<std::mutex> lock(s.mutex);
  s.writer = std::move(opened);
  return {};
} catch (...) {
  return core::exception_status("RgbdStreamSender::dial_to",
                                core::Status::Code::OutOfMemory);
}

RgbdStreamSenderStats RgbdStreamSender::stats() const {
  const Impl& s = *impl_;
  const std::lock_guard<std::mutex> lock(s.mutex);
  RgbdStreamSenderStats out = s.counts;
  out.session_open = s.writer != nullptr;
  if (s.writer) out.session = s.writer->stats();
  return out;
}

std::optional<grid::networking::ViewerNote> RgbdStreamSender::reader_note()
    const {
  const Impl& s = *impl_;
  const std::lock_guard<std::mutex> lock(s.mutex);
  if (!s.writer) return std::nullopt;
  return s.writer->reader_note();
}

std::vector<PictureTimes> RgbdStreamSender::take_times() {
  Impl& s = *impl_;
  std::vector<PictureTimes> out;
  const std::lock_guard<std::mutex> lock(s.mutex);
  out.swap(s.times);
  return out;
}

core::Status RgbdStreamSender::finish() {
  Impl& s = *impl_;
  const std::lock_guard<std::mutex> finishing(s.finish_mutex);
  {
    // Past a dial_to under way, which may still open the session; every
    // later one is refused at once.
    const std::lock_guard<std::mutex> dialing(s.dial_mutex);
    const std::lock_guard<std::mutex> lock(s.mutex);
    if (s.finished) return s.finish_status;
    s.stopping = true;
  }
  s.wake.notify_all();
  if (s.thread.joinable()) s.thread.join();
  // The thread is gone: nothing appends, and the writer is this thread's.
  core::Status done = s.writer ? s.writer->finish() : core::Status{};
  const std::lock_guard<std::mutex> lock(s.mutex);
  s.finished = true;
  s.finish_status = done;
  return done;
}

void RgbdStreamSender::flush() {
  Impl& s = *impl_;
  std::unique_lock<std::mutex> lock(s.mutex);
  s.idle.wait(lock, [&s] { return !s.slot && !s.working; });
}

void RgbdStreamSender::inject(sensor::HevcEncoder::Fault fault) {
  Impl& s = *impl_;
  const std::lock_guard<std::mutex> lock(s.mutex);
  s.fault = fault;
}

}  // namespace volumetric_kit::ios_app

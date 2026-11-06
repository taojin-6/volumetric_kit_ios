// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file rgbd_stream_sender_test.cpp
/// @brief The stream sender on this Mac's GPU and VideoToolbox: sensor's
///        `ArkitSensor` with its test pattern (`ArkitSource::Pattern`, the
///        iPad's own driver, no camera) through the sender into a session,
///        read back with grid's reader and sensor's wire form, every picture
///        decoded on VideoToolbox and its index read on the GPU
///        (`PatternReader`) against its depth record's.
///
/// Skips without a Vulkan device or a hardware HEVC encoder, unless
/// `VKC_REQUIRE_VULKAN_DEVICE` or `VS_TEST_HEVC_ENCODER` says this machine
/// has one.
///
/// `PatternSessionForTheLab` runs only with `VI_TEST_SESSION_OUT=<path>`
/// and/or `VI_TEST_LAB_HOST=<a.b.c.d:port>`, or `VI_TEST_DRY=1` for neither:
/// the iPad's stream, ARKit's 1920x1440 pattern at 60 Hz through the 10 Hz
/// gate, written to the path and dialled to the host (or coded and counted)
/// for `VI_TEST_FRAMES` frames (100), for recon's `tools/session_check`.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "TrackingLine.hpp"
#include "dial_entry.hpp"
#include "rgbd_stream_sender.hpp"
#include "session_test_support.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/core/testing/vulkan_fixture.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/grid/networking/stream_client.hpp"
#include "volumetric_kit/sensor/arkit/arkit_sensor.hpp"
#include "volumetric_kit/sensor/testing/hevc_clip.hpp"
#include "volumetric_kit/sensor/testing/index_band.hpp"
#include "volumetric_kit/sensor/testing/pattern_reader.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"
#include "volumetric_kit/sensor/video/hevc_decoder.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace {

namespace app = volumetric_kit::ios_app;
namespace net = volumetric_kit::core::net;
namespace networking = volumetric_kit::grid::networking;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
namespace vkc = volumetric_kit::core;
namespace wire = volumetric_kit::sensor::wire;

using ios_test::read_file;
using ios_test::ReadFile;
using ios_test::ReadRecord;
using ios_test::TempFile;
using namespace std::chrono_literals;

constexpr std::uint64_t kBand = std::uint64_t{1} << testing::kIndexBits;
// The smallest picture whose index band is whole, as small as the tests go.
constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;
// The pattern's rate here: fast, so a test takes its frames in tens of ms.
constexpr std::uint32_t kPatternHz = 120;
// What ARKit's untagged picture is, as the pattern tags its own: BT.601
// matrix, BT.709 transfer and primaries, full range, left-sited.
constexpr int kMatrix = 6;

bool env_set(const char* name) {
  const char* value = std::getenv(name);
  return value != nullptr && *value != '\0';
}

// Waits on the sensor's notification, which owns what it touches.
class Waker {
 public:
  std::function<void()> notify() {
    return [state = state_] {
      const std::lock_guard<std::mutex> lock(state->mutex);
      ++state->count;
      state->woken.notify_one();
    };
  }
  // Until a notification past `seen`, or `limit`; the count then.
  std::uint64_t wait(std::uint64_t seen, std::chrono::milliseconds limit) {
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->woken.wait_for(lock, limit, [&] { return state_->count != seen; });
    return state_->count;
  }

 private:
  struct State {
    std::mutex mutex;
    std::condition_variable woken;
    std::uint64_t count = 0;
  };
  std::shared_ptr<State> state_ = std::make_shared<State>();
};

// What the session holds of one frame, read back.
struct SentFrame {
  std::uint64_t set = 0;
  std::uint64_t pts = 0;
  bool key = false;
  std::optional<std::uint32_t> depth_index;
  std::optional<std::uint32_t> picture_index;
  sensor::ColorCodes codes;
  sensor::ColorExposure exposure;
  sensor::camera::Mat4d pose{1.0};
};

class RgbdStreamSenderTest : public vkc::test::VulkanDeviceTest {
 protected:
  // The camera's device, which writes the encoder's pictures too.
  vkc::DeviceRequirements requirements() const override {
    return sensor::hevc_encoder_device_requirements();
  }

  void SetUp() override {
    VulkanDeviceTest::SetUp();
    if (base_setup_incomplete()) return;
    sensor::HevcEncoder::Options probe;
    probe.width = kWidth;
    probe.height = kHeight;
    probe.color = {kMatrix, 1, 1, true, 0};
    vkc::Result<sensor::HevcEncoder> encoder =
        sensor::HevcEncoder::create(probe);
    if (!encoder.ok()) {
      ASSERT_FALSE(testing::hevc_encoder_promised())
          << encoder.status().message();
      GTEST_SKIP() << "no hardware HEVC encoder: "
                   << encoder.status().message();
    }
  }

  // The pattern, started: frames at kPatternHz and kWidth x kHeight.
  sensor::ArkitSensor& camera(std::uint32_t width = kWidth,
                              std::uint32_t height = kHeight,
                              std::uint32_t fps = kPatternHz) {
    sensor::ArkitSensor::Options o;
    o.source = sensor::ArkitSource::Pattern;
    o.device = &device();
    o.color_width = width;
    o.color_height = height;
    o.fps = fps;
    vkc::Result<sensor::ArkitSensor> opened = sensor::ArkitSensor::open(o);
    EXPECT_TRUE(opened.ok()) << opened.status().message();
    camera_.emplace(std::move(opened).value());
    EXPECT_TRUE(camera_->set_frame_notify(waker_.notify()).ok());
    EXPECT_TRUE(camera_->start().ok());
    return *camera_;
  }

  // The camera's next frame, waiting on its notification; none within 2 s.
  std::optional<sensor::RgbdFrame> next() {
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < end) {
      vkc::Result<std::optional<sensor::RgbdFrame>> got = camera_->poll();
      EXPECT_TRUE(got.ok()) << got.status().message();
      if (!got.ok()) return std::nullopt;
      if (got.value()) return std::move(got).value();
      seen_ = waker_.wait(seen_, 100ms);
    }
    ADD_FAILURE() << "no frame from the pattern within 2 s";
    return std::nullopt;
  }

  // @p count frames offered one at a time, each sent before the next, with
  // its exposure set from its sequence; their sequences, in order.
  std::vector<std::uint64_t> offer(
      app::RgbdStreamSender& sender, std::size_t count,
      const std::function<void(std::size_t)>& before = {}) {
    std::vector<std::uint64_t> sequences;
    for (std::size_t k = 0; k < count; ++k) {
      std::optional<sensor::RgbdFrame> frame = next();
      if (!frame) break;
      frame->color_exposure = {0.004f + 1e-4f * static_cast<float>(k),
                               100.0f + static_cast<float>(frame->sequence),
                               -0.5f};
      if (before) before(k);
      sender.offer(*frame);
      sender.flush();
      sequences.push_back(frame->sequence);
    }
    return sequences;
  }

  // The session at @p path read back: each frame's records, the depth's
  // index read on the host, the picture decoded on VideoToolbox and its
  // index read on the GPU.
  std::vector<SentFrame> read_back(const std::string& path) {
    std::vector<SentFrame> out;
    const ReadFile read = read_file(path);
    EXPECT_TRUE(read.finished);
    EXPECT_EQ(read.session.tracks.size(), 2u);
    if (read.session.tracks.size() != 2) return out;
    EXPECT_EQ(read.session.tracks[0].kind, stream::TrackKind::RgbdColor);
    EXPECT_EQ(read.session.tracks[1].kind, stream::TrackKind::RgbdDepth);
    auto info = wire::decode_sensor(read.session.tracks[1].params,
                                    read.session.tracks[1].sensor,
                                    sensor::ClockDomain::Host, 0);
    EXPECT_TRUE(info.ok()) << info.status().message();
    if (!info.ok()) return out;
    out = frames_of(read.records, info.value());
    return out;
  }

  std::vector<SentFrame> frames_of(const std::vector<ReadRecord>& records,
                                   const sensor::SensorInfo& info) {
    std::vector<SentFrame> out;
    sensor::HevcDecoder::Options d;
    d.device = &device();
    d.allocator = &allocator();
    vkc::Result<sensor::HevcDecoder> decoder = sensor::HevcDecoder::create(d);
    EXPECT_TRUE(decoder.ok()) << decoder.status().message();
    auto reader = testing::PatternReader::create(device(), allocator());
    EXPECT_TRUE(reader.ok()) << reader.status().message();
    if (!decoder.ok() || !reader.ok()) return out;
    std::map<std::int64_t, std::optional<std::uint32_t>> pictures;  // by pts
    const auto drain = [&] {
      for (;;) {
        auto picture = decoder.value().receive();
        EXPECT_TRUE(picture.ok()) << picture.status().message();
        if (!picture.ok() || !picture.value()) return;
        auto index = reader.value()->read(picture.value()->yuv);
        EXPECT_TRUE(index.ok()) << index.status().message();
        pictures[picture.value()->pts] =
            index.ok() ? index.value() : std::nullopt;
      }
    };
    for (const ReadRecord& r : records) {
      if (r.info.track == 0) {
        auto colour =
            wire::decode_color(info, r.payload.data(), r.payload.size());
        EXPECT_TRUE(colour.ok()) << colour.status().message();
        if (!colour.ok()) continue;
        SentFrame f;
        f.set = r.info.set;
        f.pts = r.info.pts_ns;
        f.key = r.info.key;
        f.codes = colour.value().codes;
        f.exposure = colour.value().exposure;
        out.push_back(f);
        EXPECT_TRUE(decoder.value()
                        .send(colour.value().picture,
                              colour.value().picture_bytes,
                              static_cast<std::int64_t>(r.info.pts_ns))
                        .ok());
        drain();
      } else {
        auto bytes = std::make_shared<std::vector<std::uint8_t>>(r.payload);
        auto depth =
            wire::decode_depth(info, bytes->data(), bytes->size(), bytes);
        EXPECT_TRUE(depth.ok()) << depth.status().message();
        EXPECT_FALSE(out.empty());
        if (!depth.ok() || out.empty()) continue;
        EXPECT_EQ(out.back().set, r.info.set);  // its colour, then its depth
        EXPECT_EQ(out.back().pts, r.info.pts_ns);
        out.back().depth_index = testing::depth_index(
            depth.value().depth, depth.value().depth_camera.size.width,
            depth.value().depth_camera.size.height);
        out.back().pose = depth.value().color_to_world;
      }
    }
    EXPECT_TRUE(decoder.value().send(nullptr, 0, 0).ok());
    drain();
    for (SentFrame& f : out) {
      const auto found = pictures.find(static_cast<std::int64_t>(f.pts));
      EXPECT_NE(found, pictures.end()) << "no picture decoded at set " << f.set;
      if (found != pictures.end()) f.picture_index = found->second;
    }
    return out;
  }

  app::RgbdStreamSenderOptions options(const std::string& path) {
    app::RgbdStreamSenderOptions o;
    o.session = ios_test::options_for(path);
    o.session.fps = 0;  // every frame offered
    o.device = &device();
    return o;
  }

  std::unique_ptr<app::RgbdStreamSender> open(
      const app::RgbdStreamSenderOptions& o) {
    auto sender = app::RgbdStreamSender::open(o, camera_->info());
    EXPECT_TRUE(sender.ok()) << sender.status().message();
    return sender.ok() ? std::move(sender).value() : nullptr;
  }

  void TearDown() override {
    camera_.reset();  // before the device the fixture shares
    VulkanDeviceTest::TearDown();
  }

 private:
  std::optional<sensor::ArkitSensor> camera_;
  Waker waker_;
  std::uint64_t seen_ = 0;
};

/// Every frame goes whole, its colour record then its depth record, and
/// carries its own picture: decoded on VideoToolbox, the picture's index is
/// its depth's, which is the frame's sequence. The first picture is a key,
/// the records state the stream's colour, and each frame's exposure goes as
/// the frame states it.
TEST_F(RgbdStreamSenderTest, EachFrameGoesWholeWithItsOwnPicture) {
  TempFile file("sender_frames");
  camera();
  auto sender = open(options(file.path()));
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kFrames = 10;
  const std::vector<std::uint64_t> sequences = offer(*sender, kFrames);
  ASSERT_EQ(sequences.size(), kFrames);
  const app::RgbdStreamSenderStats stats = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_EQ(stats.offered, kFrames);
  EXPECT_EQ(stats.taken, kFrames);
  EXPECT_EQ(stats.coded, kFrames);
  EXPECT_EQ(stats.written, kFrames);
  EXPECT_EQ(stats.failed, 0u);
  EXPECT_EQ(stats.busy, 0u);
  EXPECT_EQ(stats.keys, 1u);
  EXPECT_TRUE(stats.session_open);
  EXPECT_EQ(stats.session.frames, kFrames);
  EXPECT_TRUE(stats.error.empty()) << stats.error;
  EXPECT_EQ(sender->take_times().size(), kFrames);

  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), kFrames);
  for (std::size_t k = 0; k < kFrames; ++k) {
    const SentFrame& f = sent[k];
    EXPECT_EQ(f.set, k);
    EXPECT_EQ(f.key, k == 0) << "frame " << k;
    ASSERT_TRUE(f.depth_index.has_value()) << "frame " << k;
    EXPECT_EQ(*f.depth_index, sequences[k] % kBand) << "frame " << k;
    ASSERT_TRUE(f.picture_index.has_value()) << "frame " << k;
    EXPECT_EQ(*f.picture_index, *f.depth_index) << "frame " << k;
    EXPECT_EQ(f.codes.matrix, kMatrix);
    EXPECT_EQ(f.codes.transfer, 1);
    EXPECT_EQ(f.codes.primaries, 1);
    EXPECT_TRUE(f.codes.full_range);
    EXPECT_EQ(f.exposure.duration_s, 0.004f + 1e-4f * static_cast<float>(k));
    EXPECT_EQ(f.exposure.iso, 100.0f + static_cast<float>(sequences[k]));
    EXPECT_EQ(f.exposure.bias_ev, -0.5f);
  }
}

/// ARKit's map steps come out of the poses before they go: a 127 mm jump
/// under a still camera is not in any pose sent.
TEST_F(RgbdStreamSenderTest, AMapStepIsOutOfThePosesSent) {
  TempFile file("sender_steps");
  camera();
  auto sender = open(options(file.path()));
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kFrames = 8;
  for (std::size_t k = 0; k < kFrames; ++k) {
    std::optional<sensor::RgbdFrame> frame = next();
    ASSERT_TRUE(frame.has_value());
    if (k >= 4) frame->color_to_world[3][0] = 0.127;  // the map moved
    sender->offer(*frame);
    sender->flush();
  }
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_GE(sender->stats().map_steps, 1u);
  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), kFrames);
  for (const SentFrame& f : sent) {
    EXPECT_NEAR(f.pose[3][0], 0.0, 1e-3) << "set " << f.set;
  }
}

/// A picture the encoder drops leaves its frame out whole: neither record,
/// no gap in the session, the next picture a key, and the drop counted.
TEST_F(RgbdStreamSenderTest, ADroppedPictureLeavesItsFrameOutWhole) {
  TempFile file("sender_drop");
  camera();
  auto sender = open(options(file.path()));
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kFrames = 7;
  constexpr std::size_t kDropped = 3;
  const std::vector<std::uint64_t> sequences =
      offer(*sender, kFrames, [&](std::size_t k) {
        if (k == kDropped) sender->inject(sensor::HevcEncoder::Fault::Drop);
      });
  ASSERT_EQ(sequences.size(), kFrames);
  const app::RgbdStreamSenderStats stats = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_EQ(stats.taken, kFrames);
  EXPECT_EQ(stats.failed, 1u);
  EXPECT_EQ(stats.coded, kFrames - 1);
  EXPECT_EQ(stats.written, kFrames - 1);
  EXPECT_EQ(stats.keys, 2u);
  EXPECT_FALSE(stats.error.empty());

  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), kFrames - 1);
  for (std::size_t k = 0; k < sent.size(); ++k) {
    const std::size_t offered = k < kDropped ? k : k + 1;
    EXPECT_EQ(sent[k].set, k);
    EXPECT_EQ(sent[k].key, k == 0 || k == kDropped) << "set " << k;
    ASSERT_TRUE(sent[k].depth_index.has_value());
    EXPECT_EQ(*sent[k].depth_index, sequences[offered] % kBand) << "set " << k;
    EXPECT_EQ(sent[k].picture_index, sent[k].depth_index) << "set " << k;
  }
}

/// A coded frame the session refuses is not in the stream, so the next
/// picture, which would predict from it, is a key.
TEST_F(RgbdStreamSenderTest, ARefusedFrameMakesTheNextPictureAKey) {
  TempFile file("sender_refused");
  camera();
  auto sender = open(options(file.path()));
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kFrames = 6;
  constexpr std::size_t kRefused = 2;
  for (std::size_t k = 0; k < kFrames; ++k) {
    std::optional<sensor::RgbdFrame> frame = next();
    ASSERT_TRUE(frame.has_value());
    // A depth camera of another size: encode_depth refuses the frame.
    if (k == kRefused) frame->depth_camera.size.width /= 2;
    sender->offer(*frame);
    sender->flush();
  }
  const app::RgbdStreamSenderStats stats = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_EQ(stats.coded, kFrames);
  EXPECT_EQ(stats.refused, 1u);
  EXPECT_EQ(stats.written, kFrames - 1);
  EXPECT_EQ(stats.keys, 2u);
  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), kFrames - 1);
  for (std::size_t k = 0; k < sent.size(); ++k) {
    EXPECT_EQ(sent[k].key, k == 0 || k == kRefused) << "set " << k;
    EXPECT_EQ(sent[k].picture_index, sent[k].depth_index) << "set " << k;
  }
}

/// With neither a file nor a reader the sender codes and counts every frame
/// taken and writes nothing (the dry run).
TEST_F(RgbdStreamSenderTest, TheDryRunCodesAndCountsAndWritesNothing) {
  TempFile file("sender_dry");
  camera();
  app::RgbdStreamSenderOptions o = options("");
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kFrames = 5;
  ASSERT_EQ(offer(*sender, kFrames).size(), kFrames);
  const app::RgbdStreamSenderStats stats = sender->stats();
  EXPECT_EQ(stats.coded, kFrames);
  EXPECT_EQ(stats.keys, 1u);
  EXPECT_GT(stats.picture_bytes, 0u);
  EXPECT_EQ(stats.written, 0u);
  EXPECT_FALSE(stats.session_open);
  const std::vector<app::PictureTimes> times = sender->take_times();
  ASSERT_EQ(times.size(), kFrames);
  for (const app::PictureTimes& t : times) {
    EXPECT_GE(t.coded_ms, t.send_ms);
  }
  EXPECT_TRUE(sender->take_times().empty());
  EXPECT_TRUE(sender->finish().ok());
  EXPECT_FALSE(std::filesystem::exists(file.path()));
}

/// The on-screen connect opens the session on a dry sender, whose first
/// frame is a key; a reader that joins again later asks for a key, and the
/// next picture is one.
TEST_F(RgbdStreamSenderTest, AConnectOpensTheSessionAndAReaderGetsAKey) {
  auto listener = net::Listener::create("127.0.0.1", 0);
  ASSERT_TRUE(listener.ok()) << listener.status().message();
  const std::uint16_t port = listener.value()->port();
  camera();
  auto sender = open(options(""));
  ASSERT_NE(sender, nullptr);
  EXPECT_FALSE(sender->reader_note().has_value());  // no session yet
  ASSERT_EQ(offer(*sender, 3).size(), 3u);          // dry: one key, then P

  const auto take_reader = [&]() -> std::unique_ptr<networking::StreamClient> {
    const auto end = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < end) {
      auto fd = listener.value()->accept(50ms);
      if (!fd.ok() || !fd.value().valid()) continue;
      auto client = networking::StreamClient::adopt(std::move(fd).value());
      if (client.ok()) return std::move(client).value();
    }
    return nullptr;
  };
  const auto served = [&](std::size_t viewers) {
    const auto end = std::chrono::steady_clock::now() + 5s;
    while (sender->stats().session.viewers != viewers) {
      if (std::chrono::steady_clock::now() >= end) return false;
      std::this_thread::yield();
    }
    return true;
  };
  // The first key the reader reads, within 5 s.
  const auto first_record_is_a_key = [](networking::StreamClient& client) {
    stream::Record record;
    std::vector<std::uint8_t> payload;
    const auto end = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < end) {
      const vkc::Result<bool> got = client.next(20ms, &record, &payload);
      if (!got.ok()) return false;
      if (got.value()) return record.info.track == 0 && record.info.key;
    }
    return false;
  };

  ASSERT_TRUE(sender->dial_to("127.0.0.1", port).ok());
  std::unique_ptr<networking::StreamClient> first = take_reader();
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(served(1));
  ASSERT_EQ(offer(*sender, 3).size(), 3u);
  EXPECT_TRUE(first_record_is_a_key(*first));
  EXPECT_EQ(sender->stats().keys, 2u);  // the dry run's, and the session's

  // What the reader says comes out of the sender, and goes with the reader.
  EXPECT_FALSE(sender->reader_note().has_value());
  networking::ViewerNote said;
  said.line = "Looking for the tag";
  ASSERT_TRUE(first->send_note(said).ok());
  const auto said_by = std::chrono::steady_clock::now() + 5s;
  while (!sender->reader_note() && std::chrono::steady_clock::now() < said_by) {
    std::this_thread::yield();
  }
  ASSERT_TRUE(sender->reader_note().has_value());
  EXPECT_EQ(sender->reader_note()->line, said.line);

  // The reader goes, and another joins: it asks, and gets, a key.
  ASSERT_TRUE(sender->dial_to("", 0).ok());
  ASSERT_TRUE(served(0));
  EXPECT_FALSE(sender->reader_note().has_value());
  first.reset();
  ASSERT_TRUE(sender->dial_to("127.0.0.1", port).ok());
  std::unique_ptr<networking::StreamClient> second = take_reader();
  ASSERT_NE(second, nullptr);
  ASSERT_TRUE(served(1));
  ASSERT_EQ(offer(*sender, 2).size(), 2u);
  EXPECT_TRUE(first_record_is_a_key(*second));
  const app::RgbdStreamSenderStats stats = sender->stats();
  EXPECT_EQ(stats.keys, 3u);
  EXPECT_EQ(stats.written, 5u);
  EXPECT_TRUE(sender->finish().ok());
}

/// Core names no sensor type, so it reads `to_string`'s names: each has its
/// words, and normal alone reads as normal.
TEST(TrackingLineOfSensorNames, EachStateHasItsOwnWords) {
  using Tracking = sensor::ArkitTracking;
  struct Named {
    Tracking tracking;
    const char* words;
  };
  for (const Named& named :
       {Named{Tracking::NotAvailable, "No tracking"},
        Named{Tracking::Initializing, "Initializing"},
        Named{Tracking::Relocalizing, "Relocalizing"},
        Named{Tracking::ExcessiveMotion, "Moving too fast"},
        Named{Tracking::InsufficientFeatures, "Too little in view"},
        Named{Tracking::Normal, "normal"}}) {
    EXPECT_EQ(
        app::tracking_line(sensor::to_string(named.tracking), false, true).text,
        named.words);
  }
}

/// The sender refuses a sensor with no picture, a QP past 51, no device to
/// copy on, and a dry run with a file or a reader.
TEST_F(RgbdStreamSenderTest, OpenRefusesWhatItCannotSend) {
  const sensor::ArkitSensor& pattern = camera();
  sensor::SensorInfo depth_only = pattern.info();
  depth_only.color.reset();
  auto none = app::RgbdStreamSender::open(options(""), depth_only);
  EXPECT_EQ(none.status().domain(), vkc::Status::Code::InvalidArgument);
  app::RgbdStreamSenderOptions o = options("");
  o.qp = 52;
  auto past = app::RgbdStreamSender::open(o, pattern.info());
  EXPECT_EQ(past.status().domain(), vkc::Status::Code::InvalidArgument);
  o = options("");
  o.device = nullptr;
  auto no_device = app::RgbdStreamSender::open(o, pattern.info());
  EXPECT_EQ(no_device.status().domain(), vkc::Status::Code::InvalidArgument);
  const TempFile file("sender_dry_file");
  o = options(file.path());
  o.dry = true;
  auto dry_file = app::RgbdStreamSender::open(o, pattern.info());
  EXPECT_EQ(dry_file.status().domain(), vkc::Status::Code::InvalidArgument);
  EXPECT_FALSE(std::filesystem::exists(file.path()));
  o = options("");
  o.dry = true;
  o.session.dial_host = "127.0.0.1";
  auto dry_dial = app::RgbdStreamSender::open(o, pattern.info());
  EXPECT_EQ(dry_dial.status().domain(), vkc::Status::Code::InvalidArgument);
}

/// A dry launch stays one: it codes and counts, and refuses every connect,
/// so nothing it sees leaves the device.
TEST_F(RgbdStreamSenderTest, ADryRunRefusesEveryConnect) {
  camera();
  app::RgbdStreamSenderOptions o = options("");
  o.dry = true;
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  ASSERT_EQ(offer(*sender, 2).size(), 2u);
  const vkc::Status dialled = sender->dial_to("127.0.0.1", 47801);
  EXPECT_EQ(dialled.domain(), vkc::Status::Code::InvalidArgument);
  EXPECT_NE(dialled.message().find("dry run"), std::string::npos)
      << dialled.message();
  EXPECT_TRUE(sender->dial_to("", 0).ok());  // nothing to let go
  const app::RgbdStreamSenderStats stats = sender->stats();
  EXPECT_EQ(stats.coded, 2u);
  EXPECT_FALSE(stats.session_open);
  EXPECT_TRUE(sender->finish().ok());
}

/// The scanner holds its last 3 frames (its keyframe ring) while the sender
/// codes 1 frame in 12: the pattern's 4 pictures keep coming, since the
/// encoder keeps a picture of its own after coding, not the camera's.
TEST_F(RgbdStreamSenderTest, FramesHeldBesideTheStreamDoNotStopTheCamera) {
  camera();
  app::RgbdStreamSenderOptions o = options("");
  o.session.fps = 10;
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  constexpr std::size_t kHeld = 3;
  constexpr std::size_t kFrames = 40;  // a third of a second at 120 Hz
  std::vector<sensor::RgbdFrame> ring;
  for (std::size_t k = 0; k < kFrames; ++k) {
    std::optional<sensor::RgbdFrame> frame = next();
    ASSERT_TRUE(frame.has_value()) << "the camera stopped at frame " << k;
    sender->offer(*frame);
    ring.push_back(std::move(*frame));
    if (ring.size() > kHeld) ring.erase(ring.begin());
  }
  sender->flush();
  const app::RgbdStreamSenderStats stats = sender->stats();
  EXPECT_GE(stats.coded, 3u);
  EXPECT_EQ(stats.failed, 0u) << stats.error;
  ring.clear();
  EXPECT_TRUE(sender->finish().ok());
}

/// A frame due while the frame taken before still waits for the sender's
/// thread is left (busy), and leaves the gate its interval: the next frame of
/// that interval is taken. The device is held from the host so that the
/// sender's thread waits on it with a frame in hand.
TEST_F(RgbdStreamSenderTest, AFrameLeftWhileBusyLeavesTheGateItsInterval) {
  TempFile file("sender_busy");
  sensor::ArkitSensor& pattern = camera();
  app::RgbdStreamSenderOptions o = options(file.path());
  o.session.fps = 10;
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  std::optional<sensor::RgbdFrame> a = next();
  std::optional<sensor::RgbdFrame> b = next();
  ASSERT_TRUE(a.has_value() && b.has_value());
  pattern.stop();  // its frames are written on the device too
  constexpr std::uint64_t kMs = 1'000'000;
  const std::uint64_t t0 = a->timestamp_ns;
  const auto at = [&](std::uint64_t ms) {
    sensor::RgbdFrame f = *b;
    f.timestamp_ns = t0 + ms * kMs;
    return f;
  };

  vkc::Result<vkc::TimelineSemaphore> held =
      vkc::TimelineSemaphore::create(device());
  ASSERT_TRUE(held.ok()) << held.status().message();
  vkc::Result<vkc::Device::PendingSubmit> hold =
      device().submit_pending([](VkCommandBuffer) {}, {{&held.value(), 1}}, {});
  ASSERT_TRUE(hold.ok()) << hold.status().message();
  const std::uint64_t submits = device().counters().submits;
  sender->offer(*a);  // the first: taken
  // Until the sender's thread has it, and waits on the device for it.
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (device().counters().submits == submits &&
         std::chrono::steady_clock::now() < end) {
    std::this_thread::yield();
  }
  ASSERT_GT(device().counters().submits, submits);
  sender->offer(at(100));  // interval 1: taken, waiting
  sender->offer(at(200));  // interval 2, due: busy
  sender->offer(at(210));  // interval 2 still untaken: busy
  const app::RgbdStreamSenderStats waiting = sender->stats();
  EXPECT_EQ(waiting.taken, 2u);
  EXPECT_EQ(waiting.busy, 2u);
  ASSERT_TRUE(held.value().signal(1).ok());
  ASSERT_TRUE(hold.value().wait().ok());
  sender->flush();
  sender->offer(at(220));  // interval 2, now free: taken
  sender->flush();
  const app::RgbdStreamSenderStats stats = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_EQ(stats.taken, 3u);
  EXPECT_EQ(stats.busy, 2u);
  EXPECT_EQ(stats.written, 3u) << stats.error;
  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), 3u);
  const std::uint64_t pts[3] = {t0, t0 + 100 * kMs, t0 + 220 * kMs};
  for (std::size_t k = 0; k < sent.size(); ++k) {
    EXPECT_EQ(sent[k].pts, pts[k]) << "set " << k;
    EXPECT_TRUE(sent[k].picture_index.has_value()) << "set " << k;
    EXPECT_EQ(sent[k].picture_index, sent[k].depth_index) << "set " << k;
  }
}

/// A map step on a frame the gate does not take is still found: the next
/// frame sent has it out. ARKit's 60 Hz through the 10 Hz gate.
TEST_F(RgbdStreamSenderTest, AMapStepBetweenFramesSentIsOutOfTheNext) {
  TempFile file("sender_gated_step");
  camera();
  app::RgbdStreamSenderOptions o = options(file.path());
  o.session.fps = 10;
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  constexpr std::uint64_t kFrameNs = 16'666'667;  // 60 Hz
  constexpr std::size_t kFrames = 8;              // the gate takes 0 and 6
  std::uint64_t t0 = 0;
  for (std::size_t k = 0; k < kFrames; ++k) {
    std::optional<sensor::RgbdFrame> frame = next();
    ASSERT_TRUE(frame.has_value());
    if (k == 0) t0 = frame->timestamp_ns;
    frame->timestamp_ns = t0 + k * kFrameNs;
    if (k >= 4) frame->color_to_world[3][0] = 0.127;  // the map moved
    sender->offer(*frame);
    sender->flush();
  }
  const app::RgbdStreamSenderStats stats = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  EXPECT_EQ(stats.taken, 2u);
  EXPECT_GE(stats.map_steps, 1u);
  const std::vector<SentFrame> sent = read_back(file.path());
  ASSERT_EQ(sent.size(), 2u);
  EXPECT_EQ(sent[1].pts, t0 + 6 * kFrameNs);
  for (const SentFrame& f : sent) {
    EXPECT_NEAR(f.pose[3][0], 0.0, 1e-3) << "set " << f.set;
  }
}

/// T2.7's form: the iPad's stream from this Mac, for session_check.
TEST_F(RgbdStreamSenderTest, PatternSessionForTheLab) {
  if (!env_set("VI_TEST_SESSION_OUT") && !env_set("VI_TEST_LAB_HOST") &&
      !env_set("VI_TEST_DRY")) {
    GTEST_SKIP() << "set VI_TEST_SESSION_OUT and/or VI_TEST_LAB_HOST, or "
                    "VI_TEST_DRY";
  }
  camera(1920, 1440, 60);
  app::RgbdStreamSenderOptions o = options(
      env_set("VI_TEST_SESSION_OUT") ? std::getenv("VI_TEST_SESSION_OUT") : "");
  o.session.fps = 10;
  if (env_set("VI_TEST_LAB_HOST")) {
    auto target = app::parse_dial_entry(std::getenv("VI_TEST_LAB_HOST"), "");
    ASSERT_TRUE(target.ok()) << target.status().message();
    o.session.dial_host = target.value().host;
    o.session.dial_port = target.value().port;
  }
  auto sender = open(o);
  ASSERT_NE(sender, nullptr);
  const long frames =
      env_set("VI_TEST_FRAMES")
          ? std::strtol(std::getenv("VI_TEST_FRAMES"), nullptr, 10)
          : 100;
  std::vector<double> send_ms;
  std::vector<double> coded_ms;
  while (sender->stats().taken < static_cast<std::uint64_t>(frames)) {
    std::optional<sensor::RgbdFrame> frame = next();
    ASSERT_TRUE(frame.has_value());
    sender->offer(*frame);  // as the fuse thread does: no flush
  }
  sender->flush();
  for (const app::PictureTimes& t : sender->take_times()) {
    send_ms.push_back(t.send_ms);
    coded_ms.push_back(t.coded_ms);
  }
  const app::RgbdStreamSenderStats s = sender->stats();
  ASSERT_TRUE(sender->finish().ok());
  const auto at = [](std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
  };
  std::printf(
      "pattern session: offered %llu taken %llu busy %llu coded %llu keys "
      "%llu failed %llu written %llu | picture bytes %llu, depth bytes %llu | "
      "send ms p50 %.3f p95 %.3f | coded ms p50 %.2f p95 %.2f | viewers %zu "
      "dial failures %llu%s%s\n",
      static_cast<unsigned long long>(s.offered),
      static_cast<unsigned long long>(s.taken),
      static_cast<unsigned long long>(s.busy),
      static_cast<unsigned long long>(s.coded),
      static_cast<unsigned long long>(s.keys),
      static_cast<unsigned long long>(s.failed),
      static_cast<unsigned long long>(s.written),
      static_cast<unsigned long long>(s.picture_bytes),
      static_cast<unsigned long long>(s.session.depth_bytes), at(send_ms, 0.5),
      at(send_ms, 0.95), at(coded_ms, 0.5), at(coded_ms, 0.95),
      s.session.viewers,
      static_cast<unsigned long long>(s.session.dial_failures),
      s.error.empty() ? "" : " | last failure: ", s.error.c_str());
  EXPECT_EQ(s.failed, 0u);
  EXPECT_EQ(s.written, s.session_open ? s.coded : 0u);
}

}  // namespace

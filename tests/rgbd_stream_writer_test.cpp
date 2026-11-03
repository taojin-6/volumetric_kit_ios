// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file rgbd_stream_writer_test.cpp
/// @brief A sensor's frames as a `.vrst` session, read back with grid's
///        reader and sensor's wire form: the tracks, each frame's two
///        records, and the refusals that write nothing.
///
/// The frames come from a fake `IRgbdSensor` (fake_rgbd_sensor.hpp); the
/// pictures are arbitrary bytes, since the writer decodes none.

#include "rgbd_stream_writer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include <unistd.h>

#include "MapSteps.hpp"
#include "RateGate.hpp"
#include "fake_rgbd_sensor.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace {

namespace app = volumetric_kit::ios_app;
namespace fs = std::filesystem;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace testing = volumetric_kit::sensor::testing;
namespace vkc = volumetric_kit::core;
namespace wire = volumetric_kit::sensor::wire;

using ios_test::FakeRgbdSensor;

constexpr std::uint64_t kArkitNs = 16'669'375;  // 60 Hz, as ARKit's
constexpr std::uint64_t kFemtoNs = 33'333'333;  // 30 Hz

// A file of its own for each test, removed after it.
class TempFile {
 public:
  explicit TempFile(const std::string& name)
      : path_((fs::temp_directory_path() /
               ("vi_session_" + std::to_string(::getpid()) + "_" + name +
                ".vrst"))
                  .string()) {
    std::error_code ec;
    fs::remove(path_, ec);
  }
  ~TempFile() {
    std::error_code ec;
    fs::remove(path_, ec);
  }
  const std::string& path() const { return path_; }
  std::uintmax_t size() const { return fs::file_size(path_); }

 private:
  std::string path_;
};

app::RgbdStreamOptions options_for(const std::string& path) {
  app::RgbdStreamOptions o;
  o.path = path;
  o.fps = 10;
  o.session_id = {0x5e, 0x55, 0x10, 0x4e, 1, 2,  3,  4,
                  5,    6,    7,    8,    9, 10, 11, 12};
  o.created_ns = 1'793'492'974'605'774'900;
  o.writer = "scanner-test";
  return o;
}

// The colour the coded pictures state: the iPad's, BT.709 full range.
sensor::ColorCodes ipad_codes() {
  sensor::ColorCodes codes;
  codes.matrix = 1;
  codes.transfer = 1;
  codes.primaries = 1;
  codes.full_range = true;
  codes.chroma_location = 0;
  return codes;
}

// A coded picture for frame @p i: bytes that say which frame they are.
struct Picture {
  std::vector<std::uint8_t> bytes;
  app::CodedColor color;
};
Picture picture(std::uint32_t i, bool key) {
  Picture p;
  p.bytes = {0, 0, 0, 1, static_cast<std::uint8_t>(key ? 0x26 : 0x02), 1};
  for (int b = 0; b < 4; ++b) p.bytes.push_back((i >> (8 * b)) & 0xff);
  p.bytes.resize(p.bytes.size() + 3 * (i % 5));  // pictures differ in size
  p.color = {p.bytes.data(), p.bytes.size(), key, ipad_codes()};
  return p;
}

struct ReadRecord {
  stream::RecordInfo info;
  std::vector<std::uint8_t> payload;
};

// Every record of the file at @p path, in file order, and its session.
struct ReadFile {
  stream::Session session;
  bool finished = false;
  std::vector<ReadRecord> records;
};
ReadFile read_file(const std::string& path) {
  ReadFile out;
  auto reader = stream::StreamReader::open(path);
  EXPECT_TRUE(reader.ok()) << reader.status().message();
  if (!reader.ok()) return out;
  out.session = reader.value()->session();
  out.finished = reader.value()->recovery().finished;
  stream::Position pos = reader.value()->begin();
  stream::Record record;
  for (;;) {
    const vkc::Result<bool> more = reader.value()->next(&pos, &record);
    EXPECT_TRUE(more.ok()) << more.status().message();
    if (!more.ok() || !more.value()) break;
    ReadRecord r;
    r.info = record.info;
    EXPECT_TRUE(reader.value()->read(record, &r.payload).ok());
    out.records.push_back(std::move(r));
  }
  return out;
}

std::unique_ptr<app::RgbdStreamWriter> open_or_fail(
    const app::RgbdStreamOptions& options, const sensor::SensorInfo& info) {
  auto writer = app::RgbdStreamWriter::open(options, info);
  EXPECT_TRUE(writer.ok()) << writer.status().message();
  return writer.ok() ? std::move(writer).value() : nullptr;
}

// --- The session -------------------------------------------------------------

/// ARKit's sensor: its colour on track 0 and its depth on track 1, both on
/// the host's clock (0) at the rate the caller appends at, the depth track's
/// params the sensor.
TEST(RgbdStreamWriter, TheSessionNamesTheSensorAndTheRate) {
  TempFile file("session");
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = options_for(file.path());
  options.calibration = R"({"format": "test"})";
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  ASSERT_TRUE(writer->finish().ok());

  const ReadFile read = read_file(file.path());
  EXPECT_TRUE(read.finished);
  EXPECT_EQ(read.session.id, options.session_id);
  EXPECT_EQ(read.session.created_ns, options.created_ns);
  EXPECT_EQ(read.session.writer, "scanner-test");
  EXPECT_EQ(read.session.calibration, options.calibration);
  ASSERT_EQ(read.session.tracks.size(), 2u);

  const stream::Track& color = read.session.tracks[0];
  EXPECT_EQ(color.kind, stream::TrackKind::RgbdColor);
  EXPECT_EQ(color.clock, stream::kHostClock);
  EXPECT_EQ(color.rate.num, 10u);
  EXPECT_EQ(color.rate.den, 1u);
  EXPECT_EQ(color.width, 1920u);
  EXPECT_EQ(color.height, 1440u);
  EXPECT_EQ(color.sensor, info.id);
  auto codec = wire::decode_color_params(color.params);
  ASSERT_TRUE(codec.ok()) << codec.status().message();
  EXPECT_EQ(codec.value(), wire::ColorCodec::HevcMain8);

  const stream::Track& depth = read.session.tracks[1];
  EXPECT_EQ(depth.kind, stream::TrackKind::RgbdDepth);
  EXPECT_EQ(depth.clock, stream::kHostClock);
  EXPECT_EQ(depth.rate.num, 10u);
  EXPECT_EQ(depth.width, 256u);
  EXPECT_EQ(depth.height, 192u);
  EXPECT_EQ(depth.sensor, info.id);
  auto decoded = wire::decode_sensor(depth.params, depth.sensor,
                                     sensor::ClockDomain::Host, 10);
  ASSERT_TRUE(decoded.ok()) << decoded.status().message();
  EXPECT_EQ(decoded.value().id, info.id);
  EXPECT_EQ(decoded.value().model, info.model);
  EXPECT_EQ(decoded.value().pose, sensor::PoseSource::Tracked);
  ASSERT_TRUE(decoded.value().color.has_value());
  EXPECT_EQ(decoded.value().color->size.width, 1920u);
  ASSERT_TRUE(decoded.value().depth.has_value());
  EXPECT_EQ(decoded.value().depth->intrinsics.fx, info.depth->intrinsics.fx);
}

/// A sensor on its own clock takes the session's clock 1, on both tracks; a
/// rate of 0 takes the sensor's own.
TEST(RgbdStreamWriter, ADeviceClockSensorTakesClockOneAndItsOwnRate) {
  TempFile file("device_clock");
  app::RgbdStreamOptions options = options_for(file.path());
  options.fps = 0;
  auto writer = open_or_fail(options, ios_test::femto_like());
  ASSERT_NE(writer, nullptr);
  ASSERT_TRUE(writer->finish().ok());
  const ReadFile read = read_file(file.path());
  ASSERT_EQ(read.session.tracks.size(), 2u);
  for (const stream::Track& track : read.session.tracks) {
    EXPECT_EQ(track.clock, 1u);
    EXPECT_EQ(track.rate.num, 30u);
    EXPECT_EQ(track.rate.den, 1u);
  }
  EXPECT_EQ(read.session.tracks[1].width, 640u);
  EXPECT_EQ(read.session.tracks[1].height, 576u);
  auto decoded = wire::decode_sensor(read.session.tracks[1].params,
                                     read.session.tracks[1].sensor,
                                     sensor::ClockDomain::Device, 30);
  ASSERT_TRUE(decoded.ok()) << decoded.status().message();
  EXPECT_EQ(decoded.value().role, sensor::SyncRole::Primary);
  EXPECT_EQ(decoded.value().depth->distortion.k6, 0.5);
  ASSERT_TRUE(decoded.value().color_controls.has_value());
  EXPECT_EQ(decoded.value().color_controls->exposure, 8330u);
}

/// `color = false`, or a sensor with no colour camera, writes the depth
/// alone, on track 0; a picture is then refused.
TEST(RgbdStreamWriter, WithoutColourTheDepthIsTrackZero) {
  for (const bool no_camera : {false, true}) {
    TempFile file(no_camera ? "no_camera" : "no_color");
    sensor::SensorInfo info = ios_test::arkit_like();
    app::RgbdStreamOptions options = options_for(file.path());
    if (no_camera) {
      info.color.reset();
    } else {
      options.color = false;
    }
    auto writer = open_or_fail(options, info);
    ASSERT_NE(writer, nullptr);
    const FakeRgbdSensor camera(info, kArkitNs);
    const Picture p = picture(0, true);
    EXPECT_EQ(writer->append(camera.frame(0), &p.color).domain(),
              vkc::Status::Code::InvalidArgument);
    ASSERT_TRUE(writer->append(camera.frame(0), nullptr).ok());
    ASSERT_TRUE(writer->append(camera.frame(1), nullptr).ok());
    ASSERT_TRUE(writer->finish().ok());

    const ReadFile read = read_file(file.path());
    ASSERT_EQ(read.session.tracks.size(), 1u);
    EXPECT_EQ(read.session.tracks[0].kind, stream::TrackKind::RgbdDepth);
    ASSERT_EQ(read.records.size(), 2u);
    for (std::size_t k = 0; k < 2; ++k) {
      EXPECT_EQ(read.records[k].info.track, 0u);
      EXPECT_EQ(read.records[k].info.seq, k);
      EXPECT_EQ(read.records[k].info.set, k);
      EXPECT_TRUE(read.records[k].info.key);
    }
    EXPECT_EQ(writer->stats().frames, 2u);
    EXPECT_EQ(writer->stats().keys, 0u);
    EXPECT_EQ(writer->stats().color_bytes, 0u);
  }
}

// --- The records -------------------------------------------------------------

/// Each frame is its colour record, then its depth record: one set, the
/// frame's count; one pts, its capture time; seq one up on each track; keys
/// as given. Each record reads back as the frame and picture written.
TEST(RgbdStreamWriter, EachFrameIsItsColourThenItsDepth) {
  TempFile file("records");
  const sensor::SensorInfo info = ios_test::arkit_like();
  auto writer = open_or_fail(options_for(file.path()), info);
  ASSERT_NE(writer, nullptr);
  FakeRgbdSensor camera(info, kArkitNs);
  constexpr std::uint32_t kFrames = 12;
  std::vector<sensor::RgbdFrame> frames;
  std::vector<Picture> pictures;
  std::uint64_t color_bytes = 0;
  for (std::uint32_t i = 0; i < kFrames; ++i) {
    frames.push_back(camera.frame(i));
    pictures.push_back(picture(i, i % 6 == 0));
    ASSERT_TRUE(writer->append(frames.back(), &pictures.back().color).ok());
    color_bytes += wire::kColorPrefixBytes + pictures.back().bytes.size();
  }
  const app::RgbdStreamStats stats = writer->stats();
  ASSERT_TRUE(writer->finish().ok());
  EXPECT_EQ(stats.frames, kFrames);
  EXPECT_EQ(stats.keys, 2u);
  EXPECT_EQ(stats.color_bytes, color_bytes);
  EXPECT_EQ(stats.depth_bytes,
            kFrames * (wire::kDepthPrefixBytes + 3 * 256 * 192));

  const ReadFile read = read_file(file.path());
  EXPECT_TRUE(read.finished);
  ASSERT_EQ(read.records.size(), 2 * kFrames);
  auto decoded = wire::decode_sensor(read.session.tracks[1].params, info.id,
                                     sensor::ClockDomain::Host, 10);
  ASSERT_TRUE(decoded.ok());
  testing::RgbdContract contract(decoded.value());
  for (std::uint32_t i = 0; i < kFrames; ++i) {
    const ReadRecord& c = read.records[2 * i];
    const ReadRecord& d = read.records[2 * i + 1];
    EXPECT_EQ(c.info.track, 0u) << "frame " << i;
    EXPECT_EQ(d.info.track, 1u) << "frame " << i;
    EXPECT_EQ(c.info.set, i);
    EXPECT_EQ(d.info.set, i);
    EXPECT_EQ(c.info.seq, i);
    EXPECT_EQ(d.info.seq, i);
    EXPECT_EQ(c.info.pts_ns, frames[i].timestamp_ns);
    EXPECT_EQ(d.info.pts_ns, frames[i].timestamp_ns);
    EXPECT_EQ(c.info.key, i % 6 == 0);
    EXPECT_TRUE(d.info.key);

    auto colour =
        wire::decode_color(decoded.value(), c.payload.data(), c.payload.size());
    ASSERT_TRUE(colour.ok()) << colour.status().message();
    ASSERT_EQ(colour.value().picture_bytes, pictures[i].bytes.size());
    EXPECT_EQ(std::memcmp(colour.value().picture, pictures[i].bytes.data(),
                          pictures[i].bytes.size()),
              0);
    EXPECT_EQ(colour.value().codes.matrix, 1);
    EXPECT_TRUE(colour.value().codes.full_range);
    EXPECT_EQ(colour.value().exposure.iso, frames[i].color_exposure.iso);

    auto bytes = std::make_shared<std::vector<std::uint8_t>>(d.payload);
    auto depth = wire::decode_depth(decoded.value(), bytes->data(),
                                    bytes->size(), bytes);
    ASSERT_TRUE(depth.ok()) << depth.status().message();
    sensor::RgbdFrame& got = depth.value();
    EXPECT_EQ(testing::depth_index(got.depth, 256, 192), i);
    EXPECT_EQ(std::memcmp(got.depth_confidence, frames[i].depth_confidence,
                          256 * 192),
              0);
    EXPECT_EQ(got.min_confidence, 2);
    EXPECT_EQ(got.color_to_world, frames[i].color_to_world);
    // The frame a reader rebuilds: its time and number the record's.
    got.timestamp_ns = d.info.pts_ns;
    got.sequence = d.info.seq;
    got.color = frames[i].color;  // the picture, as a reader decodes it
    got.color_camera = colour.value().camera;
    contract.take(got);
  }
  sensor::SensorStats delivered;
  delivered.received = delivered.delivered = kFrames;
  EXPECT_EQ(contract.breach(delivered), "");
}

// --- Refusals ----------------------------------------------------------------

/// Each refusal writes nothing, and the next frame takes the next set and
/// seq: no gap.
TEST(RgbdStreamWriter, RefusalsWriteNothing) {
  TempFile file("refusals");
  const sensor::SensorInfo info = ios_test::arkit_like();
  auto writer = open_or_fail(options_for(file.path()), info);
  ASSERT_NE(writer, nullptr);
  const FakeRgbdSensor camera(info, kArkitNs);

  // The first picture must be a key.
  const Picture not_key = picture(0, false);
  EXPECT_EQ(writer->append(camera.frame(0), &not_key.color).domain(),
            vkc::Status::Code::InvalidArgument);
  const Picture key = picture(0, true);
  ASSERT_TRUE(writer->append(camera.frame(0), &key.color).ok());
  const Picture p1 = picture(1, false);
  ASSERT_TRUE(writer->append(camera.frame(1), &p1.color).ok());
  const std::uintmax_t size = file.size();

  const Picture p2 = picture(2, false);
  const auto refused = [&](const sensor::RgbdFrame& frame,
                           const app::CodedColor* color, vkc::Status::Code code,
                           const char* what) {
    EXPECT_EQ(writer->append(frame, color).domain(), code) << what;
    EXPECT_EQ(file.size(), size) << what;
  };
  sensor::RgbdFrame small = camera.frame(2);
  small.depth_camera.size = {128, 96};
  refused(small, &p2.color, vkc::Status::Code::InvalidArgument,
          "a depth camera of another size");
  sensor::RgbdFrame wide = camera.frame(2);
  wide.color_camera.size = {1920, 1080};
  refused(wide, &p2.color, vkc::Status::Code::InvalidArgument,
          "a colour camera of another size");
  refused(camera.frame(2), nullptr, vkc::Status::Code::InvalidArgument,
          "no picture in a colour session");
  app::CodedColor empty = p2.color;
  empty.size = 0;
  refused(camera.frame(2), &empty, vkc::Status::Code::InvalidArgument,
          "an empty picture");
  app::CodedColor bad_codes = p2.color;
  bad_codes.codes.chroma_location = 6;
  refused(camera.frame(2), &bad_codes, vkc::Status::Code::InvalidArgument,
          "a siting the wire cannot state");
  sensor::RgbdFrame earlier = camera.frame(2);
  earlier.timestamp_ns = camera.frame(0).timestamp_ns;
  refused(earlier, &p2.color, vkc::Status::Code::InvalidArgument,
          "a capture time before the last frame's");
  sensor::RgbdFrame device_only = camera.frame(2);
  device_only.depth = nullptr;
  device_only.depth_confidence = nullptr;
  device_only.min_confidence = 0;
  device_only.depth_device.buffer = std::shared_ptr<const vkc::Buffer>(
      reinterpret_cast<const vkc::Buffer*>(&info), [](const vkc::Buffer*) {});
  refused(device_only, &p2.color, vkc::Status::Code::Unsupported,
          "depth on the device only");
  EXPECT_EQ(writer->stats().frames, 2u);

  ASSERT_TRUE(writer->append(camera.frame(2), &p2.color).ok());
  ASSERT_TRUE(writer->finish().ok());
  const ReadFile read = read_file(file.path());
  ASSERT_EQ(read.records.size(), 6u);
  for (std::size_t r = 0; r < read.records.size(); ++r) {
    EXPECT_EQ(read.records[r].info.set, r / 2);
    EXPECT_EQ(read.records[r].info.seq, r / 2);
  }
}

TEST(RgbdStreamWriter, OpenRefusesWhatItCannotWrite) {
  TempFile file("open");
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = options_for(file.path());

  // An empty path writes no file: there is nothing to write to.
  app::RgbdStreamOptions no_path = options;
  no_path.path.clear();
  EXPECT_EQ(app::RgbdStreamWriter::open(no_path, info).status().domain(),
            vkc::Status::Code::InvalidArgument);

  app::RgbdStreamOptions no_id = options;
  no_id.session_id = {};
  EXPECT_EQ(app::RgbdStreamWriter::open(no_id, info).status().domain(),
            vkc::Status::Code::InvalidArgument);
  EXPECT_FALSE(fs::exists(file.path()));

  sensor::SensorInfo no_depth = info;
  no_depth.depth.reset();
  EXPECT_EQ(app::RgbdStreamWriter::open(options, no_depth).status().domain(),
            vkc::Status::Code::Unsupported);
  EXPECT_FALSE(fs::exists(file.path()));

  // A file already there is left alone.
  auto first = open_or_fail(options, info);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(app::RgbdStreamWriter::open(options, info).status().domain(),
            vkc::Status::Code::InvalidArgument);
}

/// finish is idempotent and ends the session; the destructor finishes too.
TEST(RgbdStreamWriter, FinishEndsTheSession) {
  TempFile file("finish");
  const sensor::SensorInfo info = ios_test::arkit_like();
  const FakeRgbdSensor camera(info, kArkitNs);
  const Picture key = picture(0, true);
  {
    auto writer = open_or_fail(options_for(file.path()), info);
    ASSERT_NE(writer, nullptr);
    ASSERT_TRUE(writer->append(camera.frame(0), &key.color).ok());
    ASSERT_TRUE(writer->finish().ok());
    EXPECT_TRUE(writer->finish().ok());
    EXPECT_EQ(writer->append(camera.frame(1), &key.color).domain(),
              vkc::Status::Code::InvalidArgument);
  }
  EXPECT_EQ(read_file(file.path()).records.size(), 2u);

  TempFile dropped("dropped");
  {
    auto writer = open_or_fail(options_for(dropped.path()), info);
    ASSERT_NE(writer, nullptr);
    ASSERT_TRUE(writer->append(camera.frame(0), &key.color).ok());
  }
  const ReadFile read = read_file(dropped.path());
  EXPECT_TRUE(read.finished);
  EXPECT_EQ(read.records.size(), 2u);
}

// --- The iPad's order: every pose's steps out, then the gate, then the file --

/// ARKit's 60 Hz frames, a map step among them: the step comes out of every
/// pose before the 10 Hz gate, and the session holds the gate's frames, one
/// set and seq a frame with no gap, each pose with the step taken out.
TEST(RgbdStreamWriter, StepsThenTheGateThenTheFile) {
  TempFile file("pipeline");
  const sensor::SensorInfo info = ios_test::arkit_like();
  auto writer = open_or_fail(options_for(file.path()), info);
  ASSERT_NE(writer, nullptr);
  FakeRgbdSensor camera(info, kArkitNs);
  ASSERT_TRUE(camera.start().ok());
  const glm::dmat4 step =
      glm::translate(glm::dmat4(1.0), glm::dvec3(0.09, -0.03, 0.08));
  app::MapSteps steps;
  app::RateGate gate(10);
  std::vector<std::uint32_t> sent;  // the camera's numbers of the frames sent
  for (std::uint32_t i = 0; i < 600; ++i) {
    auto polled = camera.poll();
    ASSERT_TRUE(polled.ok() && polled.value().has_value());
    sensor::RgbdFrame frame = std::move(*polled.value());
    if (i >= 250) frame.color_to_world = step * frame.color_to_world;
    frame.color_to_world = steps.take(frame.timestamp_ns, frame.color_to_world);
    if (!gate.take(frame.timestamp_ns)) continue;
    const Picture p = picture(i, sent.empty());
    ASSERT_TRUE(writer->append(frame, &p.color).ok());
    sent.push_back(i);
  }
  ASSERT_TRUE(writer->finish().ok());
  ASSERT_EQ(steps.steps().size(), 1u);
  EXPECT_EQ(steps.steps()[0].pts_ns, camera.frame(250).timestamp_ns);

  const ReadFile read = read_file(file.path());
  ASSERT_EQ(read.records.size(), 2 * sent.size());
  EXPECT_EQ(sent.size(), 100u);  // 10 s at 10 Hz
  auto decoded = wire::decode_sensor(read.session.tracks[1].params, info.id,
                                     sensor::ClockDomain::Host, 10);
  ASSERT_TRUE(decoded.ok());
  for (std::size_t k = 0; k < sent.size(); ++k) {
    const ReadRecord& d = read.records[2 * k + 1];
    EXPECT_EQ(d.info.seq, k);
    EXPECT_EQ(d.info.set, k);
    auto bytes = std::make_shared<std::vector<std::uint8_t>>(d.payload);
    auto depth = wire::decode_depth(decoded.value(), bytes->data(),
                                    bytes->size(), bytes);
    ASSERT_TRUE(depth.ok());
    EXPECT_EQ(testing::depth_index(depth.value().depth, 256, 192), sent[k]);
    const glm::dvec3 off = glm::dvec3(depth.value().color_to_world[3]) -
                           glm::dvec3(camera.pose(sent[k])[3]);
    EXPECT_LT(glm::length(off), 1e-3) << "frame " << sent[k];
  }
}

}  // namespace

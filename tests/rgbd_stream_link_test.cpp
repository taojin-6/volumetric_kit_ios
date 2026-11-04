// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file rgbd_stream_link_test.cpp
/// @brief The session writer's live link, on 127.0.0.1: the writer dials a
///        `core::net::Listener`, and grid's `StreamClient::adopt` reads what
///        it serves, as the lab's reader does.
///
/// The frames come from a fake `IRgbdSensor` (fake_rgbd_sensor.hpp); the
/// pictures are arbitrary bytes, since the writer decodes none.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dial_entry.hpp"
#include "fake_rgbd_sensor.hpp"
#include "rgbd_stream_writer.hpp"
#include "session_test_support.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/grid/networking/stream_client.hpp"
#include "volumetric_kit/grid/stream/session.hpp"

namespace {

namespace app = volumetric_kit::ios_app;
namespace net = volumetric_kit::core::net;
namespace networking = volumetric_kit::grid::networking;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace vkc = volumetric_kit::core;

using ios_test::FakeRgbdSensor;
using ios_test::open_or_fail;
using ios_test::options_for;
using ios_test::Picture;
using ios_test::picture;
using ios_test::read_file;
using ios_test::ReadFile;
using ios_test::ReadRecord;
using ios_test::TempFile;
using namespace std::chrono_literals;

constexpr std::uint64_t kArkitNs = 16'669'375;  // 60 Hz, as ARKit's
constexpr const char* kLoopback = "127.0.0.1";
// The link's times in the cases that wait on them: the redial wait a tenth of
// the app's (app::kRedialWait), and a dial limit long enough that a dial kept
// and one begun again end apart (DialToTheSameTargetKeepsTheDialUnderWay;
// app::kDialLimit is 2 s).
constexpr std::chrono::milliseconds kRedial{100};
constexpr std::chrono::milliseconds kLimit{300};

app::RgbdStreamOptions fast(app::RgbdStreamOptions options) {
  options.redial_wait = kRedial;
  options.dial_limit = kLimit;
  return options;
}

// Whether @p holds becomes true within @p limit.
template <class Predicate>
bool eventually(Predicate holds, std::chrono::milliseconds limit = 5s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!holds()) {
    if (std::chrono::steady_clock::now() >= end) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

std::unique_ptr<net::Listener> listen_or_fail() {
  auto listener = net::Listener::create(kLoopback, 0);
  EXPECT_TRUE(listener.ok()) << listener.status().message();
  return listener.ok() ? std::move(listener).value() : nullptr;
}

// The writer's next connection to @p listener, read as the lab reads one;
// null when none came within @p wait.
std::unique_ptr<networking::StreamClient> take_reader(
    net::Listener& listener, std::chrono::milliseconds wait = 5s) {
  const auto end = std::chrono::steady_clock::now() + wait;
  while (std::chrono::steady_clock::now() < end) {
    auto fd = listener.accept(50ms);
    if (!fd.ok() || !fd.value().valid()) continue;
    auto client = networking::StreamClient::adopt(std::move(fd).value());
    if (client.ok()) return std::move(client).value();
  }
  return nullptr;
}

// The next @p count records @p client reads, within @p limit.
std::vector<ReadRecord> read_records(networking::StreamClient& client,
                                     std::size_t count,
                                     std::chrono::milliseconds limit = 5s) {
  std::vector<ReadRecord> out;
  const auto end = std::chrono::steady_clock::now() + limit;
  stream::Record record;
  std::vector<std::uint8_t> payload;
  while (out.size() < count && std::chrono::steady_clock::now() < end) {
    const vkc::Result<bool> got = client.next(20ms, &record, &payload);
    EXPECT_TRUE(got.ok()) << got.status().message();
    if (!got.ok()) break;
    if (got.value()) out.push_back({record.info, payload});
  }
  return out;
}

// Whether @p client reads its sender's end within @p limit, with no record
// before it.
bool reads_the_end(networking::StreamClient& client,
                   std::chrono::milliseconds limit = 3s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  stream::Record record;
  std::vector<std::uint8_t> payload;
  while (std::chrono::steady_clock::now() < end) {
    const vkc::Result<bool> got = client.next(20ms, &record, &payload);
    if (!got.ok()) return got.status().domain() == vkc::Status::Code::IoError;
    EXPECT_FALSE(got.value()) << "a record before the end";
  }
  return false;
}

void expect_same_records(const std::vector<ReadRecord>& got,
                         const std::vector<ReadRecord>& file,
                         std::size_t first) {
  ASSERT_LE(first + got.size(), file.size());
  for (std::size_t k = 0; k < got.size(); ++k) {
    const ReadRecord& want = file[first + k];
    EXPECT_EQ(got[k].info.track, want.info.track) << "record " << k;
    EXPECT_EQ(got[k].info.key, want.info.key) << "record " << k;
    EXPECT_EQ(got[k].info.pts_ns, want.info.pts_ns) << "record " << k;
    EXPECT_EQ(got[k].info.seq, want.info.seq) << "record " << k;
    EXPECT_EQ(got[k].info.set, want.info.set) << "record " << k;
    EXPECT_EQ(got[k].payload, want.payload) << "record " << k;
  }
}

// --- The records -------------------------------------------------------------

/// A reader that joins asks for a key and waits for it: from that key on it
/// gets the file's records as the file holds them, colour then depth, and the
/// writer's end after the last. A writer opened with no link starts one at
/// its first target.
TEST(RgbdStreamLink, AReaderGetsTheFilesRecordsFromTheKeyItAskedFor) {
  TempFile file("link_records");
  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  const sensor::SensorInfo info = ios_test::arkit_like();
  const app::RgbdStreamOptions options = options_for(file.path());
  auto writer = open_or_fail(options, info);  // the file alone, at first
  ASSERT_NE(writer, nullptr);
  const FakeRgbdSensor camera(info, kArkitNs);

  // Frames before any reader: the file has them, no reader does.
  for (std::uint32_t i = 0; i < 3; ++i) {
    const Picture p = picture(i, i == 0);
    ASSERT_TRUE(writer->append(camera.frame(i), &p.color).ok());
  }
  EXPECT_FALSE(writer->key_wanted());
  EXPECT_EQ(writer->stats().key_wanted, 0u);

  ASSERT_TRUE(writer->dial_to(kLoopback, listener->port()).ok());
  auto reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  EXPECT_TRUE(writer->key_wanted());
  EXPECT_EQ(writer->stats().key_wanted, 1u);

  // Frame 3 is no key, so the reader waits past it; frame 4 is the key it
  // asked for, after which none is wanted.
  const Picture p3 = picture(3, false);
  ASSERT_TRUE(writer->append(camera.frame(3), &p3.color).ok());
  const Picture p4 = picture(4, writer->key_wanted());
  ASSERT_TRUE(p4.color.key);
  ASSERT_TRUE(writer->append(camera.frame(4), &p4.color).ok());
  EXPECT_FALSE(writer->key_wanted());
  for (std::uint32_t i = 5; i < 7; ++i) {
    const Picture p = picture(i, false);
    ASSERT_TRUE(writer->append(camera.frame(i), &p.color).ok());
  }
  const std::vector<ReadRecord> got = read_records(*reader, 6);
  ASSERT_EQ(got.size(), 6u);
  ASSERT_NE(reader->session(), nullptr);
  EXPECT_EQ(reader->session()->id, options.session_id);
  ASSERT_EQ(reader->session()->tracks.size(), 2u);
  EXPECT_EQ(reader->session()->tracks[0].kind, stream::TrackKind::RgbdColor);
  EXPECT_EQ(reader->session()->tracks[1].kind, stream::TrackKind::RgbdDepth);

  ASSERT_TRUE(writer->finish().ok());
  EXPECT_TRUE(reads_the_end(*reader));
  EXPECT_EQ(writer->stats().viewers, 0u);
  EXPECT_EQ(writer->stats().dial_failures, 0u);

  const ReadFile read = read_file(file.path());
  EXPECT_TRUE(read.finished);
  ASSERT_EQ(read.records.size(), 14u);
  expect_same_records(got, read.records, 8);  // frame 4's colour record on
}

// --- Connect and disconnect --------------------------------------------------

/// The on-screen connect moves the link to another reader, letting the first
/// go; the disconnect lets the reader go and dials no more; connecting again
/// brings the same session. The file is one valid file throughout.
TEST(RgbdStreamLink, DialToMovesTheLinkAndAnEmptyHostLetsTheReaderGo) {
  TempFile file("link_move");
  auto first = listen_or_fail();
  auto second = listen_or_fail();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = fast(options_for(file.path()));
  options.dial_host = kLoopback;
  options.dial_port = first->port();
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  const FakeRgbdSensor camera(info, kArkitNs);
  std::uint32_t next = 0;
  const auto append = [&](bool key) {
    const Picture p = picture(next, key);
    ASSERT_TRUE(writer->append(camera.frame(next), &p.color).ok());
    ++next;
  };

  auto reader = take_reader(*first);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  append(true);
  ASSERT_EQ(read_records(*reader, 2).size(), 2u);

  // Connect elsewhere: the first reader reads its end, the second joins.
  ASSERT_TRUE(writer->dial_to(kLoopback, second->port()).ok());
  EXPECT_TRUE(reads_the_end(*reader));
  ASSERT_NE(reader->session(), nullptr);
  EXPECT_EQ(reader->session()->id, options.session_id);
  reader = take_reader(*second);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  EXPECT_TRUE(writer->key_wanted());
  append(true);
  append(false);
  std::vector<ReadRecord> got = read_records(*reader, 4);
  ASSERT_EQ(got.size(), 4u);
  EXPECT_EQ(reader->session()->id, options.session_id);
  EXPECT_EQ(got[0].info.set, 1u);

  // Disconnect: the reader reads its end, and no dial follows within a redial
  // wait and more.
  ASSERT_TRUE(writer->dial_to("", 0).ok());
  EXPECT_TRUE(reads_the_end(*reader));
  EXPECT_TRUE(eventually([&] { return writer->stats().viewers == 0; }));
  EXPECT_EQ(take_reader(*second, kRedial + 100ms), nullptr);
  append(false);

  // Connect again: the same session, from the next key.
  ASSERT_TRUE(writer->dial_to(kLoopback, second->port()).ok());
  reader = take_reader(*second);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  append(writer->key_wanted());
  got = read_records(*reader, 2);
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(reader->session()->id, options.session_id);
  EXPECT_EQ(got[0].info.set, 4u);
  EXPECT_TRUE(got[0].info.key);

  ASSERT_TRUE(writer->finish().ok());
  const ReadFile read = read_file(file.path());
  EXPECT_TRUE(read.finished);
  ASSERT_EQ(read.records.size(), 2u * next);
  for (std::size_t r = 0; r < read.records.size(); ++r) {
    EXPECT_EQ(read.records[r].info.set, r / 2) << "record " << r;
    EXPECT_EQ(read.records[r].info.seq, r / 2) << "record " << r;
  }
  expect_same_records(got, read.records, 8);
}

/// A reader that leaves after a while is dialled again, and gets the same
/// session; depth alone is track 0, every record a key, so a joiner starts at
/// the next.
TEST(RgbdStreamLink, ItRedialsAReaderThatLeft) {
  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = fast(options_for(""));  // no file
  options.color = false;
  options.dial_host = kLoopback;
  options.dial_port = listener->port();
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  const FakeRgbdSensor camera(info, kArkitNs);

  auto reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  ASSERT_TRUE(writer->append(camera.frame(0), nullptr).ok());
  std::vector<ReadRecord> got = read_records(*reader, 1);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].info.track, 0u);
  EXPECT_EQ(reader->session()->tracks.size(), 1u);

  // The reader leaves after a redial wait, so no dial has failed. The next
  // connection is the redial, a watch (10 ms) after the leave: no poll for
  // the gap between them, which a 5 ms poll can miss.
  std::this_thread::sleep_for(kRedial + 50ms);
  reader.reset();
  reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  ASSERT_TRUE(writer->append(camera.frame(1), nullptr).ok());
  got = read_records(*reader, 1);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(reader->session()->id, options.session_id);
  EXPECT_EQ(got[0].info.seq, 1u);
  EXPECT_EQ(writer->stats().dial_failures, 0u);
  EXPECT_EQ(writer->stats().frames, 2u);
}

/// A reader that takes the session and closes at once, as one does that
/// refuses a session other than its first, is dialled again no more than
/// once a redial wait (the app's: a second), and each counts as a failed
/// dial; a reader that stays a redial wait starts the count again from 0.
TEST(RgbdStreamLink, AReaderThatLeavesAtOnceIsDialledOnceARedialWait) {
  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = fast(options_for(""));
  options.color = false;
  options.dial_host = kLoopback;
  options.dial_port = listener->port();
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);

  int connections = 0;
  const auto end = std::chrono::steady_clock::now() + 3 * kRedial;
  while (std::chrono::steady_clock::now() < end) {
    auto fd = listener->accept(50ms);
    if (!fd.ok() || !fd.value().valid()) continue;
    ++connections;
    auto reader = networking::StreamClient::adopt(std::move(fd).value());
    if (!reader.ok()) continue;
    const auto read_by = std::chrono::steady_clock::now() + 500ms;
    stream::Record record;
    while (reader.value()->session() == nullptr &&
           std::chrono::steady_clock::now() < read_by) {
      if (!reader.value()->next(10ms, &record, nullptr).ok()) break;
    }
    EXPECT_NE(reader.value()->session(), nullptr);
  }  // each reader closes once it has the session
  EXPECT_GE(connections, 2);
  EXPECT_LE(connections, 4);  // at 0, R and 2R, and perhaps at 3R
  EXPECT_LE(writer->stats().key_wanted, 4u);
  EXPECT_TRUE(eventually([&] { return writer->stats().dial_failures >= 2; }));

  auto reader = take_reader(*listener);  // one that stays
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  EXPECT_TRUE(eventually([&] { return writer->stats().dial_failures == 0; }));
  EXPECT_EQ(writer->stats().viewers, 1u);
}

/// A connect to the target already served keeps its reader, which goes on
/// reading every record. (A dial while a reader is served fails the redial
/// cases above, so no window here waits for a second dial.)
TEST(RgbdStreamLink, DialToTheServedTargetKeepsTheReader) {
  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = options_for("");
  options.color = false;
  options.dial_host = kLoopback;
  options.dial_port = listener->port();
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  const FakeRgbdSensor camera(info, kArkitNs);

  auto reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  // Each record is read before the next is appended: depth alone makes every
  // record a key, and the server lets a reader go on from a newer key than
  // the one it has not sent yet.
  for (std::uint32_t k = 0; k < 3; ++k) {
    if (k == 1) {  // the connect, between the first two records
      ASSERT_TRUE(writer->dial_to(kLoopback, listener->port()).ok());
    }
    ASSERT_TRUE(writer->append(camera.frame(k), nullptr).ok());
    const std::vector<ReadRecord> got = read_records(*reader, 1);
    ASSERT_EQ(got.size(), 1u) << "record " << k;
    EXPECT_EQ(got[0].info.seq, k);
  }
  EXPECT_EQ(writer->stats().viewers, 1u);
}

// --- Failed dials ------------------------------------------------------------

/// Dials to a closed port fail and are counted, one a redial wait; a connect
/// to the same target dials at once, and a new target starts the count again
/// from 0.
TEST(RgbdStreamLink, DialFailuresCountAClosedPortAndRestartAtANewTarget) {
  std::uint16_t closed = 0;
  {
    auto gone = listen_or_fail();
    ASSERT_NE(gone, nullptr);
    closed = gone->port();
  }  // its listener closed, so a dial to it is refused
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = fast(options_for(""));
  options.dial_host = kLoopback;
  options.dial_port = closed;
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  std::this_thread::sleep_for(kRedial * 5 / 2);  // dials at 0, R and 2R
  const app::RgbdStreamStats failed = writer->stats();
  EXPECT_GE(failed.dial_failures, 2u);
  EXPECT_LE(failed.dial_failures, 4u);
  EXPECT_EQ(failed.viewers, 0u);
  EXPECT_FALSE(failed.no_route);  // refused: a route, no listener

  // Just after a failure, the next dial is a redial wait away; Connect dials
  // now.
  const std::uint64_t before = writer->stats().dial_failures;
  ASSERT_TRUE(eventually([&] { return writer->stats().dial_failures > before; },
                         2 * kRedial));
  ASSERT_TRUE(writer->dial_to(kLoopback, closed).ok());
  EXPECT_EQ(writer->stats().dial_failures, 0u);
  EXPECT_TRUE(eventually([&] { return writer->stats().dial_failures == 1; },
                         kRedial * 2 / 5));

  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  ASSERT_TRUE(writer->dial_to(kLoopback, listener->port()).ok());
  EXPECT_EQ(writer->stats().dial_failures, 0u);
  auto reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  EXPECT_EQ(writer->stats().dial_failures, 0u);
}

/// A dial that finds no route to the host sets no_route, iOS's answer before
/// Local Network access is allowed; a new target clears it. macOS answers a
/// connect to 0.0.0.1 so at once.
TEST(RgbdStreamLink, NoRouteIsSetByTheDialAndClearedByANewTarget) {
  constexpr const char* kNoRoute = "0.0.0.1";
  const auto probe = net::dial(kNoRoute, 9, 500ms);
  if (probe.ok() || !app::is_no_route(probe.status())) {
    GTEST_SKIP() << "this host answers a dial to " << kNoRoute << " with "
                 << (probe.ok() ? "a connection" : probe.status().message());
  }
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = options_for("");
  options.color = false;
  options.dial_host = kNoRoute;
  options.dial_port = 9;
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().dial_failures >= 1; }));
  EXPECT_TRUE(writer->stats().no_route);

  auto listener = listen_or_fail();
  ASSERT_NE(listener, nullptr);
  ASSERT_TRUE(writer->dial_to(kLoopback, listener->port()).ok());
  EXPECT_FALSE(writer->stats().no_route);
  auto reader = take_reader(*listener);
  ASSERT_NE(reader, nullptr);
  ASSERT_TRUE(eventually([&] { return writer->stats().viewers == 1; }));
  EXPECT_FALSE(writer->stats().no_route);
}

/// A connect to the same target while a dial to it is under way keeps that
/// dial, whose failure counts. 192.0.2.1 (TEST-NET-1) answers no connect on
/// most networks, so the dial waits out its dial limit.
TEST(RgbdStreamLink, DialToTheSameTargetKeepsTheDialUnderWay) {
  constexpr const char* kSilent = "192.0.2.1";
  const auto probe = net::dial(kSilent, 9, kLimit / 3);
  if (probe.ok() || probe.status().message().find("cannot connect within") ==
                        std::string::npos) {
    GTEST_SKIP() << "this network answers a dial to " << kSilent << " with "
                 << (probe.ok() ? "a connection" : probe.status().message());
  }
  const sensor::SensorInfo info = ios_test::arkit_like();
  app::RgbdStreamOptions options = fast(options_for(""));
  options.color = false;
  options.dial_host = kSilent;
  options.dial_port = 9;
  auto writer = open_or_fail(options, info);
  ASSERT_NE(writer, nullptr);
  std::this_thread::sleep_for(kLimit / 3);  // the first dial under way
  ASSERT_TRUE(writer->dial_to(kSilent, 9).ok());
  // That dial ends at kLimit from open and counts; a dial begun again would
  // end kLimit after it.
  EXPECT_TRUE(eventually([&] { return writer->stats().dial_failures >= 1; },
                         kLimit + kLimit / 3));
}

// --- Refusals ----------------------------------------------------------------

/// A dial host must be one host's numeric IPv4 address and the port above 0,
/// at open (with no file made) and at dial_to (with the last target kept). A
/// writer with no link wants no key.
TEST(RgbdStreamLink, ATargetMustBeOneNumericAddress) {
  TempFile file("link_refusals");
  const sensor::SensorInfo info = ios_test::arkit_like();
  struct Target {
    const char* host;
    std::uint16_t port;
  };
  for (const Target& target :
       {Target{"lab-box", 7101}, Target{"0.0.0.0", 7101},
        Target{"010.8.0.1", 7101}, Target{"::1", 7101}, Target{kLoopback, 0}}) {
    app::RgbdStreamOptions options = options_for(file.path());
    options.dial_host = target.host;
    options.dial_port = target.port;
    const auto refused = app::RgbdStreamWriter::open(options, info);
    EXPECT_EQ(refused.status().domain(), vkc::Status::Code::InvalidArgument)
        << target.host << ":" << target.port;
    EXPECT_EQ(refused.status().message().rfind("RgbdStreamWriter::open: ", 0),
              0u)
        << refused.status().message();
    EXPECT_FALSE(std::filesystem::exists(file.path())) << target.host;
  }

  auto writer = open_or_fail(options_for(file.path()), info);  // no link
  ASSERT_NE(writer, nullptr);
  EXPECT_FALSE(writer->key_wanted());
  const vkc::Status bad = writer->dial_to("lab-box", 7101);
  EXPECT_EQ(bad.domain(), vkc::Status::Code::InvalidArgument);
  EXPECT_EQ(bad.message().rfind("RgbdStreamWriter::dial_to: ", 0), 0u)
      << bad.message();
  EXPECT_EQ(writer->dial_to(kLoopback, 0).domain(),
            vkc::Status::Code::InvalidArgument);
  EXPECT_TRUE(writer->dial_to("", 0).ok());  // none, and none wanted
  EXPECT_EQ(writer->stats().dial_failures, 0u);
  ASSERT_TRUE(writer->finish().ok());
  EXPECT_EQ(writer->dial_to(kLoopback, 7101).domain(),
            vkc::Status::Code::InvalidArgument);
}

}  // namespace

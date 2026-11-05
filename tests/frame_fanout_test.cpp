// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file frame_fanout_test.cpp
/// @brief The stream gets every frame at once however long fusion takes;
///        fusion takes the newest frame when it is free, the frames it skips
///        are let go at once, and a stop or a throwing fuse ends nothing it
///        should not.

#include "FrameFanout.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// Long enough never to end a passing wait, short enough to end a failing one.
constexpr std::chrono::seconds kBound{5};

// The frames alive, as the camera counts the pictures they hold.
class Pool {
 public:
  void add() {
    const std::lock_guard<std::mutex> lock(mutex_);
    peak_ = std::max(peak_, ++live_);
    changed_.notify_all();
  }
  void remove() {
    const std::lock_guard<std::mutex> lock(mutex_);
    --live_;
    changed_.notify_all();
  }
  int live() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return live_;
  }
  int peak() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return peak_;
  }
  // Whether exactly @p n come to be alive.
  bool wait_live(int n) {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, kBound, [&] { return live_ == n; });
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  int live_ = 0;
  int peak_ = 0;
};

// A frame holding one of the pool's pictures; a copy holds another, a move
// hands its own over.
class FakeFrame {
 public:
  FakeFrame(int sequence, Pool& pool) : sequence_(sequence), pool_(&pool) {
    pool_->add();
  }
  FakeFrame(const FakeFrame& o) : sequence_(o.sequence_), pool_(o.pool_) {
    if (pool_ != nullptr) pool_->add();
  }
  FakeFrame(FakeFrame&& o) noexcept
      : sequence_(o.sequence_), pool_(std::exchange(o.pool_, nullptr)) {}
  FakeFrame& operator=(const FakeFrame&) = delete;
  FakeFrame& operator=(FakeFrame&& o) noexcept {
    if (this != &o) {
      release();
      sequence_ = o.sequence_;
      pool_ = std::exchange(o.pool_, nullptr);
    }
    return *this;
  }
  ~FakeFrame() { release(); }
  int sequence() const { return sequence_; }

 private:
  void release() {
    if (pool_ != nullptr) pool_->remove();
    pool_ = nullptr;
  }
  int sequence_ = 0;
  Pool* pool_ = nullptr;
};

// A fusion that holds each frame until the test lets it through.
class Gate {
 public:
  void fuse(const FakeFrame& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_.push_back(frame.sequence());
    changed_.notify_all();
    changed_.wait(lock, [this] { return open_ || passes_ > 0; });
    if (!open_) --passes_;
  }
  // Whether fusion comes to have entered @p n frames.
  bool wait_entered(std::size_t n) {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, kBound,
                             [&] { return entered_.size() >= n; });
  }
  void pass() {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++passes_;
    changed_.notify_all();
  }
  void open() {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    changed_.notify_all();
  }
  std::vector<int> entered() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return entered_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<int> entered_;
  int passes_ = 0;
  bool open_ = false;
};

// Opens the gate as a test ends, before its fan-out stops, so a failed
// assertion cannot leave the stop waiting on a held fuse.
struct OpenAtEnd {
  Gate& gate;
  ~OpenAtEnd() { gate.open(); }
};

using Fanout = app::FrameFanout<FakeFrame>;

// A fan-out onto @p streamed and @p gate; a failure fails the test.
Fanout gated(std::vector<int>& streamed, Gate& gate) {
  return Fanout(
      [&streamed](const FakeFrame& f) { streamed.push_back(f.sequence()); },
      [&gate](const FakeFrame& f) { gate.fuse(f); },
      [](std::string_view why) { ADD_FAILURE() << why; });
}

std::vector<int> up_to(int n) {
  std::vector<int> all(static_cast<std::size_t>(n));
  std::iota(all.begin(), all.end(), 0);
  return all;
}

constexpr int kFrames = 100;

TEST(FrameFanout, StreamGetsEveryFrameInOrderWhileFusionIsHeld) {
  Pool pool;
  Gate gate;
  std::vector<int> streamed;
  Fanout fanout = gated(streamed, gate);
  const OpenAtEnd opener{gate};
  ASSERT_TRUE(fanout.start());

  fanout.offer(FakeFrame(0, pool));
  ASSERT_TRUE(gate.wait_entered(1));
  for (int i = 1; i < kFrames; ++i) fanout.offer(FakeFrame(i, pool));

  EXPECT_EQ(streamed, up_to(kFrames));
  EXPECT_EQ(gate.entered(), std::vector<int>{0});
}

TEST(FrameFanout, FusionTakesTheNewestFrameOnceFree) {
  Pool pool;
  Gate gate;
  std::vector<int> streamed;
  Fanout fanout = gated(streamed, gate);
  const OpenAtEnd opener{gate};
  ASSERT_TRUE(fanout.start());

  fanout.offer(FakeFrame(0, pool));
  ASSERT_TRUE(gate.wait_entered(1));
  for (int i = 1; i < kFrames; ++i) fanout.offer(FakeFrame(i, pool));
  gate.pass();
  ASSERT_TRUE(gate.wait_entered(2));
  gate.open();
  // Fusion lets its last frame go and holds nothing while it waits.
  ASSERT_TRUE(pool.wait_live(0));

  EXPECT_EQ(gate.entered(), (std::vector<int>{0, kFrames - 1}));
  const Fanout::Counts c = fanout.counts();
  EXPECT_EQ(c.offered, std::uint64_t{kFrames});
  EXPECT_EQ(c.taken, 2u);
  EXPECT_EQ(c.skipped, std::uint64_t{kFrames - 2});
}

TEST(FrameFanout, SkippedFramesGoBackAtOnce) {
  Pool pool;
  Gate gate;
  std::vector<int> streamed;
  Fanout fanout = gated(streamed, gate);
  const OpenAtEnd opener{gate};
  ASSERT_TRUE(fanout.start());

  fanout.offer(FakeFrame(0, pool));
  ASSERT_TRUE(gate.wait_entered(1));
  for (int i = 1; i < kFrames; ++i) {
    fanout.offer(FakeFrame(i, pool));
    // The frame in fusion and the newest waiting, no more.
    ASSERT_EQ(pool.live(), 2) << "after frame " << i;
    ASSERT_EQ(fanout.counts().skipped, std::uint64_t(i - 1));
  }
  // And the newest in hand as it is offered.
  EXPECT_LE(pool.peak(), 3);
}

TEST(FrameFanout, StopWithFusionHeldEndsCleanly) {
  Pool pool;
  Gate gate;
  std::vector<int> streamed;
  Fanout fanout = gated(streamed, gate);
  const OpenAtEnd opener{gate};
  ASSERT_TRUE(fanout.start());

  fanout.offer(FakeFrame(0, pool));
  ASSERT_TRUE(gate.wait_entered(1));
  fanout.offer(FakeFrame(1, pool));
  std::thread stopper([&fanout] { fanout.stop(); });
  // The waiting frame goes back first; the stop then waits for the fuse.
  EXPECT_TRUE(pool.wait_live(1));
  gate.open();
  stopper.join();

  EXPECT_EQ(pool.live(), 0);
  EXPECT_EQ(gate.entered(), std::vector<int>{0});
  // Once stopped, the stream still gets every frame, and fusion none.
  fanout.offer(FakeFrame(2, pool));
  EXPECT_EQ(pool.live(), 0);
  EXPECT_EQ(streamed, up_to(3));
  const Fanout::Counts c = fanout.counts();
  EXPECT_EQ(c.offered, 3u);
  EXPECT_EQ(c.taken, 1u);
  EXPECT_EQ(c.skipped, 2u);
  fanout.stop();  // again: nothing
}

TEST(FrameFanout, NotStartedStreamsEveryFrameAndFusesNone) {
  Pool pool;
  Gate gate;
  std::vector<int> streamed;
  Fanout fanout = gated(streamed, gate);
  for (int i = 0; i < 3; ++i) fanout.offer(FakeFrame(i, pool));
  EXPECT_EQ(pool.live(), 0);
  EXPECT_EQ(streamed, up_to(3));
  EXPECT_TRUE(gate.entered().empty());
  EXPECT_EQ(fanout.counts().skipped, 3u);
}

TEST(FrameFanout, AThrowingFuseIsContainedAndFusionGoesOn) {
  Pool pool;
  std::vector<int> streamed;
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<int> fused;
  std::vector<std::string> failures;
  Fanout fanout(
      [&streamed](const FakeFrame& f) { streamed.push_back(f.sequence()); },
      [&](const FakeFrame& f) {
        const std::lock_guard<std::mutex> lock(mutex);
        fused.push_back(f.sequence());
        changed.notify_all();
        if (f.sequence() == 0) throw std::runtime_error("no such frame");
      },
      [&](std::string_view why) {
        const std::lock_guard<std::mutex> lock(mutex);
        failures.emplace_back(why);
      });
  ASSERT_TRUE(fanout.start());
  const auto fused_up_to = [&](std::size_t n) {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, kBound, [&] { return fused.size() >= n; });
  };

  fanout.offer(FakeFrame(0, pool));
  ASSERT_TRUE(fused_up_to(1));
  ASSERT_TRUE(pool.wait_live(0));
  fanout.offer(FakeFrame(1, pool));
  ASSERT_TRUE(fused_up_to(2));
  ASSERT_TRUE(pool.wait_live(0));

  const std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(fused, (std::vector<int>{0, 1}));
  EXPECT_EQ(failures, std::vector<std::string>{"no such frame"});
}

}  // namespace

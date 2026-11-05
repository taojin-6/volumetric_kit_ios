// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file FrameFanout.hpp
/// @brief The camera's frames to the stream and to fusion, each at its own
///        pace: the stream is offered every frame at once, and fusion, on a
///        worker of its own, takes the newest frame each time it is free.
///
/// The camera's poller hands each frame to @ref FrameFanout::offer, which
/// gives it to the stream there and then, on the poller's thread, so a slow
/// fuse never slows the stream. The frame then waits in one slot for the
/// worker. A newer frame arriving first takes the slot, and the older one is
/// let go at once and counted skipped: no queue, so fusion works on the
/// newest frame there is whenever it starts one, and holds at most two, the
/// one it fuses and the one waiting. The worker lets its frame go before it
/// waits for the next.
///
/// Pure, so the host tests drive it with a fake frame; Bridge/FuseLoop.cpp
/// fans the camera's frames out to the stream sender and `Fusion`.
///
/// @code
/// app::FrameFanout<Frame> fanout(
///     [&](const Frame& f) { sender.offer(f); },  // every frame, at once
///     [&](const Frame& f) { fusion.fuse(f); },   // the newest, when free
///     [&](std::string_view why) noexcept { log(why); });
/// fanout.start();
/// while (running) fanout.offer(poll());
/// fanout.stop();  // the fuse under way ends; the waiting frame is let go
/// @endcode
///
/// @warning @ref start, @ref offer and @ref stop from one thread at a time,
///          the poller's; @ref counts from any.

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string_view>
#include <thread>
#include <utility>

#include "FailureHandling.hpp"

namespace volumetric_kit::ios_app {

template <typename Frame>
class FrameFanout {
 public:
  /// Each frame, on the offering thread.
  using Stream = std::function<void(const Frame&)>;
  /// The newest frame, on the worker.
  using Fuse = std::function<void(const Frame&)>;
  /// What an exception from @ref Fuse, or the worker's start, said.
  using Failure = std::function<void(std::string_view)>;

  /// @brief What the fan-out has done since it was made.
  struct Counts {
    std::uint64_t offered = 0;  ///< Frames @ref offer took.
    std::uint64_t taken = 0;    ///< Of them, handed to fusion.
    /// Of them, let go unfused: a newer frame came while fusion was busy,
    /// or fusion was not running.
    std::uint64_t skipped = 0;
  };

  FrameFanout(Stream stream, Fuse fuse, Failure failure)
      : stream_(std::move(stream)),
        fuse_(std::move(fuse)),
        failure_(std::move(failure)) {}

  /// Stops, as @ref stop.
  ~FrameFanout() { stop(); }
  FrameFanout(const FrameFanout&) = delete;
  FrameFanout& operator=(const FrameFanout&) = delete;

  /// @brief Start the worker.
  /// @return Whether it started; the failure callback is told why not.
  ///         Every frame offered while no worker runs is skipped.
  bool start() {
    if (worker_.joinable()) return false;
    set_open(true);
    const bool started = contain_frame_failure(
        [this] { worker_ = std::thread([this] { run(); }); }, failure_);
    if (!started) set_open(false);
    return started;
  }

  /// @brief The camera's next frame: to the stream at once, then into
  ///        fusion's slot, letting go of a frame still waiting there.
  ///
  /// An exception from the stream leaves here with @p frame unfused.
  void offer(Frame frame) {
    stream_(frame);
    std::optional<Frame> older;  // let go as this returns, outside the lock
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      ++counts_.offered;
      if (!open_) {
        ++counts_.skipped;
        return;
      }
      if (waiting_) {
        older.swap(waiting_);
        ++counts_.skipped;
      }
      waiting_.emplace(std::move(frame));
    }
    wake_.notify_one();
  }

  /// @brief Let the waiting frame go, wait for the fuse under way, and end
  ///        the worker. Idempotent.
  void stop() {
    std::optional<Frame> left;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      open_ = false;
      if (waiting_) {
        left.swap(waiting_);
        ++counts_.skipped;
      }
    }
    wake_.notify_one();
    left.reset();  // before the join, which may wait out a whole fuse
    if (worker_.joinable()) worker_.join();
  }

  /// @return The counts so far.
  Counts counts() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return counts_;
  }

 private:
  void set_open(bool open) {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = open;
  }

  void run() noexcept {
    for (;;) {
      std::optional<Frame> frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return waiting_.has_value() || !open_; });
        if (!waiting_) return;  // stopped
        frame.swap(waiting_);
        ++counts_.taken;
      }
      (void)contain_frame_failure([&] { fuse_(*frame); }, failure_);
      frame.reset();  // the frame goes back before the worker waits
    }
  }

  Stream stream_;
  Fuse fuse_;
  Failure failure_;
  std::thread worker_;  // the offering thread's

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  // Guarded by mutex_:
  std::optional<Frame> waiting_;
  bool open_ = false;  // the worker takes frames
  Counts counts_;
};

}  // namespace volumetric_kit::ios_app

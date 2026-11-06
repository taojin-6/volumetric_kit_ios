// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file view_picture_test.cpp
/// @brief The camera view holds at most one of the camera's pictures, lets it
///        go as soon as the frame that copied it has run, and only then wants
///        the next.

#include "ViewPicture.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// A picture whose holders can be counted: `use_count() == 1` means only the
// test still holds it.
using Picture = std::shared_ptr<const int>;
using View = app::ViewPicture<Picture>;

Picture make_picture(int id) { return std::make_shared<const int>(id); }

TEST(ViewPicture, KeepsTheFirstOfferedAndLeavesTheRest) {
  View view;
  const Picture first = make_picture(1);
  const Picture second = make_picture(2);
  EXPECT_TRUE(view.offer(first));
  EXPECT_EQ(first.use_count(), 2);
  EXPECT_FALSE(view.offer(second));
  EXPECT_EQ(second.use_count(), 1);
}

TEST(ViewPicture, NothingToTakeBeforeAnOffer) {
  View view;
  EXPECT_EQ(view.take(1), nullptr);
}

TEST(ViewPicture, TakeHandsThePictureOverOnce) {
  View view;
  const Picture picture = make_picture(1);
  ASSERT_TRUE(view.offer(picture));
  const Picture* taken = view.take(5);
  ASSERT_NE(taken, nullptr);
  EXPECT_EQ(taken->get(), picture.get());
  EXPECT_EQ(view.take(6), nullptr);
}

/// Held for as long as its frame may still read it, and refused company.
TEST(ViewPicture, HoldsThePictureUntilItsFrameHasRun) {
  View view;
  const Picture picture = make_picture(1);
  const Picture next = make_picture(2);
  ASSERT_TRUE(view.offer(picture));
  ASSERT_NE(view.take(5), nullptr);

  view.retire(4);
  EXPECT_EQ(picture.use_count(), 2);
  EXPECT_FALSE(view.offer(next));
  EXPECT_EQ(next.use_count(), 1);

  view.retire(5);
  EXPECT_EQ(picture.use_count(), 1);
  EXPECT_TRUE(view.offer(next));
  EXPECT_EQ(next.use_count(), 2);
}

/// A picture no frame has copied waits, however far the frames have run.
TEST(ViewPicture, ARetireLeavesAWaitingPicture) {
  View view;
  const Picture picture = make_picture(1);
  ASSERT_TRUE(view.offer(picture));
  view.retire(100);
  EXPECT_EQ(picture.use_count(), 2);
  EXPECT_NE(view.take(101), nullptr);
}

TEST(ViewPicture, ClearLetsItGoAndWantsTheNext) {
  View view;
  const Picture waiting = make_picture(1);
  ASSERT_TRUE(view.offer(waiting));
  view.clear();
  EXPECT_EQ(waiting.use_count(), 1);

  const Picture copied = make_picture(2);
  ASSERT_TRUE(view.offer(copied));
  ASSERT_NE(view.take(1), nullptr);
  view.clear();
  EXPECT_EQ(copied.use_count(), 1);
  EXPECT_TRUE(view.offer(waiting));
}

TEST(ViewPicture, LetsTheLastPictureGoWhenDestroyed) {
  const Picture picture = make_picture(1);
  {
    View view;
    ASSERT_TRUE(view.offer(picture));
  }
  EXPECT_EQ(picture.use_count(), 1);
}

// A picture counting how many of it are alive at once, and the most there ever
// were: the pool argument, as a number.
struct Counted {
  explicit Counted(std::atomic<int>* live_count, std::atomic<int>* most_seen)
      : live(live_count), most(most_seen) {
    arrive();
  }
  Counted(const Counted& other) : live(other.live), most(other.most) {
    arrive();
  }
  Counted& operator=(const Counted&) = delete;
  ~Counted() { live->fetch_sub(1); }

  void arrive() {
    const int now = live->fetch_add(1) + 1;
    int seen = most->load();
    while (now > seen && !most->compare_exchange_weak(seen, now)) {
    }
  }

  std::atomic<int>* live;
  std::atomic<int>* most;
};

/// The camera's poller and the render loop at once: the view never holds a
/// second picture while it has one, across a stream of offers.
TEST(ViewPicture, NeverHoldsTwoWithAPollerRunning) {
  std::atomic<int> live{0};
  std::atomic<int> most{0};
  {
    app::ViewPicture<Counted> view;
    std::atomic<bool> running{true};
    const Counted source(&live, &most);  // the camera's frame in hand
    std::thread poller([&] {
      while (running.load()) (void)view.offer(source);
    });
    std::uint64_t frame = 0;
    std::uint64_t copied = 0;
    while (copied < 2000) {
      ++frame;
      view.retire(frame - 1);  // every frame before this one has run
      if (view.take(frame) != nullptr) ++copied;
    }
    running.store(false);
    poller.join();
  }
  EXPECT_EQ(live.load(), 0);
  EXPECT_LE(most.load(), 2);  // the poller's own frame, and the view's one
}

}  // namespace

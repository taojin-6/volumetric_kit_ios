// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file picture_slot_test.cpp
/// @brief A keyframe ring slot holds its picture until it is rebound, and
///        falls back to white when its set refuses a picture.

#include "PictureSlot.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// A picture whose holders can be counted: `use_count() == 1` means only the
// test still holds it.
using Picture = std::shared_ptr<const int>;
using Slot = app::PictureSlot<Picture>;

Picture make_picture(int id) { return std::make_shared<const int>(id); }

// What a texture set would name: the picture it was last bound to, or white.
struct FakeSet {
  std::string names = "white";
  std::vector<std::string> calls;
};

std::string name_of(const Picture& p) { return std::to_string(*p); }

// Rebinds through a set that behaves as gfx's does: it accepts the picture
// unless told to refuse, and is left as it was when it refuses.
bool rebind_to(Slot& slot, FakeSet& set, std::optional<Picture> picture,
               bool refuse = false) {
  return slot.rebind(
      std::move(picture),
      [&](const Picture& p) {
        set.calls.push_back("bind " + name_of(p));
        if (refuse) return false;
        set.names = name_of(p);
        return true;
      },
      [&] {
        set.calls.push_back("white");
        set.names = "white";
      });
}

TEST(PictureSlot, StartsOnWhite) {
  const Slot slot;
  EXPECT_EQ(slot.picture(), nullptr);
}

TEST(PictureSlot, HoldsItsPictureUntilRebound) {
  Slot slot;
  FakeSet set;
  const Picture first = make_picture(1);
  ASSERT_TRUE(rebind_to(slot, set, first));
  EXPECT_EQ(set.names, "1");
  ASSERT_NE(slot.picture(), nullptr);
  EXPECT_EQ(*slot.picture(), first);
  EXPECT_EQ(first.use_count(), 2);

  // Another slot's rebind leaves this one's picture held.
  Slot other;
  FakeSet other_set;
  ASSERT_TRUE(rebind_to(other, other_set, make_picture(9)));
  EXPECT_EQ(first.use_count(), 2);

  // The old picture is still held while the set is rewritten over the new
  // one, and let go only after.
  const Picture second = make_picture(2);
  long first_holders_during_bind = 0;
  ASSERT_TRUE(slot.rebind(
      std::optional<Picture>(second),
      [&](const Picture& p) {
        first_holders_during_bind = first.use_count();
        set.names = name_of(p);
        return true;
      },
      [&] { set.names = "white"; }));
  EXPECT_EQ(first_holders_during_bind, 2);
  EXPECT_EQ(first.use_count(), 1);
  EXPECT_EQ(second.use_count(), 2);
  EXPECT_EQ(set.names, "2");
  EXPECT_EQ(*slot.picture(), second);
}

TEST(PictureSlot, RefusedPictureFallsBackToWhite) {
  Slot slot;
  FakeSet set;
  const Picture first = make_picture(1);
  ASSERT_TRUE(rebind_to(slot, set, first));

  const Picture refused = make_picture(2);
  long first_holders_during_white = 0;
  EXPECT_FALSE(slot.rebind(
      std::optional<Picture>(refused), [](const Picture&) { return false; },
      [&] {
        first_holders_during_white = first.use_count();
        set.names = "white";
      }));
  // The set left the old picture only once it named white.
  EXPECT_EQ(first_holders_during_white, 2);
  EXPECT_EQ(set.names, "white");
  EXPECT_EQ(slot.picture(), nullptr);
  EXPECT_EQ(first.use_count(), 1);
  // The refused picture is not kept either.
  EXPECT_EQ(refused.use_count(), 1);
}

TEST(PictureSlot, RefusalTriesThePictureOnce) {
  Slot slot;
  FakeSet set;
  EXPECT_FALSE(rebind_to(slot, set, make_picture(3), /*refuse=*/true));
  EXPECT_EQ(set.calls, (std::vector<std::string>{"bind 3", "white"}));
  EXPECT_EQ(set.names, "white");
}

TEST(PictureSlot, NoPictureBindsWhiteAndLetsTheOldOneGo) {
  Slot slot;
  FakeSet set;
  const Picture first = make_picture(1);
  ASSERT_TRUE(rebind_to(slot, set, first));
  set.calls.clear();

  EXPECT_FALSE(rebind_to(slot, set, std::nullopt));
  EXPECT_EQ(set.calls, (std::vector<std::string>{"white"}));
  EXPECT_EQ(slot.picture(), nullptr);
  EXPECT_EQ(first.use_count(), 1);
}

TEST(PictureSlot, TakesAPictureAgainAfterARefusal) {
  Slot slot;
  FakeSet set;
  EXPECT_FALSE(rebind_to(slot, set, make_picture(1), /*refuse=*/true));
  const Picture next = make_picture(2);
  EXPECT_TRUE(rebind_to(slot, set, next));
  EXPECT_EQ(set.names, "2");
  EXPECT_EQ(next.use_count(), 2);
}

}  // namespace

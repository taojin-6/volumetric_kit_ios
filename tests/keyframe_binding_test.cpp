// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file keyframe_binding_test.cpp
/// @brief A keyframe's planes, size and coding reach its slot's texture set
///        as they are; a keyframe with no planes, or chroma the set does not
///        sample, falls back to white.

#include "KeyframeBinding.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// Look-alikes of recon's YuvImage and gfx's YcbcrPicture: the members, the
// defaults and the enumerators the bridge's instantiation uses.
struct Image {};

enum class ChromaLocation : std::uint8_t {
  Left,
  Center,
  TopLeft,
  Top,
  BottomLeft,
  Bottom,
};

struct Keyframe {
  std::shared_ptr<const Image> image[2];
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  float kr = 0.299f;
  float kb = 0.114f;
  bool full_range = true;
  ChromaLocation chroma_location = ChromaLocation::Left;
};

enum class ChromaSiting : std::uint32_t { Center = 0, Left = 1 };

struct Extent {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

struct Coding {
  float kr = 0.299f;
  float kb = 0.114f;
  bool full_range = true;
  ChromaSiting chroma_siting = ChromaSiting::Center;
};

struct Planes {
  const Image* luma = nullptr;
  const Image* chroma = nullptr;
  Extent extent{};
  Coding coding;
};

using Slot = app::PictureSlot<Keyframe>;

// What the set was last pointed at, and every rebind, in order.
struct FakeSet {
  std::optional<Planes> names;  // none: white
  std::vector<std::optional<Planes>> calls;
};

// A BT.709 limited-range keyframe, 4:3 like ARKit's capturedImage: every
// value differs from gfx's defaults, so one the mapping drops shows.
Keyframe make_keyframe(ChromaLocation location = ChromaLocation::Left) {
  Keyframe k;
  k.image[0] = std::make_shared<const Image>();
  k.image[1] = std::make_shared<const Image>();
  k.width = 1920;
  k.height = 1440;
  k.kr = 0.2126f;
  k.kb = 0.0722f;
  k.full_range = false;
  k.chroma_location = location;
  return k;
}

const char* rebind_to(Slot& slot, FakeSet& set, Keyframe keyframe) {
  return app::rebind_keyframe<Planes>(
      slot, std::move(keyframe),
      [&](const Planes& planes) {
        set.calls.emplace_back(planes);
        set.names = planes;
        return true;
      },
      [&] {
        set.calls.emplace_back(std::nullopt);
        set.names.reset();
      });
}

TEST(KeyframeBinding, PlanesSizeAndCodingReachTheSet) {
  Slot slot;
  FakeSet set;
  const Keyframe k = make_keyframe(ChromaLocation::Left);
  EXPECT_EQ(rebind_to(slot, set, k), nullptr);

  ASSERT_EQ(set.calls.size(), 1u);
  ASSERT_TRUE(set.names.has_value());
  const Planes& p = *set.names;
  EXPECT_EQ(p.luma, k.image[0].get());
  EXPECT_EQ(p.chroma, k.image[1].get());
  EXPECT_EQ(p.extent.width, 1920u);
  EXPECT_EQ(p.extent.height, 1440u);
  EXPECT_FLOAT_EQ(p.coding.kr, 0.2126f);
  EXPECT_FLOAT_EQ(p.coding.kb, 0.0722f);
  EXPECT_FALSE(p.coding.full_range);
  EXPECT_EQ(p.coding.chroma_siting, ChromaSiting::Left);

  // The slot holds the very planes the set names.
  ASSERT_NE(slot.picture(), nullptr);
  EXPECT_EQ(slot.picture()->image[0], k.image[0]);
  EXPECT_EQ(slot.picture()->image[1], k.image[1]);
}

TEST(KeyframeBinding, CentredChromaStaysCentred) {
  Slot slot;
  FakeSet set;
  Keyframe k = make_keyframe(ChromaLocation::Center);
  k.full_range = true;
  EXPECT_EQ(rebind_to(slot, set, std::move(k)), nullptr);
  ASSERT_TRUE(set.names.has_value());
  EXPECT_EQ(set.names->coding.chroma_siting, ChromaSiting::Center);
  EXPECT_TRUE(set.names->coding.full_range);
}

TEST(KeyframeBinding, OtherSitingsFallBackToWhite) {
  for (const ChromaLocation location :
       {ChromaLocation::TopLeft, ChromaLocation::Top,
        ChromaLocation::BottomLeft, ChromaLocation::Bottom}) {
    Slot slot;
    FakeSet set;
    const Keyframe before = make_keyframe();
    ASSERT_EQ(rebind_to(slot, set, before), nullptr);
    set.calls.clear();

    const Keyframe k = make_keyframe(location);
    EXPECT_NE(rebind_to(slot, set, k), nullptr);
    // Never offered to the set: white, holding neither picture.
    ASSERT_EQ(set.calls.size(), 1u);
    EXPECT_FALSE(set.calls[0].has_value());
    EXPECT_EQ(slot.picture(), nullptr);
    EXPECT_EQ(before.image[0].use_count(), 1);
    EXPECT_EQ(k.image[0].use_count(), 1);
  }
}

// What the renderer gets for a mesh that was not textured, a refused import
// among the reasons: no planes, or only one.
TEST(KeyframeBinding, KeyframeWithoutPlanesBindsWhiteQuietly) {
  Keyframe luma_only = make_keyframe();
  luma_only.image[1].reset();
  Keyframe chroma_only = make_keyframe();
  chroma_only.image[0].reset();
  for (const Keyframe& planeless : {Keyframe{}, luma_only, chroma_only}) {
    Slot slot;
    FakeSet set;
    const Keyframe before = make_keyframe();
    ASSERT_EQ(rebind_to(slot, set, before), nullptr);
    set.calls.clear();

    EXPECT_EQ(rebind_to(slot, set, planeless), nullptr);
    ASSERT_EQ(set.calls.size(), 1u);
    EXPECT_FALSE(set.calls[0].has_value());
    EXPECT_EQ(slot.picture(), nullptr);
    EXPECT_EQ(before.image[0].use_count(), 1);
  }
}

TEST(KeyframeBinding, SetRefusalFallsBackToWhite) {
  Slot slot;
  FakeSet set;
  int offered = 0;
  bool white = false;
  // The set's own reason is the caller's to report.
  EXPECT_EQ(app::rebind_keyframe<Planes>(
                slot, make_keyframe(),
                [&](const Planes&) {
                  ++offered;
                  return false;
                },
                [&] { white = true; }),
            nullptr);
  EXPECT_EQ(offered, 1);
  EXPECT_TRUE(white);
  EXPECT_EQ(slot.picture(), nullptr);
}

}  // namespace

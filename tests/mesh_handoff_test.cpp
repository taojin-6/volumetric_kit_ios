// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file mesh_handoff_test.cpp
/// @brief A frame draws the live mesh only when it has buffers and triangles,
///        and the bring-up triangle only before the first commit; a committed
///        mesh's keyframe lands in the next ring slot, and the mesh commits
///        even when its keyframe cannot be bound.

#include "MeshHandoff.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;

// A look-alike of grid's DeviceMesh: the members live_draw reads.
struct Mesh {
  std::uint64_t generation = 0;
  bool buffers = false;
  std::uint32_t triangle_count = 0;
  bool valid() const noexcept { return buffers; }
  bool empty() const noexcept { return triangle_count == 0; }
};

TEST(LiveDraw, BeforeAnyCommitIsTheBringUpTriangle) {
  EXPECT_EQ(app::live_draw(Mesh{}), app::LiveDraw::kBringUp);
}

TEST(LiveDraw, ACommittedMeshWithTrianglesIsDrawn) {
  EXPECT_EQ(app::live_draw(Mesh{1, true, 12}), app::LiveDraw::kMesh);
}

TEST(LiveDraw, AnEmptyMeshDrawsNothingWithOrWithoutBuffers) {
  // grid's contract lets an extract that meshed nothing name null buffers;
  // binding them would be undefined, and the triangle is only for before the
  // first commit.
  EXPECT_EQ(app::live_draw(Mesh{3, false, 0}), app::LiveDraw::kNothing);
  EXPECT_EQ(app::live_draw(Mesh{3, true, 0}), app::LiveDraw::kNothing);
}

TEST(LiveDraw, ANullBufferIsNeverDrawn) {
  EXPECT_EQ(app::live_draw(Mesh{4, false, 12}), app::LiveDraw::kNothing);
}

// A ring slot that records what it was bound to.
struct Slot {
  std::vector<std::string> bound;
};

TEST(CommitKeyframe, BindsIntoTheNextSlotAndMakesItLive) {
  Slot slots[3];
  std::size_t live = 0;
  const auto bind = [](Slot& slot, std::string keyframe) {
    slot.bound.push_back(std::move(keyframe));
  };
  EXPECT_TRUE(app::commit_keyframe(slots, live, std::string("a"), bind));
  EXPECT_EQ(live, 1u);
  EXPECT_TRUE(app::commit_keyframe(slots, live, std::string("b"), bind));
  EXPECT_EQ(live, 2u);
  // Round the ring: slot 0 is rebound third, after the two before it.
  EXPECT_TRUE(app::commit_keyframe(slots, live, std::string("c"), bind));
  EXPECT_EQ(live, 0u);
  EXPECT_EQ(slots[1].bound, std::vector<std::string>{"a"});
  EXPECT_EQ(slots[2].bound, std::vector<std::string>{"b"});
  EXPECT_EQ(slots[0].bound, std::vector<std::string>{"c"});
}

TEST(CommitKeyframe, CommitsWhenTheKeyframeCannotBeBound) {
  // A refused keyframe leaves the slot white and the mesh still commits:
  // parking it would hold every later mesh back.
  Slot slots[3];
  std::size_t live = 2;
  int refused = 0;
  EXPECT_TRUE(app::commit_keyframe(slots, live, std::string("x"),
                                   [&](Slot& slot, std::string) {
                                     slot.bound.push_back("white");
                                     ++refused;
                                   }));
  EXPECT_EQ(live, 0u);
  EXPECT_EQ(refused, 1);
  EXPECT_EQ(slots[0].bound, std::vector<std::string>{"white"});
}

}  // namespace

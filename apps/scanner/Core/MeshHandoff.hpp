// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file MeshHandoff.hpp
/// @brief The renderer's two rules on its side of grid's `MeshExchange`: what
///        a frame draws for the live mesh, and how a committed mesh's keyframe
///        lands in the keyframe ring.
///
/// Pure, so both can be tested on the host: the renderer
/// (Bridge/VolumetricRenderer.mm) instantiates them with grid's `DeviceMesh`
/// and the bridge's `AtlasSlot`; a test with look-alikes.

#include <cstddef>
#include <cstdint>
#include <utility>

namespace volumetric_kit::ios_app {

/// @brief What a frame draws for the exchange's live mesh.
enum class LiveDraw : std::uint8_t {
  /// Nothing committed yet: the bring-up triangle.
  kBringUp,
  /// An empty mesh is live. It draws nothing, and may name null buffers.
  kNothing,
  /// Draw the mesh.
  kMesh,
};

/// @tparam Mesh  grid's `DeviceMesh`: `generation`, `valid()` and `empty()`.
/// @param live   The exchange's live mesh, generation 0 before the first
///               commit.
template <typename Mesh>
LiveDraw live_draw(const Mesh& live) noexcept {
  if (live.generation == 0) {
    return LiveDraw::kBringUp;
  }
  return live.valid() && !live.empty() ? LiveDraw::kMesh : LiveDraw::kNothing;
}

/// @brief The renderer's commit: bind @p keyframe into the ring slot after
///        @p live, and make that slot live.
///
/// Always commits, also when @p bind refuses the keyframe: the slot then
/// samples white and the mesh draws with its textured triangles white.
/// Parking the mesh instead would hold back every later mesh for as long as
/// binding fails. One slot per commit, so a slot is rebound only after the
/// ring has gone round, which outlasts the frames in flight.
///
/// @param slots     The ring.
/// @param live      The slot the live mesh's keyframe is bound in; advanced.
/// @param keyframe  The committed mesh's keyframe, moved into @p bind.
/// @param bind      `void(Slot&, Keyframe)`: points the slot at the keyframe,
///                  or at white when it cannot.
/// @return `true`: commit the mesh.
template <typename Slot, std::size_t N, typename Keyframe, typename Bind>
bool commit_keyframe(Slot (&slots)[N], std::size_t& live, Keyframe&& keyframe,
                     Bind&& bind) {
  live = (live + 1) % N;
  bind(slots[live], std::forward<Keyframe>(keyframe));
  return true;
}

}  // namespace volumetric_kit::ios_app

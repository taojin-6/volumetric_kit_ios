// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file MeshHandoff.hpp
/// @brief The scanner's rules on top of grid's `MeshExchange`: how deep the
///        mesh ring must be, what a frame draws for the live mesh, and how a
///        committed mesh's keyframe lands in the keyframe ring.
///
/// Pure, so all three can be tested on the host: Bridge/Fusion.mm checks the
/// depth, and the renderer (Bridge/VolumetricRenderer.mm) instantiates the
/// other two with grid's `DeviceMesh` and the bridge's `AtlasSlot`; a test
/// with look-alikes.

#include <cstddef>
#include <cstdint>
#include <utility>

namespace volumetric_kit::ios_app {

/// @brief Why a mesh ring of @p mesh_slots cannot serve a renderer keeping
///        @p frames_in_flight frames, or null if it can.
///
/// grid's `slot_count` rule for this producer, which never extracts while a
/// mesh waits to be taken, and this consumer, which never drops one: one slot
/// more than the frames in flight. A shallower ring is refused extracts while
/// frames still draw every slot, so the scan meshes in fits.
///
/// @param mesh_slots        `FusionConfig::mesh_slots`.
/// @param frames_in_flight  `MeshExchangeConfig::frames_in_flight`.
inline const char* mesh_ring_refusal(std::uint32_t mesh_slots,
                                     std::uint32_t frames_in_flight) noexcept {
  if (frames_in_flight == 0) {
    return "the renderer keeps no frame in flight";
  }
  // Not `mesh_slots < frames_in_flight + 1`, which wraps at the top.
  if (mesh_slots <= frames_in_flight) {
    return "the ring needs one slot more than the renderer's frames in flight";
  }
  return nullptr;
}

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

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file AtlasRing.hpp
/// @brief The keyframes the textured mesh samples, one per mesh slot.
///
/// Each keyframe is the camera's picture where it is: ARKit's `capturedImage`,
/// its two planes imported by recon's `PixelBufferImporter` and sampled by
/// gfx's texture set, with no copy. A ring rather than one set for the same
/// reason the mesh slots are a ring: a slot's set is rewritten, and its old
/// picture let go, only once no pending frame has bound it. Indexed by
/// `mesh_slot`, since a mesh and the keyframe its `uv0` address are one value.

#include <cstddef>
#include <memory>

#include "PictureSlot.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_texture_set.hpp"
#include "volumetric_kit/recon/sensor/yuv_image.hpp"

namespace volumetric_kit::ios_app {

namespace vkc = volumetric_kit::core;

/// @brief How many keyframes the ring holds -- one per mesh slot.
///
/// The depth is the renderer's: one more than the frames the CPU runs ahead,
/// so a slot rebound now was last bound by a frame that has retired.
/// @ref RendererImpl static_asserts that its `kMeshSlots` agrees.
inline constexpr std::size_t kRingSlots = 3;

/// @brief A keyframe: the camera picture's two plane images and how they code
///        R'G'B'. The images hold the picture's pixel buffer.
using Keyframe = volumetric_kit::recon::sensor::YuvImage;

/// @brief One mesh slot's keyframe.
struct AtlasSlot {
  /// The hybrid pipeline's set 0: made at bring-up over the white texture,
  /// rebound with every mesh that lands in the slot.
  std::unique_ptr<volumetric_kit::gfx::pipelines::HybridMeshTextureSet> set;
  /// The picture @ref set samples, held until the slot is rebound.
  PictureSlot<Keyframe> picture;
};

/// @brief The renderer's keyframes.
///
/// @warning Holds gfx resources, so it must be declared *after* the
///          `WindowedApp` that made them. See the teardown note on
///          @ref RendererImpl.
struct AtlasRing {
  /// Indexed by `mesh_slot`.
  AtlasSlot slots[kRingSlots];
};

/// @brief Point @p slot at @p keyframe, or at @p white when it has no planes
///        or cannot be sampled.
///
/// The slot then holds @p keyframe until it is next rebound, and has let go of
/// the picture it held before. Core's `rebind_keyframe` (KeyframeBinding.hpp)
/// decides what the set samples; this binds it through gfx's set.
///
/// @pre No pending frame has bound the slot's set, and none being recorded
///      has: the ring depth guarantees it for the slot a new mesh lands in.
/// @param slot      The slot, its set made.
/// @param keyframe  The mesh's keyframe; no planes when the mesh carries none.
/// @param white     The white texture's view, in `SHADER_READ_ONLY_OPTIMAL`.
/// @return OK, or why @p keyframe cannot be sampled: its chroma siting, or
///         the set's refusal. The slot samples white then.
vkc::Status bind_keyframe(AtlasSlot& slot, Keyframe keyframe,
                          VkImageView white);

}  // namespace volumetric_kit::ios_app

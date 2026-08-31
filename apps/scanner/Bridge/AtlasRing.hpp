// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file AtlasRing.hpp
/// @brief The keyframe images the textured mesh samples, one per mesh slot.
///
/// A ring rather than one image for the same reason the mesh slots are a ring:
/// a slot the GPU may still be sampling cannot be rewritten. Indexed by
/// `mesh_slot` deliberately -- a mesh and the keyframe its `uv0` address are
/// one value, and binding them crossed samples the wrong place on every
/// textured triangle while still looking like a plausible image.

#import "RendererImpl.hpp"

#include <cstdint>

#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"

namespace volumetric_kit::ios_app {

/// Record the buffer -> image copy that publishes a staged keyframe, with the
/// barriers either side of it.
///
/// Recorded into the frame's own command buffer rather than submitted on its
/// own, which is the whole reason the atlas ring is affordable at remesh rate:
/// no second submit, no fence, and no contention for the submit mutex recon and
/// gfx share on this platform (see the warning at the top of Fusion.hpp).
///
/// @param first_write  True when @p image has never been written, so it is
///                     still in `VK_IMAGE_LAYOUT_UNDEFINED`. Naming the wrong
///                     old layout is undefined rather than diagnosed here --
///                     this build ships without validation layers -- and
///                     UNDEFINED is also the correct choice on a first write
///                     for the reason it exists: its contents need not be
///                     preserved, so the driver may discard rather than move
///                     them.
void record_atlas_upload(VkCommandBuffer cmd, VkBuffer staging, VkImage image,
                         std::uint32_t width, std::uint32_t height,
                         bool first_write);

/// @brief Give every atlas slot a real image at the colour camera's size.
///
/// **All or nothing.** Everything is built into locals and moved into `impl`
/// only once every slot has succeeded, so a refusal partway through leaves the
/// renderer exactly as it found it and frees what it had already taken on the
/// way out. The previous shape wrote each slot as it went and returned early on
/// the first failure, which left up to two 11 MB images and two 11 MB mapped
/// staging buffers owned by slots nothing would ever bind -- `atlas_ready`
/// stays false -- and nothing would ever free, because the only writer of those
/// slots is a later successful call. ~44 MB converted to unreachable resident
/// memory by a transient allocation failure, on the device where MemoryBudget
/// exists precisely because the working set is the binding constraint, and it
/// made the *next* allocation likelier to fail for the same reason.
///
/// Allocates no descriptor sets. They are handed out once at bring-up, and that
/// is what makes the retry this function's caller advertises actually possible:
/// gfx's `DescriptorPool::create` passes no flags and its header states the kit
/// does not free sets individually, so a set consumed by a slot built before a
/// mid-way failure was gone for good. Against a pool sized at exactly
/// `kMeshSlots + 1`, the second attempt then failed at `allocate` with
/// `VK_ERROR_OUT_OF_POOL_MEMORY` -- and so did every attempt after it, for the
/// life of the process. The white fallback stayed bound, every textured
/// triangle sampled one white texel, and the read-out called it transient.
///
/// Writing the descriptors here is safe only because `atlas_ready` is false for
/// the whole time this runs, including across a retry: no frame binds a slot
/// set until the flag goes up, so nothing is reading what this writes.
vg::Status build_atlas_ring(RendererImpl& impl, std::uint32_t width,
                            std::uint32_t height);

}  // namespace volumetric_kit::ios_app

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
///
/// @warning This is **not** `Fusion::atlas_ring_`, a different ring of the same
///          name in the same namespace one file over. That one is the producer
///          side: two host `std::vector<std::uint32_t>` entries written on the
///          fuse thread, alternating so `take_mesh` can hand one out while the
///          next remesh fills the other. This one is the consumer side:
///          @ref kRingSlots VkImages plus mapped staging buffers, written on
///          the render thread. Every load-bearing property differs, the depths
///          included, and neither depth argument transfers to the other ring --
///          Fusion's rests on the uncollected guard in `remesh`, this one's on
///          @ref kRingSlots below.

#include <cstddef>
#include <cstdint>

#include "volumetric_kit/gfx/core/allocator.hpp"
#include "volumetric_kit/gfx/core/buffer.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/texture.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"

namespace volumetric_kit::ios_app {

namespace vg = volumetric_kit::gfx;

/// @brief How many keyframe images the ring holds -- one per mesh slot.
///
/// The renderer's two rings are the same depth and have to be: this one is
/// indexed by `mesh_slot`, so slot i's keyframe belongs to slot i's mesh. The
/// depth is a *renderer* property -- one more than the frames the CPU runs
/// ahead, so a slot written now is untouched again by the time the ring comes
/// back round -- and it is stated here only because this is the file that
/// declares the arrays. @ref RendererImpl static_asserts that its
/// `kFramesInFlight` still agrees, so deepening the CPU-ahead pipeline without
/// deepening the rings is a build failure rather than a slot overwritten under
/// a live read.
inline constexpr std::size_t kRingSlots = 3;

/// @brief One keyframe image, paired with the mesh slot whose `uv0` index it.
///
/// Persistent, and that is the design rather than an optimisation. gfx's
/// `upload_texture` creates a fresh image plus a staging buffer and blocks on
/// a fence -- right for an asset loaded once, wrong at remesh rate.
/// `FusionConfig::remesh_every` is 1, so that shape would mean an 11 MB image
/// creation and a blocking graphics submit *every frame*, on the queue whose
/// submit mutex the warning at the top of Fusion.hpp is entirely about. These
/// are built once at the colour camera's size and rewritten in place: one host
/// memcpy into a mapped staging buffer, then a copy recorded into the frame's
/// own command buffer. Nothing is allocated, submitted or waited on per frame.
struct AtlasSlot {
  vg::Texture texture;
  /// Host-visible and persistently mapped: the render thread writes here and
  /// the GPU copies out inside the frame already being recorded, so there is no
  /// second submit and no fence.
  ///
  /// What makes that *safe* is the ring depth, and it is a second, independent
  /// use of the same argument the mesh ring rests on rather than a consequence
  /// of it. The host memcpy overwrites a buffer that a previously recorded
  /// `vkCmdCopyBufferToImage` may still be reading at the transfer stage, and
  /// nothing waits on that read. @ref kRingSlots being `kFramesInFlight + 1` is
  /// what covers it: a slot is rewritten only after that many accepted meshes,
  /// by which point `begin_frame` has waited on a fence newer than the frame
  /// that read it. Give the atlas slots a shallower index of their own, or
  /// decouple the depth from `kFramesInFlight`, and this wants re-deriving on
  /// its own terms -- the mesh ring's version of the argument is about a
  /// GpuMesh being replaced and does not reach the staging buffers.
  vg::Buffer staging;
  vg::DescriptorSet set;
};

/// @brief The renderer's keyframe images, and the state saying which may be
///        bound.
///
/// One member of @ref RendererImpl rather than eleven spread across it, so the
/// functions below can take what they actually operate on.
///
/// @warning Holds gfx resources, so it must be declared *after* the
///          `WindowedApp` whose allocator produced them. See the teardown note
///          on @ref RendererImpl.
struct AtlasRing {
  /// Indexed by `mesh_slot`, deliberately: a mesh and the keyframe its `uv0`
  /// address are one value, so slot i's atlas belongs to slot i's mesh. Binding
  /// them crossed samples the wrong place on every textured triangle -- and
  /// looks like a plausible image, not like an error.
  AtlasSlot slots[kRingSlots];
  /// The colour camera's dimensions the ring was built for, or 0 before the
  /// first textured mesh arrives. ARKit does not change `imageResolution`
  /// mid-session, but a ring built for one size and fed another would read past
  /// the staged image; the upload checks this rather than assuming it.
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  /// Whether @ref slots hold real images yet; until then every frame binds the
  /// 1x1 white set.
  bool ready = false;
  /// Which slots may be *bound*. Not a statement about image layout -- see
  /// @ref slot_in_undefined_layout, which is.
  ///
  /// Cleared whenever a slot must stop being bindable for a reason that leaves
  /// its image exactly as it was: a keyframe that could not be staged, or
  /// `drawMesh` switched off while meshes keep arriving. A slot that is not
  /// bindable binds the 1x1 white set instead, which is wrong-looking and
  /// honest rather than a plausible photograph of somewhere else.
  bool slot_written[kRingSlots] = {};
  /// Which slots are still in `VK_IMAGE_LAYOUT_UNDEFINED` -- never uploaded
  /// since their image was created.
  ///
  /// Separate from @ref slot_written, and not interchangeable with it even
  /// though the two start out equal. That one is a bindability flag and is
  /// cleared for reasons that do not touch the image, so deriving
  /// `first_write` from it eventually claims UNDEFINED for an image sitting in
  /// `SHADER_READ_ONLY_OPTIMAL`. Legal in itself -- UNDEFINED merely discards
  /// contents -- but it also drops the upload barrier's source scope to
  /// `TOP_OF_PIPE` with an empty access mask, deleting the dependency on the
  /// fragment-shader read that @ref record_atlas_upload's barrier exists to
  /// order. Only @ref build_atlas_ring sets this, because only it creates
  /// images; only a completed upload clears it.
  bool slot_in_undefined_layout[kRingSlots] = {};
};

/// @brief The staging-buffer size for a @p width x @p height keyframe, and the
///        one definition of it.
///
/// The allocation and the `memcpy` that fills it are in different translation
/// units, so a format change away from a four-byte texel could be applied to
/// one and not the other -- and getting only the allocation writes past the
/// mapped buffer, with no diagnostic on a build that ships without validation
/// layers. The same argument the `bufferRowLength` comment inside
/// @ref record_atlas_upload makes for not restating the packing.
constexpr VkDeviceSize atlas_staging_bytes(std::uint32_t width,
                                           std::uint32_t height) noexcept {
  return static_cast<VkDeviceSize>(width) * height * sizeof(std::uint32_t);
}

/// Record the buffer -> image copy that publishes a staged keyframe, with the
/// barriers either side of it.
///
/// Recorded into the frame's own command buffer rather than submitted on its
/// own, which is the whole reason the atlas ring is affordable at remesh rate:
/// no second submit, no fence, and no contention for the submit mutex recon and
/// gfx share on this platform (see the warning at the top of Fusion.hpp).
///
/// @param cmd          The frame's command buffer, recording, and outside any
///                     render pass instance -- see the precondition below.
/// @param staging      A mapped host-visible buffer holding at least
///                     @ref atlas_staging_bytes bytes for @p width x @p height,
///                     already filled with the keyframe.
/// @param image        The slot's image, created `TRANSFER_DST | SAMPLED` at
///                     exactly @p width x @p height, one mip and one layer.
/// @param width        The keyframe's width, which must equal the image's.
/// @param height       The keyframe's height, which must equal the image's.
/// @param first_write  True when @p image has never been written, so it is
///                     still in `VK_IMAGE_LAYOUT_UNDEFINED`. Naming the wrong
///                     old layout is undefined rather than diagnosed here --
///                     this build ships without validation layers -- and
///                     UNDEFINED is also the correct choice on a first write
///                     for the reason it exists: its contents need not be
///                     preserved, so the driver may discard rather than move
///                     them. Take it from `AtlasRing::slot_in_undefined_layout`
///                     and not from `slot_written`, which answers a different
///                     question; that member says why.
///
/// @pre @p cmd is **outside a render pass instance**. `vkCmdCopyBufferToImage`
///      is an outside-only command, so recording this below the frame's
///      `target->begin(...)` is invalid usage -- and with no validation layers
///      it is not diagnosed: on MoltenVK the copy is silently dropped or
///      mis-ordered, which reads as a keyframe that "sometimes" lands rather
///      than as an error. The renderer satisfies this by position, uploading
///      above `begin` while nearly every other `vkCmd*` in that file is below
///      it, so a new caller has nothing local to copy.
void record_atlas_upload(VkCommandBuffer cmd, VkBuffer staging, VkImage image,
                         std::uint32_t width, std::uint32_t height,
                         bool first_write);

/// @brief Give every atlas slot a real image at the colour camera's size.
///
/// **All or nothing.** Everything is built into locals and moved into @p ring
/// only once every slot has succeeded, so a refusal partway through leaves the
/// ring exactly as it found it and frees what it had already taken on the way
/// out. The previous shape wrote each slot as it went and returned early on the
/// first failure, which left up to two 11 MB images and two 11 MB mapped
/// staging buffers owned by slots nothing would ever bind -- `ready` stays
/// false -- and nothing would ever free, because the only writer of those slots
/// is a later successful call. ~44 MB converted to unreachable resident memory
/// by a transient allocation failure, on the device where MemoryBudget exists
/// precisely because the working set is the binding constraint, and it made the
/// *next* allocation likelier to fail for the same reason.
///
/// Allocates no descriptor sets. They are handed out once at bring-up, and that
/// is what makes the retry this function's caller advertises actually possible:
/// gfx's `DescriptorPool::create` passes no flags and its header states the kit
/// does not free sets individually, so a set consumed by a slot built before a
/// mid-way failure was gone for good. Against a pool sized at exactly
/// `kRingSlots + 1`, the second attempt then failed at `allocate` with
/// `VK_ERROR_OUT_OF_POOL_MEMORY` -- and so did every attempt after it, for the
/// life of the process. The white fallback stayed bound, every textured
/// triangle sampled one white texel, and the read-out called it transient.
///
/// @param ring       The ring to fill. Must not already be built.
/// @param allocator  Produces the images and staging buffers, and must outlive
///                   @p ring -- see the teardown note on @ref RendererImpl.
/// @param sampler    Written into every slot's descriptor alongside its view.
/// @param width      ARKit's colour width, which sizes every slot.
/// @param height     ARKit's colour height.
///
/// All three preconditions are **checked and refused** with a `Status` rather
/// than left to the single caller that satisfies them today. This is a header
/// symbol now, reachable from every bridge translation unit:
///
/// - `!ring.ready`. Rebuilding a live ring frees images that frames in flight
///   are binding -- `vg::Texture`'s move-assignment destroys eagerly, so the
///   commit loop would `vkDestroyImage` up to @ref kRingSlots images and their
///   mapped staging buffers while submitted command buffers still sample them,
///   then rewrite the descriptor sets those buffers bound. No queue drain, no
///   validation layer: it surfaces as `VK_ERROR_DEVICE_LOST` out of a fence
///   wait several submissions downstream. The obvious second caller is an
///   `imageResolution` change, which is exactly the path the renderer's
///   extent-mismatch branch refuses today for this reason.
/// - Every `ring.slots[i].set` already allocated. Writing a descriptor opens
///   with a `VG_CHECK` that aborts in every build, and it would do so
///   *after* the six allocations -- leaving images and staging buffers moved
///   into @p ring with `width`/`height` still 0, the exact non-atomic state the
///   all-or-nothing shape above exists to prevent.
/// - `sampler != VK_NULL_HANDLE`, for the same reason: the renderer's sampler
///   is an engaged-only `std::optional`, and dereferencing an empty one here
///   would be undefined behaviour a hundred lines from where it is created.
///
/// Writing the descriptors here is safe only because `ring.ready` is false for
/// the whole time this runs, including across a retry: no frame binds a slot
/// set until the flag goes up, so nothing is reading what this writes.
vg::Status build_atlas_ring(AtlasRing& ring, vg::Allocator& allocator,
                            VkSampler sampler, std::uint32_t width,
                            std::uint32_t height);

}  // namespace volumetric_kit::ios_app

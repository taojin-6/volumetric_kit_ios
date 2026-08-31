// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "AtlasRing.hpp"

#include <cstddef>
#include <utility>

namespace volumetric_kit::ios_app {

void record_atlas_upload(VkCommandBuffer cmd, VkBuffer staging, VkImage image,
                         std::uint32_t width, std::uint32_t height,
                         bool first_write) {
  // Zero-initialised and then stamped, like every other Vulkan struct here.
  // Naming sType in the braces leaves the remaining fields to aggregate
  // initialisation, which -Wextra reports as a missing initialiser -- and this
  // target builds with -Wall -Wextra since the review of #29.
  VkImageMemoryBarrier to_dst{};
  to_dst.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  to_dst.srcAccessMask = first_write ? 0 : VK_ACCESS_SHADER_READ_BIT;
  to_dst.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
  to_dst.oldLayout = first_write ? VK_IMAGE_LAYOUT_UNDEFINED
                                 : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  to_dst.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  to_dst.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  to_dst.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  to_dst.image = image;
  to_dst.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  // The source scope is the FRAGMENT shader, not TOP_OF_PIPE: the previous
  // frame that bound this slot sampled it there, and this copy must not begin
  // until that read has finished. The slot ring makes that frame an old one in
  // the steady state, but "old" is not a dependency -- gfx exposes no fence to
  // wait on and no semaphore may cross this seam, so the barrier is what
  // orders them.
  vkCmdPipelineBarrier(cmd,
                       first_write ? VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT
                                   : VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                       VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                       nullptr, 1, &to_dst);

  VkBufferImageCopy region{};
  region.bufferOffset = 0;
  // Zero means "tightly packed to imageExtent", which the staging buffer is:
  // it is sized exactly width * height * 4 and written by one memcpy from a
  // vector of the same shape. Stating the row length instead would be a second
  // place for the packing to be described, and so a second place to be wrong.
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  region.imageOffset = {0, 0, 0};
  region.imageExtent = {width, height, 1};
  vkCmdCopyBufferToImage(cmd, staging, image,
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

  VkImageMemoryBarrier to_read = to_dst;
  to_read.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
  to_read.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  to_read.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  to_read.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                       VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0,
                       nullptr, 1, &to_read);
}

vg::Status build_atlas_ring(RendererImpl& impl, std::uint32_t width,
                            std::uint32_t height) {
  if (width == 0 || height == 0) {
    return vg::Status::invalid_argument("atlas ring: zero colour extent");
  }
  const VkDeviceSize bytes =
      static_cast<VkDeviceSize>(width) * height * sizeof(std::uint32_t);

  // Staged here, committed below. Destroying these on an early return is the
  // whole point -- see the note above.
  vg::Texture textures[RendererImpl::kMeshSlots];
  vg::Buffer stagings[RendererImpl::kMeshSlots];

  for (std::size_t i = 0; i < RendererImpl::kMeshSlots; ++i) {
    vg::TextureDesc tex_desc;
    tex_desc.extent = {width, height};
    // _SRGB, matching `fuse_render`'s atlas and for its two reasons.
    //
    // The atlas holds canonical-encoded 8-bit R'G'B' -- what ARKitCapture
    // publishes and what the TSDF fuses -- while `Vertex::color`, the other
    // albedo source the shader picks between per triangle, is **linear**:
    // marching cubes decodes the encoded voxel attribute at the corner gather
    // (recon's mesh.hpp says so where it declares the field). An _SRGB view
    // makes the sampler decode too, so both sources arrive linear and a surface
    // does not change brightness at the seam where texturing stops. _UNORM
    // leaves the atlas encoded, which reads lighter and flatter than the voxel
    // colour beside it -- a real mismatch, not a subtle one.
    //
    // It also fixes the filtering, which is the half easy to miss: bilinear on
    // an _UNORM view averages ENCODED values, and an average of encoded values
    // is not the encoding of the average. _SRGB decodes before filtering.
    tex_desc.format = VK_FORMAT_R8G8B8A8_SRGB;
    tex_desc.usage =
        VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    vg::Result<vg::Texture> tex = impl.app.allocator().create_image(tex_desc);
    if (!tex) {
      return tex.status();
    }

    vg::BufferDesc buf_desc;
    buf_desc.size = bytes;
    buf_desc.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buf_desc.memory = vg::MemoryUsage::HostVisible;
    buf_desc.mapped = true;
    // Sequential: this is written by one memcpy front to back and never read
    // back, which on a write-combined mapping is the difference between a
    // streaming store and a read-modify-write per cache line.
    buf_desc.host_access = vg::HostAccess::SequentialWrite;
    vg::Result<vg::Buffer> staging =
        impl.app.allocator().create_buffer(buf_desc);
    if (!staging) {
      return staging.status();
    }

    textures[i] = std::move(tex).value();
    stagings[i] = std::move(staging).value();
  }

  // --- Commit. Nothing below can fail ---------------------------------------
  for (std::size_t i = 0; i < RendererImpl::kMeshSlots; ++i) {
    impl.atlas_slots[i].texture = std::move(textures[i]);
    impl.atlas_slots[i].staging = std::move(stagings[i]);
    // Written once, here: the image the set points at never changes, only its
    // contents. That is what makes the per-frame path a copy rather than a
    // descriptor update. The set itself was allocated at bring-up.
    impl.atlas_slots[i].set.write_combined_image_sampler(
        0, impl.atlas_slots[i].texture.view(), impl.atlas_sampler->handle(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    // Fresh images, so every slot is back in VK_IMAGE_LAYOUT_UNDEFINED and its
    // first upload must transition from there rather than from
    // SHADER_READ_ONLY_OPTIMAL. This is also what the *bind* consults: a slot
    // that has not been written yet is not bindable, whatever `atlas_ready`
    // says. See where frame_info.atlas is chosen.
    impl.atlas_slot_written[i] = false;
  }

  impl.atlas_width = width;
  impl.atlas_height = height;
  impl.atlas_ready = true;
  return vg::Status();
}

}  // namespace volumetric_kit::ios_app

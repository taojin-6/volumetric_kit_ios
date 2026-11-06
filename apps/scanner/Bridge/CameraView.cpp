// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "CameraView.hpp"

#include <utility>

#include <glm/vec2.hpp>

#include "KeyframeBinding.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/gfx/camera/image_view_2d.hpp"

namespace volumetric_kit::ios_app {
namespace {

namespace vp = volumetric_kit::gfx::pipelines;

core::Status record_update(CameraView& view, core::Allocator& allocator,
                           VkCommandBuffer cmd,
                           const volumetric_kit::sensor::YuvImage& picture) {
  if (!keyframe_sampled(picture)) {
    return core::Status::unsupported(
        "the camera's picture is not two NV12 plane images, chroma sited left "
        "or centre");
  }
  if (!view.texture.valid()) {
    vp::ImageTextureDesc desc;
    desc.extent = {picture.width, picture.height};
    desc.mapping = vp::ImageMapping::Nv12;
    core::Result<vp::ImageTexture> made =
        vp::ImageTexture::create(view.pipeline, allocator, desc);
    if (!made) return made.status();
    view.texture = std::move(made).value();
  }
  vp::ImageUpdate update;
  update.planes[0].image = picture.image[0].get();
  update.planes[1].image = picture.image[1].get();
  set_coding(update, picture);
  return view.pipeline.record_update(cmd, view.texture, update);
}

}  // namespace

core::Status update_camera_view(CameraView& view, core::Allocator& allocator,
                                VkCommandBuffer cmd, std::uint64_t frame,
                                std::uint64_t completed) {
  view.picture.retire(completed);
  const volumetric_kit::sensor::YuvImage* picture = view.picture.take(frame);
  if (picture == nullptr) return {};
  core::Status updated = record_update(view, allocator, cmd, *picture);
  if (!updated.ok()) view.picture.clear();  // nothing recorded reads it
  return updated;
}

void draw_camera_view(const CameraView& view, VkCommandBuffer cmd,
                      VkExtent2D extent) {
  if (!view.texture.valid()) return;
  const VkExtent2D image = view.texture.extent();
  const volumetric_kit::gfx::camera::ImageView2D fitted(
      glm::vec2(static_cast<float>(image.width),
                static_cast<float>(image.height)),
      glm::vec2(0.0f, 0.0f),
      glm::vec2(static_cast<float>(extent.width),
                static_cast<float>(extent.height)));
  const vp::ImageDraw draw =
      vp::image_draw(view.texture, fitted, vp::ImageFilter::Linear);
  view.pipeline.submit(cmd, {extent, &draw, 1});
}

}  // namespace volumetric_kit::ios_app

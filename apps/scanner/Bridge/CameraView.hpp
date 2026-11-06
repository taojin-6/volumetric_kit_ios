// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file CameraView.hpp
/// @brief What the client draws: the camera's newest picture, fitted to the
///        drawable with its aspect kept, through gfx's image pipeline.
///
/// Each picture is copied on the GPU into the texture's own planes and
/// converted there, in the frame that first draws it; the frames after draw
/// what the texture keeps. So only the frame that copies a picture holds it
/// (Core/ViewPicture.hpp), and the picture becomes nothing on the host.
///
/// gfx draws a picture unturned, so the view stands upright by the layer's own
/// turn, which `ScannerViewController` sets from `picture_quarter_turns`.

#include <cstdint>
#include <string>

#include "ViewPicture.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/image_pipeline.hpp"
#include "volumetric_kit/sensor/yuv_image.hpp"

namespace volumetric_kit::core {
class Allocator;
}  // namespace volumetric_kit::core

namespace volumetric_kit::ios_app {

/// @brief The camera view's state.
///
/// @warning Holds gfx resources, so it must be declared *after* the
///          `WindowedApp` that made them. See the teardown note on
///          @ref RendererImpl.
struct CameraView {
  /// Made at bring-up, in either mode: no image of its own.
  volumetric_kit::gfx::pipelines::ImagePipeline pipeline;
  /// The view's own image, made at the first picture and freed with the mode:
  /// a 1920 x 1440 picture's planes and display image hold about 19 MB.
  volumetric_kit::gfx::pipelines::ImageTexture texture;
  /// Offered by the camera's poller, taken by @ref update_camera_view.
  ViewPicture<volumetric_kit::sensor::YuvImage> picture;
};

/// @brief Let go of the picture the frames up to @p completed copied, and
///        record the copy and conversion of the next, if one waits.
/// @param view       The view.
/// @param allocator  Makes the texture at the first picture.
/// @param cmd        The frame's command buffer, outside its rendering scope.
/// @param frame      The frame's number (`windowing::Frame::number`).
/// @param completed  The newest frame number whose GPU work has completed.
/// @return OK; or why the picture cannot be drawn: it is let go, nothing is
///         recorded, and the view keeps its last picture.
core::Status update_camera_view(CameraView& view, core::Allocator& allocator,
                                VkCommandBuffer cmd, std::uint64_t frame,
                                std::uint64_t completed);

/// @brief Draw the view's last picture into @p extent, inside the frame's
///        rendering scope; nothing before its first.
void draw_camera_view(const CameraView& view, VkCommandBuffer cmd,
                      VkExtent2D extent);

}  // namespace volumetric_kit::ios_app

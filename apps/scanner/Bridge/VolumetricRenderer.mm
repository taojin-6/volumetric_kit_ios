// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Objective-C++ seam. This is the one translation unit where a
// CAMetalLayer* and a vg::app::WindowedApp are both first-class, which is the
// entire reason the bridge is .mm rather than Swift.

#import "VolumetricRenderer.h"

#include <cstdint>
#include <memory>
#include <string>

#include "triangle_frag.spv.hpp"
#include "triangle_vert.spv.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/graphics_pipeline.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/shader.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"

namespace vg = volumetric_kit::gfx;

namespace {

NSString* const kErrorDomain = @"io.taojin.volumetrickit.renderer";

// Surface the library's Status as an NSError so Swift sees a native failure
// instead of a status code it would have to interpret.
void set_error(NSError** error, const vg::Status& status, const char* stage) {
  if (error == nullptr) {
    return;
  }
  NSString* message =
      [NSString stringWithFormat:@"%s: %s", stage, status.message().c_str()];
  *error = [NSError errorWithDomain:kErrorDomain
                               code:static_cast<NSInteger>(status.code())
                           userInfo:@{NSLocalizedDescriptionKey : message}];
}

std::string api_version_string(std::uint32_t v) {
  return std::to_string(VK_API_VERSION_MAJOR(v)) + "." +
         std::to_string(VK_API_VERSION_MINOR(v)) + "." +
         std::to_string(VK_API_VERSION_PATCH(v));
}

}  // namespace

// Everything C++ lives here so the header stays Objective-C only and Swift
// never sees a move-only type.
struct RendererImpl {
  vg::app::WindowedApp app;
  vg::ShaderModule vertex_shader;
  vg::ShaderModule fragment_shader;
  vg::GraphicsPipeline pipeline;
  std::uint64_t frames_presented = 0;
};

@implementation VolumetricRenderer {
  std::unique_ptr<RendererImpl> _impl;
}

- (nullable instancetype)initWithLayer:(CAMetalLayer*)layer
                                 error:(NSError**)error {
  self = [super init];
  if (self == nil) {
    return nil;
  }
  _impl = std::make_unique<RendererImpl>();

  vg::app::WindowedAppConfig config;
  config.app_name = "volumetric_kit_ios scanner";
  // No validation layer: MoltenVK is linked directly on iOS, with no loader to
  // interpose one. Diagnostics come from MoltenVK's own logging.
  config.enable_validation = false;
  // What GLFW would hand us on desktop. The windowing tier is window-system
  // agnostic by design, so the platform surface extension is the consumer's to
  // name -- on iOS that is VK_EXT_metal_surface.
  config.instance_extensions = {VK_KHR_SURFACE_EXTENSION_NAME,
                                VK_EXT_METAL_SURFACE_EXTENSION_NAME};
  config.swapchain.extent = {
      static_cast<std::uint32_t>(layer.drawableSize.width),
      static_cast<std::uint32_t>(layer.drawableSize.height)};
  // FIFO rather than the MAILBOX default: on a phone, tearing-free vsync at the
  // display's cadence is what we want, and MAILBOX keeps the GPU busy producing
  // frames that are then discarded -- straight thermal cost for no benefit.
  config.swapchain.preferred_present_mode = VK_PRESENT_MODE_FIFO_KHR;

  vg::Result<vg::app::WindowedApp> app = vg::app::WindowedApp::create(
      config, [layer](VkInstance instance) -> vg::Result<VkSurfaceKHR> {
        // Resolve through vkGetInstanceProcAddr rather than linking the symbol:
        // it works whether MoltenVK is linked directly (as here) or reached
        // through a loader later.
        auto create_metal_surface =
            reinterpret_cast<PFN_vkCreateMetalSurfaceEXT>(
                vkGetInstanceProcAddr(instance, "vkCreateMetalSurfaceEXT"));
        if (create_metal_surface == nullptr) {
          return vg::Status::unsupported(
              "vkCreateMetalSurfaceEXT unavailable (VK_EXT_metal_surface not "
              "enabled?)");
        }
        VkMetalSurfaceCreateInfoEXT info{};
        info.sType = VK_STRUCTURE_TYPE_METAL_SURFACE_CREATE_INFO_EXT;
        info.pLayer = layer;
        VkSurfaceKHR surface = VK_NULL_HANDLE;
        const VkResult r =
            create_metal_surface(instance, &info, nullptr, &surface);
        if (r != VK_SUCCESS) {
          return vg::vk_error(r, "vkCreateMetalSurfaceEXT");
        }
        return surface;
      });
  if (!app) {
    set_error(error, app.status(), "WindowedApp::create");
    return nil;
  }
  _impl->app = std::move(app).value();

  VkDevice device = _impl->app.device().handle();
  vg::Result<vg::ShaderModule> vert = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_vert_spv),
      vi_triangle_vert_spv_size);
  if (!vert) {
    set_error(error, vert.status(), "vertex ShaderModule::create");
    return nil;
  }
  _impl->vertex_shader = std::move(vert).value();

  vg::Result<vg::ShaderModule> frag = vg::ShaderModule::create(
      device, reinterpret_cast<const std::uint32_t*>(vi_triangle_frag_spv),
      vi_triangle_frag_spv_size);
  if (!frag) {
    set_error(error, frag.status(), "fragment ShaderModule::create");
    return nil;
  }
  _impl->fragment_shader = std::move(frag).value();

  vg::GraphicsPipelineDesc desc;
  desc.vertex_shader = &_impl->vertex_shader;
  desc.fragment_shader = &_impl->fragment_shader;
  desc.layout = _impl->app.swapchain().layout();
  // Procedural vertices: positions come from gl_VertexIndex, so no vertex
  // buffer and no input bindings.
  desc.vertex_bindings = nullptr;
  desc.vertex_binding_count = 0;

  vg::Result<vg::GraphicsPipeline> pipeline =
      vg::GraphicsPipeline::create(device, desc);
  if (!pipeline) {
    set_error(error, pipeline.status(), "GraphicsPipeline::create");
    return nil;
  }
  _impl->pipeline = std::move(pipeline).value();

  return self;
}

- (BOOL)renderFrameWithDrawableSize:(CGSize)size error:(NSError**)error {
  const VkExtent2D extent{static_cast<std::uint32_t>(size.width),
                          static_cast<std::uint32_t>(size.height)};

  vg::Result<std::optional<vg::windowing::Frame>> frame =
      _impl->app.begin_frame(extent);
  if (!frame) {
    set_error(error, frame.status(), "begin_frame");
    return NO;
  }
  if (!frame.value()) {
    // No drawable this tick (the view is off-screen or mid-rebuild). The
    // protocol working as designed, not a failure.
    return YES;
  }
  const vg::windowing::Frame& f = *frame.value();

  vg::RenderTargetBeginInfo begin{};
  begin.load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  begin.clear_color = {{0.05f, 0.06f, 0.09f, 1.0f}};
  f.target->begin(f.cmd, begin);

  vkCmdBindPipeline(f.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                    _impl->pipeline.handle());

  // Viewport and scissor are dynamic state, so they follow the swapchain
  // through a rotation without rebuilding the pipeline.
  VkViewport viewport{};
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = static_cast<float>(extent.width);
  viewport.height = static_cast<float>(extent.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;
  vkCmdSetViewport(f.cmd, 0, 1, &viewport);

  VkRect2D scissor{};
  scissor.offset = {0, 0};
  scissor.extent = extent;
  vkCmdSetScissor(f.cmd, 0, 1, &scissor);

  vkCmdDraw(f.cmd, 3, 1, 0, 0);

  f.target->end(f.cmd);

  const vg::Status end = _impl->app.end_frame(f);
  if (!end.ok()) {
    // A stale swapchain is the normal signal that the drawable changed size;
    // the next begin_frame rebuilds. Only a genuine error propagates.
    if (!vg::windowing::swapchain_stale(end)) {
      set_error(error, end, "end_frame");
      return NO;
    }
    return YES;
  }
  ++_impl->frames_presented;
  return YES;
}

- (void)waitIdle {
  if (_impl && _impl->app.valid()) {
    (void)_impl->app.wait_idle();
  }
}

- (NSString*)deviceName {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(_impl->app.device().physical_device(), &props);
  return [NSString stringWithUTF8String:props.deviceName];
}

- (NSString*)apiVersion {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(_impl->app.device().physical_device(), &props);
  return [NSString
      stringWithUTF8String:api_version_string(props.apiVersion).c_str()];
}

- (uint64_t)framesPresented {
  return _impl->frames_presented;
}

@end

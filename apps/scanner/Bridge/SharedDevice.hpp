// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file SharedDevice.hpp
/// @brief One Vulkan device satisfying both libraries, for each to adopt.
///
/// The embedder half of the create/adopt seam recon and gfx both expose (the
/// 2026-07-04 interop decision). Neither library is consulted about the other:
/// each publishes a `DeviceRequirements`, this bootstrap satisfies the union,
/// and the same handles go to `recon::Device::adopt` and
/// `gfx::app::WindowedApp::adopt`.
///
/// Why it matters here rather than being an optimisation: a `VkBuffer` is valid
/// only on the `VkDevice` that created it, so the zero-copy mesh handoff needs
/// *one* device — two devices on the same GPU would still cost a round trip
/// through host memory.
///
/// This is deliberately the app's own bootstrap, not a shared library. recon's
/// `examples/viewer/shared_device.hpp` does the same job for GLFW on desktop,
/// and the two differ exactly where they must (surface creation, and what a
/// phone's driver offers). Promoting the common part waits on a decision about
/// where it would live; see the note in README.md.

#include <mutex>
#include <string>
#include <vector>

#include "volumetric_kit/gfx/core/device.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/recon/core/device.hpp"

namespace volumetric_kit::ios_app {

namespace vr = volumetric_kit::recon;
namespace vg = volumetric_kit::gfx;

/// @brief How the two libraries' queues were carved out of the shared device.
enum class QueuePlan {
  /// Two queues from one family: independent submission, and no queue-family
  /// ownership transfer on shared buffers. What we ask for first.
  kTwoQueues,
  /// One queue shared under a mutex. Submits from the fuse thread and the
  /// render thread serialize; Vulkan requires queue submits be externally
  /// synchronized, and both `adopt` paths route through the mutex to do it.
  kSharedQueue,
};

/// @brief The shared device and everything each library needs to adopt it.
///
/// Owns the instance, device and surface, and destroys them in order — after
/// both adopters have released their wrappers, which is why this must outlive
/// them.
class SharedDevice {
 public:
  SharedDevice() = default;
  ~SharedDevice();

  SharedDevice(const SharedDevice&) = delete;
  SharedDevice& operator=(const SharedDevice&) = delete;

  /// @brief Build the instance, surface and device from both libraries' merged
  ///        requirements.
  ///
  /// @param metal_layer  A `CAMetalLayer*`, which becomes the `VkSurfaceKHR`
  ///                     via `VK_EXT_metal_surface`.
  /// @param app_name     Reported to the driver in `VkApplicationInfo`.
  /// @return OK, or why the driver could not satisfy the union. Treat a failure
  ///         as fatal rather than falling back to two devices: that would
  ///         silently give up the shared-buffer seam this exists to establish.
  vr::Status build(const void* metal_layer, const std::string& app_name);

  /// @return The payload `recon::Device::adopt` needs.
  vr::AdoptedDevice recon_payload();

  /// @return The payload `gfx::app::WindowedApp::adopt` needs.
  vg::AdoptedDevice gfx_payload();

  /// @brief Give up ownership of the surface to the caller.
  ///
  /// gfx's surface-factory contract is that the app adopts and later destroys
  /// what the factory returns — but this bootstrap had to create the surface
  /// *first*, because picking a physical device requires testing present
  /// support against a real one. So it is created here and handed over, and
  /// this stops tracking it. Destroying it twice would be a use-after-free at
  /// teardown.
  VkSurfaceKHR release_surface() noexcept {
    VkSurfaceKHR released = surface_;
    surface_ = VK_NULL_HANDLE;
    return released;
  }

  VkInstance instance() const noexcept { return instance_; }
  VkPhysicalDevice physical_device() const noexcept { return physical_; }
  VkDevice device() const noexcept { return device_; }
  VkSurfaceKHR surface() const noexcept { return surface_; }
  QueuePlan queue_plan() const noexcept { return queue_plan_; }
  bool valid() const noexcept { return device_ != VK_NULL_HANDLE; }

  /// @return A one-line description of what was built, for the read-out.
  std::string summary() const;

 private:
  VkInstance instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice physical_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  VkSurfaceKHR surface_ = VK_NULL_HANDLE;

  std::uint32_t graphics_family_ = 0;
  std::uint32_t compute_family_ = 0;
  VkQueue graphics_queue_ = VK_NULL_HANDLE;  ///< gfx's; also presents.
  VkQueue compute_queue_ = VK_NULL_HANDLE;   ///< recon's.
  QueuePlan queue_plan_ = QueuePlan::kSharedQueue;

  /// Guards a queue both libraries submit to. Non-null only under
  /// @ref QueuePlan::kSharedQueue; both `adopt` paths take it when set.
  std::mutex submit_mutex_;
  bool needs_submit_mutex_ = false;

  /// Exactly what was passed to `vkCreateDevice`, never restated by hand — each
  /// `adopt` verifies its needs against this declaration, so a hand-written
  /// value would turn that check into a no-op.
  std::vector<const char*> enabled_extensions_;
  VkPhysicalDeviceFeatures enabled_features_{};
  bool enabled_timeline_semaphore_ = false;
  bool enabled_scalar_block_layout_ = false;

  std::uint32_t api_version_ = 0;
  std::string device_name_;
};

}  // namespace volumetric_kit::ios_app

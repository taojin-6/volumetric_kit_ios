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
/// and the two differ exactly where they must (surface creation, and the app's
/// `Status`-returning error style in place of the example's stderr). Everything
/// load-bearing — the plan order, the pre-create support checks, the feature
/// merge — is kept deliberately in step with it; see "The duplicated
/// bootstrap" in README.md for why it is copied rather than promoted, and what
/// would trigger promoting it.

#include <cstdint>
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
///
/// Ordered best-first: @ref SharedDevice::build takes the first plan the
/// hardware allows, and @ref SharedDevice::summary reports which. The order is
/// not cosmetic — each step down costs something real.
enum class QueuePlan {
  /// One family, two queues: independent submission *and* no queue-family
  /// ownership transfer on a buffer recon writes and gfx reads. What interop
  /// seam B assumes.
  kTwoQueuesOneFamily,
  /// Two families, one queue each: still independent submission, but a shared
  /// buffer will need `VK_SHARING_MODE_CONCURRENT` or an explicit
  /// release/acquire pair when seam B lands. This is the plan that actually
  /// runs on MoltenVK, which reports several graphics + compute + present
  /// families of exactly one queue each — so tier 1 is unreachable on a phone
  /// and *this* is what the first tier's absence would have cost.
  kTwoFamilies,
  /// One family, one queue shared under a mutex. Last resort: submits from the
  /// fuse thread and the render thread serialize, which is precisely what the
  /// background fuse thread exists to avoid.
  ///
  /// @warning Not merely slower. gfx takes the same mutex across a full queue
  ///          drain when it recreates the swapchain (every rotation), so once
  ///          seam B lands a recon submit waiting on a gfx-signalled timeline
  ///          value can deadlock against a concurrent rotation. Nothing the
  ///          embedder can do about that — Vulkan requires the external
  ///          synchronization — which is why this plan is last and why the two
  ///          above it are worth the search.
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
  // Moves deleted too, not just copies: @ref submit_mutex_'s *address* is
  // handed to both libraries and borrowed for their whole lifetime, so it has
  // to stay put.
  SharedDevice(SharedDevice&&) = delete;
  SharedDevice& operator=(SharedDevice&&) = delete;

  /// @brief Build the instance, surface and device from both libraries' merged
  ///        requirements.
  ///
  /// @param metal_layer  A `CAMetalLayer*`, which becomes the `VkSurfaceKHR`
  ///                     via `VK_EXT_metal_surface`.
  /// @param app_name     Reported to the driver in `VkApplicationInfo`.
  /// @return OK, or why the driver could not satisfy the union — naming the
  ///         missing extension or feature rather than letting `vkCreateDevice`
  ///         fail opaquely. Treat a failure as fatal rather than falling back
  ///         to two devices: that would silently give up the shared-buffer seam
  ///         this exists to establish.
  vr::Status build(const void* metal_layer, const std::string& app_name);

  /// @return The payload `recon::Device::adopt` needs.
  vr::AdoptedDevice recon_payload();

  /// @return The payload `gfx::app::WindowedApp::adopt` needs.
  vg::AdoptedDevice gfx_payload();

  /// @brief Block until every queue this bootstrap handed out is idle.
  ///
  /// Neither adopter can do this for us: gfx's `Device::wait_idle` covers only
  /// the queues *it* was assigned, and recon's `Device` exposes no wait at all
  /// — so under @ref QueuePlan::kTwoFamilies recon's queue is one nobody else
  /// would ever drain. Safe to call with both adopters alive: it takes the same
  /// mutex their submits do when the queue is shared.
  void wait_idle() noexcept;

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
  VkDevice device() const noexcept { return device_; }

  /// @return A one-line description of what was built, for the read-out. Names
  ///         the queue plan that was actually taken, so a phone that lands on
  ///         @ref QueuePlan::kSharedQueue says so rather than reading the same
  ///         as one that got two queues.
  std::string summary() const;

 private:
  /// @return The mutex both libraries must hold to submit, or `nullptr` when
  ///         each got a queue of its own. Derived from @ref queue_plan_ rather
  ///         than tracked alongside it: a second field encoding the same fact
  ///         is a second field to keep in agreement by hand.
  std::mutex* submit_mutex() noexcept {
    return queue_plan_ == QueuePlan::kSharedQueue ? &submit_mutex_ : nullptr;
  }

  VkInstance instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice physical_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  VkSurfaceKHR surface_ = VK_NULL_HANDLE;

  std::uint32_t graphics_family_ = 0;
  std::uint32_t compute_family_ = 0;
  VkQueue graphics_queue_ = VK_NULL_HANDLE;  ///< gfx's; also presents.
  VkQueue compute_queue_ = VK_NULL_HANDLE;   ///< recon's.
  /// The most conservative plan, so a payload taken before a successful
  /// @ref build hands out the mutex rather than omitting it.
  QueuePlan queue_plan_ = QueuePlan::kSharedQueue;

  /// Guards a queue both libraries submit to. Handed out (via
  /// @ref submit_mutex) only under @ref QueuePlan::kSharedQueue.
  std::mutex submit_mutex_;

  /// Owns the bytes of the enabled extension names. gfx documents its
  /// requirement vectors as *borrowing* their `const char*`s from the
  /// `DeviceConfig` they were derived from, which here is a `build` local — so
  /// the names are copied rather than aliased, and @ref enabled_extensions_
  /// points into this. Both payloads hand that array out after `build` has
  /// returned, so aliasing the locals would be a use-after-free the moment a
  /// requirement is backed by anything but a literal.
  std::vector<std::string> extension_storage_;

  /// Exactly what was passed to `vkCreateDevice`, never restated by hand — each
  /// `adopt` verifies its needs against this declaration, so a hand-written
  /// value would turn that check into a no-op. Points into
  /// @ref extension_storage_, and is filled only once that has stopped growing.
  std::vector<const char*> enabled_extensions_;
  VkPhysicalDeviceFeatures enabled_features_{};
  bool enabled_timeline_semaphore_ = false;
  bool enabled_scalar_block_layout_ = false;

  std::uint32_t api_version_ = 0;
  std::string device_name_;
};

}  // namespace volumetric_kit::ios_app

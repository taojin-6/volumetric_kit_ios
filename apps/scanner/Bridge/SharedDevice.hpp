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
  /// @warning It also forbids a cross-library GPU-side wait, in *either*
  ///          direction — a different sync design, not just a slower one.
  ///
  /// `Swapchain::recreate` drains on every rotation through gfx's
  /// `Device::wait_idle`, which takes the mutex and *then* calls
  /// `vkQueueWaitIdle`. On a shared queue that drain waits on **both**
  /// libraries' in-flight work while holding the lock either one needs
  /// to submit anything further.
  ///
  /// So once seam B lands, any command buffer already on the queue whose
  /// completion depends on a submit the sibling has not made yet will
  /// deadlock against a concurrent rotation: gfx's draw waiting on recon's
  /// mesh-ready value and recon's extract waiting on a gfx-signalled value
  /// are both that shape.
  ///
  /// Nothing the embedder can undo — Vulkan requires the external
  /// synchronization, and the drain is gfx's to make. Under this plan both
  /// sides must check readiness on the host and skip, never submit a wait
  /// on a value the sibling has yet to signal. Avoiding that is what this
  /// plan being last is for.
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
  /// would ever drain. Safe to call with both adopters alive **under every
  /// queue plan**: it takes the same mutex each queue's submits do, so it
  /// excludes a concurrent `vkQueueSubmit` rather than racing one.
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
  /// @name The mutex guarding each handed-out queue
  ///
  /// One per queue, and always non-null. The earlier shape returned `nullptr`
  /// whenever the two libraries got queues of their own, on the reasoning that
  /// an unshared queue needs no lock — but Vulkan requires a `VkQueue` be
  /// externally synchronized for **every** host-side operation on it, and
  /// @ref wait_idle is a third thread touching both. With no mutex to take it
  /// called `vkQueueWaitIdle` bare, racing recon's fuse thread mid-submit on
  /// exactly the @ref QueuePlan::kTwoFamilies plan a phone actually gets.
  ///
  /// Per queue rather than one global: under the multi-queue plans gfx and
  /// recon take different mutexes, so their submits still do not serialize
  /// against each other — only against a drain, which is the point. Under
  /// @ref QueuePlan::kSharedQueue both resolve to the same object, which is the
  /// single mutex the shared queue always needed.
  /// @{
  std::mutex* graphics_submit_mutex() noexcept {
    return &graphics_submit_mutex_;
  }
  std::mutex* compute_submit_mutex() noexcept {
    return queue_plan_ == QueuePlan::kSharedQueue ? &graphics_submit_mutex_
                                                  : &compute_submit_mutex_;
  }
  /// @}

 public:
  /// @name The queue families the two libraries were handed
  ///
  /// A buffer one library writes and the other reads must be created naming
  /// **both**, or it is `VK_SHARING_MODE_EXCLUSIVE` and reading it from the
  /// family that does not own it is undefined -- silently, with the contents
  /// simply not guaranteed to be there. Under @ref QueuePlan::kTwoFamilies
  /// these differ, which is the plan a phone actually gets.
  ///
  /// Pass both to the producer unconditionally: recon's `BufferDesc` reduces
  /// them to distinct entries and picks EXCLUSIVE when they turn out to be one
  /// family, so a caller never has to branch on the plan.
  /// @{
  std::uint32_t graphics_family() const noexcept { return graphics_family_; }
  std::uint32_t compute_family() const noexcept { return compute_family_; }
  /// @}

 private:
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

  /// Guards submits to gfx's queue, and to recon's as well whenever the two are
  /// one queue. Always handed out — see @ref graphics_submit_mutex.
  std::mutex graphics_submit_mutex_;
  /// Guards submits to recon's queue under the multi-queue plans. Unused (and
  /// never handed out) under @ref QueuePlan::kSharedQueue, where both adopters
  /// take @ref graphics_submit_mutex_ instead.
  std::mutex compute_submit_mutex_;

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

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "SharedDevice.hpp"

#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <cstring>

namespace volumetric_kit::ios_app {
namespace {

/// Add @p name unless it is already present. `vkCreateDevice` rejects a
/// duplicated extension name, and the two libraries' requirement lists overlap
/// by construction (both ask for timeline semaphores).
void add_unique(std::vector<std::string>& names, const char* name) {
  const bool present =
      std::any_of(names.begin(), names.end(),
                  [&](const std::string& n) { return n == name; });
  if (!present) {
    names.emplace_back(name);
  }
}

/// Bitwise OR of two core feature sets. `VkPhysicalDeviceFeatures` is a flat
/// block of `VkBool32`, so the union is a field-wise OR -- done over the raw
/// words rather than by naming 55 fields, which would silently drop whichever
/// one a future requirement added.
VkPhysicalDeviceFeatures merge_features(const VkPhysicalDeviceFeatures& a,
                                        const VkPhysicalDeviceFeatures& b) {
  VkPhysicalDeviceFeatures out{};
  static_assert(sizeof(VkPhysicalDeviceFeatures) % sizeof(VkBool32) == 0,
                "VkPhysicalDeviceFeatures must be a whole number of VkBool32");
  const auto* wa = reinterpret_cast<const VkBool32*>(&a);
  const auto* wb = reinterpret_cast<const VkBool32*>(&b);
  auto* wo = reinterpret_cast<VkBool32*>(&out);
  for (std::size_t i = 0; i < sizeof(out) / sizeof(VkBool32); ++i) {
    wo[i] = (wa[i] != VK_FALSE || wb[i] != VK_FALSE) ? VK_TRUE : VK_FALSE;
  }
  return out;
}

/// Index of the first bit set in @p wanted but clear in @p supported, or -1
/// when @p wanted is a subset. Same flat-block reasoning as @ref
/// merge_features: naming fields would go stale the moment a requirement adds
/// one.
int first_unsupported_feature(const VkPhysicalDeviceFeatures& wanted,
                              const VkPhysicalDeviceFeatures& supported) {
  const auto* want = reinterpret_cast<const VkBool32*>(&wanted);
  const auto* have = reinterpret_cast<const VkBool32*>(&supported);
  for (std::size_t i = 0; i < sizeof(wanted) / sizeof(VkBool32); ++i) {
    if (want[i] != VK_FALSE && have[i] == VK_FALSE) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string version_string(std::uint32_t version) {
  return std::to_string(VK_API_VERSION_MAJOR(version)) + "." +
         std::to_string(VK_API_VERSION_MINOR(version));
}

}  // namespace

SharedDevice::~SharedDevice() {
  // Drain first: `vkDestroyDevice` requires every queue idle, and no adopter
  // can promise that on our behalf -- gfx waits only on the queues it was
  // assigned, and recon's Device exposes no wait at all. Both adopters must
  // already be gone: their wrappers borrow these handles and destroy nothing,
  // which is the whole point of `adopt`, so tearing down while one is alive
  // would free objects still in use.
  wait_idle();

  // Reverse creation order.
  if (device_ != VK_NULL_HANDLE) {
    vkDestroyDevice(device_, nullptr);
  }
  if (surface_ != VK_NULL_HANDLE) {
    vkDestroySurfaceKHR(instance_, surface_, nullptr);
  }
  if (instance_ != VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
  }
}

void SharedDevice::wait_idle() noexcept {
  if (device_ == VK_NULL_HANDLE) {
    return;
  }
  // Per queue rather than `vkDeviceWaitIdle`: on a shared queue the wait is a
  // queue operation like any other and must hold the mutex every submit holds,
  // which `vkDeviceWaitIdle` gives no way to scope.
  auto drain = [this](VkQueue queue) {
    if (queue == VK_NULL_HANDLE) {
      return;
    }
    if (std::mutex* guard = submit_mutex(); guard != nullptr) {
      const std::lock_guard<std::mutex> lock(*guard);
      vkQueueWaitIdle(queue);
      return;
    }
    vkQueueWaitIdle(queue);
  };
  drain(graphics_queue_);
  if (compute_queue_ != graphics_queue_) {
    drain(compute_queue_);
  }
}

vr::Status SharedDevice::build(const void* metal_layer,
                               const std::string& app_name) {
  // --- 1. Merge what each library publishes --------------------------------
  // Neither is consulted about the other: each states its needs, the embedder
  // satisfies the union.
  vg::DeviceConfig gfx_config;
  gfx_config.needs_present = true;
  const vr::DeviceRequirements recon_req = vr::Device::requirements({});
  const vg::DeviceRequirements gfx_req = vg::Device::requirements(gfx_config);

  // The higher floor wins, and it must NOT inherit recon's 1.2: MoltenVK caps
  // the apiVersion a physical device advertises to whatever its instance asked
  // for, so requesting 1.2 here would make gfx's 1.3 floor look unsupported on
  // hardware that fully supports it. (recon's CLAUDE.md records this trap.)
  api_version_ = std::max(recon_req.api_version, gfx_req.api_version);

  std::uint32_t instance_ceiling = VK_API_VERSION_1_0;
  if (vkEnumerateInstanceVersion(&instance_ceiling) != VK_SUCCESS) {
    instance_ceiling = VK_API_VERSION_1_0;
  }
  if (instance_ceiling < api_version_) {
    return vr::Status::unsupported("the Vulkan implementation supports " +
                                   version_string(instance_ceiling) +
                                   " but the merged floor is " +
                                   version_string(api_version_));
  }

  // --- 2. Instance ----------------------------------------------------------
  VkApplicationInfo app{};
  app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.pApplicationName = app_name.c_str();
  app.apiVersion = api_version_;

  // VK_EXT_metal_surface is the platform surface on Apple; no portability
  // enumeration, because MoltenVK is linked directly here with no loader to
  // interpose it (and it does not exist in that configuration).
  const char* instance_exts[] = {VK_KHR_SURFACE_EXTENSION_NAME,
                                 VK_EXT_METAL_SURFACE_EXTENSION_NAME};
  VkInstanceCreateInfo ici{};
  ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  ici.pApplicationInfo = &app;
  ici.enabledExtensionCount = 2;
  ici.ppEnabledExtensionNames = instance_exts;
  if (const VkResult r = vkCreateInstance(&ici, nullptr, &instance_);
      r != VK_SUCCESS) {
    return vr::Status::backend_error(
        r, "vkCreateInstance failed for the shared device");
  }

  // --- 3. Surface -----------------------------------------------------------
  auto create_metal_surface = reinterpret_cast<PFN_vkCreateMetalSurfaceEXT>(
      vkGetInstanceProcAddr(instance_, "vkCreateMetalSurfaceEXT"));
  if (create_metal_surface == nullptr) {
    return vr::Status::unsupported("vkCreateMetalSurfaceEXT unavailable");
  }
  VkMetalSurfaceCreateInfoEXT sci{};
  sci.sType = VK_STRUCTURE_TYPE_METAL_SURFACE_CREATE_INFO_EXT;
  // ARC needs the ownership transfer spelled out to reach an Obj-C type from
  // a void*; __bridge is the right one -- the layer is owned by the view.
  sci.pLayer = (__bridge const CAMetalLayer*)metal_layer;
  if (const VkResult r =
          create_metal_surface(instance_, &sci, nullptr, &surface_);
      r != VK_SUCCESS) {
    return vr::Status::backend_error(r, "vkCreateMetalSurfaceEXT failed");
  }

  // --- 4. Physical device + queue plan --------------------------------------
  std::uint32_t gpu_count = 0;
  vkEnumeratePhysicalDevices(instance_, &gpu_count, nullptr);
  std::vector<VkPhysicalDevice> gpus(gpu_count);
  if (gpu_count > 0) {
    if (const VkResult r =
            vkEnumeratePhysicalDevices(instance_, &gpu_count, gpus.data());
        r != VK_SUCCESS && r != VK_INCOMPLETE) {
      return vr::Status::backend_error(r, "vkEnumeratePhysicalDevices failed");
    }
  }

  const VkQueueFlags gfx_flags = gfx_req.queue_flags | VK_QUEUE_GRAPHICS_BIT;
  // recon's compute bit and nothing more: a conformant compute family may
  // legally not advertise VK_QUEUE_TRANSFER_BIT, and compute implies transfer.
  // recon's own header warns an embedder against adding it.
  const VkQueueFlags recon_flags = recon_req.queue_flags;

  // Best plan first, and the order is load-bearing -- see @ref QueuePlan. The
  // point of searching all three is that a phone lands on kTwoFamilies:
  // MoltenVK reports several graphics + compute + present families of one queue
  // each, so stopping at the first matching family would take kSharedQueue and
  // hand back the concurrency the fuse thread exists for.
  bool found = false;
  bool any_device_at_floor = false;
  for (VkPhysicalDevice candidate : gpus) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(candidate, &props);
    if (props.apiVersion < api_version_) {
      continue;
    }
    any_device_at_floor = true;

    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count,
                                             families.data());

    auto has = [&](std::uint32_t i, VkQueueFlags flags) {
      return (families[i].queueFlags & flags) == flags;
    };
    auto presents = [&](std::uint32_t i) {
      VkBool32 present = VK_FALSE;
      const VkResult r = vkGetPhysicalDeviceSurfaceSupportKHR(
          candidate, i, surface_, &present);
      return r == VK_SUCCESS && present == VK_TRUE;
    };
    auto take = [&](QueuePlan plan, std::uint32_t graphics,
                    std::uint32_t compute) {
      physical_ = candidate;
      queue_plan_ = plan;
      graphics_family_ = graphics;
      compute_family_ = compute;
      device_name_ = props.deviceName;
      found = true;
    };

    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (has(i, gfx_flags) && has(i, recon_flags) &&
          families[i].queueCount >= 2 && presents(i)) {
        take(QueuePlan::kTwoQueuesOneFamily, i, i);
      }
    }
    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (!has(i, gfx_flags) || !presents(i)) {
        continue;
      }
      for (std::uint32_t j = 0; j < family_count && !found; ++j) {
        if (j != i && has(j, recon_flags)) {
          take(QueuePlan::kTwoFamilies, i, j);
        }
      }
    }
    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (has(i, gfx_flags) && has(i, recon_flags) && presents(i)) {
        take(QueuePlan::kSharedQueue, i, i);
      }
    }
    if (found) {
      break;
    }
  }
  if (!found) {
    // Three different failures, three different messages: reporting the API
    // floor as a queue-topology problem sends the reader looking at the wrong
    // thing, and a device rejected before its families were even queried has
    // nothing to say about them.
    if (gpus.empty()) {
      return vr::Status::unsupported("no Vulkan physical device");
    }
    if (!any_device_at_floor) {
      return vr::Status::unsupported(
          "every Vulkan device is below the merged " +
          version_string(api_version_) + " floor gfx requires");
    }
    return vr::Status::unsupported(
        "no device at the merged floor exposes a present-capable graphics "
        "family alongside a compute family");
  }

  // --- 5. Verify before creating -------------------------------------------
  // Every extension and feature is checked against the device first, so a
  // shortfall names itself instead of collapsing into "vkCreateDevice failed".
  std::uint32_t available_count = 0;
  vkEnumerateDeviceExtensionProperties(physical_, nullptr, &available_count,
                                       nullptr);
  std::vector<VkExtensionProperties> available(available_count);
  if (available_count > 0) {
    if (const VkResult r = vkEnumerateDeviceExtensionProperties(
            physical_, nullptr, &available_count, available.data());
        r != VK_SUCCESS && r != VK_INCOMPLETE) {
      return vr::Status::backend_error(
          r, "vkEnumerateDeviceExtensionProperties failed");
    }
  }
  auto device_supports = [&available](const char* name) {
    return std::any_of(available.begin(), available.end(),
                       [&](const VkExtensionProperties& p) {
                         return std::strcmp(p.extensionName, name) == 0;
                       });
  };

  for (const char* name : recon_req.device_extensions) {
    add_unique(extension_storage_, name);
  }
  for (const char* name : gfx_req.device_extensions) {
    add_unique(extension_storage_, name);
  }
  // The spec requires enabling VK_KHR_portability_subset whenever a device
  // exposes it, which MoltenVK always does. Neither library lists it -- it is
  // the device creator's obligation, and that is this bootstrap.
  if (device_supports("VK_KHR_portability_subset")) {
    add_unique(extension_storage_, "VK_KHR_portability_subset");
  }
  for (const std::string& name : extension_storage_) {
    if (!device_supports(name.c_str())) {
      return vr::Status::unsupported(
          "the device does not support a required extension: " + name);
    }
  }
  // Pointers taken only now that the storage has stopped growing: a `push_back`
  // after this would reallocate the strings out from under them.
  enabled_extensions_.reserve(extension_storage_.size());
  for (const std::string& name : extension_storage_) {
    enabled_extensions_.push_back(name.c_str());
  }

  enabled_features_ = merge_features(recon_req.features, gfx_req.features);
  enabled_timeline_semaphore_ =
      recon_req.timeline_semaphore || gfx_req.timeline_semaphore;
  enabled_scalar_block_layout_ = recon_req.scalar_block_layout;
  const bool want_dynamic_rendering = gfx_req.dynamic_rendering;

  // Queried through the same 1.2/1.3 aggregates the enable path uses below, so
  // the check and the request cannot drift apart. scalarBlockLayout in
  // particular stays optional at the 1.3 floor this enforces, so a device
  // lacking it is a real outcome and not a theoretical one.
  VkPhysicalDeviceVulkan13Features supported13{};
  supported13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  VkPhysicalDeviceVulkan12Features supported12{};
  supported12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  supported12.pNext = &supported13;
  VkPhysicalDeviceFeatures2 supported{};
  supported.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  supported.pNext = &supported12;
  vkGetPhysicalDeviceFeatures2(physical_, &supported);

  struct FeatureCheck {
    bool wanted;
    VkBool32 supported;
    const char* name;
  };
  for (const FeatureCheck& check :
       {FeatureCheck{enabled_timeline_semaphore_, supported12.timelineSemaphore,
                     "timelineSemaphore"},
        FeatureCheck{enabled_scalar_block_layout_,
                     supported12.scalarBlockLayout, "scalarBlockLayout"},
        FeatureCheck{want_dynamic_rendering, supported13.dynamicRendering,
                     "dynamicRendering"}}) {
    if (check.wanted && check.supported == VK_FALSE) {
      return vr::Status::unsupported(
          std::string("the device does not support a required feature: ") +
          check.name);
    }
  }
  if (const int missing =
          first_unsupported_feature(enabled_features_, supported.features);
      missing >= 0) {
    return vr::Status::unsupported(
        "the device does not support VkPhysicalDeviceFeatures bit " +
        std::to_string(missing));
  }

  // --- 6. Device ------------------------------------------------------------
  VkPhysicalDeviceVulkan13Features f13{};
  f13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  f13.dynamicRendering = want_dynamic_rendering ? VK_TRUE : VK_FALSE;
  // gfx publishes any *further* extended-feature structs it would enable as an
  // opaque chain and documents enabling them as the embedder's job -- its
  // Device::adopt cannot introspect one. Splice it on the tail. The const_cast
  // is safe: vkCreateDevice only reads pNext. Null today; this keeps it from
  // being silently dropped the day it is not, which would leave adopt
  // succeeding and the renderer faulting at first use.
  f13.pNext = const_cast<void*>(gfx_req.feature_chain);

  VkPhysicalDeviceVulkan12Features f12{};
  f12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  f12.timelineSemaphore = enabled_timeline_semaphore_ ? VK_TRUE : VK_FALSE;
  f12.scalarBlockLayout = enabled_scalar_block_layout_ ? VK_TRUE : VK_FALSE;
  f12.pNext = &f13;

  VkPhysicalDeviceFeatures2 features2{};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.features = enabled_features_;
  features2.pNext = &f12;

  // One VkDeviceQueueCreateInfo per distinct family, and two queues from it
  // only under the first plan.
  const std::uint32_t compute_queue_index =
      queue_plan_ == QueuePlan::kTwoQueuesOneFamily ? 1u : 0u;
  const float priorities[2] = {1.0f, 1.0f};
  std::vector<VkDeviceQueueCreateInfo> queue_infos;
  VkDeviceQueueCreateInfo qci{};
  qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  qci.pQueuePriorities = priorities;
  qci.queueFamilyIndex = graphics_family_;
  qci.queueCount = compute_queue_index == 1u ? 2u : 1u;
  queue_infos.push_back(qci);
  if (compute_family_ != graphics_family_) {
    qci.queueFamilyIndex = compute_family_;
    qci.queueCount = 1u;
    queue_infos.push_back(qci);
  }

  VkDeviceCreateInfo dci{};
  dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  dci.pNext = &features2;
  dci.queueCreateInfoCount = static_cast<std::uint32_t>(queue_infos.size());
  dci.pQueueCreateInfos = queue_infos.data();
  dci.enabledExtensionCount =
      static_cast<std::uint32_t>(enabled_extensions_.size());
  dci.ppEnabledExtensionNames = enabled_extensions_.data();
  // pEnabledFeatures must stay null when VkPhysicalDeviceFeatures2 is chained.
  if (const VkResult r = vkCreateDevice(physical_, &dci, nullptr, &device_);
      r != VK_SUCCESS) {
    return vr::Status::backend_error(
        r, "vkCreateDevice failed for the merged requirements");
  }

  vkGetDeviceQueue(device_, graphics_family_, 0, &graphics_queue_);
  vkGetDeviceQueue(device_, compute_family_, compute_queue_index,
                   &compute_queue_);
  return vr::Status();
}

vr::AdoptedDevice SharedDevice::recon_payload() {
  vr::AdoptedDevice adopted;
  adopted.instance = instance_;
  adopted.physical_device = physical_;
  adopted.device = device_;
  adopted.compute_family = compute_family_;
  adopted.compute_queue = compute_queue_;
  adopted.submit_mutex = submit_mutex();
  adopted.enabled_device_extensions = enabled_extensions_.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(enabled_extensions_.size());
  adopted.enabled_features = enabled_features_;
  // Read back from what this bootstrap enabled, never asserted: recon's adopt
  // trusts the declaration in place of a query Vulkan does not offer, so a
  // hand-written `true` would turn its verification into a no-op.
  adopted.enabled_timeline_semaphore = enabled_timeline_semaphore_;
  adopted.enabled_scalar_block_layout = enabled_scalar_block_layout_;
  return adopted;
}

vg::AdoptedDevice SharedDevice::gfx_payload() {
  vg::AdoptedDevice adopted;
  adopted.instance = instance_;
  adopted.physical_device = physical_;
  adopted.device = device_;
  adopted.graphics_family = graphics_family_;
  adopted.graphics_queue = graphics_queue_;
  // The family was chosen for its present support, so present is that queue.
  adopted.has_present = true;
  adopted.present_family = graphics_family_;
  adopted.present_queue = graphics_queue_;
  adopted.submit_mutex = submit_mutex();
  adopted.enabled_device_extensions = enabled_extensions_.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(enabled_extensions_.size());
  return adopted;
}

std::string SharedDevice::summary() const {
  const std::string gfx_family = std::to_string(graphics_family_);
  switch (queue_plan_) {
    case QueuePlan::kTwoQueuesOneFamily:
      return device_name_ + ", family " + gfx_family +
             ", 2 queues (gfx + recon)";
    case QueuePlan::kTwoFamilies:
      return device_name_ + ", family " + gfx_family + " (gfx) + family " +
             std::to_string(compute_family_) + " (recon), 1 queue each";
    case QueuePlan::kSharedQueue:
      return device_name_ + ", family " + gfx_family +
             ", 1 shared queue (mutex-guarded, serializes)";
  }
  return device_name_;
}

}  // namespace volumetric_kit::ios_app

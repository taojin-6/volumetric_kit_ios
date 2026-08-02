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
void add_unique(std::vector<const char*>& names, const char* name) {
  const bool present =
      std::any_of(names.begin(), names.end(),
                  [&](const char* n) { return std::strcmp(n, name) == 0; });
  if (!present) {
    names.push_back(name);
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

bool supports_extension(VkPhysicalDevice physical, const char* name) {
  std::uint32_t count = 0;
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> props(count);
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, props.data());
  return std::any_of(props.begin(), props.end(),
                     [&](const VkExtensionProperties& p) {
                       return std::strcmp(p.extensionName, name) == 0;
                     });
}

}  // namespace

SharedDevice::~SharedDevice() {
  // Reverse creation order. Both adopters must already be gone: their wrappers
  // borrow these handles and destroy nothing, which is the whole point of
  // `adopt`, so tearing down while one is alive would free objects still in
  // use.
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
    return vr::Status::unsupported(
        "the Vulkan implementation is below the merged 1.3 floor gfx requires");
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

  // --- 4. Physical device + queue family ------------------------------------
  std::uint32_t gpu_count = 0;
  vkEnumeratePhysicalDevices(instance_, &gpu_count, nullptr);
  std::vector<VkPhysicalDevice> gpus(gpu_count);
  vkEnumeratePhysicalDevices(instance_, &gpu_count, gpus.data());

  // One family carrying graphics + compute + present. Insisting on a single
  // family is not a simplification: it is what avoids a queue-family ownership
  // transfer on every buffer the two libraries share. iOS offers exactly one
  // family, so this always holds here -- the search exists so a failure says so
  // rather than misbehaving.
  const VkQueueFlags needed = gfx_req.queue_flags | recon_req.queue_flags;
  for (VkPhysicalDevice candidate : gpus) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(candidate, &props);
    if (props.apiVersion < api_version_) {
      continue;
    }
    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count,
                                             families.data());
    for (std::uint32_t i = 0; i < family_count; ++i) {
      if ((families[i].queueFlags & needed) != needed) {
        continue;
      }
      VkBool32 present = VK_FALSE;
      vkGetPhysicalDeviceSurfaceSupportKHR(candidate, i, surface_, &present);
      if (present != VK_TRUE) {
        continue;
      }
      physical_ = candidate;
      graphics_family_ = compute_family_ = i;
      queue_plan_ = families[i].queueCount >= 2 ? QueuePlan::kTwoQueues
                                                : QueuePlan::kSharedQueue;
      device_name_ = props.deviceName;
      break;
    }
    if (physical_ != VK_NULL_HANDLE) {
      break;
    }
  }
  if (physical_ == VK_NULL_HANDLE) {
    return vr::Status::unsupported(
        "no device has one queue family with graphics + compute + present at "
        "the merged API floor");
  }

  // --- 5. Device ------------------------------------------------------------
  for (const char* name : recon_req.device_extensions) {
    add_unique(enabled_extensions_, name);
  }
  for (const char* name : gfx_req.device_extensions) {
    add_unique(enabled_extensions_, name);
  }
  // The spec requires enabling VK_KHR_portability_subset whenever a device
  // exposes it, which MoltenVK always does. Neither library lists it -- it is
  // the device creator's obligation, and that is this bootstrap.
  if (supports_extension(physical_, "VK_KHR_portability_subset")) {
    add_unique(enabled_extensions_, "VK_KHR_portability_subset");
  }

  enabled_features_ = merge_features(recon_req.features, gfx_req.features);
  enabled_timeline_semaphore_ =
      recon_req.timeline_semaphore || gfx_req.timeline_semaphore;
  enabled_scalar_block_layout_ = recon_req.scalar_block_layout;

  VkPhysicalDeviceVulkan13Features f13{};
  f13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  f13.dynamicRendering = gfx_req.dynamic_rendering ? VK_TRUE : VK_FALSE;

  VkPhysicalDeviceVulkan12Features f12{};
  f12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  f12.timelineSemaphore = enabled_timeline_semaphore_ ? VK_TRUE : VK_FALSE;
  f12.scalarBlockLayout = enabled_scalar_block_layout_ ? VK_TRUE : VK_FALSE;
  f12.pNext = &f13;

  VkPhysicalDeviceFeatures2 features2{};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.features = enabled_features_;
  features2.pNext = &f12;

  const float priorities[2] = {1.0f, 1.0f};
  VkDeviceQueueCreateInfo qci{};
  qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  qci.queueFamilyIndex = graphics_family_;
  qci.queueCount = queue_plan_ == QueuePlan::kTwoQueues ? 2u : 1u;
  qci.pQueuePriorities = priorities;

  VkDeviceCreateInfo dci{};
  dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  dci.pNext = &features2;
  dci.queueCreateInfoCount = 1;
  dci.pQueueCreateInfos = &qci;
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
  if (queue_plan_ == QueuePlan::kTwoQueues) {
    vkGetDeviceQueue(device_, compute_family_, 1, &compute_queue_);
  } else {
    // One queue, two submitters: Vulkan requires queue submits be externally
    // synchronized, so both adopt paths take the mutex below.
    compute_queue_ = graphics_queue_;
    needs_submit_mutex_ = true;
  }
  return vr::Status();
}

vr::AdoptedDevice SharedDevice::recon_payload() {
  vr::AdoptedDevice adopted;
  adopted.instance = instance_;
  adopted.physical_device = physical_;
  adopted.device = device_;
  adopted.compute_family = compute_family_;
  adopted.compute_queue = compute_queue_;
  adopted.submit_mutex = needs_submit_mutex_ ? &submit_mutex_ : nullptr;
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
  adopted.submit_mutex = needs_submit_mutex_ ? &submit_mutex_ : nullptr;
  adopted.enabled_device_extensions = enabled_extensions_.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(enabled_extensions_.size());
  return adopted;
}

std::string SharedDevice::summary() const {
  const std::string plan = queue_plan_ == QueuePlan::kTwoQueues
                               ? "2 queues (gfx + recon)"
                               : "1 shared queue (mutex-guarded)";
  return device_name_ + ", family " + std::to_string(graphics_family_) + ", " +
         plan;
}

}  // namespace volumetric_kit::ios_app

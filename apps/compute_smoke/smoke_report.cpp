// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The on-device de-risk gate. CLAUDE.md's standing rule for this family is
// "validate MoltenVK compute on the target Apple GPU early -- prove the path
// before building on it". This is that gate, applied to iOS: it exercises the
// real driver on real hardware, in four stages that fail independently so a
// late failure still leaves the earlier evidence on screen.
//
// Everything here is plain C++ against recon's public headers; nothing is
// iOS-specific except that it happens to be running there. main.mm supplies the
// app shell that displays the report.

#include "smoke_report.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "fill_comp.spv.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;

namespace volumetric_kit::ios_app {
namespace {

/// Accumulates the report and tracks whether anything failed.
class Report {
 public:
  void line(const std::string& text) { text_ += text + "\n"; }
  void blank() { text_ += "\n"; }

  void section(const std::string& title) {
    blank();
    line("== " + title + " ==");
  }

  /// Record a stage outcome; a failure marks the whole run failed.
  void check(bool ok, const std::string& what) {
    line((ok ? "  [PASS] " : "  [FAIL] ") + what);
    if (!ok) {
      failed_ = true;
    }
  }

  /// A stage could not run at all (a prerequisite failed) -- distinct from a
  /// stage that ran and produced the wrong answer.
  void abort_stage(const std::string& why) {
    line("  [FAIL] " + why);
    failed_ = true;
  }

  void field(const std::string& key, const std::string& value) {
    line("  " + key + ": " + value);
  }

  bool failed() const { return failed_; }
  const std::string& text() const { return text_; }

 private:
  std::string text_;
  bool failed_ = false;
};

std::string yes_no(bool v) { return v ? "yes" : "no"; }

std::string api_version_string(std::uint32_t v) {
  return std::to_string(VK_API_VERSION_MAJOR(v)) + "." +
         std::to_string(VK_API_VERSION_MINOR(v)) + "." +
         std::to_string(VK_API_VERSION_PATCH(v));
}

// --- Stage 0: what MoltenVK actually exposes on this GPU --------------------
// The three headline questions for this family on iOS: does the device reach
// the 1.2 floor recon requires (and the 1.3 floor gfx requires, for the shared
// device later); is scalarBlockLayout there (recon's whole buffer ABI rests on
// it); and are timeline semaphores there (the interop-seam handoff).
//
// IMPORTANT: this stage must NOT reuse recon's VkInstance. recon negotiates
// VkApplicationInfo::apiVersion = 1.2 (it needs no more), and MoltenVK caps the
// apiVersion a physical device *advertises* to what its instance asked for --
// so querying through recon's instance can never report above 1.2 and would
// make gfx's 1.3 floor look unsupported on hardware that in fact supports it.
// The question here is "what can this GPU do", not "what did recon ask for", so
// we probe through our own instance created at the implementation's maximum.
struct ProbeInstance {
  VkInstance handle = VK_NULL_HANDLE;
  ~ProbeInstance() {
    if (handle != VK_NULL_HANDLE) {
      vkDestroyInstance(handle, nullptr);
    }
  }
  ProbeInstance() = default;
  ProbeInstance(const ProbeInstance&) = delete;
  ProbeInstance& operator=(const ProbeInstance&) = delete;
};

// Create an instance at the highest API version the implementation supports, so
// the physical device reports its true ceiling.
bool create_probe_instance(ProbeInstance& out, std::uint32_t api_version) {
  VkApplicationInfo app{};
  app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.pApplicationName = "volumetric_kit iOS capability probe";
  app.apiVersion = api_version;

  // Portability enumeration is how a portability driver (MoltenVK) becomes
  // visible when a real loader is in play. Directly linked it is unnecessary,
  // so a failure here retries without it rather than giving up.
  const char* portability_ext = "VK_KHR_portability_enumeration";
  VkInstanceCreateInfo ci{};
  ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  ci.pApplicationInfo = &app;
  ci.enabledExtensionCount = 1;
  ci.ppEnabledExtensionNames = &portability_ext;
#ifdef VK_KHR_portability_enumeration
  ci.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif
  if (vkCreateInstance(&ci, nullptr, &out.handle) == VK_SUCCESS) {
    return true;
  }

  ci.flags = 0;
  ci.enabledExtensionCount = 0;
  ci.ppEnabledExtensionNames = nullptr;
  return vkCreateInstance(&ci, nullptr, &out.handle) == VK_SUCCESS;
}

void stage_device_caps(Report& report, VkPhysicalDevice recon_physical) {
  report.section("Stage 0: device capabilities");

  // The instance-level ceiling: the most any instance on this implementation
  // may request.
  std::uint32_t instance_version = VK_API_VERSION_1_0;
  if (vkEnumerateInstanceVersion(&instance_version) != VK_SUCCESS) {
    instance_version = VK_API_VERSION_1_0;
  }
  report.field("instance API ceiling", api_version_string(instance_version));

  ProbeInstance probe;
  VkPhysicalDevice physical = recon_physical;
  bool probed = false;
  if (create_probe_instance(probe, instance_version)) {
    std::uint32_t count = 0;
    vkEnumeratePhysicalDevices(probe.handle, &count, nullptr);
    if (count > 0) {
      std::vector<VkPhysicalDevice> devices(count);
      if (vkEnumeratePhysicalDevices(probe.handle, &count, devices.data()) ==
          VK_SUCCESS) {
        physical = devices[0];
        probed = true;
      }
    }
  }
  report.field("capability source",
               probed ? "dedicated max-version probe instance"
                      : "recon's 1.2 instance (probe failed; API version and "
                        "1.3 features below are CAPPED and not conclusive)");

  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  report.field("device", props.deviceName);
  report.field("Vulkan API", api_version_string(props.apiVersion));
  report.field("driver version", std::to_string(props.driverVersion));

  // For contrast: what recon's own 1.2 instance sees. Expected to read 1.2 even
  // on a 1.3+ device -- that is the cap described above, not a limitation.
  VkPhysicalDeviceProperties recon_props{};
  vkGetPhysicalDeviceProperties(recon_physical, &recon_props);
  report.field("as advertised to recon's 1.2 instance",
               api_version_string(recon_props.apiVersion));

  VkPhysicalDeviceVulkan12Features features12{};
  features12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  VkPhysicalDeviceVulkan13Features features13{};
  features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  features12.pNext = &features13;
  VkPhysicalDeviceFeatures2 features2{};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.pNext = &features12;
  vkGetPhysicalDeviceFeatures2(physical, &features2);

  report.check(props.apiVersion >= VK_API_VERSION_1_2,
               "Vulkan 1.2 (recon's floor)");
  report.check(props.apiVersion >= VK_API_VERSION_1_3,
               "Vulkan 1.3 (gfx's floor -- needed for the shared device)");
  report.check(features12.scalarBlockLayout == VK_TRUE,
               "scalarBlockLayout (recon's host/GLSL buffer ABI)");
  report.check(features12.timelineSemaphore == VK_TRUE,
               "timelineSemaphore (the interop-seam handoff)");
  report.check(features13.dynamicRendering == VK_TRUE,
               "dynamicRendering (gfx's only rendering path)");

  const VkPhysicalDeviceLimits& limits = props.limits;
  report.field("maxStorageBufferRange",
               std::to_string(limits.maxStorageBufferRange) + " B (" +
                   std::to_string(limits.maxStorageBufferRange / (1u << 20)) +
                   " MiB)");
  report.field("maxComputeWorkGroupCount[0]",
               std::to_string(limits.maxComputeWorkGroupCount[0]));
  report.field("maxComputeWorkGroupInvocations",
               std::to_string(limits.maxComputeWorkGroupInvocations));
  report.field("maxComputeSharedMemorySize",
               std::to_string(limits.maxComputeSharedMemorySize) + " B");

  // Unified memory decides how the ARKit upload path should work: on a UMA GPU
  // a host-visible DEVICE_LOCAL heap means a CVPixelBuffer copy lands straight
  // in GPU-visible memory with no staging blit.
  VkPhysicalDeviceMemoryProperties mem{};
  vkGetPhysicalDeviceMemoryProperties(physical, &mem);
  bool host_visible_device_local = false;
  VkDeviceSize largest_heap = 0;
  for (std::uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
    const VkMemoryPropertyFlags flags = mem.memoryTypes[i].propertyFlags;
    if ((flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0 &&
        (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0) {
      host_visible_device_local = true;
    }
  }
  for (std::uint32_t i = 0; i < mem.memoryHeapCount; ++i) {
    if (mem.memoryHeaps[i].size > largest_heap) {
      largest_heap = mem.memoryHeaps[i].size;
    }
  }
  report.field("unified memory (host-visible DEVICE_LOCAL)",
               yes_no(host_visible_device_local));
  report.field("largest heap",
               std::to_string(largest_heap / (1024ull * 1024ull)) + " MiB");
}

// --- Stage 1: the compute chain end to end ----------------------------------
void stage_compute_dispatch(Report& report, vr::Device& device,
                            vr::Allocator& allocator) {
  report.section("Stage 1: compute dispatch");

  constexpr std::uint32_t kCount = 1024;
  vr::BufferDesc buffer_desc;
  buffer_desc.size = kCount * sizeof(std::uint32_t);
  buffer_desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_desc.memory = vr::MemoryUsage::HostVisible;
  buffer_desc.mapped = true;
  vr::Result<vr::Buffer> buffer = allocator.create_buffer(buffer_desc);
  if (!buffer) {
    report.abort_stage("buffer create: " + buffer.status().message());
    return;
  }

  vr::Result<vr::ShaderModule> shader = vr::ShaderModule::create(
      device.handle(), reinterpret_cast<const std::uint32_t*>(vi_fill_comp_spv),
      vi_fill_comp_spv_size);
  if (!shader) {
    report.abort_stage("shader create: " + shader.status().message());
    return;
  }

  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binding.descriptorCount = 1;
  binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  vr::Result<vr::DescriptorSetLayout> layout =
      vr::DescriptorSetLayout::create(device.handle(), &binding, 1);
  if (!layout) {
    report.abort_stage("descriptor layout: " + layout.status().message());
    return;
  }

  VkDescriptorPoolSize pool_size{};
  pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  pool_size.descriptorCount = 1;
  vr::Result<vr::DescriptorPool> pool =
      vr::DescriptorPool::create(device.handle(), &pool_size, 1, 1);
  if (!pool) {
    report.abort_stage("descriptor pool: " + pool.status().message());
    return;
  }
  vr::Result<vr::DescriptorSet> set = pool.value().allocate(layout->handle());
  if (!set) {
    report.abort_stage("descriptor set: " + set.status().message());
    return;
  }
  set->write_storage_buffer(0, buffer->handle(), 0, VK_WHOLE_SIZE);

  VkPushConstantRange push{};
  push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  push.offset = 0;
  push.size = sizeof(std::uint32_t);
  VkDescriptorSetLayout set_layout = layout->handle();
  vr::ComputePipelineDesc pipeline_desc;
  pipeline_desc.shader = &shader.value();
  pipeline_desc.set_layouts = &set_layout;
  pipeline_desc.set_layout_count = 1;
  pipeline_desc.push_ranges = &push;
  pipeline_desc.push_range_count = 1;
  vr::Result<vr::ComputePipeline> pipeline =
      vr::ComputePipeline::create(device.handle(), pipeline_desc);
  if (!pipeline) {
    report.abort_stage("compute pipeline: " + pipeline.status().message());
    return;
  }
  report.check(true, "pipeline built from embedded SPIR-V");

  const VkDescriptorSet descriptor_set = set->handle();
  const std::uint32_t count = kCount;
  const vr::Status submitted =
      device.submit_single_time([&](VkCommandBuffer cmd) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          pipeline->handle());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipeline->layout(), 0, 1, &descriptor_set, 0,
                                nullptr);
        vkCmdPushConstants(cmd, pipeline->layout(), VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(count), &count);
        vkCmdDispatch(cmd, (count + 63u) / 64u, 1, 1);

        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0,
                             nullptr, 0, nullptr);
      });
  if (!submitted) {
    report.abort_stage("dispatch: " + submitted.message());
    return;
  }
  report.check(true, "dispatched " + std::to_string(kCount) + " threads");

  const auto* out = static_cast<const std::uint32_t*>(buffer->mapped());
  if (out == nullptr) {
    report.abort_stage("buffer was not host-mapped");
    return;
  }
  std::uint32_t mismatches = 0;
  for (std::uint32_t i = 0; i < kCount; ++i) {
    if (out[i] != i) {
      ++mismatches;
    }
  }
  report.check(
      mismatches == 0,
      "readback matched (" + std::to_string(mismatches) + " mismatches)");
}

// The grid shape the remaining stages share: 5 mm voxels in 8^3 blocks, sized
// small enough that the whole thing is comfortable inside an iOS app's memory
// budget.
vol::VoxelGridParams smoke_grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.005f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 1024;
  grid.num_blocks = 1024 * 8;
  grid.max_chain = 128;
  return grid;
}

// --- Stage 2: the scalar-block-layout ABI -----------------------------------
// The genuine iOS risk. recon's host PODs (HashEntry, BlockIndex -- both
// carrying a Vec3i) are byte-identical to their GLSL mirrors only under
// GL_EXT_scalar_block_layout; std430 would 16-byte-align the vec3 and shift
// every field. If MoltenVK's iOS SPIR-V -> MSL translation got that wrong, the
// coordinates written here would come back garbled.
void stage_scalar_abi(Report& report, vr::Device& device,
                      vr::Allocator& allocator) {
  report.section("Stage 2: scalar block layout (host/GLSL ABI)");

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device, allocator, smoke_grid_params(), attrs, 2);
  if (!grid_result) {
    report.abort_stage("VoxelBlockGrid::create: " +
                       grid_result.status().message());
    return;
  }
  vol::VoxelBlockGrid vbg = std::move(grid_result).value();
  report.check(true, "block grid created (tsdf + weight attributes)");

  // Asymmetric, non-zero coordinates in every component: a layout bug that
  // shifted or dropped a field could not survive this unnoticed.
  std::vector<vol::BlockIndex> blocks;
  for (int i = 0; i < 8; ++i) {
    vol::BlockIndex b{};
    b.coord = vr::Vec3i(i - 3, 2 * i + 1, -5 * i);
    blocks.push_back(b);
  }
  vr::Result<std::uint32_t> overflow = vbg.map().allocate(
      blocks.data(), static_cast<std::uint32_t>(blocks.size()));
  if (!overflow) {
    report.abort_stage("allocate: " + overflow.status().message());
    return;
  }
  report.check(overflow.value() == 0, "allocated " +
                                          std::to_string(blocks.size()) +
                                          " blocks with no overflow");

  vr::Result<std::vector<vol::BlockIndex>> active =
      vbg.map().compact_active_blocks();
  if (!active) {
    report.abort_stage("compact_active_blocks: " + active.status().message());
    return;
  }
  report.check(active.value().size() == blocks.size(),
               "active set round-tripped " +
                   std::to_string(active.value().size()) + " of " +
                   std::to_string(blocks.size()) + " blocks");

  // Every coordinate written must come back exactly, in some order.
  std::uint32_t found = 0;
  for (const vol::BlockIndex& want : blocks) {
    for (const vol::BlockIndex& got : active.value()) {
      if (got.coord == want.coord) {
        ++found;
        break;
      }
    }
  }
  report.check(found == blocks.size(),
               "every Vec3i coordinate survived the GPU round-trip (" +
                   std::to_string(found) + "/" + std::to_string(blocks.size()) +
                   ")");
}

// --- Stage 3: the vertical slice --------------------------------------------
// A synthetic posed depth frame through the real spine:
// allocate_from_depth -> TSDF integrate -> marching cubes. This is the shape
// an ARKit frame will drive, with the frame's own depth in place of the plane.
void stage_vertical_slice(Report& report, vr::Device& device,
                          vr::Allocator& allocator) {
  report.section("Stage 3: vertical slice (allocate -> fuse -> mesh)");

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device, allocator, smoke_grid_params(), attrs, 2);
  if (!grid_result) {
    report.abort_stage("VoxelBlockGrid::create: " +
                       grid_result.status().message());
    return;
  }
  vol::VoxelBlockGrid vbg = std::move(grid_result).value();

  // A 160x120 depth frame at a constant 0.5 m -- a flat wall filling the view.
  // Deliberately close to ARKit's 256x192 sceneDepth resolution rather than a
  // desktop 640x480, so the dispatch shape is representative.
  vr::DepthCameraParams cam{};
  cam.width = 160;
  cam.height = 120;
  cam.fx = 130.0f;
  cam.fy = 130.0f;
  cam.cx = static_cast<float>(cam.width) * 0.5f;
  cam.cy = static_cast<float>(cam.height) * 0.5f;
  cam.min_depth = 0.1f;
  cam.max_depth = 5.0f;
  cam.cam_to_world = vr::Mat4f(1.0f);

  const float plane_z = 0.5f;
  const std::vector<float> depth(
      static_cast<std::size_t>(cam.width) * cam.height, plane_z);

  vr::Result<std::uint32_t> overflow =
      vbg.map().allocate_from_depth(depth.data(), cam);
  if (!overflow) {
    report.abort_stage("allocate_from_depth: " + overflow.status().message());
    return;
  }
  vr::Result<std::vector<vol::BlockIndex>> active =
      vbg.map().compact_active_blocks();
  if (!active) {
    report.abort_stage("compact_active_blocks: " + active.status().message());
    return;
  }
  report.check(!active.value().empty() && overflow.value() == 0,
               "allocated " + std::to_string(active.value().size()) +
                   " blocks from a " + std::to_string(cam.width) + "x" +
                   std::to_string(cam.height) + " depth frame");

  vr::Result<tsdf::TsdfIntegrator> integ_result =
      tsdf::TsdfIntegrator::create(device, allocator);
  if (!integ_result) {
    report.abort_stage("TsdfIntegrator::create: " +
                       integ_result.status().message());
    return;
  }
  tsdf::TsdfIntegrator integ = std::move(integ_result).value();

  const vr::Status fused = integ.integrate(vbg, depth.data(), cam);
  if (!fused) {
    report.abort_stage("integrate: " + fused.message());
    return;
  }
  report.check(true, "TSDF integrated one posed depth frame");

  // The plane sits at z = 0.5 m, so a zero crossing must exist and the
  // extracted surface must be non-empty.
  vr::Result<mesh::MarchingCubes> mc_result =
      mesh::MarchingCubes::create(device, allocator);
  if (!mc_result) {
    report.abort_stage("MarchingCubes::create: " +
                       mc_result.status().message());
    return;
  }
  mesh::MarchingCubes mc = std::move(mc_result).value();

  vr::Result<mesh::Mesh> extracted = mc.extract(vbg);
  if (!extracted) {
    report.abort_stage("extract: " + extracted.status().message());
    return;
  }
  const std::size_t triangles = extracted.value().indices.size() / 3;
  report.check(triangles > 0,
               "marching cubes extracted " + std::to_string(triangles) +
                   " triangles / " +
                   std::to_string(extracted.value().vertices.size()) +
                   " vertices");
}

}  // namespace

std::string run_smoke_report() {
  Report report;
  report.line("volumetric_kit_recon -- iOS / MoltenVK smoke");

  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    report.abort_stage("Instance::create: " + instance.status().message());
    return report.text();
  }
  vr::Result<VkPhysicalDevice> gpu = instance->select_physical_device();
  if (!gpu) {
    report.abort_stage("select_physical_device: " + gpu.status().message());
    return report.text();
  }

  stage_device_caps(report, gpu.value());

  vr::Result<vr::Device> device =
      vr::Device::create(instance->handle(), gpu.value(), {});
  if (!device) {
    report.abort_stage("Device::create: " + device.status().message());
    return report.text();
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance->handle(), device.value());
  if (!allocator) {
    report.abort_stage("Allocator::create: " + allocator.status().message());
    return report.text();
  }

  stage_compute_dispatch(report, device.value(), allocator.value());
  stage_scalar_abi(report, device.value(), allocator.value());
  stage_vertical_slice(report, device.value(), allocator.value());

  report.blank();
  report.line(report.failed() ? "RESULT: FAILED" : "RESULT: ALL STAGES PASSED");
  return report.text();
}

}  // namespace volumetric_kit::ios_app

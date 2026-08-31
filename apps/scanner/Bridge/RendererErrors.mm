// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "RendererErrors.hpp"

#import "BridgeStrings.hpp"

#include <optional>
#include <string>

#include "volumetric_kit/gfx/core/vulkan.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

namespace vg = volumetric_kit::gfx;
namespace vr = volumetric_kit::recon;

namespace {

// The one line both renderings of a failure carry: the NSError's localized
// description, and the frame trace's banner. Written once so a dump collected
// off a device and the error Swift showed cannot name the same fault two
// different ways.
std::string described(const char* stage, std::optional<VkResult> vk_result,
                      const std::string& message) {
  std::string out = std::string(stage) + ": " + message;
  if (vk_result) {
    out += " (";
    out += std::string(vg::to_string(*vk_result));
    out += ")";
  }
  return out;
}

// Which VkResult a Status carries, if it carries one. Both libraries keep the
// backend code in a domain-specific slot, so the test is per-library even
// though the answer is the same type.
std::optional<VkResult> vulkan_result(const vg::Status& status) {
  return status.domain() == vg::Status::Code::Vulkan
             ? std::optional<VkResult>(status.code())
             : std::nullopt;
}

std::optional<VkResult> vulkan_result(const vr::Status& status) {
  return status.domain() == vr::Status::Code::Backend
             ? std::optional<VkResult>(static_cast<VkResult>(status.detail()))
             : std::nullopt;
}

void set_error(NSError* _Nullable* _Nullable error, const char* stage,
               VolumetricRendererError code, std::optional<VkResult> vk_result,
               const std::string& message) {
  if (error == nullptr) {
    return;
  }
  NSMutableDictionary* info = [NSMutableDictionary dictionary];
  if (vk_result) {
    info[VolumetricRendererVulkanResultKey] = @(static_cast<int>(*vk_result));
  }
  info[NSLocalizedDescriptionKey] =
      to_ns_string(described(stage, vk_result, message));
  *error = [NSError errorWithDomain:VolumetricRendererErrorDomain
                               code:code
                           userInfo:info];
}

}  // namespace

VolumetricRendererError error_code(vg::Status::Code domain) noexcept {
  switch (domain) {
    case vg::Status::Code::Ok:
      return VolumetricRendererErrorUnknown;
    case vg::Status::Code::InvalidArgument:
      return VolumetricRendererErrorInvalidArgument;
    case vg::Status::Code::NotFound:
      return VolumetricRendererErrorNotFound;
    case vg::Status::Code::Unsupported:
      return VolumetricRendererErrorUnsupported;
    case vg::Status::Code::OutOfMemory:
      return VolumetricRendererErrorOutOfMemory;
    case vg::Status::Code::IoError:
      return VolumetricRendererErrorIoError;
    case vg::Status::Code::Vulkan:
      return VolumetricRendererErrorVulkan;
  }
  // Unreachable while the switch is exhaustive. That is enforced rather than
  // hoped for: this target builds with `-Werror=switch`, so a domain added
  // upstream -- and both siblings track main, not a pin -- stops the compile
  // here instead of reporting itself as `Unknown` to Swift.
  return VolumetricRendererErrorUnknown;
}

VolumetricRendererError error_code(vr::Status::Code domain) noexcept {
  switch (domain) {
    case vr::Status::Code::Ok:
      return VolumetricRendererErrorUnknown;
    case vr::Status::Code::InvalidArgument:
      return VolumetricRendererErrorInvalidArgument;
    case vr::Status::Code::NotFound:
      return VolumetricRendererErrorNotFound;
    case vr::Status::Code::Unsupported:
      return VolumetricRendererErrorUnsupported;
    case vr::Status::Code::OutOfMemory:
      return VolumetricRendererErrorOutOfMemory;
    case vr::Status::Code::IoError:
      return VolumetricRendererErrorIoError;
    // recon keeps its backend neutral, but here the backend *is* Vulkan and the
    // backend code is the VkResult.
    case vr::Status::Code::Backend:
      return VolumetricRendererErrorVulkan;
  }
  return VolumetricRendererErrorUnknown;
}

std::string describe(const vg::Status& status, const char* stage) {
  return described(stage, vulkan_result(status), status.message());
}

std::string describe(const vr::Status& status, const char* stage) {
  return described(stage, vulkan_result(status), status.message());
}

void set_error(NSError* _Nullable* _Nullable error, const vg::Status& status,
               const char* stage) {
  set_error(error, stage, error_code(status.domain()), vulkan_result(status),
            status.message());
}

// The VkResult is carried through rather than flattened into `unsupported`: a
// device-creation failure on a user's phone should name its VkResult, not read
// as a capability the driver lacks.
void set_error(NSError* _Nullable* _Nullable error, const vr::Status& status,
               const char* stage) {
  set_error(error, stage, error_code(status.domain()), vulkan_result(status),
            status.message());
}

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

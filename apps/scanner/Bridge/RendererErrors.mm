// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "RendererErrors.hpp"

#import "BridgeStrings.hpp"

#include <optional>
#include <string>

#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

namespace vkc = volumetric_kit::core;

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
    out += std::string(vkc::to_string(*vk_result));
    out += ")";
  }
  return out;
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

VolumetricRendererError error_code(vkc::Status::Code domain) noexcept {
  switch (domain) {
    case vkc::Status::Code::Ok:
      return VolumetricRendererErrorUnknown;
    case vkc::Status::Code::InvalidArgument:
      return VolumetricRendererErrorInvalidArgument;
    case vkc::Status::Code::NotFound:
      return VolumetricRendererErrorNotFound;
    case vkc::Status::Code::Unsupported:
      return VolumetricRendererErrorUnsupported;
    case vkc::Status::Code::OutOfMemory:
      return VolumetricRendererErrorOutOfMemory;
    case vkc::Status::Code::IoError:
      return VolumetricRendererErrorIoError;
    case vkc::Status::Code::Numerical:
      return VolumetricRendererErrorNumerical;
    // The core keeps its backend neutral, but here the backend *is* Vulkan and
    // the backend code is the VkResult.
    case vkc::Status::Code::Backend:
      return VolumetricRendererErrorVulkan;
  }
  // Unreachable while the switch is exhaustive. That is enforced rather than
  // hoped for: this target builds with `-Werror=switch`, so a domain added
  // upstream stops the compile here, at the pin bump that brings it, instead
  // of reporting itself as `Unknown` to Swift.
  return VolumetricRendererErrorUnknown;
}

std::string describe(const vkc::Status& status, const char* stage) {
  return described(stage, vkc::vk_result(status), status.message());
}

// The VkResult is carried through rather than flattened into `unsupported`: a
// device-creation failure on a user's phone should name its VkResult, not read
// as a capability the driver lacks.
void set_error(NSError* _Nullable* _Nullable error, const vkc::Status& status,
               const char* stage) {
  set_error(error, stage, error_code(status.domain()), vkc::vk_result(status),
            status.message());
}

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

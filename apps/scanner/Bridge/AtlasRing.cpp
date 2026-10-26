// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "AtlasRing.hpp"

#include <utility>

#include "KeyframeBinding.hpp"
#include "volumetric_kit/gfx/pipelines/ycbcr.hpp"

namespace volumetric_kit::ios_app {

namespace vp = volumetric_kit::gfx::pipelines;

vkc::Status bind_keyframe(AtlasSlot& slot, Keyframe keyframe,
                          VkImageView white) {
  vkc::Status refused;
  const char* const unsampled = rebind_keyframe<vp::YcbcrPicture>(
      slot.picture, std::move(keyframe),
      [&](const vp::YcbcrPicture& planes) {
        refused = slot.set->rebind(planes);
        return refused.ok();
      },
      [&] { slot.set->rebind(white); });
  if (unsampled != nullptr) {
    return vkc::Status::unsupported(unsampled);
  }
  return refused;
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "AtlasRing.hpp"

#include <optional>
#include <utility>

#include "volumetric_kit/gfx/pipelines/ycbcr.hpp"
#include "volumetric_kit/recon/sensor/chroma_location.hpp"

namespace volumetric_kit::ios_app {

namespace vp = volumetric_kit::gfx::pipelines;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// gfx samples chroma sited Left or Center; the other sitings have no
// counterpart there.
std::optional<vp::ChromaSiting> siting_of(sensor::ChromaLocation location) {
  switch (location) {
    case sensor::ChromaLocation::Left:
      return vp::ChromaSiting::Left;
    case sensor::ChromaLocation::Center:
      return vp::ChromaSiting::Center;
    case sensor::ChromaLocation::TopLeft:
    case sensor::ChromaLocation::Top:
    case sensor::ChromaLocation::BottomLeft:
    case sensor::ChromaLocation::Bottom:
      return std::nullopt;
  }
  return std::nullopt;
}

}  // namespace

vkc::Status bind_keyframe(AtlasSlot& slot, Keyframe keyframe,
                          VkImageView white) {
  std::optional<Keyframe> picture;
  if (keyframe.image[0] != nullptr && keyframe.image[1] != nullptr) {
    picture = std::move(keyframe);
  }
  vkc::Status refused;
  slot.picture.rebind(
      std::move(picture),
      [&](const Keyframe& k) {
        const std::optional<vp::ChromaSiting> siting =
            siting_of(k.chroma_location);
        if (!siting) {
          refused = vkc::Status::unsupported(
              "keyframe: its chroma siting is neither left nor centre");
          return false;
        }
        vp::YcbcrPicture planes;
        planes.luma = k.image[0].get();
        planes.chroma = k.image[1].get();
        planes.extent = {k.width, k.height};
        planes.coding.kr = k.kr;
        planes.coding.kb = k.kb;
        planes.coding.full_range = k.full_range;
        planes.coding.chroma_siting = *siting;
        refused = slot.set->rebind(planes);
        return refused.ok();
      },
      [&] { slot.set->rebind(white); });
  return refused;
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file KeyframeBinding.hpp
/// @brief What a ring slot's texture set samples for a keyframe: its two
///        planes, their size and how they code R'G'B'.
///
/// Pure, so the mapping can be tested on the host: the bridge
/// (Bridge/AtlasRing.cpp) instantiates it with sensor's `YuvImage` and gfx's
/// `YcbcrPicture`; a test with look-alikes that have the same members.

#include <optional>
#include <utility>

#include "PictureSlot.hpp"

namespace volumetric_kit::ios_app {

/// @return Whether a ring slot's set samples chroma sited at @p location: left
///         or centre, which gfx's picture names; no other.
/// @tparam Location  sensor's `ChromaLocation`.
template <typename Location>
bool chroma_sampled(Location location) {
  // Every location named, so one added upstream stops the bridge's compile
  // (-Werror=switch) rather than rendering white.
  switch (location) {
    case Location::Left:
    case Location::Center:
      return true;
    case Location::TopLeft:
    case Location::Top:
    case Location::BottomLeft:
    case Location::Bottom:
      return false;
  }
  return false;
}

/// @return Whether @p keyframe is one a ring slot samples, short of the set's
///         own refusal: both planes, its chroma sited left or centre
///         (@ref chroma_sampled). Fusion textures a mesh from a picture only
///         when this holds, so a textured mesh never draws against white.
/// @tparam Keyframe  sensor's `YuvImage`, as @ref rebind_keyframe takes it.
template <typename Keyframe>
bool keyframe_sampled(const Keyframe& keyframe) {
  return keyframe.image[0] != nullptr && keyframe.image[1] != nullptr &&
         chroma_sampled(keyframe.chroma_location);
}

/// @brief Sets @p coding to how @p keyframe's values code R'G'B': its matrix
///        weights, range and chroma siting, which @ref chroma_sampled allows.
/// @tparam Coding    gfx's `YcbcrCoding`: `kr`, `kb`, `full_range` and
///                   `chroma_siting`.
/// @tparam Keyframe  sensor's `YuvImage`.
template <typename Coding, typename Keyframe>
void set_coding(Coding& coding, const Keyframe& keyframe) {
  using Location = decltype(keyframe.chroma_location);
  using Siting = decltype(coding.chroma_siting);
  coding.chroma_siting = keyframe.chroma_location == Location::Left
                             ? Siting::Left
                             : Siting::Center;
  coding.kr = keyframe.kr;
  coding.kb = keyframe.kb;
  coding.full_range = keyframe.full_range;
}

/// @brief Points @p slot at @p keyframe through @p bind, or at white through
///        @p bind_white when the keyframe has no planes, its chroma is sited
///        other than left or centre, or @p bind refuses it.
///
/// @p bind gets the keyframe's luma and chroma images, its luma size, matrix
/// weights, range and chroma siting. A keyframe has no planes when its mesh
/// was not textured, as when the import of its picture was refused. Call only
/// when no pending frame has bound the slot's set.
///
/// @tparam Planes    gfx's `YcbcrPicture`: `luma`, `chroma`, `extent` and
///                   `coding` (`kr`, `kb`, `full_range`, `chroma_siting`).
/// @tparam Keyframe  sensor's `YuvImage`: `image[2]`, `width`, `height`, `kr`,
///                   `kb`, `full_range` and `chroma_location`.
/// @param bind        `bool(const Planes&)`: rewrites the set over the planes,
///                    or returns false and leaves the set as it was.
/// @param bind_white  `void()`: rewrites the set over white; cannot fail.
/// @return Why the keyframe's chroma cannot be sampled, or null.
template <typename Planes, typename Keyframe, typename Bind, typename BindWhite>
const char* rebind_keyframe(PictureSlot<Keyframe>& slot, Keyframe keyframe,
                            Bind&& bind, BindWhite&& bind_white) {
  std::optional<Keyframe> picture;
  if (keyframe.image[0] != nullptr && keyframe.image[1] != nullptr) {
    picture = std::move(keyframe);
  }
  const char* refusal = nullptr;
  slot.rebind(
      std::move(picture),
      [&](const Keyframe& k) {
        Planes planes;
        if (!chroma_sampled(k.chroma_location)) {
          refusal = "keyframe: its chroma siting is neither left nor centre";
          return false;
        }
        set_coding(planes.coding, k);
        planes.luma = k.image[0].get();
        planes.chroma = k.image[1].get();
        planes.extent = {k.width, k.height};
        return static_cast<bool>(bind(planes));
      },
      std::forward<BindWhite>(bind_white));
  return refusal;
}

}  // namespace volumetric_kit::ios_app

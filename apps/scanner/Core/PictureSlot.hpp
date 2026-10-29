// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file PictureSlot.hpp
/// @brief The order in which a ring slot's texture set and the picture it
///        samples change hands.
///
/// Pure, so the renderer's keyframe ring (Bridge/AtlasRing.hpp) can be tested
/// on the host: the bridge instantiates it with sensor's imported camera
/// picture and binds through gfx's texture set; a test binds through stubs.

#include <optional>
#include <utility>

namespace volumetric_kit::ios_app {

/// @brief The picture a ring slot's texture set samples, held until the slot
///        is rebound.
///
/// The set names the picture's images and does not own them, so the slot
/// holds the picture for as long as the set may be bound. A rebind rewrites
/// the set first and lets go of the old picture only after, so the set never
/// names a picture nothing holds. When there is no picture, or the set refuses
/// it, the slot falls back to white.
///
/// @tparam Picture  A movable value whose lifetime holds the picture.
template <typename Picture>
class PictureSlot {
 public:
  /// @brief Points the slot at @p picture through @p bind, or at white
  ///        through @p bind_white when there is none or @p bind refuses it.
  ///
  /// Call only when no pending frame has bound the slot's set.
  /// @param picture     The new picture, or none.
  /// @param bind        `bool(const Picture&)`: rewrites the set over the
  ///                    picture, or returns false and leaves the set as it was.
  /// @param bind_white  `void()`: rewrites the set over white; cannot fail.
  /// @return Whether the slot now samples @p picture.
  template <typename Bind, typename BindWhite>
  bool rebind(std::optional<Picture> picture, Bind&& bind,
              BindWhite&& bind_white) {
    if (picture && bind(*picture)) {
      held_ = std::move(picture);
      return true;
    }
    bind_white();
    held_.reset();
    return false;
  }

  /// @return The picture the slot samples, or null while it samples white.
  const Picture* picture() const noexcept { return held_ ? &*held_ : nullptr; }

 private:
  std::optional<Picture> held_;
};

}  // namespace volumetric_kit::ios_app

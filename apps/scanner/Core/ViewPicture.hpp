// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file ViewPicture.hpp
/// @brief The camera view's one picture: taken from the camera's poller, copied
///        by one frame, and let go as soon as that frame has run.
///
/// The camera's pictures are few (the pattern has 4), and every one held is one
/// the camera cannot use. So the view holds at most one, whether it waits for a
/// frame or is being copied by one, and asks for the next only once it has let
/// that go: its frame copies the picture into the view's own image on the GPU
/// (gfx's `ImagePipeline::record_update`), which keeps it from then on. The
/// hold never depends on a newer picture coming, so it cannot stop the camera.
/// The view updates at the pace this allows; the stream is offered every frame
/// first, and is not held up by it.
///
/// Pure, so the host tests count the holds; Bridge/CameraView.cpp instantiates
/// it with sensor's `YuvImage`.
///
/// @code
/// // The camera's poller, each frame, after the stream:
/// picture.offer(frame.color);
/// // The render thread, each frame:
/// picture.retire(frames_completed);
/// if (const YuvImage* p = picture.take(frame_number)) record_copy(*p);
/// @endcode
///
/// @warning @ref offer from one thread, the poller's; the rest from one other
///          at a time, the renderer's.

#include <cstdint>
#include <mutex>
#include <optional>

namespace volumetric_kit::ios_app {

/// @tparam Picture  A copyable value whose lifetime holds the picture.
template <typename Picture>
class ViewPicture {
 public:
  /// @brief The camera's next picture, kept only while the view wants one.
  /// @return Whether it was kept.
  bool offer(const Picture& picture) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (picture_) return false;
    picture_.emplace(picture);
    return true;
  }

  /// @brief The picture waiting, for frame @p frame to copy. It is held until
  ///        @ref retire sees that frame done, or @ref clear.
  /// @return The picture, valid until then; null when none waits.
  const Picture* take(std::uint64_t frame) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!picture_ || copied_by_) return nullptr;
    copied_by_ = frame;
    return &*picture_;
  }

  /// @brief Let the picture go, and want the next, once its frame is done.
  /// @param completed  The newest frame number whose GPU work has completed.
  void retire(std::uint64_t completed) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (copied_by_ && *copied_by_ <= completed) {
      picture_.reset();
      copied_by_.reset();
    }
  }

  /// @brief Let the picture go and want the next, whatever it was waiting for.
  /// @pre No frame not yet run has copied it: the GPU is idle, or the copy was
  ///      not recorded.
  void clear() {
    const std::lock_guard<std::mutex> lock(mutex_);
    picture_.reset();
    copied_by_.reset();
  }

 private:
  mutable std::mutex mutex_;
  // Guarded by mutex_. Released under it, so a new picture is never kept
  // before the old one has gone.
  std::optional<Picture> picture_;
  std::optional<std::uint64_t> copied_by_;  // the frame that copies picture_
};

}  // namespace volumetric_kit::ios_app

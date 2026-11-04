// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file FuseLoop.hpp
/// @brief The fuse thread's body: the camera's frames into the stream and
///        fusion, and the run's console lines every `kReportEverySeconds`.

namespace volumetric_kit::ios_app {

struct RendererImpl;

/// @brief Poll `impl.camera`, offer each frame it hands out to the stream
///        (`impl.stream`, which codes and sends a tenth of them on its own
///        thread) and fuse it, and print two lines every
///        `kReportEverySeconds` (Core/CaptureReport.hpp): the camera's and
///        fusion's counts, times, states and colour codes ("sa"), and the
///        stream's ("st"), until `impl.fusing` goes false; then print the
///        stop's lines ("sa stop", "st stop").
///
/// Waits on `impl.frame_signal` while the camera holds nothing. Each frame
/// is checked against sensor's frame contract (`testing::RgbdContract`) and,
/// from the pattern, for the index its depth carries; a failed poll or fuse is
/// reported through `Fusion::note_error` and skipped, so one bad frame does
/// not end the scan.
/// @pre `impl.camera` is open and started; nothing else polls it.
void run_fuse_loop(RendererImpl& impl);

}  // namespace volumetric_kit::ios_app

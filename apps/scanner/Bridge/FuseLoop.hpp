// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file FuseLoop.hpp
/// @brief The fuse thread's body: the camera's frames into fusion, and the
///        run's console line every `kReportEverySeconds`.

namespace volumetric_kit::ios_app {

struct RendererImpl;

/// @brief Poll `impl.camera`, fuse each frame it hands out, and print one
///        line of counts, times, states and colour codes every
///        `kReportEverySeconds` (Core/CaptureReport.hpp), until `impl.fusing`
///        goes false; then print the run's last line.
///
/// Waits on `impl.frame_signal` while the camera holds nothing. Each frame
/// is checked against sensor's frame contract (`testing::RgbdContract`) and,
/// from the pattern, for the index its depth carries; a failed poll or fuse is
/// reported through `Fusion::note_error` and skipped, so one bad frame does
/// not end the scan.
/// @pre `impl.camera` is open and started; nothing else polls it.
void run_fuse_loop(RendererImpl& impl);

}  // namespace volumetric_kit::ios_app

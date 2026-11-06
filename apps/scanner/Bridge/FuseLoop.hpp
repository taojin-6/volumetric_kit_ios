// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file FuseLoop.hpp
/// @brief The fuse thread's body: the camera's frames into the stream and
///        fusion, or in the client the stream and the camera view, and the
///        run's console lines every `kReportEverySeconds`.

namespace volumetric_kit::ios_app {

struct RendererImpl;

/// @brief Poll `impl.camera` and offer each frame it hands out to the stream
///        (`impl.stream`, which codes and sends a tenth of them on its own
///        thread) at once, and to fusion, which fuses on a worker of its own
///        the newest frame each time it is free (Core/FrameFanout.hpp), so
///        the stream keeps its rate however long a fuse takes. Print two
///        lines every `kReportEverySeconds` (Core/CaptureReport.hpp): the
///        camera's and fusion's counts, times, states and colour codes and
///        the device's thermal state ("sa"), and the stream's ("st"), until
///        `impl.fusing` goes false; then end the worker, after the fuse under
///        way, and print the stop's lines ("sa stop", "st stop").
///
/// In the client (`mode_plan`) no worker runs and nothing is fused: each frame
/// goes to the stream, then to the camera view, which keeps it only when it
/// wants one (Core/ViewPicture.hpp). Fused and skipped read 0.
///
/// The client also asks for a withheld frame's picture on each empty poll
/// (`no_frame_turn`), so the view stays live while tracking is lost. It goes to
/// the view alone: the stream, the contract and every frame's count never see
/// it. From the first ask the camera holds one picture while tracking is lost,
/// for its life, so the loop goes on asking until the stop gives it back.
///
/// The model never asks. Launched as the model, the camera holds nothing for
/// it. Switched from the client, the camera's hold stays: the model keeps at
/// most 4 through a loss (the ring's 3 and one published keyframe), so with the
/// hold it needs no more of the pool than the ring, the frame it fuses and the
/// one waiting already do.
///
/// Waits on `impl.frame_signal` while the camera holds nothing. Each frame
/// is checked against sensor's frame contract (`testing::RgbdContract`) and,
/// from the pattern, for the index its depth carries; a failed poll or fuse is
/// reported through `Fusion::note_error` and skipped, so one bad frame does
/// not end the scan.
///
/// Holds at most one frame more than a loop that fused each frame it polled:
/// the newest, waiting for the worker, let go the moment a newer one comes.
/// @pre `impl.camera` is open and started; nothing else polls it.
void run_fuse_loop(RendererImpl& impl);

}  // namespace volumetric_kit::ios_app

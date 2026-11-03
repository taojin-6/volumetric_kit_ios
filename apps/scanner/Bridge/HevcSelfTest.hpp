// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file HevcSelfTest.hpp
/// @brief `-self-test hevc`: sensor's HEVC encoder rules and the encoder's
///        timing on this device's hardware, on synthetic pictures only.
///
/// Every picture is sensor's test pattern in an IOSurface pixel buffer at
/// ARKit's 1920x1440 (`testing/nv12_pattern.hpp`,
/// `testing/nv12_pixel_buffer.hpp`). The camera is never opened, nothing is
/// written to a file and nothing is sent. What the report says, and whether
/// it passes, is `Core/SelfTest.hpp`'s.

#include <functional>
#include <string>

namespace volumetric_kit::ios_app {

/// @brief Run the self-test on the calling thread (not the main one), which
///        takes about 20 s (17 s on an M5 Max).
///
/// Its report, line by line, as each is made: the device and its thermal
/// state, the GPU the decode check (C6) runs on, the encoder's facts, one
/// line per case of `testing::hevc_encoder_breach` (C1-C8 and C10-C13 on
/// 1920x1440 pixel buffers: BT.601 at either range, BT.709 centre-sited, the
/// QP ends, and every colour code and siting in the encoder's set, each
/// decoded back on the GPU, and run again without the decode when that is
/// what failed), the timing of one session with one picture in flight
/// (send return time, send to access unit, access unit bytes), the thermal
/// state at the end, then the verdict, last.
///
/// With no GPU, the GPU's line says why, each case's name ends in
/// ", no decode", and the verdict fails.
///
/// @param line     Handed each line, without a newline, on the calling thread.
/// @param thermal  The device's thermal state, read when its line is made.
/// @return Whether the cases were decoded, every case kept the rules and every
///         timed picture coded.
bool run_hevc_self_test(const std::function<void(const std::string&)>& line,
                        const std::function<std::string()>& thermal);

}  // namespace volumetric_kit::ios_app

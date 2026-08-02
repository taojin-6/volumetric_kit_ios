// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file smoke_report.hpp
/// @brief The on-device de-risk gate: runs volumetric_kit_recon's compute path
///        against MoltenVK on a real iOS GPU and reports what it found.

#include <string>

namespace volumetric_kit::ios_app {

/// @brief Run every smoke stage and return a human-readable report.
///
/// Stages run in dependency order and each reports independently, so a failure
/// late in the pipeline still leaves the earlier evidence on screen:
///
///   0. **Device capabilities** — which Vulkan version MoltenVK exposes on this
///      GPU and whether the features recon's ABI depends on are present.
///   1. **Compute dispatch** — Instance → Device → Allocator → Buffer →
///      Descriptor → ComputePipeline → dispatch → readback.
///   2. **Scalar block layout** — a `VoxelHashMap` round-trip, which is the
///      real iOS risk: recon's host PODs and their GLSL mirrors agree only
///      under `GL_EXT_scalar_block_layout`.
///   3. **The vertical slice** — allocate from a synthetic depth frame, fuse
///      TSDF, extract a mesh with marching cubes.
///
/// @return The report text, one section per stage.
std::string run_smoke_report();

}  // namespace volumetric_kit::ios_app

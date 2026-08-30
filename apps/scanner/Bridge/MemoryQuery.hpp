// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file MemoryQuery.hpp
/// @brief Reading the jetsam ledger from the kernel.
///
/// The half of the memory read-out that needs mach and a live task port. What
/// the numbers *mean* -- the REV guards, the clamped-headroom alarm, the
/// derived ceiling -- is in `Core/MemoryBudget.hpp`, which is pure and host
/// tested. This file is only the call.

#include "MemoryBudget.hpp"

namespace volumetric_kit::ios_app {

/// @brief Read this process's memory position from the kernel.
///
/// One `task_info` call, plus a `sysctlbyname` on the first call only
/// (`hw.memsize` is immutable for the process's lifetime, so it is cached).
/// Cheap enough for a read-out polled at a few hertz, and not intended for a
/// per-frame path.
///
/// @return The reading, interpreted by @ref interpret_memory_budget -- or a
///         default-constructed value (@ref MemoryBudget::valid `== false`, with
///         @ref MemoryBudget::task_info_status carrying the kernel's code) when
///         the kernel declined to answer.
MemoryBudget query_memory_budget() noexcept;

}  // namespace volumetric_kit::ios_app

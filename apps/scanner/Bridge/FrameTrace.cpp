// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "FrameTrace.hpp"

#include <os/log.h>

#include <algorithm>
#include <cstdio>

namespace volumetric_kit::ios_app {

void FrameTrace::dump(const char* why) const {
  const std::uint64_t count = std::min<std::uint64_t>(next, kCapacity);
  os_log_error(OS_LOG_DEFAULT, "vk-trace: %{public}s -- last %llu frames:", why,
               static_cast<unsigned long long>(count));
  std::fprintf(stderr, "vk-trace: %s -- last %llu frames:\n", why,
               static_cast<unsigned long long>(count));
  for (std::uint64_t i = 0; i < count; ++i) {
    const Entry& e = entries[(next - count + i) % kCapacity];
    // Sized against the widest line this can produce rather than the widest
    // one seen: every %llu here can render 20 digits, and snprintf truncates
    // in silence -- which would cut the tail of the line, where `alloc` and
    // `extract` are.
    char occupancy[24];
    if (e.occupancy_known) {
      std::snprintf(occupancy, sizeof(occupancy), "%.1f%%",
                    100.0 * static_cast<double>(e.occupancy));
    } else {
      // Not a number, because the number is a fabricated 1.0 the allocate
      // guard uses to fail safe. See Entry::occupancy_known.
      std::snprintf(occupancy, sizeof(occupancy), "unreadable");
    }
    char line[384];
    std::snprintf(
        line, sizeof(line),
        "f=%llu drew=%d gen=%llu slot=%zu released<=%llu tris=%u/%u "
        "arena=%llu blocks=%u occ=%s alloc=%s fuse=%.0fms extract=%.1fms",
        static_cast<unsigned long long>(e.frame), e.drew_mesh ? 1 : 0,
        static_cast<unsigned long long>(e.generation), e.mesh_slot,
        static_cast<unsigned long long>(e.released_through), e.triangles,
        e.triangle_capacity, static_cast<unsigned long long>(e.arena_bytes),
        e.active_blocks, occupancy, allocation_stop_tag(e.stop),
        static_cast<double>(e.ms_since_fuse),
        static_cast<double>(e.extract_ms));
    os_log_error(OS_LOG_DEFAULT, "vk-trace: %{public}s", line);
    std::fprintf(stderr, "vk-trace: %s\n", line);
  }
  std::fflush(stderr);
}

}  // namespace volumetric_kit::ios_app

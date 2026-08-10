// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "MemoryBudget.hpp"

#include <mach/mach.h>
#include <mach/task_info.h>
#include <os/proc.h>
#include <sys/sysctl.h>
#include <sys/types.h>

#include <cstddef>

namespace volumetric_kit::ios_app {
namespace {

/// `phys_footprint`, the ledger jetsam decides the kill against.
///
/// `TASK_VM_INFO` rather than `TASK_BASIC_INFO`'s `resident_size`: see
/// MemoryBudget::footprint_bytes for why the two disagree on this app, and by
/// enough that reporting the smaller one would read as comfortable headroom
/// right up to the SIGKILL.
///
/// @return The footprint in bytes, or 0 when the kernel refused.
std::uint64_t phys_footprint() noexcept {
  task_vm_info_data_t info{};
  mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
  const kern_return_t kr =
      task_info(mach_task_self(), TASK_VM_INFO,
                reinterpret_cast<task_info_t>(&info), &count);
  if (kr != KERN_SUCCESS) {
    return 0;
  }
  return static_cast<std::uint64_t>(info.phys_footprint);
}

/// Installed RAM. Context for the ceiling, never a substitute for it.
///
/// @return `hw.memsize` in bytes, or 0 when the sysctl is unavailable -- which
///         is reported as 0 rather than guessed at, since the whole point of
///         this file is to stop treating a memory figure as known.
std::uint64_t device_ram() noexcept {
  std::uint64_t bytes = 0;
  std::size_t size = sizeof(bytes);
  if (sysctlbyname("hw.memsize", &bytes, &size, nullptr, 0) != 0) {
    return 0;
  }
  return bytes;
}

}  // namespace

MemoryBudget query_memory_budget() noexcept {
  const std::uint64_t footprint = phys_footprint();
  if (footprint == 0) {
    // The one call here that distinguishes failure from a legitimate zero. See
    // MemoryBudget::valid: without the footprint the ceiling is underivable,
    // and reporting the headroom alone as the limit would understate how full
    // the process is by exactly what it is already holding.
    return MemoryBudget{};
  }

  MemoryBudget budget;
  budget.footprint_bytes = footprint;
  budget.available_bytes =
      static_cast<std::uint64_t>(os_proc_available_memory());
  budget.limit_bytes = budget.footprint_bytes + budget.available_bytes;
  budget.device_ram_bytes = device_ram();
  budget.valid = true;
  return budget;
}

}  // namespace volumetric_kit::ios_app

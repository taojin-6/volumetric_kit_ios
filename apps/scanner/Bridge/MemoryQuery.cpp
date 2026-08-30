// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "MemoryQuery.hpp"

#include <mach/mach.h>
#include <mach/task_info.h>
#include <sys/sysctl.h>
#include <sys/types.h>

#include <cstddef>

namespace volumetric_kit::ios_app {
namespace {

/// Installed RAM, read once. Context for the ceiling, never a substitute.
///
/// Cached because `hw.memsize` cannot change while the process lives, and this
/// is called from a read-out polled at a few hertz -- re-asking the kernel
/// every tick for a constant is the kind of cost that is invisible until it is
/// in a profile.
///
/// @return `hw.memsize` in bytes, or 0 when the sysctl is unavailable -- which
///         is reported as 0 rather than guessed at, since the whole point of
///         this file is to stop treating a memory figure as known.
std::uint64_t device_ram() noexcept {
  static const std::uint64_t cached = []() noexcept -> std::uint64_t {
    std::uint64_t bytes = 0;
    std::size_t size = sizeof(bytes);
    if (sysctlbyname("hw.memsize", &bytes, &size, nullptr, 0) != 0) {
      return 0;
    }
    return bytes;
  }();
  return cached;
}

}  // namespace

MemoryBudget query_memory_budget() noexcept {
  // One call for every per-process figure on this read-out.
  //
  // `TASK_VM_INFO` rather than `TASK_BASIC_INFO`'s `resident_size`: see
  // MemoryBudget::footprint_bytes for why the two disagree on this app, and by
  // enough that reporting the smaller one would read as comfortable headroom
  // right up to the SIGKILL.
  //
  // And rather than `os_proc_available_memory()`, which <os/proc.h> documents
  // as returning the same quantity as `limit_bytes_remaining` while noting that
  // `task_info` is what computes it. Taking both from one struct is what makes
  // the footprint and the headroom two halves of a single instant rather than
  // two samples of a value the fuse thread is moving underneath us -- see
  // MemoryBudget::limit_bytes. It also drops a syscall and gets the peak for
  // free.
  task_vm_info_data_t info{};
  mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
  const kern_return_t kr =
      task_info(mach_task_self(), TASK_VM_INFO,
                reinterpret_cast<task_info_t>(&info), &count);

  // Which revision introduced which field is mach knowledge, so it is resolved
  // here rather than passed on: the interpreter is told whether a field is
  // present, not how to work that out.
  TaskMemoryReading reading;
  reading.task_info_status = static_cast<int>(kr);
  reading.ok = kr == KERN_SUCCESS;
  reading.device_ram_bytes = device_ram();
  if (reading.ok) {
    reading.has_peak = count >= TASK_VM_INFO_REV3_COUNT;
    reading.has_remaining = count >= TASK_VM_INFO_REV4_COUNT;
    reading.phys_footprint = static_cast<std::uint64_t>(info.phys_footprint);
    reading.ledger_phys_footprint_peak =
        static_cast<std::uint64_t>(info.ledger_phys_footprint_peak);
    reading.limit_bytes_remaining = info.limit_bytes_remaining;
  }
  return interpret_memory_budget(reading);
}

}  // namespace volumetric_kit::ios_app

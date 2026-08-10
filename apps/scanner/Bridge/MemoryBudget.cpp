// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "MemoryBudget.hpp"

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

  MemoryBudget budget;
  budget.task_info_status = static_cast<int>(kr);
  // Reported even when the task port refuses, because it is still true and
  // still worth showing: suppressing the whole row would drop a reading that
  // did not fail along with the one that did.
  budget.device_ram_bytes = device_ram();
  if (kr != KERN_SUCCESS) {
    // Without the footprint the ceiling is underivable, and reporting the
    // headroom alone as the limit would understate how full the process is by
    // exactly what it is already holding.
    return budget;
  }

  budget.footprint_bytes = static_cast<std::uint64_t>(info.phys_footprint);
  budget.valid = true;

  // The kernel may answer with a shorter struct than it was asked for, so each
  // field is taken only once `count` covers the revision that introduced it.
  // `limit_bytes_remaining` arrived in REV4 and `ledger_phys_footprint_peak` in
  // REV3; both are present on every OS this app targets, and the guards are
  // here so a short reply degrades to "not known" rather than to a stack
  // reading printed as a measurement.
  if (count >= TASK_VM_INFO_REV3_COUNT && info.ledger_phys_footprint_peak > 0) {
    budget.peak_footprint_bytes =
        static_cast<std::uint64_t>(info.ledger_phys_footprint_peak);
  }
  if (count >= TASK_VM_INFO_REV4_COUNT) {
    budget.available_bytes = info.limit_bytes_remaining;
    // 0 means the process is at or past its limit -- the kernel clamps the
    // remainder rather than reporting a negative one. Deriving a ceiling from
    // it would produce `limit == footprint`: a read-out showing the limit
    // rising to meet the footprint and calling it 100%, exactly when the true
    // ceiling is the one number worth having. So it is reported as the alarm it
    // is and the ceiling is left unknown.
    budget.at_limit = budget.available_bytes == 0;
    budget.limit_known = !budget.at_limit;
    if (budget.limit_known) {
      budget.limit_bytes = budget.footprint_bytes + budget.available_bytes;
    }
  }
  return budget;
}

}  // namespace volumetric_kit::ios_app

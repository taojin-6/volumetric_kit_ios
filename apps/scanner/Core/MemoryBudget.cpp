// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "MemoryBudget.hpp"

namespace volumetric_kit::ios_app {

MemoryBudget interpret_memory_budget(
    const TaskMemoryReading& reading) noexcept {
  MemoryBudget budget;
  budget.task_info_status = reading.task_info_status;
  // Reported even when the task port refuses, because it is still true and
  // still worth showing: suppressing the whole row would drop a reading that
  // did not fail along with the one that did.
  budget.device_ram_bytes = reading.device_ram_bytes;
  if (!reading.ok) {
    // Without the footprint the ceiling is underivable, and reporting the
    // headroom alone as the limit would understate how full the process is by
    // exactly what it is already holding.
    return budget;
  }

  budget.footprint_bytes = reading.phys_footprint;
  budget.valid = true;

  // The kernel may answer with a shorter struct than it was asked for, so each
  // field is taken only once the reply covers the revision that introduced it.
  // `limit_bytes_remaining` arrived in REV4 and `ledger_phys_footprint_peak` in
  // REV3; both are present on every OS this app targets, and the guards are
  // here so a short reply degrades to "not known" rather than to a stack
  // reading printed as a measurement.
  if (reading.has_peak && reading.ledger_phys_footprint_peak > 0) {
    budget.peak_footprint_bytes = reading.ledger_phys_footprint_peak;
  }
  if (reading.has_remaining) {
    budget.available_bytes = reading.limit_bytes_remaining;
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

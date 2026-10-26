// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <cstddef>

namespace ios_test {

// Fail this thread's allocations after `allowed` successful ones. Arm only
// around host-only API work, never a driver call or a GoogleTest assertion.
class AllocationFailure {
 public:
  explicit AllocationFailure(std::size_t allowed = 0,
                             bool persistent = false) noexcept;
  ~AllocationFailure();
  AllocationFailure(const AllocationFailure&) = delete;
  AllocationFailure& operator=(const AllocationFailure&) = delete;
  AllocationFailure(AllocationFailure&&) = delete;
  AllocationFailure& operator=(AllocationFailure&&) = delete;

  // Disarm before assertions, and report whether an allocation failed.
  bool stop() noexcept;
  // Allocation attempts observed before stop(), including a failed one.
  std::size_t allocations() const noexcept { return allocations_; }

 private:
  bool stopped_ = false;
  bool triggered_ = false;
  std::size_t allocations_ = 0;
};

}  // namespace ios_test

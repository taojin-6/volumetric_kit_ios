// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Complete replacements live apart from the tests so optimizers and the
// sanitizers see matching new/delete families, including sized and aligned
// forms. Only the dedicated allocation-failure test binary links this file.

#include "allocation_failure.hpp"

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>

namespace {

thread_local bool armed = false;
thread_local bool persistent_failure = false;
thread_local bool triggered = false;
thread_local std::size_t remaining = 0;
thread_local std::size_t attempts = 0;

void fail_if_armed() {
  if (!armed) return;
  ++attempts;
  if (remaining != 0) {
    --remaining;
    return;
  }
  triggered = true;
  armed = persistent_failure;
  throw std::bad_alloc();
}

void* allocate(std::size_t size) {
  fail_if_armed();
  if (void* p = std::malloc(size == 0 ? 1 : size)) return p;
  throw std::bad_alloc();
}

void* allocate_aligned(std::size_t size, std::align_val_t alignment) {
  fail_if_armed();
  const auto align = static_cast<std::size_t>(alignment);
  // Round up for aligned_alloc's multiple-of-alignment size contract.
  const std::size_t bytes = size == 0 ? align : size;
  if (bytes > std::numeric_limits<std::size_t>::max() - (align - 1)) {
    throw std::bad_alloc();
  }
  const std::size_t padded = ((bytes + align - 1) / align) * align;
  if (void* p = std::aligned_alloc(align, padded)) return p;
  throw std::bad_alloc();
}

}  // namespace

namespace ios_test {

AllocationFailure::AllocationFailure(std::size_t allowed,
                                     bool persistent) noexcept {
  remaining = allowed;
  persistent_failure = persistent;
  triggered = false;
  attempts = 0;
  armed = true;
}

AllocationFailure::~AllocationFailure() { stop(); }

bool AllocationFailure::stop() noexcept {
  if (!stopped_) {
    armed = false;
    triggered_ = triggered;
    allocations_ = attempts;
    stopped_ = true;
  }
  return triggered_;
}

}  // namespace ios_test

// The standard library declares these replaceable functions with reserved
// parameter names; application definitions use ordinary portable names.
// NOLINTBEGIN(readability-inconsistent-declaration-parameter-name)
void* operator new(std::size_t size) { return allocate(size); }
void* operator new[](std::size_t size) { return allocate(size); }
void* operator new(std::size_t size, std::align_val_t alignment) {
  return allocate_aligned(size, alignment);
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
  return allocate_aligned(size, alignment);
}
void* operator new(std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return allocate(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, const std::nothrow_t& /*tag*/) noexcept {
  try {
    return allocate(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t& /*tag*/) noexcept {
  try {
    return allocate_aligned(size, alignment);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t& /*tag*/) noexcept {
  try {
    return allocate_aligned(size, alignment);
  } catch (...) {
    return nullptr;
  }
}
// NOLINTEND(readability-inconsistent-declaration-parameter-name)
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t /*size*/) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t /*size*/) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t /*alignment*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t /*alignment*/) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t /*size*/,
                     std::align_val_t /*alignment*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t /*size*/,
                       std::align_val_t /*alignment*/) noexcept {
  std::free(p);
}
void operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}
void operator delete(void* p, std::align_val_t /*alignment*/,
                     const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t /*alignment*/,
                       const std::nothrow_t& /*tag*/) noexcept {
  std::free(p);
}

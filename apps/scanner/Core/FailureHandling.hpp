// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <atomic>
#include <cstdint>
#include <exception>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace volumetric_kit::ios_app {

/// Run one capture/fusion callback without letting a C++ exception escape its
/// platform or thread entry point. Failure receives borrowed exception text;
/// even a failure reporter that throws cannot escape this boundary.
template <class Work, class Failure>
bool contain_frame_failure(Work&& work, Failure&& failure) noexcept {
  try {
    std::forward<Work>(work)();
    return true;
  } catch (const std::exception& error) {
    try {
      std::forward<Failure>(failure)(std::string_view(error.what()));
    } catch (...) {
    }
  } catch (...) {
    try {
      std::forward<Failure>(failure)(std::string_view("unknown exception"));
    } catch (...) {
    }
  }
  return false;
}

/// Start only an idle, joined worker. The flag is set before launch so its
/// loop can run immediately, and restored on construction failure. A prior
/// joinable thread is never overwritten, even when its stop flag is set.
/// Callers retain capture before this call and release it on false only when
/// they have already excluded a running/joinable prior worker.
template <class Launch, class Failure>
bool start_worker(std::atomic<bool>& running, std::thread& worker,
                  Launch&& launch, Failure&& failure) noexcept {
  if (running.load() || worker.joinable()) return false;
  running.store(true);
  const bool started =
      contain_frame_failure([&] { worker = std::forward<Launch>(launch)(); },
                            std::forward<Failure>(failure));
  if (!started || !worker.joinable()) {
    running.store(false);
    return false;
  }
  return true;
}

/// Count an error even if its diagnostic cannot be stored. On allocation
/// failure clear the previous text rather than show it as the newest error.
/// The caller supplies synchronization for these two fields.
void record_error(std::uint64_t& count, std::string& stored,
                  std::string_view message,
                  std::string_view context = {}) noexcept;

}  // namespace volumetric_kit::ios_app

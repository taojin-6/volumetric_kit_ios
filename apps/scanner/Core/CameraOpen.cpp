// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "CameraOpen.hpp"

#include <algorithm>
#include <utility>

namespace volumetric_kit::ios_app {

bool CameraOpenPolicy::begin(CameraOpenCause cause,
                             std::uint64_t now_ns) noexcept {
  const bool due = state_ == CameraOpenState::Idle ||
                   (state_ == CameraOpenState::Retrying &&
                    (cause == CameraOpenCause::Wake || now_ns >= retry_at_ns_));
  if (!due) return false;
  state_ = CameraOpenState::Opening;
  ++opens_;
  return true;
}

void CameraOpenPolicy::end(CameraOpenEnd end, std::string error,
                           std::uint64_t now_ns) {
  if (state_ != CameraOpenState::Opening) return;
  switch (end) {
    case CameraOpenEnd::Opened:
      state_ = CameraOpenState::Open;
      failed_ = 0;
      error_.clear();
      return;
    case CameraOpenEnd::NotFound: {
      state_ = CameraOpenState::Retrying;
      // 2 s, 4 s, 8 s, 16 s, then 30 s: the shift stops before it could pass
      // the cap, however many opens failed.
      const std::uint32_t doublings = std::min<std::uint32_t>(failed_, 5);
      retry_at_ns_ =
          now_ns + std::min(kFirstBackoffNs << doublings, kMaxBackoffNs);
      ++failed_;
      error_ = std::move(error);
      return;
    }
    case CameraOpenEnd::Refused:
      state_ = CameraOpenState::Failed;
      error_ = std::move(error);
      return;
  }
}

void CameraOpenPolicy::refuse(std::string why) {
  if (state_ == CameraOpenState::Open) return;
  state_ = CameraOpenState::Failed;
  error_ = std::move(why);
}

std::string camera_open_text(const CameraOpenPolicy& policy,
                             std::uint64_t now_ns) {
  const std::uint32_t retries = policy.retries();
  switch (policy.state()) {
    case CameraOpenState::Idle:
      return "not opened yet";
    case CameraOpenState::Opening:
      if (retries == 0) return "opening";
      return "opening again (retry " + std::to_string(retries) +
             ") after: " + policy.error();
    case CameraOpenState::Open:
      if (retries == 0) return "open";
      return "open after " + std::to_string(retries) +
             (retries == 1 ? " retry" : " retries");
    case CameraOpenState::Retrying: {
      const std::uint64_t at = policy.retry_at_ns();
      // Whole seconds, rounded up: a tick due in 0.4 s says 1 s.
      const std::uint64_t left_s =
          at > now_ns ? (at - now_ns + 999'999'999) / 1'000'000'000 : 0;
      return policy.error() + "; retry " + std::to_string(retries + 1) +
             " when the app is in front again, or in " +
             std::to_string(left_s) + " s";
    }
    case CameraOpenState::Failed:
      return policy.error();
  }
  return policy.error();
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "LinkState.hpp"

namespace volumetric_kit::ios_app {

LinkState link_state(bool target, std::size_t viewers,
                     std::uint64_t dial_failures, bool no_route) noexcept {
  if (!target) return LinkState::Off;
  if (viewers > 0) return LinkState::Connected;
  if (dial_failures == 0) return LinkState::Dialing;
  return no_route ? LinkState::NeedsLocalNetwork : LinkState::Retrying;
}

}  // namespace volumetric_kit::ios_app

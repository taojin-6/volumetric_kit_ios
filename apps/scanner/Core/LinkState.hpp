// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file LinkState.hpp
/// @brief What the scanner's live link shows on screen: off, dialling,
///        connected, retrying, or waiting for Local Network access.
///
/// The writer counts (`RgbdStreamStats`: readers served, dials failed since
/// the target was set, whether the last found no route); this decides what
/// they mean to the person at the entry. The Objective-C
/// `VolumetricLinkState` mirrors @ref LinkState value for value, and
/// `VolumetricLink.mm` static_asserts the two agree.

#include <cstddef>
#include <cstdint>

namespace volumetric_kit::ios_app {

/// @brief The live link, as the screen states it.
enum class LinkState {
  /// No reader named: nothing is dialled.
  Off = 0,
  /// A reader named, none connected, and no dial to it has failed yet.
  Dialing,
  /// A reader is being served.
  Connected,
  /// Dials to the reader failed; the link dials again each second.
  Retrying,
  /// The last dial found no route to the host, as iOS answers until the app
  /// is allowed Local Network access. The link goes on dialling, which keeps
  /// iOS's prompt up; the screen asks the person to allow it.
  NeedsLocalNetwork,
};

/// @brief The state of a link.
/// @param target         Whether a reader is named (a host is set).
/// @param viewers        Readers served now.
/// @param dial_failures  Dials failed since the target was last set.
/// @param no_route       The last dial found no route to the host.
/// @return Off without a target; Connected while a reader is served, however
///         dials went before; else Dialing before any failure, then
///         NeedsLocalNetwork after a dial with no route, Retrying after any
///         other.
LinkState link_state(bool target, std::size_t viewers,
                     std::uint64_t dial_failures, bool no_route) noexcept;

}  // namespace volumetric_kit::ios_app

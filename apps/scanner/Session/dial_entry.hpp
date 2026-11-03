// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file dial_entry.hpp
/// @brief The reader the scanner dials, as its on-screen entry names it: an
///        address field and a port field, read with the core's checks
///        (`core/net/address.hpp`), whose reasons the screen shows as they
///        are.
///
/// @code
/// namespace app = volumetric_kit::ios_app;
/// auto target = app::parse_dial_entry("10.8.0.1", "7101");
/// if (!target.ok()) show_under_the_field(target.status().message());
/// else VKC_TRY(writer->dial_to(target->host, target->port));  // "" dials none
/// @endcode

#include <cstdint>
#include <string>
#include <string_view>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"

namespace volumetric_kit::ios_app {

/// @brief A reader to dial.
struct DialTarget {
  std::string host;  ///< One host's numeric IPv4 address; empty dials none.
  std::uint16_t port = grid::networking::kDefaultPort;  ///< Its port.
};

/// @brief Read the on-screen entry. Spaces and tabs around either field are
///        dropped, as a pasted address carries them.
/// @param address  The address field: a numeric IPv4 address, or one with
///                 its port after a ':' ("10.8.0.1:7101"), which takes that
///                 port; empty dials none.
/// @param port     The port field: 1 to 65535, checked even with an empty
///                 address; empty takes `grid::networking::kDefaultPort`.
/// @return The target; `Status::Code::InvalidArgument` whose message is the
///         core's sentence alone, for the screen to show under the field:
///         `core::net::parse_port`'s for the port field,
///         `core::net::parse_host_port`'s and `core::net::parse_endpoint`'s
///         for the address (a name, IPv6, the wildcard 0.0.0.0, a bad port
///         after the ':').
core::Result<DialTarget> parse_dial_entry(std::string_view address,
                                          std::string_view port);

/// @return Whether @p dial_failure, a refused `core::net::dial` or
///         `StreamServer::add_viewer`, failed with no route to the host
///         (EHOSTUNREACH, in the system's words that end the message). iOS
///         answers a connect to the local network so until the person allows
///         the app Local Network access, and asks them only while the app
///         keeps trying.
bool is_no_route(const core::Status& dial_failure);

}  // namespace volumetric_kit::ios_app

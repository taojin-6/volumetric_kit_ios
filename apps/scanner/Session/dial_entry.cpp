// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "dial_entry.hpp"

#include <cerrno>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "volumetric_kit/core/base/exception_status.hpp"
#include "volumetric_kit/core/net/address.hpp"

namespace volumetric_kit::ios_app {

namespace net = core::net;

namespace {

std::string_view trimmed(std::string_view text) {
  constexpr std::string_view kSpace = " \t";
  const std::size_t first = text.find_first_not_of(kSpace);
  if (first == std::string_view::npos) return {};
  const std::size_t last = text.find_last_not_of(kSpace);
  return text.substr(first, last - first + 1);
}

}  // namespace

core::Result<DialTarget> parse_dial_entry(std::string_view address,
                                          std::string_view port) try {
  DialTarget target;
  const std::string_view port_text = trimmed(port);
  if (!port_text.empty()) {
    VKC_ASSIGN(target.port, net::parse_port(port_text));
  }
  const std::string_view host_text = trimmed(address);
  if (host_text.empty()) return target;  // dials none
  VKC_ASSIGN(net::HostPort typed, net::parse_host_port(host_text, target.port));
  VKC_TRY(net::parse_endpoint(typed.host, typed.port).status());
  target.host = std::move(typed.host);
  target.port = typed.port;
  return target;
} catch (...) {
  return core::exception_status("parse_dial_entry",
                                core::Status::Code::InvalidArgument);
}

bool is_no_route(const core::Status& dial_failure) {
  if (dial_failure.ok()) return false;
  // The core ends a socket call's failure with the system's words for its
  // errno (`std::generic_category`), and keeps no errno.
  const std::string words = std::generic_category().message(EHOSTUNREACH);
  const std::string& message = dial_failure.message();
  return message.size() >= words.size() &&
         message.compare(message.size() - words.size(), words.size(), words) ==
             0;
}

}  // namespace volumetric_kit::ios_app

// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file dial_entry_test.cpp
/// @brief The on-screen entry's two fields read as a reader to dial, each
///        refusal the core's own sentence; and the dial failure that means
///        iOS has not allowed Local Network access yet.

#include "dial_entry.hpp"

#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <string>
#include <system_error>

#include "volumetric_kit/core/net/address.hpp"
#include "volumetric_kit/core/net/dial.hpp"

namespace {

namespace app = volumetric_kit::ios_app;
namespace net = volumetric_kit::core::net;
namespace networking = volumetric_kit::grid::networking;
namespace vkc = volumetric_kit::core;

app::DialTarget parsed(const char* address, const char* port) {
  auto target = app::parse_dial_entry(address, port);
  EXPECT_TRUE(target.ok()) << address << " / " << port << ": "
                           << target.status().message();
  return target.ok() ? target.value() : app::DialTarget{};
}

TEST(DialEntry, AnAddressAndAPort) {
  app::DialTarget t = parsed("10.8.0.1", "7101");
  EXPECT_EQ(t.host, "10.8.0.1");
  EXPECT_EQ(t.port, 7101);
  // An empty port field takes the default port.
  t = parsed("192.168.21.217", "");
  EXPECT_EQ(t.host, "192.168.21.217");
  EXPECT_EQ(t.port, networking::kDefaultPort);
  // Spaces and tabs around a pasted field are dropped.
  t = parsed(" 10.8.0.1\t", " 7101 ");
  EXPECT_EQ(t.host, "10.8.0.1");
  EXPECT_EQ(t.port, 7101);
}

/// An address with its own port takes that one, whatever the port field
/// says.
TEST(DialEntry, AnAddressWithItsPortTakesThatPort) {
  app::DialTarget t = parsed("10.8.0.1:7101", "");
  EXPECT_EQ(t.host, "10.8.0.1");
  EXPECT_EQ(t.port, 7101);
  t = parsed("10.8.0.1:7101", "47801");
  EXPECT_EQ(t.port, 7101);
}

/// An empty address dials nothing; the port field is still checked.
TEST(DialEntry, AnEmptyAddressDialsNothing) {
  EXPECT_EQ(parsed("", "").host, "");
  EXPECT_EQ(parsed("  ", "7101").host, "");
  EXPECT_EQ(app::parse_dial_entry("", "abc").status().domain(),
            vkc::Status::Code::InvalidArgument);
}

/// Each refusal is InvalidArgument, its message the core's sentence alone,
/// so the screen shows it as it is.
TEST(DialEntry, RefusalsAreTheCoresSentences) {
  const auto refused = [](const char* address, const char* port,
                          const vkc::Status& core) {
    const vkc::Status got = app::parse_dial_entry(address, port).status();
    EXPECT_EQ(got.domain(), vkc::Status::Code::InvalidArgument)
        << address << " / " << port;
    EXPECT_FALSE(core.ok());
    EXPECT_EQ(got.message(), core.message()) << address << " / " << port;
  };
  refused("10.8.0.1", "0", net::parse_port("0").status());
  refused("10.8.0.1", "65536", net::parse_port("65536").status());
  refused("10.8.0.1", "7101x", net::parse_port("7101x").status());
  refused("lab-box", "", net::parse_endpoint("lab-box", 47801).status());
  refused("0.0.0.0", "", net::parse_endpoint("0.0.0.0", 47801).status());
  refused("010.8.0.1", "", net::parse_endpoint("010.8.0.1", 47801).status());
  refused("[::1]:7101", "", net::parse_host_port("[::1]:7101", 47801).status());
  refused("10.8.0.1:7101:1", "",
          net::parse_host_port("10.8.0.1:7101:1", 47801).status());
  refused("10.8.0.1:0", "", net::parse_host_port("10.8.0.1:0", 47801).status());
}

/// "No route to host" ends the core's message for a dial iOS has not allowed
/// onto the local network yet; any other failure, and OK, is not it.
TEST(DialEntry, NoRouteIsTheFailureThatEndsInTheSystemsWordsForIt) {
  const std::string no_route = std::generic_category().message(EHOSTUNREACH);
  const std::string refused = std::generic_category().message(ECONNREFUSED);
  EXPECT_TRUE(app::is_no_route(vkc::Status::io_error(
      "dial: 192.168.1.20:47801: cannot connect: " + no_route)));
  EXPECT_FALSE(app::is_no_route(vkc::Status::io_error(
      "dial: 192.168.1.20:47801: cannot connect: " + refused)));
  EXPECT_FALSE(app::is_no_route(vkc::Status::io_error(
      "dial: 192.168.1.20:47801: cannot connect within 2000 ms")));
  EXPECT_FALSE(app::is_no_route(vkc::Status::io_error(no_route + ", once")));
  EXPECT_FALSE(app::is_no_route(vkc::Status{}));

  // The core ends a real refusal with the system's words, as the rule reads.
  std::uint16_t closed = 0;
  {
    auto gone = net::Listener::create("127.0.0.1", 0);
    ASSERT_TRUE(gone.ok()) << gone.status().message();
    closed = gone.value()->port();
  }
  const auto fd =
      net::dial("127.0.0.1", closed, std::chrono::milliseconds{2'000});
  ASSERT_FALSE(fd.ok());
  const std::string& message = fd.status().message();
  ASSERT_GE(message.size(), refused.size());
  EXPECT_EQ(message.substr(message.size() - refused.size()), refused)
      << message;
  EXPECT_FALSE(app::is_no_route(fd.status()));
}

}  // namespace

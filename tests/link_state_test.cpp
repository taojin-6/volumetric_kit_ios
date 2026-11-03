// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "LinkState.hpp"

#include <gtest/gtest.h>

namespace {

namespace app = volumetric_kit::ios_app;
using app::LinkState;

TEST(LinkState, NoTargetIsOffWhateverTheCounts) {
  EXPECT_EQ(app::link_state(false, 0, 0, false), LinkState::Off);
  EXPECT_EQ(app::link_state(false, 1, 3, true), LinkState::Off);
}

TEST(LinkState, AServedReaderIsConnectedHoweverDialsWentBefore) {
  EXPECT_EQ(app::link_state(true, 1, 0, false), LinkState::Connected);
  EXPECT_EQ(app::link_state(true, 1, 4, false), LinkState::Connected);
  EXPECT_EQ(app::link_state(true, 1, 4, true), LinkState::Connected);
}

TEST(LinkState, DialingUntilADialFails) {
  EXPECT_EQ(app::link_state(true, 0, 0, false), LinkState::Dialing);
}

/// After a failure: no route asks for Local Network access, as iOS answers
/// until it is allowed; any other failure is a retry.
TEST(LinkState, AFailedDialRetriesOrAsksForLocalNetworkAccess) {
  EXPECT_EQ(app::link_state(true, 0, 1, false), LinkState::Retrying);
  EXPECT_EQ(app::link_state(true, 0, 7, false), LinkState::Retrying);
  EXPECT_EQ(app::link_state(true, 0, 1, true), LinkState::NeedsLocalNetwork);
  EXPECT_EQ(app::link_state(true, 0, 9, true), LinkState::NeedsLocalNetwork);
}

}  // namespace

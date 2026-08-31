// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "GrowthPolicy.hpp"

namespace volumetric_kit::ios_app {

GrowthPlan plan_growth(const GrowthInputs& in) noexcept {
  GrowthPlan plan;

  if (!growth_due(in)) {
    return plan;
  }

  plan.grow_to = grow_target(in.num_buckets, in.max_buckets);
  plan.needed_bytes = grid_bytes_for(plan.grow_to);

  // Only a reading that is both valid and carries a real ceiling can decline a
  // grow. See plan_growth's contract: an unreadable budget is not evidence of
  // a full one.
  const bool budget_can_refuse = in.budget.valid && in.budget.limit_known;
  plan.action =
      budget_can_refuse && in.budget.available_bytes < plan.needed_bytes
          ? GrowthAction::DeclinedForMemory
          : GrowthAction::Resize;
  return plan;
}

}  // namespace volumetric_kit::ios_app

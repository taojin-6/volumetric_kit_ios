// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "GrowthPolicy.hpp"

namespace volumetric_kit::ios_app {

GrowthPlan plan_resize(std::int32_t num_buckets, std::int32_t max_buckets,
                       const MemoryBudget& budget) noexcept {
  GrowthPlan plan;
  if (num_buckets >= max_buckets) return plan;
  plan.grow_to = grow_target(num_buckets, max_buckets);
  plan.needed_bytes = grid_bytes_for(plan.grow_to);
  // An exhausted reading cannot derive the total ceiling, but it still says
  // there is no headroom. Missing or failed readings carry neither flag.
  const bool cannot_afford =
      budget.valid &&
      (budget.at_limit ||
       (budget.limit_known && budget.available_bytes < plan.needed_bytes));
  plan.action =
      cannot_afford ? GrowthAction::DeclinedForMemory : GrowthAction::Resize;
  return plan;
}

GrowthPlan plan_growth(const GrowthInputs& in) noexcept {
  if (!growth_due(in)) {
    return {};
  }
  return plan_resize(in.num_buckets, in.max_buckets, in.budget);
}

}  // namespace volumetric_kit::ios_app

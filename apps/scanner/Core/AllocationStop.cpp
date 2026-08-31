// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "AllocationStop.hpp"

#include <string>

namespace volumetric_kit::ios_app {

AllocationStopText allocation_stop_text(AllocationStop stop) noexcept {
  switch (stop) {
    case AllocationStop::None:
      return {"", "", false};
    case AllocationStop::VolumeFull:
      return {"volume full",
              "Existing surface keeps refining, but new areas will not be "
              "added. Finish here, or restart with a coarser voxel size.",
              false};
    case AllocationStop::OccupancyUnknown:
      return {"occupancy unreadable",
              "The volume is not known to be full -- the table's load factor "
              "could not be read, and allocation refused on a fabricated "
              "figure. The cause is on the errors row.",
              true};
    case AllocationStop::BlocksDropped:
      return {"blocks dropped",
              "The allocate hit a capacity limit and dropped this frame's "
              "blocks. This can fire well below the occupancy guard -- at the "
              "bucket ceiling, or with the frame's grow budget spent.",
              true};
  }
  // Unreachable while the switch above is exhaustive, which `-Werror=switch`
  // is what actually enforces. Here rather than a trailing `default:` so that
  // adding a cause is a compile error rather than a silent fall-through to a
  // blank phrase -- which would read as a healthy scan on all four renderings
  // at once.
  return {"", "", false};
}

std::string allocation_stop_row(AllocationStop stop) {
  if (stop == AllocationStop::None) {
    return {};
  }
  const AllocationStopText text = allocation_stop_text(stop);
  std::string row = "ALLOCATION STOPPED — ";
  row += text.headline;
  if (text.on_errors_row) {
    row += "  (see the errors row)";
  }
  return row;
}

const char* allocation_stop_tag(AllocationStop stop) noexcept {
  switch (stop) {
    case AllocationStop::None:
      return "ok";
    case AllocationStop::VolumeFull:
      return "full";
    case AllocationStop::OccupancyUnknown:
      return "unknown";
    case AllocationStop::BlocksDropped:
      return "dropped";
  }
  return "ok";
}

}  // namespace volumetric_kit::ios_app

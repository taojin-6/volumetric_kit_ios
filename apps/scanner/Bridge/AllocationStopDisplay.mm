// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "AllocationStopDisplay.hpp"

namespace volumetric_kit::ios_app {

const char* allocation_stop_note(AllocationStop stop) noexcept {
  switch (stop) {
    case AllocationStop::None:
      return "";
    case AllocationStop::VolumeFull:
      return "  -- NOT TAKING NEW GEOMETRY (volume full)";
    case AllocationStop::OccupancyUnknown:
      return "  -- NOT TAKING NEW GEOMETRY (occupancy unreadable, see error)";
    case AllocationStop::BlocksDropped:
      return "  -- NOT TAKING NEW GEOMETRY (blocks dropped, see error)";
  }
  return "";
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

AllocationStopText allocation_stop_text(AllocationStop stop) noexcept {
  switch (stop) {
    case AllocationStop::None:
      return {"", ""};
    case AllocationStop::VolumeFull:
      return {"volume full",
              "Existing surface keeps refining, but new areas will not be "
              "added. Finish here, or restart with a coarser voxel size."};
    case AllocationStop::OccupancyUnknown:
      return {"occupancy unreadable",
              "The volume is not known to be full -- the table's load factor "
              "could not be read, and allocation refused on a fabricated "
              "figure. The cause is on the errors row."};
    case AllocationStop::BlocksDropped:
      return {"blocks dropped",
              "The allocate hit a capacity limit and dropped this frame's "
              "blocks. This can fire well below the occupancy guard -- at the "
              "bucket ceiling, or with the frame's grow budget spent."};
  }
  return {"", ""};
}

/// The same cause, as the Swift-facing enum.
///
/// Switched rather than cast, though the two enumerations are declared in the
/// same order: a cast makes that order load-bearing across two files that no
/// build step compares, and the failure is a sample reporting the
/// *neighbouring* cause -- a wrong answer that looks exactly like a right one.

VolumetricAllocationStop allocation_stop_value(AllocationStop stop) noexcept {
  switch (stop) {
    case AllocationStop::None:
      return VolumetricAllocationStopNone;
    case AllocationStop::VolumeFull:
      return VolumetricAllocationStopVolumeFull;
    case AllocationStop::OccupancyUnknown:
      return VolumetricAllocationStopOccupancyUnknown;
    case AllocationStop::BlocksDropped:
      return VolumetricAllocationStopBlocksDropped;
  }
  return VolumetricAllocationStopNone;
}

}  // namespace volumetric_kit::ios_app

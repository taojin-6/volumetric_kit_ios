// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "AllocationStopDisplay.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

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

NS_ASSUME_NONNULL_END

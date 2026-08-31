// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file RendererErrors.hpp
/// @brief Reducing a library `Status` to the `NSError` Swift sees.
///
/// The two libraries' Status types are structurally alike -- domain, optional
/// backend code, message -- but neither imports the other, so each gets its own
/// overload here rather than a shared adapter neither could own.
///
/// Why the *domain* is the NSError code, and the VkResult rides in the user
/// info instead, is argued on @ref VolumetricRendererError in
/// VolumetricRenderer.h; this file is what implements that decision.

#import <Foundation/Foundation.h>

#import "VolumetricRenderer.h"

#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::ios_app {

/// @brief The `NSError.code` for a gfx failure domain.
VolumetricRendererError error_code(
    volumetric_kit::gfx::Status::Code domain) noexcept;

/// @brief The `NSError.code` for a recon failure domain.
VolumetricRendererError error_code(
    volumetric_kit::recon::Status::Code domain) noexcept;

/// @brief Surface a gfx `Status` as an `NSError`, no-op when @p error is null.
/// @param stage  What was being attempted, prefixed to the description.
void set_error(NSError** error, const volumetric_kit::gfx::Status& status,
               const char* stage);

/// @brief Surface a recon `Status` as an `NSError`, no-op when @p error is
/// null.
void set_error(NSError** error, const volumetric_kit::recon::Status& status,
               const char* stage);

}  // namespace volumetric_kit::ios_app

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

#include <string>

#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/recon/core/result.hpp"

NS_ASSUME_NONNULL_BEGIN

namespace volumetric_kit::ios_app {

/// @brief The `NSError.code` for a gfx failure domain.
VolumetricRendererError error_code(
    volumetric_kit::gfx::Status::Code domain) noexcept;

/// @brief The `NSError.code` for a recon failure domain.
VolumetricRendererError error_code(
    volumetric_kit::recon::Status::Code domain) noexcept;

/// @brief One line naming what failed, what it said, and its `VkResult`.
///
/// The same string `set_error` puts in `NSLocalizedDescriptionKey`, exposed
/// because the `NSError` is not the only place a failure has to be legible.
/// The frame trace's banner is read out of a collected device log, long after
/// any `NSError` is gone, and it used to print `status.message()` alone -- for
/// every gfx fence wait that is the bare string `"vkWaitForFences"`, the
/// stringified call rather than its result, so a `VK_ERROR_DEVICE_LOST` and a
/// `VK_TIMEOUT` produced byte-identical dumps. The `VkResult` is what the ring
/// is opened to find, and it was in hand at the call site the whole time.
std::string describe(const volumetric_kit::gfx::Status& status,
                     const char* stage);

/// @brief One line naming what failed, what it said, and its `VkResult`.
std::string describe(const volumetric_kit::recon::Status& status,
                     const char* stage);

/// @brief Surface a gfx `Status` as an `NSError`, no-op when @p error is null.
/// @param stage  What was being attempted, prefixed to the description.
///
/// Both pointers are explicitly `_Nullable` rather than left to the audited
/// region above, which would make them `_Nonnull` and contradict the no-op this
/// documents. An `NSError**` out-parameter is nullable on both levels by Cocoa
/// convention: callers that do not want the error pass none.
void set_error(NSError* _Nullable* _Nullable error,
               const volumetric_kit::gfx::Status& status, const char* stage);

/// @brief Surface a recon `Status` as an `NSError`, no-op when @p error is
/// null.
void set_error(NSError* _Nullable* _Nullable error,
               const volumetric_kit::recon::Status& status, const char* stage);

}  // namespace volumetric_kit::ios_app

NS_ASSUME_NONNULL_END

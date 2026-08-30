// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file ViewOrientation.hpp
/// @brief The turn from ARKit's sensor basis to the interface's viewport.
///
/// Pure math, deliberately: no Vulkan, no recon, no Objective-C, no UIKit. The
/// Objective-C `VolumetricViewOrientation` mirrors @ref ViewOrientation value
/// for value and `VolumetricRenderer.mm` static_asserts the two agree, so this
/// header is reachable from a host test while the enum Swift sees stays in the
/// public bridge header where Swift can see it.
///
/// Several places name the orientation the interface is in: the enum in
/// VolumetricRenderer.h, Swift's map from UIInterfaceOrientation, the
/// read-out's label. This is the only thing that says what that means in
/// radians, deliberately. A zero and a sign restated in a second place is
/// precisely how the two come to disagree, and this mapping has now been wrong
/// twice, in two different directions.

namespace volumetric_kit::ios_app {

/// @brief Which way up the interface is, as quarter turns.
///
/// The raw values are load-bearing: the turn is computed by subtracting two of
/// them, so they must stay consecutive and in this order. @ref viewport_turn
/// static_asserts each one, and `VolumetricRenderer.mm` static_asserts that the
/// Objective-C mirror still agrees — so reordering either is a build failure
/// rather than a silently rotated scan.
///
/// Fusion is unaffected — the pose and the intrinsics are mutually consistent
/// in the sensor frame whatever this says — so it is a render-camera concern
/// only.
enum class ViewOrientation {
  LandscapeLeft = 0,
  Portrait = 1,
  LandscapeRight = 2,
  PortraitUpsideDown = 3,
};

/// The orientation whose viewport already coincides with ARKit's sensor basis,
/// and so needs no turn at all.
///
/// The mapping's single free parameter: every other orientation's turn is
/// measured from here, so this constant *is* the convention. Two independent
/// readings put it at the interface's landscape-right:
///
///   - `ARCamera.transform`'s +X "always points along the long axis of the
///     device, from the front-facing camera toward the Home button". The Home
///     button end is to the right exactly in `UIDeviceOrientationLandscapeLeft`
///     -- which is the *interface's* LandscapeRight, the two being inverses of
///     each other (UIApplication.h states the equivalence outright).
///   - The sensor's own image says the same thing without reference to the
///     prose: a back-camera portrait frame is EXIF orientation 6, whose 0th
///     column is the visual top, so image +u runs *down* the screen in
///     portrait, toward the Home button. The camera's +X is +u -- the CV
///     conversion negates the second and third basis columns and leaves the
///     first untouched -- so +X points the same way.
///
/// Both were already argued before the zero sat here; what moved is which of
/// them the code follows. The reading that shipped instead came from one
/// uncontrolled sighting, that portrait rendered upside down under an earlier
/// build's mapping. A second sighting, on the build that sighting produced,
/// reports portrait upside down *again* -- and the two builds differ by exactly
/// 180 degrees in portrait and by nothing at all in landscape, so they cannot
/// both be describing portrait. They reconcile if the first was taken in
/// landscape, which both readings agree the earlier build had wrong.
///
/// **MEASURED**, 2026-08-10, on an iPad Pro 11-inch (M5): landscape-right and
/// portrait both render upright, with `orient` read off the console at each to
/// confirm the renderer held the value being tested. That is the first time any
/// part of this mapping has been settled by anything but a reading of the
/// documentation, and it took two orientations because one cannot do it:
///
///   - **Landscape-right pins the zero.** It is the orientation this constant
///     names, so the turn applied there is 0 -- and being upright is therefore
///     the statement that the sensor basis and the viewport genuinely coincide
///     here. The build before this one turned 180 degrees at this orientation
///     and was upside down, which is the same observation from the other side.
///   - **Portrait pins the step.** It is the *only* orientation that can: the
///     turn is +-90 x (raw - 2), and at both landscapes that lands on 0 or 180,
///     each its own negation. So the two landscapes look identical whichever
///     way the step runs, and a sign error there is invisible by construction.
///     Portrait is where the two differ, and it came up upright.
///
/// Two points determine the line, so landscape-left (raw 0, half a turn)
/// follows from these rather than resting on the prose above -- it is computed
/// by the same single expression, from a zero and a step that were both
/// measured. Worth a glance, not worth blocking on. Info.plist declares
/// Portrait, LandscapeLeft and LandscapeRight only, so raw 3 is unreachable and
/// cannot be tested at all.
///
/// One caveat left standing: this was measured on an iPad. `ARCamera.transform`
/// documents its basis without reference to the device, and the two readings
/// above are device-independent, so there is no reason to expect an iPhone to
/// differ -- but the sighting that sent this the wrong way in the first place
/// may well have been taken on one, and nobody has checked.
inline constexpr ViewOrientation kSensorBasisOrientation =
    ViewOrientation::LandscapeRight;

/// The turn from the sensor basis to @p orientation's viewport, in radians
/// about the **GL** camera's +Z.
///
/// Which frame this acts in matters as much as the angle, so it is stated here
/// rather than left to the call site: the caller applies this after
/// `cv_from_gl_camera`, where +Z points out of the screen at the viewer and a
/// positive angle turns the basis counterclockwise on screen. recon's own poses
/// are CV (+Z along the view direction), and the identical rule applied to one
/// of those comes out with the opposite sign.
float viewport_turn(ViewOrientation orientation) noexcept;

}  // namespace volumetric_kit::ios_app

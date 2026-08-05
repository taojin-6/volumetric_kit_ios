# volumetric_kit_ios

The iOS application shell of the `volumetric_kit` family: ARKit capture →
[`volumetric_kit_recon`](https://github.com/taojin-6/volumetric_kit_recon)
fusion → [`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx)
rendering, all on MoltenVK, on a phone.

This repo holds only what is genuinely iOS: the cross-compilation toolchain, the
app targets, and the Xcode packaging. The libraries stay independent siblings —
neither needed a single source change to build for iOS.

## Why there is a repo here at all

The family's rule is that `recon` and `gfx` each build and release on their own.
An iOS app needs an Xcode project, a bundle identifier, a signing team, and a
provisioning profile — none of which belong in a library that also ships on
Linux and Windows. So the app lives here and consumes both libraries the way any
other downstream would.

The one thing iOS genuinely needs is a **Vulkan implementation**: the platform
ships no ICD loader and no `libvulkan`, so MoltenVK's static library *is* Vulkan
here, linked directly (and therefore with no validation layers on device).
`cmake/ios.toolchain.cmake` seeds `Vulkan_LIBRARY` / `Vulkan_INCLUDE_DIR` with
MoltenVK's iOS xcframework, which is enough for CMake's `FindVulkan` to build a
`Vulkan::Vulkan` target — so every `find_package(Vulkan)` in recon and gfx
resolves without either repo knowing iOS exists.

## Build

```sh
tools/fetch_moltenvk.sh          # once — pulls MoltenVK's iOS release

cmake -S . -B build-ios -G Xcode \
  -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
  -DVI_DEVELOPMENT_TEAM=<your team id>

cd build-ios && xcodebuild -project volumetric_kit_ios.xcodeproj \
  -target compute_smoke -configuration Debug -allowProvisioningUpdates build
```

Find your team id with `security find-identity -v -p codesigning` (the
parenthesised code) or in Xcode → Settings → Accounts.

To build against a local checkout of a sibling instead of the pinned remote:

```sh
-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=/path/to/volumetric_kit_recon
```

### Install and run

```sh
xcrun devicectl list devices
xcrun devicectl device install app --device <id> \
  build-ios/apps/compute_smoke/Debug-iphoneos/compute_smoke.app
xcrun devicectl device process launch --console --device <id> \
  io.taojin.volumetrickit.computesmoke
```

The device must be **unlocked** — mounting the developer disk image fails on a
locked device.

## Development

```sh
pre-commit install     # once — formatting + hygiene hooks on every commit
```

The hooks mirror the sibling repos — the same pinned `clang-format` (22.1.5),
the same `cmake-format`, and the same rule that Vulkan is reached only through
gfx's `core/vulkan.hpp` umbrella — plus two of this repo's own: `swift-format`
from the Xcode toolchain, and `shellcheck`.

Two configuration notes specific to here:

- `.clang-format` carries an **ObjC section as well as a Cpp one**. A config with
  only `Language: Cpp` does not merely fall back for `.mm` files — clang-format
  refuses them outright (*"Configuration file(s) do(es) not support
  Objective-C"*). The Cpp section is byte-identical to the siblings'.
- `swift-format` runs via `xcrun`, from whichever Xcode builds the app, rather
  than a pinned pre-commit environment — there is no Swift toolchain to install
  on a Linux hook runner. That is why the lint CI job runs on macOS while the
  siblings lint on Linux.

CI (`.github/workflows/`) cross-compiles every app target for iOS arm64 and runs
the same hooks. The build is **unsigned** (`CODE_SIGNING_ALLOWED=NO`), so no
certificate or provisioning secret is needed, and **build-only**: MoltenVK ships
no simulator slice and ARKit scene depth needs LiDAR, so nothing here is
runnable in CI. It still earns its place — it catches a sibling change that
stops Xcode-generating, a toolchain regression, or a Swift/Objective-C++ seam
that no longer compiles, all of which happened while standing this repo up. A
final step asserts each bundle is really iOS arm64 (`LC_BUILD_VERSION`
`platform 2`), since a host-vs-target mixup would otherwise pass silently.

It runs on push and pull request only — there is **no scheduled run**. recon is
consumed at `GIT_TAG main` rather than a fixed tag, so the sibling breakage above
arrives on upstream's schedule, not on ours; a nightly turned that into a red
`main` on mornings with no work queued against it, so drift is now surfaced by
the next push here instead, when there is a change in hand to fix it against. The
dependency cache is still keyed on recon's resolved tip commit and carries no
`restore-keys`: a key that ignored the moving ref would go on hitting a pre-drift
source tree and report green against stale sources.

## Language split

Swift owns the app; Objective-C++ owns the seam; C++ is the engine.

```
Swift          app shell, UI, lifecycle (later: ARSession config, permissions)
    ↓ bridging header
Obj-C++ (.mm)  VolumetricRenderer — CAMetalLayer → VkSurfaceKHR, ARFrame → PODs
    ↓
C++            volumetric_kit_recon + volumetric_kit_gfx
```

The bridge is Objective-C++ rather than Swift because an `.mm` is the one
translation unit where a `CAMetalLayer*` and a `vg::app::WindowedApp` are both
first-class — no marshalling layer needed. Swift's C++ interop struggles exactly
where these libraries live: recon and gfx are move-only types returning
`Result<T>`, and Vulkan's `pNext` struct chains are unpleasant from Swift. So the
seam stays a narrow Objective-C class that hands Swift plain values and
`NSError`s.

Each app is therefore **two CMake targets** — a static library for the
Obj-C++/C++ bridge, and the Swift app linking it through a bridging header.
Mixing Swift and C++ in a *single* target under the Xcode generator is the
fragile configuration.

## Apps

### `scanner`

The live reconstruction app. Currently: gfx brought up on a `CAMetalLayer` and
drawing a procedural triangle, driven by a `CADisplayLink`, with an on-screen
read-out of GPU, API version, drawable size, presented frames, and fps.

This step exists to retire two risks together — the `CAMetalLayer` →
`VK_EXT_metal_surface` → swapchain → present path, and the mixed-language build.
The triangle is deliberately *procedural* (positions from `gl_VertexIndex`, no
vertex buffer), so it proves the whole **graphics** pipeline works under MoltenVK
on iOS — shader modules, spirv-cross reflection, dynamic rendering, rasterised
interpolation — without the distraction of geometry. `compute_smoke` proved the
compute path; this proves the render path.

Verified on an iPad Pro M5: Vulkan 1.3 instance, 3 swapchain images at the native
2420×1668, triangle on screen.

#### The duplicated bootstrap

`Bridge/SharedDevice.{hpp,mm}` builds one `VkDevice` from
`vr::Device::requirements()` ∪ `vg::Device::requirements()` and hands the same
handles to `recon::Device::adopt` and `gfx::app::WindowedApp::adopt`. recon's
`examples/viewer/shared_device.hpp` does the same job for GLFW on desktop, so
this is a second copy of one algorithm.

It is copied rather than shared because there is nowhere yet for it to live:
neither library can own it (it is the one piece that is *neither* library's, and
both libraries' RAII owners start after it), and a third package for ~400 lines
with two consumers buys less than it costs. What genuinely differs is small —
surface creation (`VK_EXT_metal_surface` against a `CAMetalLayer` rather than
GLFW) and returning a `Status` instead of printing to stderr.

The rest is deliberately kept in step, and the parts that are load-bearing are
the ones a copy loses quietly: the **queue-plan order** (one family with two
queues, then two families, then a shared queue — MoltenVK reports several
graphics + compute + present families of one queue each, so a phone lands on the
second and taking the third hands back the concurrency the fuse thread exists
for), the **pre-create support checks** (so a shortfall names the missing
extension or feature instead of collapsing into `vkCreateDevice failed`), the
**`feature_chain` splice** (gfx documents enabling its opaque chain as the
embedder's job; it is null today), and the **field-wise feature merge**.

Promote it to a shared package when a third consumer appears, or the first time
the two copies disagree about any of the above — the family's rule is that
duplication is answered at the second consumer, and the second consumer here is
what makes the drift possible rather than what makes it wrong.

### `compute_smoke`

The de-risk gate. The family's standing rule is *validate MoltenVK compute on
the target Apple GPU early — prove the path before building on it*; this is that
rule applied to iOS. It runs recon's real compute path against MoltenVK on real
hardware in four independently-reporting stages:

| Stage | What it proves |
| --- | --- |
| 0. Device capabilities | The Vulkan version MoltenVK exposes, and whether `scalarBlockLayout`, `timelineSemaphore` and `dynamicRendering` are present |
| 1. Compute dispatch | Instance → Device → Allocator → Buffer → Descriptor → ComputePipeline → dispatch → readback |
| 2. Scalar block layout | A `VoxelHashMap` round-trip — recon's host PODs and their GLSL mirrors agree only under `GL_EXT_scalar_block_layout` |
| 3. Vertical slice | `allocate_from_depth` → TSDF integrate → marching cubes on a synthetic posed depth frame |

Stage 2 is the real risk. recon's buffer ABI is scalar block layout, not
`std430`, because its PODs embed `Vec3i` block coordinates that `std430` would
16-byte-align; if MoltenVK's iOS SPIR-V → MSL translation got that wrong, the
coordinates would come back garbled rather than failing loudly.

Stage 3 is shaped like the real thing on purpose: a 160×120 depth frame, close
to ARKit's 256×192 `sceneDepth`, rather than a desktop 640×480.

## Platform notes

- **Device only, arm64.** MoltenVK's iOS release ships no simulator slice, and
  ARKit scene depth needs LiDAR hardware anyway.
- **LiDAR required** for the ARKit slice: iPhone 12 Pro and later Pro models,
  iPad Pro 2020 and later. Guard with
  `ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)`.
- **Shaders are compiled on the host** by `glslc` and embedded into the binary
  as constexpr byte arrays, so there is no runtime shader file and nothing to
  resolve in the app bundle.
- **glm is vendored header-only.** Homebrew's glm 1.0.3 defines `glm::glm` as a
  *macOS* dylib; recon links that canonical name deliberately (older packagings
  lack `glm::glm-header-only`), so this repo supplies a header-only glm via
  `OVERRIDE_FIND_PACKAGE` rather than pushing the problem upstream.

## Roadmap

1. ✅ **`compute_smoke`** — recon's compute path on device.
2. ✅ **`scanner` bring-up** — gfx rendering to a `CAMetalLayer` surface via
   `VK_EXT_metal_surface`, proving the render path and the Swift/Obj-C++ build.
3. **ARKit capture** — an `ICameraCapture` source in recon's `sensor` tier
   feeding `sceneDepth` (256×192 float metres), `capturedImage` (YCbCr 420, needs
   conversion), and `camera.transform`. ARKit is +Y up / −Z forward while recon
   projects +Z forward, so poses convert as
   `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`. Depth and colour have
   different resolutions, which recon already models as separate
   `DepthCameraParams` and `ColorCameraParams`. ARKit's depth is *registered* to
   the colour camera, so the two share a pose and differ only in intrinsics
   scale — which avoids the unregistered-camera caveats recon's integrator
   documents.
4. **Live fusion** — fuse and render live in `scanner`: ARKit frames feed recon
   on a background thread while the render thread draws the growing mesh, the
   `fuse_viewer` model. Then one shared `VkDevice` built from
   `vr::Device::requirements()` ∪ `vg::Device::requirements()` and handed to both
   via `Device::adopt`. Starts on interop seam A (host mesh, as `fuse_viewer`
   does) and moves to seam B (indirect draw over a mesh ring with a timeline
   handoff) once it works.

## License

MIT — see [LICENSE](LICENSE).

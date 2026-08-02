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

## Apps

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
2. **`triangle`** — gfx rendering to a `CAMetalLayer` surface via
   `VK_EXT_metal_surface`, proving the render path.
3. **ARKit capture** — an `ICameraCapture` source in recon's `sensor` tier
   feeding `sceneDepth` (256×192 float metres), `capturedImage` (YCbCr 420, needs
   conversion), and `camera.transform`. ARKit is +Y up / −Z forward while recon
   projects +Z forward, so poses convert as
   `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`. Depth and colour have
   different resolutions, which recon already models as separate
   `DepthCameraParams` and `ColorCameraParams`. Records sequences to disk so
   captures replay on desktop through recon's `fuse_replica`.
4. **`scanner`** — fuse and render live, one shared `VkDevice` built from
   `vr::Device::requirements()` ∪ `vg::Device::requirements()` and handed to both
   via `Device::adopt`. Starts on interop seam A (host mesh, as `fuse_viewer`
   does) and moves to seam B (indirect draw over a mesh ring with a timeline
   handoff) once it works.

## License

MIT — see [LICENSE](LICENSE).

# volumetric_kit_ios

The iOS application shell of the `volumetric_kit` family: ARKit capture →
[`volumetric_kit_recon`](https://github.com/taojin-6/volumetric_kit_recon)
fusion on
[`volumetric_kit_grid`](https://github.com/taojin-6/volumetric_kit_grid)'s
voxel grid and meshing →
[`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx)
rendering, all on MoltenVK, on a phone.

This repo holds only what is genuinely iOS: the cross-compilation toolchain, the
app targets, and the Xcode packaging. The libraries stay independent siblings —
neither needed a single source change to build for iOS.

[AGENTS.md](AGENTS.md) is the shared working guide for Codex and Claude Code;
`CLAUDE.md` imports it. Use its task map for the relevant build, architecture,
and platform sections below.

## Why there is a repo here at all

The family's rule is that each library builds and releases on its own.
An iOS app needs an Xcode project, a bundle identifier, a signing team, and a
provisioning profile — none of which belong in a library that also ships on
Linux and Windows. So the app lives here and consumes both libraries the way any
other downstream would.

The one thing iOS genuinely needs is a **Vulkan implementation**: the platform
ships no ICD loader and no `libvulkan`, so MoltenVK's static library *is* Vulkan
here, linked directly (and therefore with no validation layers on device).
`cmake/ios.toolchain.cmake` names the platform and points `MoltenVK_ROOT` at
the fetched release; volumetric_kit_core's iOS module (`cmake/vkc_ios.cmake`)
then sets `Vulkan_LIBRARY` / `Vulkan_INCLUDE_DIR` from it, so every
`find_package(Vulkan)` in the siblings makes `Vulkan::Vulkan` from MoltenVK's
iOS xcframework.

## Build

```sh
brew install shaderc glm         # once — glslc and glm's headers, host tools
tools/fetch_moltenvk.sh          # once — pulls MoltenVK's iOS release

cmake -S . -B build-ios -G Xcode \
  -DCMAKE_TOOLCHAIN_FILE=cmake/ios.toolchain.cmake \
  -DVI_DEVELOPMENT_TEAM=<your team id>

cd build-ios && xcodebuild -project volumetric_kit_ios.xcodeproj \
  -target compute_smoke -configuration Debug -allowProvisioningUpdates build
```

Find your team id with `security find-identity -v -p codesigning` (the
parenthesised code) or in Xcode → Settings → Accounts.

`CMakeLists.txt` fetches the siblings by commit, in this order:

| Sibling | What the apps take from it |
| --- | --- |
| `volumetric_kit_core` | the shared device, `Status`, the scenario parser |
| `volumetric_kit_sensor` | the camera parameters, colour encoding and conventions ARKitCapture fills (`sensor_interface`), and the `PixelBufferImporter` the keyframes use (`sensor_apple`) |
| `volumetric_kit_grid` | the voxel block grid and Marching Cubes (`grid_volume`, `grid_mesh`) |
| `volumetric_kit_recon` | TSDF fusion and the projective texturer (`recon_tsdf`, `recon_texture`) |
| `volumetric_kit_gfx` | the renderer |

FetchContent keeps a name's first declaration, so these pins are the ones every
sibling builds against, recon's own core, sensor and grid pins included. Each
floor is its pin's version (`vkc_require_core`, `vkc_require_sibling`), so a
local checkout older than the pin is refused. To build against one instead of
the pinned remote:

```sh
-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON=/path/to/volumetric_kit_recon
```

A checkout that needs a newer core, sensor or grid fails to configure or
compile against these; point that sibling at a matching checkout too, with its
own `-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_<NAME>`.

### Scanner configuration

The app bundles [`apps/scanner/config/scanner.json`](apps/scanner/config/scanner.json).
Edit this file to select voxel size, truncation, allocation limits, fusion and
remeshing cadence, texturing, and stage timing, then rebuild the app. The bridge
also accepts a configuration file path, so a caller can select another scenario.
Core owns the versioned JSON format, defaults, and all static validation. A
configuration error reaches the dashboard before GPU bring-up. Hardware support,
resource creation, and runtime allocation checks remain in the scanner and engines.

Scenario files share the format used by recon, gfx, and calib. Core rejects fields
that the selected application cannot use; scanner configurations require the live
ARKit source. Drawable size and queue/ring resources still come from the running
app.

### The host tests

`vi_core` — the scanner's pure logic — and the bundled configuration test
build and run on the host:

```sh
cmake -S . -B build-host -DVI_HOST_TESTS=ON
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

No toolchain file, no MoltenVK, no sibling library, no Xcode project — that mode
short-circuits before any of them. It fetches only glm, the CPU-only core
configuration tier, its private JSON dependency, and the test framework.

It exists because **everything else here is build-only by nature**: MoltenVK
ships no simulator slice and ARKit scene depth needs LiDAR, so nothing that
touches Vulkan, the libraries or ARKit can execute anywhere but a real device. That left
the parts of this app that genuinely *decide* things — a sign, a threshold,
whether a kernel reading can be trusted — checkable only by installing a build
and looking at it.

`apps/scanner/Core/` is the subset that carries no such dependency: plain C++
over glm, no Vulkan, no sibling library, no Objective-C, no ARKit, no mach. The
rule for what belongs there is in its `CMakeLists.txt`. Where a decision needs
the device, the split is done at the seam rather than abandoned —
`Bridge/MemoryQuery.cpp` makes the `task_info` call, `Core/MemoryBudget.cpp`
decides what the answer means, and only the second half is testable. That is
what puts the case that matters most within reach of a test: a kernel-clamped
headroom of 0, which must be reported as an alarm rather than turned into a
ceiling, and which cannot otherwise be reached without first persuading a real
process to run out of memory.

Capture and worker entry points contain C++ exceptions. A failed frame
conversion is rejected before its scratch buffers are published; a failed
thread construction restores the stopped state and releases its capture so a
later start can retry. Error counters still advance when memory pressure
prevents storing the diagnostic text. These failure policies are covered by
host allocation-injection and worker-start tests; ARKit and GPU execution
still require a device.

Two of the facts pinned here were settled on hardware and are expensive to
re-establish — the sensor-basis-to-viewport turn, which had been wrong twice in
two directions before an iPad Pro M5 settled it, and the takeover heading in
`OrbitCamera`, which snapped the scene half a turn for any phone aimed above the
horizon. Both are now regressions rather than sightings.

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

The hooks mirror the sibling repos — the same pinned `clang-format` (22.1.8),
the same `cmake-format`, and the same rule that Vulkan is reached only through
the core's `volumetric_kit/core/vulkan/vulkan.hpp` umbrella — plus two of this
repo's own: `swift-format`
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

CI (`.github/workflows/`) cross-compiles every app target for iOS arm64, runs
the same hooks, and runs the `vi_core` host tests. The build is **unsigned**
(`CODE_SIGNING_ALLOWED=NO`), so no certificate or provisioning secret is needed,
and **build-only**: MoltenVK ships no simulator slice and ARKit scene depth needs
LiDAR, so no *app* target is runnable in CI. The test leg is the one that
executes — see [the host tests](#the-host-tests) for why that is a separate
target rather than a wish. It still earns its place — it catches a sibling change that
stops Xcode-generating, a toolchain regression, or a Swift/Objective-C++ seam
that no longer compiles, all of which happened while standing this repo up. A
final step asserts each bundle is really iOS arm64 (`LC_BUILD_VERSION`
`platform 2`), since a host-vs-target mixup would otherwise pass silently.

It runs on push and pull request only — there is **no scheduled run**. The
siblings are pinned by commit in `CMakeLists.txt`, so sibling changes arrive
only when a PR here bumps a pin, and CI builds exactly that set. The dependency cache is
keyed on the hash of `CMakeLists.txt`, which holds every pin, and carries no
`restore-keys`, so a bumped pin always re-fetches.

Until 2026-10-03 the siblings were consumed at `GIT_TAG main`. Upstream breakage
then arrived on upstream's schedule, with no commit here to fix it against, and
a nightly reported it as a red `main`. Pinning ends that drift at the cost of
adopting upstream deliberately; it also lets the family's shared
`volumetric_kit_core` change the libraries' `Status` without breaking this
build mid-migration.

## Language split

Swift owns the app; Objective-C++ owns the seam; C++ is the engine.

```
Swift          app shell, UI, lifecycle (later: ARSession config, permissions)
    ↓ bridging header
Obj-C++ (.mm)  VolumetricRenderer — CAMetalLayer → VkSurfaceKHR, ARFrame → PODs
    ↓
C++            vi_core (pure, host tested) + volumetric_kit_sensor, _grid,
               _recon and _gfx
```

`vi_core` (`apps/scanner/Core/`) is a fourth layer only in the sense that it sits
under the bridge and depends on nothing above it. It is the app's own logic with
the platform taken out — see [the host tests](#the-host-tests).

The bridge is Objective-C++ rather than Swift because an `.mm` is the one
translation unit where a `CAMetalLayer*` and a `vg::app::WindowedApp` are both
first-class — no marshalling layer needed. Swift's C++ interop struggles exactly
where these libraries live: their types are move-only and return
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

#### The shared device

One `VkDevice` serves every library, built by volumetric_kit_core's
`SharedDevice` (`volumetric_kit/core/vulkan/shared_device.hpp`) from the
compute side's requirements -- grid's, recon's and sensor's, merged with
`vkc::merge`; sensor's add the optional extensions its picture importer takes
-- ∪ `vg::device_requirements()` (with `needs_present`), and adopted by each
side: sensor, grid and recon share one `vkc::Device::adopt` of the shared
device's `compute_payload()`, gfx goes through `WindowedApp::adopt` with its
`graphics_payload()`. A `VkBuffer` is valid only on the device that made
it, so the zero-copy mesh handoff needs the one device.

The app used to keep its own copy of this bootstrap, beside recon's desktop
viewer's; both now use the core's, which owns the parts a copy loses quietly --
the queue-plan order (one family with two queues, then two families, then a
shared queue, so a phone lands on two families and the fuse thread keeps its
own queue), the pre-create support checks, and the mutex that serializes a
shared queue. What stays here is what is the platform's:
`VolumetricRenderer.mm` asks for `VK_KHR_surface` and `VK_EXT_metal_surface`
and makes the surface from the view's `CAMetalLayer` in the `make_surface`
callback. The core requests portability enumeration only where a loader offers
it, which the directly linked MoltenVK does not.

Two behaviours come with the core's device that the app's own bootstrap did not
have:

- **Allocation stops at Metal's working set.** The core enables
  `VK_EXT_memory_budget` wherever it is offered, and its `Allocator` refuses new
  device memory past a heap's budget. MoltenVK 1.4.2 reports the unified heap's
  budget on iOS 16+ as `MTLDevice.recommendedMaxWorkingSetSize` and its usage
  as `currentAllocatedSize`, so a volume resize or mesh-arena allocation that
  would take Metal past the working set now fails as an over-budget
  `vkc::Status` on its stage, where before only jetsam stopped it. That is the
  ceiling the read-out's GPU working-set row already shows. This comes from the
  linked `libMoltenVK.a`; a long scan on a LiDAR device has yet to confirm
  where it binds.
- **Debug labels in Debug builds only.** The core's instance enables
  `VK_EXT_debug_utils` by default, so the libraries' kernel labels and object names
  reach an Xcode GPU capture as Metal debug groups and labels. A Release build
  clears `request_debug_utils`, as the old bootstrap always did.

### `compute_smoke`

The de-risk gate. The family's standing rule is *validate MoltenVK compute on
the target Apple GPU early — prove the path before building on it*; this is that
rule applied to iOS. It runs grid's and recon's real compute path against
MoltenVK on real hardware in four independently-reporting stages, on a device
with their requirements (`scalarBlockLayout`):

| Stage | What it proves |
| --- | --- |
| 0. Device capabilities | The Vulkan version MoltenVK exposes, and whether `scalarBlockLayout`, `timelineSemaphore` and `dynamicRendering` are present |
| 1. Compute dispatch | Instance → Device → Allocator → Buffer → Descriptor → ComputePipeline → dispatch → readback |
| 2. Scalar block layout | A `VoxelHashMap` round-trip — grid's host PODs and their GLSL mirrors agree only under `GL_EXT_scalar_block_layout` |
| 3. Vertical slice | `allocate_from_depth` → TSDF integrate → marching cubes on a synthetic posed depth frame |

Stage 2 is the real risk. grid's buffer ABI is scalar block layout, not
`std430`, because its PODs embed `Vec3i` block coordinates that `std430` would
16-byte-align; if MoltenVK's iOS SPIR-V → MSL translation got that wrong, the
coordinates would come back garbled rather than failing loudly.

Stage 3 is shaped like the real thing on purpose: a 160×120 depth frame, close
to ARKit's 256×192 `sceneDepth`, rather than a desktop 640×480.

## Platform notes

- **Device only, arm64.** MoltenVK's iOS release ships no simulator slice, and
  ARKit scene depth needs LiDAR hardware anyway.
- **iOS 26.0 or newer** (`cmake/ios.toolchain.cmake`).
- **LiDAR required** for the ARKit slice: iPhone 12 Pro and later Pro models,
  iPad Pro 2020 and later. Guard with
  `ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)`.
- **The scanner's keyframes are ARKit's own pictures.** Each mesh slot samples
  the `capturedImage` its mesh was textured from where ARKit left it, imported
  by sensor's `PixelBufferImporter`, so the ring holds up to three of ARKit's
  pixel buffers beyond the frames being captured and fused. Fusion still
  converts each frame's colour on the CPU.
- **Shaders are compiled on the host** by `glslc` and embedded into the binary
  as constexpr byte arrays, so there is no runtime shader file and nothing to
  resolve in the app bundle.
- **glm is header-only.** Homebrew's glm 1.0.3 defines `glm::glm` as a *macOS*
  dylib, which the iOS linker refuses; the core's iOS module answers every
  `find_package(glm)` with the host's glm headers as a header-only target
  instead. The host tests fetch glm 1.0.3 header-only.

## Roadmap

1. ✅ **`compute_smoke`** — the compute path on device.
2. ✅ **`scanner` bring-up** — gfx rendering to a `CAMetalLayer` surface via
   `VK_EXT_metal_surface`, proving the render path and the Swift/Obj-C++ build.
3. **ARKit capture** — an `ICameraCapture` source in recon's `sensor` tier
   feeding `sceneDepth` (256×192 float metres), `capturedImage` (YCbCr 420, needs
   conversion), and `camera.transform`. ARKit is +Y up / −Z forward while
   sensor's cameras project +Z forward, so poses convert as
   `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`. Depth and colour have
   different resolutions, which sensor already models as separate
   `DepthCameraParams` and `ColorCameraParams`. ARKit's depth is *registered* to
   the colour camera, so the two share a pose and differ only in intrinsics
   scale — which avoids the unregistered-camera caveats recon's integrator
   documents.
4. **Live fusion** — fuse and render live in `scanner`: ARKit frames feed recon
   on a background thread while the render thread draws the growing mesh, the
   `fuse_viewer` model. Then one shared `VkDevice`, the core's `SharedDevice`,
   built from the compute libraries' requirements ∪ `vg::device_requirements()`
   and handed to both sides via `adopt`. Starts on interop seam A (host mesh, as `fuse_viewer`
   does) and moves to seam B (indirect draw over a mesh ring with a timeline
   handoff) once it works.

## License

MIT — see [LICENSE](LICENSE).

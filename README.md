# volumetric_kit_ios

The iOS application shell of the `volumetric_kit` family: ARKit capture → an
RGBD stream to the server that holds the scene, with the camera on screen
through
[`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx); and, on
request,
[`volumetric_kit_recon`](https://github.com/taojin-6/volumetric_kit_recon)
fusion on
[`volumetric_kit_grid`](https://github.com/taojin-6/volumetric_kit_grid)'s
voxel grid and meshing → gfx rendering, all on MoltenVK, on a phone.

The scanner runs as the client by default, streaming and showing the camera with
no grid or mesh on the device; `-model`, or the panel's switch, adds the
on-device TSDF, mesh and textured model, with or without a stream
(`apps/scanner/Core/ScannerMode.hpp`).

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

`-DVI_EXTENDED_VIRTUAL_ADDRESSING=ON` also signs the scanner with the
extended-virtual-addressing entitlement; it needs a paid Apple Developer Program
team.

`CMakeLists.txt` fetches the siblings by commit, in this order:

| Sibling | What the apps take from it |
| --- | --- |
| `volumetric_kit_core` | the shared device, `Status`, the scenario parser, and the address checks and dial the live link uses (`core_net`) |
| `volumetric_kit_sensor` | the camera, `ArkitSensor`, which owns the ARSession and hands out `RgbdFrame`s (`sensor_arkit`, `sensor_interface`); the GPU pass that prepares each frame for fusion (`sensor_prep`); a picture's own buffer and tags (`sensor_apple`); the HEVC encoder the stream codes each picture on, and its rules that `-self-test hevc` runs (`sensor_video`, `sensor_video_testing`); and the frames' wire form the session writer writes (`sensor_wire`) |
| `volumetric_kit_grid` | the voxel block grid and Marching Cubes (`grid_volume`, `grid_mesh`), the `.vrst` container (`grid_stream`), and the server that serves it live (`grid_networking`) |
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

`vi_core` (the scanner's pure logic), `vi_session` (the session writer),
`vi_stream` (the stream sender) and the bundled configuration test build and
run on the host:

```sh
brew install glm molten-vk vulkan-headers vulkan-loader shaderc  # once
cmake -S . -B build-host -DVI_HOST_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

No toolchain file, no MoltenVK release, no recon or gfx, no Xcode project —
that mode short-circuits before any of them. It fetches the core, sensor, grid
and the test framework, and builds only the tiers the tests link: the core's,
sensor's interface and wire form (`sensor_wire`), grid's `.vrst` container
and its server (`grid_stream`, `grid_networking`), and for `vi_stream`'s
tests sensor's ARKit driver, encoder, decoder and frame prep (`sensor_arkit`,
`sensor_video`, `sensor_prep`) and the core's GoogleTest fixtures. It takes the
Vulkan SDK and glm from Homebrew. Only `vi_stream_tests` opens a device, the
Mac's GPU and VideoToolbox, and skips without them unless
`VKC_REQUIRE_VULKAN_DEVICE` or `VS_TEST_HEVC_ENCODER` says they are there. CI
runs it on the family's self-hosted Mac (label `mac`).

`apps/scanner/Session/` is `vi_session`: `RgbdStreamWriter` writes any
`IRgbdSensor`'s frames as one `.vrst` session, each frame its colour record
(the caller's coded picture) then its depth record (sensor's wire form), with
no decode and no device, to a file, to a live reader, or both. Live, it dials
and listens on nothing: grid's `StreamServer` serves with no listener, and a
thread of the writer's dials the reader (`core::net::dial`) and hands it the
socket, dialling again at most once a second while no reader is served. A
joining reader asks for a key (`key_wanted`); `dial_to` moves the link, or
ends it, within the one session. `parse_dial_entry` reads the on-screen entry
with the core's checks. Its tests write a fake sensor's frames, read the file
back with grid's reader, and read the link on 127.0.0.1 through grid's
`StreamClient` on a core `Listener`, as the lab does. The pose and rate rules
it is fed through are `vi_core`'s: `MapSteps` takes ARKit's map steps out of
every pose and logs them, then `RateGate` cuts 60 Hz to the stream's 10 Hz.

`vi_stream` (`Session/rgbd_stream_sender.hpp`) is what feeds it on the iPad:
`RgbdStreamSender::offer` takes each frame the fuse loop polls, at once and
whatever fusion is doing (`Core/FrameFanout.hpp`: fusion takes the newest
frame on a worker of its own when it is free, and the frames it cannot keep
up with are let go at once), through `MapSteps` and `RateGate`, and a thread
of the sender's own copies each taken frame's picture on the compute device
into one of the hardware encoder's own (`HevcEncoder::target`), codes it (one
picture in flight), then appends the frame. The copy is because
VideoToolbox's low-latency session keeps the last picture it coded until it
codes the next: with ARKit's own, one of the camera's few pictures would be
out for the 100 ms between codings, beside the scanner's keyframe ring, which
on the pattern's 4 stops the camera for good.
A frame the fuse loop let go unseen is a gap to `MapSteps`, which finds no
step across 100 ms or more. A picture the encoder drops leaves its frame out
whole, and the next is a key; so is the session's first frame and the next
after a reader joins. The encoder opens at the first picture from its own
tags. With neither a file nor a reader the sender codes and counts only: the
dry run. Its tests run sensor's `ArkitSensor` pattern through it on the Mac,
decode every picture on VideoToolbox and check, on the GPU, that each frame
carries its own picture. With `VI_TEST_SESSION_OUT=<path>`
or `VI_TEST_LAB_HOST=<a.b.c.d:port>` (or `VI_TEST_DRY=1`),
`PatternSessionForTheLab` sends the iPad's own stream, 1920×1440 at 10 Hz, for
`VI_TEST_FRAMES` frames, for recon's `tools/session_check`.

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

### `-self-test hevc`

Launched with `-self-test hevc`, the scanner runs sensor's HEVC encoder check
instead of scanning, and never opens the camera:

```sh
xcrun devicectl device process launch --console --device <id> \
  io.taojin.volumetrickit.scanner -- -self-test hevc
```

It codes sensor's test pattern in IOSurface pixel buffers at ARKit's
1920×1440 and writes no file and sends nothing. Its report, on the console
and the screen, gives:

- the device and its thermal state, the GPU and the encoder's own settings
  line, which says whether it takes a QP a picture;
- one line per run of sensor's encoder rules (`hevc_encoder_breach`, C1–C8 and
  C10–C13), each decoded back on the GPU: BT.601 at either range, BT.709
  centre-sited, QP 1 and 51, then every colour code and siting the encoder
  accepts. `kept` or `BROKEN` (a rule's name); `FAILED` for a status from a
  session that opens, `NOT RUN` for one that does not. A run that failed or
  broke C6 runs again without the decode and ends in `; without C6: kept` or
  what broke. With no GPU, no case decodes, each name ends in `, no decode`,
  and the run fails;
- one session's timing, one picture in flight: `send` return time, send to
  access unit (an upper bound on the encode), and access-unit bytes, at p50,
  p95 and max;
- the thermal state at the end, and last a PASS or FAIL line.

The report stays on the screen for 20 s after it ends, then the app exits: 0
passed, 1 failed, 2 for an unknown test name, so `--console` returns about 20 s
after the last line. `Core/SelfTest.hpp` decides what the report says and
whether it passed, and is host-tested; `Bridge/HevcSelfTest.cpp` runs it.

### `-source pattern`, `-minutes N`, `-record <name>`, `-connect <address>`, `-dry`, `-fail-opens N`

Launched with `-source pattern`, the scanner scans sensor's test pattern
(`ArkitSource::Pattern`) instead of the camera, which never opens: each
frame's index coded in its depth and its picture, at the camera's sizes and
rate, through the same queue, frame prep and fusion. `-source camera` is the
default. `-minutes N` (1 to 1440) stops the scan after N minutes, prints the
stop's lines, finishes the stream, and exits; the minutes count from launch.
`-record <name>` writes the stream's session to `Documents/<name>` (a plain
name ending `.vrst`; an existing file is never overwritten), and
`-connect <a.b.c.d:port>` connects the live link once the scanner is up, as
the on-screen Connect does. Without either, and until a Connect, the stream
codes and counts only. `-dry` keeps it so for the whole launch: no file, and
every connect refused, the on-screen one too, so nothing the camera sees
leaves the iPad; it takes no `-record` or `-connect`. `-fail-opens N` (1 to
9, with `-source pattern` alone) tests the camera's retry: the first N opens
wait 1 s and come back `NotFound`, as a camera that gave no frame in time,
and the next opens the pattern. An argument it cannot read is refused, and
then no camera opens at all.

```sh
xcrun devicectl device process launch --console --device <id> \
  io.taojin.volumetrickit.scanner -- -source pattern -minutes 2 \
  -record pattern.vrst -connect 192.168.1.20:47801
```

Every 10 s the fuse thread prints one `sa` line of counts, times, states and
colour codes, never a pixel, a depth value, a pose or a path: the camera's
tracking and counters (`SensorStats`, `ArkitStatus`), pictures imported and
surfaces made since the camera opened (a pool keeps surfaces near its size),
frames fused and frames fusion skipped (a newer one came while it fused), the
device's thermal state, seconds to the first frame, poll and fuse (prep +
allocate + integrate) milliseconds at p50 and p95 over the window, the
picture's H.273 colour codes, sensor's frame contract
(`testing::RgbdContract`), and for the pattern how many frames carried their
own index. Each stop prints an `sa stop` line, a
lock or backgrounding as well as the run's end; the next start counts its
seconds and counters from 0 again, so its first frame says how soon the camera
came back. Beside it an `st` line says what the stream did: `dry` or
`session`; frames taken, busy (due but left, the one before still waiting to
be coded), coded (keys), failed, written and refused since launch; this
window's video and depth Mbit/s, send and send-to-access-unit ms at p50 and
p95; the live readers and failed dials. The Capture card shows the skipped
and busy counts and the thermal state too. `Core/CaptureReport.hpp` reads the
arguments and writes both lines, and is host-tested.

The pattern has 4 pictures, and a tick that finds all 4 held is skipped and
counted `missed`. The scanner and the camera's queue hold up to 9 (3 in the
keyframe ring, the published keyframe, the frame being fused, the newest
waiting for fusion, the frame waiting to be polled, and the stream's frame
being copied and coded and the one waiting behind it), so a pattern run skips
ticks by design: `received + missed` is the pattern's 60 Hz, and `received`
alone is not. The ring and the frame being fused take all 4 there, so the
pattern runs at fusion's pace; every holder but the ring lets its picture go
without waiting for a newer frame, so none can stop the camera. The stream's
encoder holds none: it codes and keeps pictures of its own.

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
the same hooks, and runs the `vi_core`, `vi_session` and `vi_stream` host
tests on the self-hosted Mac. The build is **unsigned**
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
C++            vi_core (pure, host tested), vi_session and vi_stream
               (host tested)
               + volumetric_kit_sensor, _grid, _recon and _gfx
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

#### The camera's open

Bring-up opens the camera last (`ArkitSensor::open`), with its stream on it.
An open that gets no frame with depth in time (`NotFound`) is opened again,
one open at a time: at the app's wakes and, while the scan runs, at the
read-out's tick once a backoff has passed. Fusion and the stream start when it
opens; any other failure is final. `Core/CameraOpen.hpp` decides which open
begins and when, and is host-tested. The read-out's `camera:` line says what
the open is doing (opening, why the last found no camera with the retry count
and when the next opens, or the refusal), and each open's begin and end prints
a `scanner: camera:` line on the console.

#### The live link

The dashboard's first card, on iPad and iPhone, names the reader the scanner
dials: an IPv4 address (or `address:port`) and a port, empty for 47801, both
remembered between launches. Connect dials it, Disconnect lets it go, and the
line beneath says what the link does: dialling, connected, retrying with the
failed count, or, on "No route to host", allow Local Network access if iOS
asked, or check the address. iOS answers a connect to the local network so
until the person allows it. The link dials again at most once a second, also
after a reader that refuses the session; Connect dials at once. A refused
entry shows the core's reason under the fields. The session is the stream
sender's (`vi_stream`), opened with the camera with `-record` or else at the
first connect, and lasts while the app runs, so a reconnect, to the same reader or
another, is the same session. It describes the camera as `ArkitSensor` opened
it (`SensorInfo`), and its frames are the camera's at 10 Hz, each its HEVC
picture then its depth; the line says how many frames it has sent, which a
reader that stalls has not all read. A `-dry` launch refuses the connect.

The reader can say one line up the same socket, with a tone (grid's
`ViewerNote`; `RgbdStreamWriter::reader_note`), and the top bar's first chip
shows it as it stands, in both modes: how its tag search goes, today. The
scanner parses no word of it, so new words need no new app. The chip is hidden
until the first note and once the reader leaves, when the link's own chip takes
over. A reader that says nothing shows nothing. A note 3 s old or more reads
`Server quiet for N s` (`Core/ReaderLine.hpp`), the scanner's one line of its
own.

While ARKit does not track normally, as for some seconds after a lock, the
camera withholds its frames and the stream pauses. The client's camera view goes
on with their pictures (`ArkitSensor::poll_withheld_picture`), which reach
nothing else, and the top bar's chips say why (`Core/TrackingLine.hpp`). The
model never asks for them.

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
- **LiDAR required** for the camera: iPhone 12 Pro and later Pro models, iPad
  Pro 2020 and later. `ArkitSensor::open` refuses a device without scene depth
  (`Unsupported`), and the read-out says so.
- **The camera is sensor's `ArkitSensor`.** It owns the ARSession, withholds
  frames ARKit did not track normally, resumes after an interruption in the
  same world, and hands out `RgbdFrame`s: depth as millimetres with ARKit's
  confidence beside it, and the picture imported where ARKit left it. Fusion
  sends each frame through sensor's `GpuFramePrep` (the confidence gate at
  medium, the picture's Y'CbCr → R'G'B', both on the GPU) and recon's and
  grid's device overloads read the prepared buffers in place; nothing is
  converted on the CPU.
- **The scanner's keyframes are ARKit's own pictures.** Each mesh slot samples
  the picture its mesh was textured from, the frame's own `RgbdFrame::color`,
  so the ring holds up to three of ARKit's pixel buffers beyond the frames
  being captured and fused.
- **Local Network access.** The live link's first connect to a local address
  raises iOS's Local Network prompt (`NSLocalNetworkUsageDescription`); until
  it is allowed, every connect fails with "No route to host".
- **Shaders are compiled on the host** by `glslc` and embedded into the binary
  as constexpr byte arrays, so there is no runtime shader file and nothing to
  resolve in the app bundle.
- **glm is header-only.** Homebrew's glm 1.0.3 defines `glm::glm` as a *macOS*
  dylib, which the iOS linker refuses; the core's iOS module answers every
  `find_package(glm)` with the host's glm headers as a header-only target
  instead. The host tests, a macOS build, take Homebrew's.

## Roadmap

1. ✅ **`compute_smoke`** — the compute path on device.
2. ✅ **`scanner` bring-up** — gfx rendering to a `CAMetalLayer` surface via
   `VK_EXT_metal_surface`, proving the render path and the Swift/Obj-C++ build.
3. **ARKit capture** — sensor's `ArkitSensor`, an `IRgbdSensor`: ARKit's
   depth (256×192, millimetres and confidence), its picture where ARKit left
   it, and its pose in sensor's camera axes. ARKit's depth is *registered* to
   its picture, so the two share a pose and differ only in intrinsics scale.
4. **Live fusion** — fuse and render live in `scanner`: ARKit frames feed recon
   on a background thread while the render thread draws the growing mesh, the
   `fuse_viewer` model. Then one shared `VkDevice`, the core's `SharedDevice`,
   built from the compute libraries' requirements ∪ `vg::device_requirements()`
   and handed to both sides via `adopt`. Starts on interop seam A (host mesh, as `fuse_viewer`
   does) and moves to seam B (indirect draw over a mesh ring with a timeline
   handoff) once it works.

## License

MIT — see [LICENSE](LICENSE).

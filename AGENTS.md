# Shared repository instructions

This is the canonical working guide for Codex, Claude Code, and other agents.
Keep shared instructions here. `CLAUDE.md` imports this file with `@AGENTS.md`;
do not duplicate these rules or create a circular reference.

## Project and boundaries

`volumetric_kit_ios` is the iOS application shell: ARKit capture →
`volumetric_kit_recon` fusion → `volumetric_kit_gfx` rendering on MoltenVK.
This repository owns the app, platform bridge, toolchain, and packaging.
The libraries remain independent siblings with their own guidance.

- Swift owns the app/UI/lifecycle; Objective-C++ owns the platform bridge;
  C++ owns the engine and host-testable app logic.
- `apps/scanner/Core/` stays plain C++ over glm: no Vulkan, recon, gfx,
  Objective-C, ARKit, or mach dependencies. Keep platform queries in `Bridge/`
  and the decisions made from their results in `Core/`.
- The bridge presents plain values and `NSError` to Swift. Keep the bridge's
  static-library target separate from the Swift app target under Xcode.
- Share one `VkDevice` between recon and gfx through their create/adopt seam.
  Preserve the documented requirements merge, queue selection, and lifetimes.
- Include Vulkan through gfx's `core/vulkan.hpp` umbrella, never directly
  through `<vulkan/...>`.

## Read what the task needs

| Task | Read |
| --- | --- |
| Build, dependencies, signing, device deployment | [README build guide](README.md#build), relevant CMake/toolchain files |
| Pure app logic and host tests | [Host-test boundary](README.md#the-host-tests), `apps/scanner/Core/CMakeLists.txt`, affected tests |
| Swift/Objective-C++ seam | [Language split](README.md#language-split), affected bridge headers and Swift callers |
| Shared Vulkan device | [Bootstrap contract](README.md#the-duplicated-bootstrap), `Bridge/SharedDevice.{hpp,mm}` |
| Benchmarking incremental extraction | [Measurement build](README.md#the-measurement-build), scanner CMake and fusion configuration |
| Hardware capability or simulator assumptions | [Platform notes](README.md#platform-notes), relevant runtime guards |
| Formatting and CI | [Development](README.md#development), `.pre-commit-config.yaml`, `.github/workflows/` |

Read the relevant sections rather than the whole README before each edit.
Use the current source and CMake targets to verify implementation status;
roadmap and bring-up notes can describe earlier stages.

## Working with Git

- Use a dedicated branch and worktree under `.worktrees/` for each task.
  Concurrent Claude/Codex tasks use separate worktrees.
- Preserve unrelated local files and other tasks' worktrees. Remove your
  worktree after its PR merges.
- Use absolute paths for `git -C`, `cmake -S/-B`, and file operations so work
  cannot spill into a sibling repository.
- Use Conventional Commits, e.g. `fix(scanner): …`, `build: …`, `docs: …`.
- Assign PRs to the authenticated user (`gh pr create --assignee @me`).
- Keep shared decisions and handoff context in committed documentation.

## Build and validation

Run from the task's worktree. Pure app logic uses the host configuration:

```sh
ios_root="$(git rev-parse --show-toplevel)"
cmake -S "$ios_root" -B "$ios_root/build-host" \
  -DVI_HOST_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$ios_root/build-host" --parallel
ctest --test-dir "$ios_root/build-host" --output-on-failure
pre-commit run --all-files --show-diff-on-failure
git -C "$ios_root" diff --check
```

The host mode needs no iOS toolchain, MoltenVK, recon, or gfx. Changes to the
bridge, Swift, shaders, or app packaging need the iOS build on macOS with Xcode
and host `glslc` installed:

```sh
"$ios_root/tools/fetch_moltenvk.sh"
cmake -S "$ios_root" -B "$ios_root/build-ios" -G Xcode \
  -DCMAKE_TOOLCHAIN_FILE="$ios_root/cmake/ios.toolchain.cmake" \
  -DVI_INCREMENTAL_BENCHMARK=OFF
cmake --build "$ios_root/build-ios" --config Debug -- \
  CODE_SIGNING_ALLOWED=NO CODE_SIGNING_REQUIRED=NO CODE_SIGN_IDENTITY=""
```

- Keep host and iOS build trees separate. Use `--config Release` for Xcode
  performance measurements and report the build type, device, and workload.
- Add regression coverage for changed pure logic. For runtime platform
  behavior, distinguish compilation, host tests, and actual device checks.
- App targets are device-only iOS arm64. This MoltenVK package has no simulator
  slice; ARKit scene depth requires supported LiDAR hardware and a runtime
  capability check. An unsigned CI build is not a device runtime test.
- `VI_INCREMENTAL_BENCHMARK=ON` deliberately publishes no geometry and is
  cached. Use a separate benchmark tree or explicitly set it OFF for app runs.
  Read the reported `incremental` flag to distinguish the fast path from fallback.
- Use the pinned hooks. Keep both Cpp and ObjC sections in `.clang-format`;
  Swift formatting uses `xcrun swift-format` from the Xcode toolchain.
- Documentation-only changes need formatting, link, and consistency checks;
  no build is needed. Hooks can be scoped with `pre-commit run --files`.
- For device installation, follow README's signing and deployment instructions;
  keep local signing state and provisioning files out of commits.

## Dependency and documentation upkeep

- recon and gfx are pinned by commit in `CMakeLists.txt`. Adopt upstream
  changes by bumping a pin in its own PR, and say which commits the build used.
  When recon and gfx depend on `volumetric_kit_core`, declare the core first so
  both build against one pinned copy (the core's README).
- Local sibling source overrides use CMake's
  `FETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON` / `_VOLUMETRIC_KIT_GFX` options;
  keep machine-specific paths out of committed build configuration.
- Keep shared rules here concise (roughly 100–200 lines). Detailed build,
  architecture, and platform rationale belong in the relevant README section
  or a dedicated supporting document, linked from the task map.
- Update a changed contract and its rationale in the same commit as the code;
  distinguish shipped behavior from plans and hardware observations.
- `CLAUDE.md` stays an import. Add only genuinely Claude-specific instructions
  below it if needed; shared guidance is edited here only.

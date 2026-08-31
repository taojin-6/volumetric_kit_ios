# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vi_core: the scanner's pure logic.
#
# Everything here is plain C++ over glm -- no Vulkan, no recon, no gfx, no
# Objective-C, no ARKit, no mach. That is a deliberate boundary rather than an
# accident of what happened to have no dependencies: it is what makes this the
# one part of the app a host can build and a test can run, on a repo whose CI
# cannot run anything else (MoltenVK ships no simulator slice and ARKit scene
# depth needs LiDAR, so the app targets are build-only by nature).
#
# The rule for what belongs here: if it decides something -- a sign, a
# threshold, whether a reading is trustworthy -- and it can decide it without
# the device, it belongs here and it gets a test. If it needs a live task port,
# a VkDevice or an ARFrame, it stays in Bridge/ and this side gets the part that
# reasons about what it returned. Bridge/MemoryQuery.cpp is that split done
# once: the syscall there, the meaning here.
add_library(vi_core STATIC AllocationStop.cpp MemoryBudget.cpp OrbitCamera.cpp
                           ViewOrientation.cpp)

# The same warning set the sibling libraries build under, and here for a reason
# beyond consistency: AllocationStop.cpp turns an enum into what four different
# media say about it, and every one of those is an exhaustive switch whose
# guarantee -- "a cause added stops the compile" -- is a bare warning without
# `-Werror=switch`. Left as a warning, a new cause links cleanly and reports
# itself as a healthy scan on every rendering at once.
target_compile_options(vi_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

target_include_directories(vi_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(vi_core PUBLIC glm::glm)

# glm targets OpenGL's [-1, 1] clip depth; Vulkan's is [0, 1]. PUBLIC and set on
# the lowest target that touches glm, so every consumer compiles glm the same
# way -- glm is header-only, so the convention is fixed per translation unit at
# instantiation and a TU that missed it would silently build a projection the
# depth buffer disagrees with.
#
# Nothing here calls `glm::perspective` today, so nothing here depends on it
# yet. It is set anyway because the trap is silent and the guard is free: the
# moment any projection math moves into this library, it arrives already correct
# rather than needing this to be remembered.
target_compile_definitions(vi_core PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)

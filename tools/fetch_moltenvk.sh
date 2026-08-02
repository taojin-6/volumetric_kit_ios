#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin
#
# Fetch the MoltenVK iOS distribution into third_party/.
#
# MoltenVK is the Vulkan implementation on iOS (there is no loader and no
# libvulkan on the platform), so it is a hard prerequisite for any iOS build
# here. The upstream release tarball carries a static ios-arm64 slice plus the
# matching Vulkan headers -- both of which cmake/ios.toolchain.cmake points
# find_package(Vulkan) at.
#
# This is not vendored into git: it is a 34 MB binary artifact with a stable
# upstream URL, so we fetch it once and .gitignore the result.

set -euo pipefail

# Keep in lockstep with the Homebrew molten-vk used for desktop builds, so the
# same MoltenVK version backs macOS and iOS and a driver-specific bug cannot
# show up on only one of them.
MOLTENVK_VERSION="${MOLTENVK_VERSION:-v1.4.2}"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dest="${repo_root}/third_party"
tarball="${dest}/MoltenVK-ios-${MOLTENVK_VERSION}.tar"
url="https://github.com/KhronosGroup/MoltenVK/releases/download/${MOLTENVK_VERSION}/MoltenVK-ios.tar"

lib="${dest}/MoltenVK/MoltenVK/static/MoltenVK.xcframework/ios-arm64/libMoltenVK.a"
if [[ -f "${lib}" ]]; then
  echo "MoltenVK already present: ${lib}"
  exit 0
fi

mkdir -p "${dest}"
echo "Fetching MoltenVK ${MOLTENVK_VERSION} for iOS..."
curl --fail --location --progress-bar --output "${tarball}" "${url}"

echo "Extracting..."
tar -xf "${tarball}" -C "${dest}"
rm -f "${tarball}"

if [[ ! -f "${lib}" ]]; then
  echo "error: expected ${lib} after extraction; upstream layout may have changed" >&2
  exit 1
fi

echo "MoltenVK ready: ${lib}"

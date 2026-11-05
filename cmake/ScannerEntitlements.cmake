# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vi_write_scanner_entitlements(<output> <extended_virtual_addressing>)
#
# Writes the scanner's entitlements from apps/scanner/scanner.entitlements.in to
# <output>. increased-memory-limit is in every build;
# extended-virtual-addressing only when <extended_virtual_addressing> is true.
# The app's configure and the host test (tests/scanner_entitlements_test.cmake)
# both call this, so the test checks the file the app is signed with.
function(vi_write_scanner_entitlements output extended_virtual_addressing)
  # The key goes on from the end of the line before it, so an OFF build leaves
  # no blank line inside <dict>.
  if(extended_virtual_addressing)
    set(VI_EXTENDED_VIRTUAL_ADDRESSING_KEY
        "\n\t<key>com.apple.developer.kernel.extended-virtual-addressing</key>\n\t<true/>"
    )
  else()
    set(VI_EXTENDED_VIRTUAL_ADDRESSING_KEY "")
  endif()
  configure_file(
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../apps/scanner/scanner.entitlements.in"
    "${output}" @ONLY)
endfunction()

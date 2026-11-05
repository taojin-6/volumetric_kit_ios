# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The scanner's entitlements, written by the app's own function both ways
# VI_EXTENDED_VIRTUAL_ADDRESSING can be set, into OUT_DIR:
#
# ~~~
#   cmake -DOUT_DIR=<dir> -P scanner_entitlements_test.cmake
# ~~~
#
# Each must lint, carry increased-memory-limit, carry
# extended-virtual-addressing exactly when asked, and hold no doubled hyphen
# inside an XML comment: plutil accepts one, but AMFI's parser rejects the file
# (scanner.entitlements.in).
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ScannerEntitlements.cmake")

if(NOT OUT_DIR)
  message(FATAL_ERROR "pass -DOUT_DIR=<dir>")
endif()

find_program(PLUTIL plutil)
if(NOT PLUTIL)
  # tests/CMakeLists.txt marks the test skipped on this line.
  message("SKIP: plutil not found")
  return()
endif()

set(memory_key "com.apple.developer.kernel.increased-memory-limit")
set(address_key "com.apple.developer.kernel.extended-virtual-addressing")

foreach(extended_va OFF ON)
  set(out "${OUT_DIR}/scanner-${extended_va}.entitlements")
  vi_write_scanner_entitlements("${out}" ${extended_va})

  execute_process(
    COMMAND "${PLUTIL}" -lint "${out}"
    RESULT_VARIABLE lint_result
    OUTPUT_VARIABLE lint_output
    ERROR_VARIABLE lint_output)
  if(NOT lint_result EQUAL 0)
    message(FATAL_ERROR "${extended_va}: plutil -lint failed: ${lint_output}")
  endif()

  # As JSON, because plutil's keypaths split on the dots in these keys.
  execute_process(
    COMMAND "${PLUTIL}" -convert json -o - "${out}"
    RESULT_VARIABLE json_result
    OUTPUT_VARIABLE json
    ERROR_VARIABLE json_error)
  if(NOT json_result EQUAL 0)
    message(FATAL_ERROR "${extended_va}: plutil -convert failed: ${json_error}")
  endif()

  # Exactly these keys, each true: increased-memory-limit always,
  # extended-virtual-addressing when asked for and only then.
  set(expected "${memory_key}=ON")
  if(extended_va)
    list(APPEND expected "${address_key}=ON")
  endif()
  set(found "")
  string(JSON count LENGTH "${json}")
  if(count GREATER 0)
    math(EXPR last "${count} - 1")
    foreach(index RANGE ${last})
      string(JSON key MEMBER "${json}" ${index})
      string(JSON value GET "${json}" ${key})
      list(APPEND found "${key}=${value}")
    endforeach()
  endif()
  list(SORT expected)
  list(SORT found)
  if(NOT found STREQUAL expected)
    message(FATAL_ERROR "${extended_va}: carries [${found}], not [${expected}]")
  endif()

  # Every comment, opener to the first closer after it: its text must hold no
  # doubled hyphen and must not end in a hyphen (that closes it with three).
  file(READ "${out}" text)
  string(FIND "${text}" "<!--" open)
  while(open GREATER -1)
    math(EXPR body_start "${open} + 4")
    string(SUBSTRING "${text}" ${body_start} -1 rest)
    string(FIND "${rest}" "-->" close)
    if(close EQUAL -1)
      message(FATAL_ERROR "${extended_va}: an XML comment is never closed")
    endif()
    string(SUBSTRING "${rest}" 0 ${close} body)
    string(FIND "${body}" "--" doubled)
    if(doubled GREATER -1 OR body MATCHES "-$")
      message(
        FATAL_ERROR
          "${extended_va}: a doubled hyphen inside an XML comment, which AMFI "
          "rejects")
    endif()
    math(EXPR after "${close} + 3")
    string(SUBSTRING "${rest}" ${after} -1 text)
    string(FIND "${text}" "<!--" open)
  endwhile()
endforeach()

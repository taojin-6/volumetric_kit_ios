# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The scanner's build stamp: which commit an installed app was built from.
#
# vi_scanner_version(<source_dir> <git> <local_sources>)
#
# Sets, in the caller's scope:
#
# ~~~
#   VI_BUNDLE_VERSION  CFBundleVersion: the number of first-parent commits up
#                      to HEAD (`git rev-list --count --first-parent HEAD`)
#   VI_GIT_COMMIT      VIGitCommit: HEAD's full hash, then "-dirty" when a
#                      tracked file differs from HEAD, then "+local" when
#                      <local_sources> is not empty
#   VI_VERSION_NOTE    one line for the build log saying what was stamped
# ~~~
#
# CFBundleShortVersionString stays the project's version. Apple documents it and
# CFBundleVersion as period-separated integers, so neither can hold a hash;
# `devicectl device info apps` shows the two as Version and Bundle Version.
# main's first-parent count rises with every merge to it (by one for a merge
# commit or a squash, by the branch's commits for a rebase merge), and names one
# commit of main: the Nth line of `git rev-list --first-parent --reverse
# origin/main`. A branch counts its own commits on top of where it left main, so
# its number can equal a later main's; VIGitCommit names the commit exactly.
#
# The hash is this checkout's alone. A dependency built from a local tree
# (FETCHCONTENT_SOURCE_DIR_<NAME>, as a local sibling is) is in no commit here,
# so no commit rebuilds that app: <local_sources>, the dependencies' names as
# vi_scanner_local_sources gives them, marks the hash "+local" and goes in the
# build log.
#
# Fallbacks, so a build never fails for want of git: with no git, outside a
# checkout, before the first commit, or in a source tree that is not itself the
# checkout's top (a copy inside some other repository, whose commit would be the
# wrong one), the stamp is 0 and "unknown". A shallow clone, such as CI's, has a
# hash but no true count, so it stamps 0 and the hash rather than a wrong count.
#
# Untracked files do not make a build dirty, as with `git describe --dirty`. The
# status read takes no optional locks, so a build running beside a git command
# never leaves that command an index.lock to trip on.
#
# vi_scanner_local_sources(<var>)
#
# Sets <var> to the names of the dependencies the configure takes from a local
# tree, every FETCHCONTENT_SOURCE_DIR_<NAME> that is set, as <NAME>s joined by
# commas; "" when there are none.
#
# vi_configure_scanner_info_plist(<template> <output> <source_dir> <git>
# <command_var>)
#
# The configure's half. Configures <template> to <output>.stamp.in with @ONLY
# and the caller's variables, all but the stamp's two, which stay
# @VI_BUNDLE_VERSION@ and @VI_GIT_COMMIT@; stamps that to <output>; and sets
# <command_var> to the command that stamps it again, which the build runs. Every
# other value in the template is the configure's, so a build cannot blank one.
#
# vi_stamp_scanner_info_plist(<stamp_template> <output> <source_dir> <git>
# <local_sources>)
#
# The stamp: puts the two values in <stamp_template>'s place and writes <output>
# only when its text changes, so an unchanged stamp changes nothing Xcode
# tracks.
#
# apps/scanner/CMakeLists.txt runs the command from a target the app depends on,
# so a commit, a checkout or an edit reaches the next xcodebuild with no
# configure in between. It is this file in script mode, with -DSTAMP_TEMPLATE=,
# -DOUTPUT=, -DSOURCE_DIR=, -DGIT_EXECUTABLE= (empty for none) and
# -DLOCAL_SOURCES=. The host test (tests/scanner_version_test.cmake) calls these
# functions on repositories of its own and runs that same command.

# One git command in the caller's <source_dir> with the caller's <git>: its
# stripped stdout in `out`, and whether it exited 0 in `ok`. A missing
# executable is a failed command, not an error.
macro(_vi_scanner_git)
  execute_process(
    COMMAND "${git}" -C "${source_dir}" ${ARGN}
    RESULT_VARIABLE _vi_git_result
    OUTPUT_VARIABLE out
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_vi_git_result STREQUAL "0")
    set(ok TRUE)
  else()
    set(ok FALSE)
  endif()
endmacro()

function(vi_scanner_version source_dir git local_sources)
  set(build "0")
  set(commit "unknown")
  set(notes "")

  if(NOT git)
    list(APPEND notes "git not found")
  else()
    _vi_scanner_git(rev-parse --show-toplevel)
    if(ok)
      file(REAL_PATH "${out}" top)
      file(REAL_PATH "${source_dir}" source)
    endif()
    if(NOT ok)
      list(APPEND notes "not a git checkout")
    elseif(NOT top STREQUAL source)
      list(APPEND notes "inside ${top}, not a checkout of its own")
    else()
      _vi_scanner_git(rev-parse --verify --quiet HEAD)
      # SHA-1, or SHA-256 for a repository that uses it.
      if(NOT ok OR NOT out MATCHES "^[0-9a-f]+$")
        list(APPEND notes "no commit yet")
      else()
        set(commit "${out}")
        _vi_scanner_git(--no-optional-locks status --porcelain
                        --untracked-files=no)
        # A status that cannot be read cannot vouch for a clean tree either.
        if(NOT ok OR NOT out STREQUAL "")
          string(APPEND commit "-dirty")
        endif()
        _vi_scanner_git(rev-parse --is-shallow-repository)
        if(ok AND out STREQUAL "false")
          _vi_scanner_git(rev-list --count --first-parent HEAD)
          if(ok AND out MATCHES "^[1-9][0-9]*$")
            set(build "${out}")
          else()
            list(APPEND notes "commit count unreadable")
          endif()
        else()
          list(APPEND notes "shallow clone, no commit count")
        endif()
      endif()
    endif()
  endif()

  if(NOT local_sources STREQUAL "")
    string(APPEND commit "+local")
    list(APPEND notes "local sources: ${local_sources}")
  endif()

  set(line "CFBundleVersion ${build}, VIGitCommit ${commit}")
  if(NOT notes STREQUAL "")
    list(JOIN notes "; " notes)
    string(APPEND line " (${notes})")
  endif()
  set(VI_BUNDLE_VERSION
      "${build}"
      PARENT_SCOPE)
  set(VI_GIT_COMMIT
      "${commit}"
      PARENT_SCOPE)
  set(VI_VERSION_NOTE
      "${line}"
      PARENT_SCOPE)
endfunction()

function(vi_scanner_local_sources var)
  set(names "")
  get_cmake_property(variables VARIABLES)
  foreach(variable IN LISTS variables)
    if(variable MATCHES "^FETCHCONTENT_SOURCE_DIR_(.+)$")
      set(name "${CMAKE_MATCH_1}")
      if(NOT "${${variable}}" STREQUAL "")
        list(APPEND names "${name}")
      endif()
    endif()
  endforeach()
  list(SORT names)
  list(JOIN names "," names)
  set(${var}
      "${names}"
      PARENT_SCOPE)
endfunction()

function(vi_stamp_scanner_info_plist stamp_template output source_dir git
         local_sources)
  vi_scanner_version("${source_dir}" "${git}" "${local_sources}")
  file(READ "${stamp_template}" text)
  string(REPLACE "@VI_BUNDLE_VERSION@" "${VI_BUNDLE_VERSION}" text "${text}")
  string(REPLACE "@VI_GIT_COMMIT@" "${VI_GIT_COMMIT}" text "${text}")
  set(written "")
  if(EXISTS "${output}")
    file(READ "${output}" written)
  endif()
  string(COMPARE NOTEQUAL "${text}" "${written}" changed)
  if(changed)
    file(WRITE "${output}" "${text}")
  endif()
  message(STATUS "scanner Info.plist: ${VI_VERSION_NOTE}")
endfunction()

function(vi_configure_scanner_info_plist template output source_dir git
         command_var)
  set(stamp_template "${output}.stamp.in")
  set(VI_BUNDLE_VERSION "@VI_BUNDLE_VERSION@")
  set(VI_GIT_COMMIT "@VI_GIT_COMMIT@")
  configure_file("${template}" "${stamp_template}" @ONLY)
  vi_scanner_local_sources(local_sources)
  vi_stamp_scanner_info_plist("${stamp_template}" "${output}" "${source_dir}"
                              "${git}" "${local_sources}")
  set(${command_var}
      "${CMAKE_COMMAND}"
      "-DSTAMP_TEMPLATE=${stamp_template}"
      "-DOUTPUT=${output}"
      "-DSOURCE_DIR=${source_dir}"
      "-DGIT_EXECUTABLE=${git}"
      "-DLOCAL_SOURCES=${local_sources}"
      -P
      "${CMAKE_CURRENT_FUNCTION_LIST_FILE}"
      PARENT_SCOPE)
endfunction()

# Script mode, as the build runs it.
if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
  foreach(required STAMP_TEMPLATE OUTPUT SOURCE_DIR)
    if(NOT DEFINED ${required})
      message(FATAL_ERROR "pass -D${required}=")
    endif()
  endforeach()
  vi_stamp_scanner_info_plist("${STAMP_TEMPLATE}" "${OUTPUT}" "${SOURCE_DIR}"
                              "${GIT_EXECUTABLE}" "${LOCAL_SOURCES}")
endif()

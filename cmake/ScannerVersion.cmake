# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The scanner's build stamp: which commit an installed app was built from.
#
# vi_scanner_version(<source_dir> <git>)
#
# Sets, in the caller's scope:
#
# ~~~
#   VI_BUNDLE_VERSION  CFBundleVersion: the number of first-parent commits up
#                      to HEAD (`git rev-list --count --first-parent HEAD`)
#   VI_GIT_COMMIT      VIGitCommit: HEAD's full hash, with "-dirty" appended
#                      when a tracked file differs from HEAD
#   VI_VERSION_NOTE    one line for the build log saying what was stamped
# ~~~
#
# CFBundleShortVersionString stays the project's version. Apple documents it and
# CFBundleVersion as period-separated integers, so neither can hold a hash;
# `devicectl device info apps` shows the two as Version and Bundle Version. main
# takes every change as a merge, so its first-parent count rises by one a merge
# and names one commit of main: the Nth line of `git rev-list --first-parent
# --reverse origin/main`. A branch counts its own commits on top of where it
# left main, so its number can equal a later main's; VIGitCommit is what names
# any build exactly.
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
# vi_write_scanner_info_plist(<template> <output> <source_dir> <git>)
#
# Stamps, then configures <template> to <output> with @ONLY. configure_file
# leaves <output> untouched when its text would not change, so an unchanged
# stamp changes nothing Xcode tracks. The template also takes VI_APP_NAME,
# VI_BUNDLE_ID and PROJECT_VERSION from the caller.
#
# The scanner's configure writes the plist this way, and so does every build:
# apps/scanner/CMakeLists.txt runs this file as a script from a target the app
# depends on, so a commit, a checkout or an edit reaches the next xcodebuild
# with no configure in between. In script mode it takes -DTEMPLATE=, -DOUTPUT=,
# -DSOURCE_DIR=, -DGIT_EXECUTABLE= (empty for none) and the three template
# values. The host test (tests/scanner_version_test.cmake) calls the same two
# functions on repositories of its own.

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

function(vi_scanner_version source_dir git)
  set(build "0")
  set(commit "unknown")
  set(note "")

  if(NOT git)
    set(note "git not found")
  else()
    _vi_scanner_git(rev-parse --show-toplevel)
    if(ok)
      file(REAL_PATH "${out}" top)
      file(REAL_PATH "${source_dir}" source)
    endif()
    if(NOT ok)
      set(note "not a git checkout")
    elseif(NOT top STREQUAL source)
      set(note "inside ${top}, not a checkout of its own")
    else()
      _vi_scanner_git(rev-parse --verify --quiet HEAD)
      # SHA-1, or SHA-256 for a repository that uses it.
      if(NOT ok OR NOT out MATCHES "^[0-9a-f]+$")
        set(note "no commit yet")
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
            set(note "commit count unreadable")
          endif()
        else()
          set(note "shallow clone, no commit count")
        endif()
      endif()
    endif()
  endif()

  set(line "CFBundleVersion ${build}, VIGitCommit ${commit}")
  if(note)
    string(APPEND line " (${note})")
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

function(vi_write_scanner_info_plist template output source_dir git)
  vi_scanner_version("${source_dir}" "${git}")
  configure_file("${template}" "${output}" @ONLY)
  message(STATUS "scanner Info.plist: ${VI_VERSION_NOTE}")
endfunction()

# Script mode, as the build runs it.
if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
  foreach(required TEMPLATE OUTPUT SOURCE_DIR VI_APP_NAME VI_BUNDLE_ID
                   PROJECT_VERSION)
    if(NOT DEFINED ${required})
      message(FATAL_ERROR "pass -D${required}=")
    endif()
  endforeach()
  vi_write_scanner_info_plist("${TEMPLATE}" "${OUTPUT}" "${SOURCE_DIR}"
                              "${GIT_EXECUTABLE}")
endif()

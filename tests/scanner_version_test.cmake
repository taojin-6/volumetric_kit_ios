# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The scanner's build stamp (cmake/ScannerVersion.cmake), on repositories made
# here in OUT_DIR:
#
# ~~~
#   cmake -DOUT_DIR=<dir> -P scanner_version_test.cmake
# ~~~
#
# The count is first-parent, a merged branch adding one; a tracked edit marks
# the hash -dirty and an untracked file does not; a local dependency marks it
# +local; a linked worktree, whose .git is a file, stamps its own HEAD; a
# shallow clone stamps its hash and no count; and with no git, no checkout, no
# commit, or a tree inside another repository, the stamp is 0 and unknown. Then
# the app's own Info.plist.in, by the app's own functions: the configure's
# write, and the command the build runs, in a cmake of its own as Xcode runs it.
# The plist must lint, carry the stamp in integer versions, keep every value the
# configure filled in through a build's re-stamp, and be left untouched when the
# stamp has not changed, and rewritten when it has. That the Xcode project runs
# the command and builds its output into the app is CI's to check
# (.github/workflows/build.yml), as only an iOS build has the project.
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ScannerVersion.cmake")

if(NOT OUT_DIR)
  message(FATAL_ERROR "pass -DOUT_DIR=<dir>")
endif()

find_program(GIT git)
find_program(PLUTIL plutil)
if(NOT GIT OR NOT PLUTIL)
  # tests/CMakeLists.txt marks the test skipped on this line.
  message("SKIP: git or plutil not found")
  return()
endif()

# The repositories here are this test's alone: no variable from a calling git (a
# hook's GIT_DIR would aim every command below at the real repository), and none
# of the user's configuration (signing, hooks, a default branch name).
foreach(variable GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE GIT_COMMON_DIR
                 GIT_OBJECT_DIRECTORY GIT_CEILING_DIRECTORIES)
  unset(ENV{${variable}})
endforeach()
set(ENV{GIT_CONFIG_GLOBAL} /dev/null)
set(ENV{GIT_CONFIG_NOSYSTEM} 1)
set(ENV{GIT_AUTHOR_NAME} test)
set(ENV{GIT_AUTHOR_EMAIL} test@example.invalid)
set(ENV{GIT_COMMITTER_NAME} test)
set(ENV{GIT_COMMITTER_EMAIL} test@example.invalid)

file(REMOVE_RECURSE "${OUT_DIR}")
file(MAKE_DIRECTORY "${OUT_DIR}")

# run_git <dir> <args...>: run git there, and fail the test if it fails.
function(run_git dir)
  execute_process(
    COMMAND "${GIT}" -C "${dir}" ${ARGN}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "git ${ARGN} in ${dir}: ${err}")
  endif()
  set(out
      "${out}"
      PARENT_SCOPE)
endfunction()

function(commit dir message)
  run_git("${dir}" commit --allow-empty --no-verify -m "${message}")
endfunction()

# expect(<case> <dir> <git> <build> <commit> [<local sources>])
function(expect case dir git build commit)
  vi_scanner_version("${dir}" "${git}" "${ARGN}")
  if(NOT VI_BUNDLE_VERSION STREQUAL build OR NOT VI_GIT_COMMIT STREQUAL commit)
    message(
      FATAL_ERROR
        "${case}: stamped ${VI_BUNDLE_VERSION} ${VI_GIT_COMMIT}, expected "
        "${build} ${commit} (${VI_VERSION_NOTE})")
  endif()
endfunction()

# --- A repository whose main took a branch as a merge ------------------------
set(repo "${OUT_DIR}/repo")
file(MAKE_DIRECTORY "${repo}")
run_git("${repo}" init --initial-branch=main)
expect("no commit" "${repo}" "${GIT}" 0 unknown)

file(WRITE "${repo}/tracked.txt" "one\n")
run_git("${repo}" add tracked.txt)
commit("${repo}" c1)
commit("${repo}" c2)
run_git("${repo}" switch -c side)
commit("${repo}" s1)
commit("${repo}" s2)
commit("${repo}" s3)
run_git("${repo}" switch main)
commit("${repo}" c3)
run_git("${repo}" merge --no-ff --no-edit --no-verify side)
run_git("${repo}" rev-parse HEAD)
set(head "${out}")
# c1 c2 c3 and the merge: four on the first-parent line, of seven in all.
expect("merge" "${repo}" "${GIT}" 4 "${head}")

# An untracked file is not a change to the build's sources.
file(WRITE "${repo}/untracked.txt" "new\n")
expect("untracked" "${repo}" "${GIT}" 4 "${head}")

# A tracked edit is, staged or not; the count stays HEAD's.
file(WRITE "${repo}/tracked.txt" "two\n")
expect("edited" "${repo}" "${GIT}" 4 "${head}-dirty")
run_git("${repo}" add tracked.txt)
expect("staged" "${repo}" "${GIT}" 4 "${head}-dirty")
run_git("${repo}" reset --hard HEAD)

# The next commit stamps itself, so a rebuild after it differs.
commit("${repo}" c4)
run_git("${repo}" rev-parse HEAD)
set(next "${out}")
expect("next commit" "${repo}" "${GIT}" 5 "${next}")

# A dependency from a local tree is in no commit here.
expect("local" "${repo}" "${GIT}" 5 "${next}+local" VOLUMETRIC_KIT_RECON)
file(WRITE "${repo}/tracked.txt" "three\n")
expect("edited, local" "${repo}" "${GIT}" 5 "${next}-dirty+local"
       "VOLUMETRIC_KIT_GFX,VOLUMETRIC_KIT_RECON")
run_git("${repo}" reset --hard HEAD)

# Which dependencies are local: the FETCHCONTENT_SOURCE_DIR_<NAME>s that are
# set, a cache entry or a variable alike. A function, so they go with it.
function(expect_local_sources case expected)
  vi_scanner_local_sources(names)
  if(NOT names STREQUAL expected)
    message(FATAL_ERROR "${case}: local sources '${names}', expected "
                        "'${expected}'")
  endif()
endfunction()
expect_local_sources("no local sources" "")
function(local_sources_set)
  set(FETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON "${OUT_DIR}/recon")
  set(FETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE "")
  set(FETCHCONTENT_SOURCE_DIR_SPIRV-CROSS "${OUT_DIR}/spirv-cross")
  expect_local_sources("local sources" "SPIRV-CROSS,VOLUMETRIC_KIT_RECON")
endfunction()
local_sources_set()

# --- A linked worktree: .git is a file there ---------------------------------
set(worktree "${OUT_DIR}/worktree")
# Detached at the merge, which a commit given in place of a branch gives.
run_git("${repo}" worktree add "${worktree}" "${head}")
if(IS_DIRECTORY "${worktree}/.git")
  message(FATAL_ERROR "worktree: .git is a directory, not the case under test")
endif()
expect("worktree" "${worktree}" "${GIT}" 4 "${head}")
commit("${worktree}" w1)
run_git("${worktree}" rev-parse HEAD)
expect("worktree commit" "${worktree}" "${GIT}" 5 "${out}")

# --- A shallow clone has a hash and no true count ----------------------------
# Of main, the repository's checked-out branch.
set(shallow "${OUT_DIR}/shallow")
run_git("${OUT_DIR}" clone --depth=1 "file://${repo}" "${shallow}")
expect("shallow" "${shallow}" "${GIT}" 0 "${next}")

# --- No commit to read --------------------------------------------------------
# Inside the repository but not its top: its commit is not this tree's.
file(MAKE_DIRECTORY "${repo}/nested")
expect("nested" "${repo}/nested" "${GIT}" 0 unknown)
set(plain "${OUT_DIR}/plain")
file(MAKE_DIRECTORY "${plain}")
set(ENV{GIT_CEILING_DIRECTORIES} "${OUT_DIR}")
expect("not a checkout" "${plain}" "${GIT}" 0 unknown)
unset(ENV{GIT_CEILING_DIRECTORIES})
expect("no git" "${repo}" "" 0 unknown)
expect("git not found" "${repo}" "GIT-NOTFOUND" 0 unknown)
expect("git missing" "${repo}" "${OUT_DIR}/no-such-git" 0 unknown)

# --- The app's Info.plist ----------------------------------------------------
# The template with a value of the configure's own added, as a later key would
# add one: a build's re-stamp must keep it, though the build passes it no value.
file(READ "${CMAKE_CURRENT_LIST_DIR}/../apps/scanner/Info.plist.in" text)
set(end "</dict>\n</plist>\n")
string(FIND "${text}" "${end}" at REVERSE)
string(LENGTH "${text}" length)
string(LENGTH "${end}" end_length)
math(EXPR at_end "${at} + ${end_length}")
if(at EQUAL -1 OR NOT at_end EQUAL length)
  message(FATAL_ERROR "Info.plist.in no longer ends ${end}")
endif()
string(SUBSTRING "${text}" 0 ${at} text)
string(APPEND text "  <key>VIExample</key>\n  <string>@VI_EXAMPLE@</string>\n"
       "${end}")
set(template "${OUT_DIR}/Info.plist.in")
file(WRITE "${template}" "${text}")

set(plist "${OUT_DIR}/Info.plist")
set(VI_APP_NAME scanner)
set(VI_BUNDLE_ID io.example.scanner)
set(PROJECT_VERSION 1.2.3)
set(VI_EXAMPLE configured-value)

# plist_value(<key> <var>)
function(plist_value key var)
  execute_process(
    COMMAND "${PLUTIL}" -extract "${key}" raw -o - "${plist}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE value
    ERROR_VARIABLE err
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Info.plist: no ${key}: ${err}")
  endif()
  set(${var}
      "${value}"
      PARENT_SCOPE)
endfunction()

# check_plist(<case> <build> <commit>)
function(check_plist case build commit)
  execute_process(
    COMMAND "${PLUTIL}" -lint "${plist}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE lint
    ERROR_VARIABLE lint)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "${case}: plutil -lint failed: ${lint}")
  endif()
  plist_value(CFBundleShortVersionString short)
  plist_value(CFBundleVersion bundle)
  plist_value(VIGitCommit stamped)
  # Apple's form for both: up to three period-separated integers.
  foreach(version "${short}" "${bundle}")
    if(NOT version MATCHES "^[0-9]+(\\.[0-9]+)?(\\.[0-9]+)?$")
      message(FATAL_ERROR "${case}: version ${version} is not integers")
    endif()
  endforeach()
  if(NOT short STREQUAL "1.2.3"
     OR NOT bundle STREQUAL build
     OR NOT stamped STREQUAL commit)
    message(
      FATAL_ERROR
        "${case}: carries ${short} (${bundle}) ${stamped}, expected 1.2.3 "
        "(${build}) ${commit}")
  endif()
  # And the configure's values, all of them still there.
  plist_value(CFBundleExecutable executable)
  plist_value(CFBundleIdentifier identifier)
  plist_value(VIExample example)
  if(NOT executable STREQUAL "scanner"
     OR NOT identifier STREQUAL "io.example.scanner"
     OR NOT example STREQUAL "configured-value")
    message(
      FATAL_ERROR
        "${case}: carries ${executable} ${identifier} VIExample '${example}', "
        "expected scanner io.example.scanner VIExample 'configured-value'")
  endif()
endfunction()

# build_stamp(<case>): the command the build runs, in a cmake of its own with
# none of this script's variables, as Xcode runs it.
function(build_stamp case)
  execute_process(
    COMMAND ${command}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "${case}: the build's stamp failed: ${out}${err}")
  endif()
endfunction()

# plist_age(<var>): the plist's year, 2000 while it is untouched.
function(plist_age var)
  file(TIMESTAMP "${plist}" year "%Y")
  set(${var}
      "${year}"
      PARENT_SCOPE)
endfunction()

vi_configure_scanner_info_plist("${template}" "${plist}" "${worktree}" "${GIT}"
                                command)
run_git("${worktree}" rev-parse HEAD)
set(worktree_head "${out}")
check_plist("configure" 5 "${worktree_head}")
build_stamp("build")
check_plist("build" 5 "${worktree_head}")

# An unchanged stamp leaves the file alone, at a configure or a build, or Xcode
# would re-process and re-sign the app at every build.
execute_process(COMMAND touch -t 200001010000 "${plist}")
build_stamp("unchanged build")
vi_configure_scanner_info_plist("${template}" "${plist}" "${worktree}" "${GIT}"
                                command)
plist_age(age)
if(NOT age STREQUAL "2000")
  message(FATAL_ERROR "plist: rewritten with an unchanged stamp")
endif()

# A commit since the configure is in the next build's plist, with every value
# the configure filled in kept.
commit("${worktree}" w2)
run_git("${worktree}" rev-parse HEAD)
set(worktree_next "${out}")
build_stamp("build after a commit")
check_plist("build after a commit" 6 "${worktree_next}")
plist_age(age)
if(age STREQUAL "2000")
  message(FATAL_ERROR "plist: not rewritten after a commit")
endif()

# A local dependency at the configure reaches every build's stamp.
function(configure_with_local_sources)
  set(FETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_RECON "${OUT_DIR}/recon")
  vi_configure_scanner_info_plist("${template}" "${plist}" "${worktree}"
                                  "${GIT}" command)
  set(command
      "${command}"
      PARENT_SCOPE)
endfunction()
configure_with_local_sources()
file(REMOVE "${plist}")
build_stamp("build with a local source")
check_plist("build with a local source" 6 "${worktree_next}+local")

# And the fallback still makes a valid plist, at the configure and the build.
vi_configure_scanner_info_plist("${template}" "${plist}" "${plain}" "${GIT}"
                                command)
check_plist("configure outside a checkout" 0 unknown)
file(REMOVE "${plist}")
build_stamp("build outside a checkout")
check_plist("build outside a checkout" 0 unknown)

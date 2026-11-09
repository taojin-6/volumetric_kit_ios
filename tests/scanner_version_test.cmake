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
# the hash -dirty and an untracked file does not; a linked worktree, whose .git
# is a file, stamps its own HEAD; a shallow clone stamps its hash and no count;
# and with no git, no checkout, no commit, or a tree inside another repository,
# the stamp is 0 and unknown. Then the app's own Info.plist.in, written by the
# app's own function: it must lint, carry the stamp in integer versions, and be
# left untouched when the stamp has not changed, and rewritten when it has.
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

# expect(<case> <dir> <git> <build> <commit>)
function(expect case dir git build commit)
  vi_scanner_version("${dir}" "${git}")
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
set(template "${CMAKE_CURRENT_LIST_DIR}/../apps/scanner/Info.plist.in")
set(plist "${OUT_DIR}/Info.plist")
set(VI_APP_NAME scanner)
set(VI_BUNDLE_ID io.example.scanner)
set(PROJECT_VERSION 1.2.3)

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
endfunction()

vi_write_scanner_info_plist("${template}" "${plist}" "${worktree}" "${GIT}")
run_git("${worktree}" rev-parse HEAD)
set(worktree_head "${out}")
check_plist("plist" 5 "${worktree_head}")

# An unchanged stamp leaves the file alone, or Xcode would re-process and
# re-sign the app at every build.
execute_process(COMMAND touch -t 200001010000 "${plist}")
file(TIMESTAMP "${plist}" before "%Y")
vi_write_scanner_info_plist("${template}" "${plist}" "${worktree}" "${GIT}")
file(TIMESTAMP "${plist}" after "%Y")
if(NOT before STREQUAL "2000" OR NOT after STREQUAL "2000")
  message(FATAL_ERROR "plist: rewritten with an unchanged stamp")
endif()

# A commit since is in the next write.
commit("${worktree}" w2)
run_git("${worktree}" rev-parse HEAD)
vi_write_scanner_info_plist("${template}" "${plist}" "${worktree}" "${GIT}")
check_plist("plist after a commit" 6 "${out}")
file(TIMESTAMP "${plist}" after "%Y")
if(after STREQUAL "2000")
  message(FATAL_ERROR "plist: not rewritten after a commit")
endif()

# And the fallback still makes a valid plist.
vi_write_scanner_info_plist("${template}" "${plist}" "${plain}" "${GIT}")
check_plist("plist outside a checkout" 0 unknown)

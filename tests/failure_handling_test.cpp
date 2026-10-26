// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "FailureHandling.hpp"

#include <atomic>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "allocation_failure.hpp"

namespace app = volumetric_kit::ios_app;

TEST(FailureHandling, FailedFramePreparationRejectsWithoutPublishingScratch) {
  std::vector<int> published{9, 8};
  std::vector<int> scratch_depth{1, 2};
  std::vector<int> scratch_color;
  int rejected = 0;
  const auto submit = [&] {
    return app::contain_frame_failure(
        [&] {
          scratch_depth[0] = 42;
          scratch_color.resize(8192);
          published.swap(scratch_depth);
        },
        [&](std::string_view) { ++rejected; });
  };
  ios_test::AllocationFailure failure(0, true);
  const bool accepted = submit();
  const bool failed = failure.stop();
  EXPECT_TRUE(failed);
  EXPECT_FALSE(accepted);
  EXPECT_EQ(rejected, 1);
  EXPECT_EQ(published, (std::vector<int>{9, 8}));
  EXPECT_EQ(scratch_depth[0], 42);

  EXPECT_TRUE(submit());
  EXPECT_EQ(rejected, 1);
  EXPECT_EQ(published, (std::vector<int>{42, 2}));
  EXPECT_EQ(scratch_color.size(), 8192u);
}

TEST(FailureHandling, PersistentDiagnosticFailureStillCountsAndAllowsRetry) {
  std::uint64_t errors = 0;
  std::string stored = "previous error";
  constexpr std::string_view message =
      "the operation failed while preparing the next captured frame";
  ios_test::AllocationFailure failure(0, true);
  app::record_error(errors, stored, message, "worker: ");
  const bool failed = failure.stop();
  EXPECT_TRUE(failed);
  EXPECT_EQ(errors, 1u);
  EXPECT_TRUE(stored.empty());

  app::record_error(errors, stored, message, "worker: ");
  EXPECT_EQ(errors, 2u);
  EXPECT_EQ(stored, std::string("worker: ") + std::string(message));
}

TEST(FailureHandling, WorkerReportsAnExceptionUnderPersistentMemoryFailure) {
  std::uint64_t errors = 0;
  std::string stored;
  ios_test::AllocationFailure failure(0, true);
  const bool completed = app::contain_frame_failure(
      [] { throw std::bad_alloc(); },
      [&](std::string_view message) noexcept {
        app::record_error(errors, stored, message, "fuse thread: ");
      });
  failure.stop();
  EXPECT_FALSE(completed);
  EXPECT_EQ(errors, 1u);
  EXPECT_TRUE(stored.empty());
  EXPECT_TRUE(app::contain_frame_failure([] {}, [](std::string_view) {}));
}

TEST(FailureHandling, ReporterExceptionCannotEscapeCallback) {
  bool reported = false;
  EXPECT_FALSE(app::contain_frame_failure([] { throw 42; },
                                          [&](std::string_view message) {
                                            reported =
                                                message == "unknown exception";
                                            throw std::bad_alloc();
                                          }));
  EXPECT_TRUE(reported);
}

TEST(FailureHandling, ThreadConstructionFailureRestoresFlagAndCanRetry) {
  std::atomic<bool> running{false};
  std::thread worker;
  std::atomic<bool> entered{false};
  std::uint64_t errors = 0;
  std::string stored;
  const auto launch = [&] {
    return std::thread([&] {
      entered.store(running.load());
      running.store(false);
    });
  };
  const auto report = [&](std::string_view message) noexcept {
    app::record_error(errors, stored, message, "fuse thread startup: ");
  };
  // Fails the actual std::thread's host state allocation, before native launch.
  ios_test::AllocationFailure failure(0, true);
  const bool started = app::start_worker(running, worker, launch, report);
  const bool failed = failure.stop();
  EXPECT_TRUE(failed);
  EXPECT_FALSE(started);
  EXPECT_FALSE(running.load());
  EXPECT_FALSE(worker.joinable());
  EXPECT_FALSE(entered.load());
  EXPECT_EQ(errors, 1u);

  ASSERT_TRUE(app::start_worker(running, worker, launch, report));
  worker.join();
  EXPECT_TRUE(entered.load());
  EXPECT_FALSE(running.load());
  EXPECT_EQ(errors, 1u);
}

TEST(FailureHandling, StoppedButJoinableWorkerIsNeverOverwritten) {
  std::atomic<bool> running{false};
  std::thread worker([] {});
  const auto previous = worker.get_id();
  bool launched = false;
  bool reported = false;
  EXPECT_FALSE(app::start_worker(
      running, worker,
      [&] {
        launched = true;
        return std::thread([] {});
      },
      [&](std::string_view) { reported = true; }));
  EXPECT_FALSE(launched);
  EXPECT_FALSE(reported);
  EXPECT_EQ(worker.get_id(), previous);
  EXPECT_TRUE(worker.joinable());
  worker.join();
}

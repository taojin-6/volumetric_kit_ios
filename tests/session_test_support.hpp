// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file session_test_support.hpp
/// @brief What the session writer's tests share: a file of each test's own,
///        the writer's options, the iPad's colour codes, a picture that says
///        which frame it is, and a file read back whole.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <unistd.h>

#include "rgbd_stream_writer.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"

namespace ios_test {

namespace app = volumetric_kit::ios_app;
namespace stream = volumetric_kit::grid::stream;

/// @brief A file of its own for each test, removed after it.
class TempFile {
 public:
  explicit TempFile(const std::string& name)
      : path_((std::filesystem::temp_directory_path() /
               ("vi_session_" + std::to_string(::getpid()) + "_" + name +
                ".vrst"))
                  .string()) {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  ~TempFile() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
  const std::string& path() const { return path_; }
  std::uintmax_t size() const { return std::filesystem::file_size(path_); }

 private:
  std::string path_;
};

/// @return A writer's options for the file at @p path, at 10 Hz.
inline app::RgbdStreamOptions options_for(const std::string& path) {
  app::RgbdStreamOptions o;
  o.path = path;
  o.fps = 10;
  o.session_id = {0x5e, 0x55, 0x10, 0x4e, 1, 2,  3,  4,
                  5,    6,    7,    8,    9, 10, 11, 12};
  o.created_ns = 1'793'492'974'605'774'900;
  o.writer = "scanner-test";
  return o;
}

/// @return The colour the coded pictures state: the iPad's, BT.709 full
///         range.
inline volumetric_kit::sensor::ColorCodes ipad_codes() {
  volumetric_kit::sensor::ColorCodes codes;
  codes.matrix = 1;
  codes.transfer = 1;
  codes.primaries = 1;
  codes.full_range = true;
  codes.chroma_location = 0;
  return codes;
}

/// @brief A coded picture for a frame: bytes that say which frame they are.
struct Picture {
  std::vector<std::uint8_t> bytes;
  app::CodedColor color;
};

/// @return Frame @p i 's picture, a key or not.
inline Picture picture(std::uint32_t i, bool key) {
  Picture p;
  p.bytes = {0, 0, 0, 1, static_cast<std::uint8_t>(key ? 0x26 : 0x02), 1};
  for (int b = 0; b < 4; ++b) p.bytes.push_back((i >> (8 * b)) & 0xff);
  p.bytes.resize(p.bytes.size() + 3 * (i % 5));  // pictures differ in size
  p.color = {p.bytes.data(), p.bytes.size(), key, ipad_codes()};
  return p;
}

/// @brief One record, its header and payload.
struct ReadRecord {
  stream::RecordInfo info;
  std::vector<std::uint8_t> payload;
};

/// @brief Every record of a file, in file order, and its session.
struct ReadFile {
  stream::Session session;
  bool finished = false;
  std::vector<ReadRecord> records;
};

/// @return The file at @p path, read whole.
inline ReadFile read_file(const std::string& path) {
  ReadFile out;
  auto reader = stream::StreamReader::open(path);
  EXPECT_TRUE(reader.ok()) << reader.status().message();
  if (!reader.ok()) return out;
  out.session = reader.value()->session();
  out.finished = reader.value()->recovery().finished;
  stream::Position pos = reader.value()->begin();
  stream::Record record;
  for (;;) {
    const volumetric_kit::core::Result<bool> more =
        reader.value()->next(&pos, &record);
    EXPECT_TRUE(more.ok()) << more.status().message();
    if (!more.ok() || !more.value()) break;
    ReadRecord r;
    r.info = record.info;
    EXPECT_TRUE(reader.value()->read(record, &r.payload).ok());
    out.records.push_back(std::move(r));
  }
  return out;
}

/// @return The writer, or null with the failure recorded.
inline std::unique_ptr<app::RgbdStreamWriter> open_or_fail(
    const app::RgbdStreamOptions& options,
    const volumetric_kit::sensor::SensorInfo& info) {
  auto writer = app::RgbdStreamWriter::open(options, info);
  EXPECT_TRUE(writer.ok()) << writer.status().message();
  return writer.ok() ? std::move(writer).value() : nullptr;
}

}  // namespace ios_test

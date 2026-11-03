// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "CaptureReport.hpp"

#include <cstddef>
#include <cstdio>

#include "SelfTest.hpp"  // percentile

namespace volumetric_kit::ios_app {
namespace {

// "p50 a p95 b (n)" of @p values at @p decimals; "- (0)" for none.
std::string spread(const std::vector<double>& values, int decimals) {
  if (values.empty()) return "- (0)";
  char text[96];
  std::snprintf(text, sizeof text, "p50 %.*f p95 %.*f (%zu)", decimals,
                percentile(values, 50), decimals, percentile(values, 95),
                values.size());
  return text;
}

// @p text as a whole number from 1 to kMaxRunMinutes, or none.
std::optional<std::uint32_t> minutes_of(const std::string& text) {
  if (text.empty() || text.size() > 4) return std::nullopt;
  std::uint32_t value = 0;
  for (const char c : text) {
    if (c < '0' || c > '9') return std::nullopt;
    value = value * 10 + static_cast<std::uint32_t>(c - '0');
  }
  if (value < 1 || value > kMaxRunMinutes) return std::nullopt;
  return value;
}

}  // namespace

const char* to_string(CaptureSource source) noexcept {
  switch (source) {
    case CaptureSource::Camera:
      return "camera";
    case CaptureSource::Pattern:
      return "pattern";
  }
  return "camera";
}

LaunchOptions launch_options(const std::vector<std::string>& arguments) {
  LaunchOptions out;
  const auto refuse = [&out](std::string why) {
    if (out.refusal.empty()) out.refusal = std::move(why);
    out.source.reset();
  };
  for (std::size_t i = 1; i < arguments.size(); ++i) {
    const std::string& flag = arguments[i];
    if (flag != "-source" && flag != "-minutes") continue;
    if (i + 1 == arguments.size()) {
      refuse(flag + " needs a value after it");
      break;
    }
    const std::string& value = arguments[++i];
    if (flag == "-source") {
      if (value == "camera") {
        if (out.refusal.empty()) out.source = CaptureSource::Camera;
      } else if (value == "pattern") {
        if (out.refusal.empty()) out.source = CaptureSource::Pattern;
      } else {
        refuse("-source takes camera or pattern, not \"" + value + "\"");
      }
    } else if (const std::optional<std::uint32_t> minutes = minutes_of(value)) {
      out.minutes = *minutes;
    } else {
      refuse("-minutes takes a whole number from 1 to " +
             std::to_string(kMaxRunMinutes) + ", not \"" + value + "\"");
    }
  }
  return out;
}

std::string capture_report_line(const CaptureReport& r) {
  char head[160];
  std::snprintf(head, sizeof head, "sa%s %s %.1f s: tracking %s%s",
                r.stop ? " stop" : "", to_string(r.source), r.seconds,
                r.tracking.empty() ? "-" : r.tracking.c_str(),
                r.interrupted ? ", interrupted" : "");
  std::string line = head;
  char counts[256];
  std::snprintf(counts, sizeof counts,
                " | received %llu delivered %llu dropped %llu failed %llu "
                "(withheld %llu refused %llu) missed %llu | fused %llu",
                static_cast<unsigned long long>(r.received),
                static_cast<unsigned long long>(r.delivered),
                static_cast<unsigned long long>(r.dropped),
                static_cast<unsigned long long>(r.failed),
                static_cast<unsigned long long>(r.withheld),
                static_cast<unsigned long long>(r.refused),
                static_cast<unsigned long long>(r.missed),
                static_cast<unsigned long long>(r.fused));
  line += counts;
  if (r.first_frame_s) {
    char first[48];
    std::snprintf(first, sizeof first, " | first frame %.2f s",
                  *r.first_frame_s);
    line += first;
  } else {
    line += " | first frame -";
  }
  line += " | poll ms " + spread(r.poll_ms, 3);
  line += " | fuse ms " + spread(r.fuse_ms, 2);
  line += ", prep " + spread(r.prep_ms, 2);
  if (r.codes_known) {
    char codes[96];
    std::snprintf(codes, sizeof codes, " | codes %d/%d/%d %s loc %d", r.matrix,
                  r.transfer, r.primaries, r.full_range ? "full" : "limited",
                  r.chroma_location);
    line += codes;
  } else {
    line += " | codes -";
  }
  line += r.breach.empty() ? " | contract kept" : " | contract: " + r.breach;
  if (r.source == CaptureSource::Pattern) {
    line += " | index " + std::to_string(r.index_checked - r.index_mismatches) +
            "/" + std::to_string(r.index_checked);
  }
  if (!r.session_error.empty()) {
    line += " | session failed: " + r.session_error;
  }
  return line;
}

}  // namespace volumetric_kit::ios_app

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

bool ends_with(const std::string& text, const std::string& end) {
  return text.size() >= end.size() &&
         text.compare(text.size() - end.size(), end.size(), end) == 0;
}

// A plain file name: no directory, nothing hidden, a session's extension.
bool record_name(const std::string& text) {
  if (text.size() <= 5 || text.size() > kMaxRecordName || text[0] == '.' ||
      !ends_with(text, ".vrst")) {
    return false;
  }
  for (const char c : text) {
    const bool word = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '.' || c == '_' ||
                      c == '-';
    if (!word) return false;
  }
  return true;
}

// What a numeric address and port can hold; the link checks the rest.
bool connect_text(const std::string& text) {
  if (text.empty() || text.size() > 21) return false;  // 255.255.255.255:65535
  for (const char c : text) {
    if (!((c >= '0' && c <= '9') || c == '.' || c == ':')) return false;
  }
  return true;
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
    if (flag == "-dry") {
      out.dry = true;
      continue;
    }
    if (flag == "-model") {
      out.mode = ScannerMode::Model;
      continue;
    }
    if (flag != "-source" && flag != "-minutes" && flag != "-record" &&
        flag != "-connect" && flag != "-fail-opens") {
      continue;
    }
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
    } else if (flag == "-record") {
      if (record_name(value)) {
        out.record = value;
      } else {
        refuse(
            "-record takes a file name ending .vrst (letters, digits, . _ "
            "-, at most " +
            std::to_string(kMaxRecordName) + "), not \"" + value + "\"");
      }
    } else if (flag == "-connect") {
      if (connect_text(value)) {
        out.connect = value;
      } else {
        refuse("-connect takes a numeric address, a.b.c.d:port, not \"" +
               value + "\"");
      }
    } else if (flag == "-fail-opens") {
      if (value.size() == 1 && value[0] >= '1' &&
          value[0] <= static_cast<char>('0' + kMaxFailOpens)) {
        out.fail_opens = static_cast<std::uint32_t>(value[0] - '0');
      } else {
        refuse("-fail-opens takes a whole number from 1 to " +
               std::to_string(kMaxFailOpens) + ", not \"" + value + "\"");
      }
    } else if (const std::optional<std::uint32_t> minutes = minutes_of(value)) {
      out.minutes = *minutes;
    } else {
      refuse("-minutes takes a whole number from 1 to " +
             std::to_string(kMaxRunMinutes) + ", not \"" + value + "\"");
    }
  }
  if (out.dry && (!out.record.empty() || !out.connect.empty())) {
    refuse(
        "-dry records and sends nothing, so it takes no -record or "
        "-connect");
  }
  // A test of the retry, never of a camera run.
  if (out.fail_opens > 0 && out.source != CaptureSource::Pattern) {
    refuse("-fail-opens is a test of the retry, with -source pattern alone");
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
  char counts[336];
  std::snprintf(counts, sizeof counts,
                " | received %llu delivered %llu dropped %llu failed %llu "
                "(withheld %llu refused %llu) missed %llu | imported %llu "
                "surfaces %llu | fused %llu skipped %llu",
                static_cast<unsigned long long>(r.received),
                static_cast<unsigned long long>(r.delivered),
                static_cast<unsigned long long>(r.dropped),
                static_cast<unsigned long long>(r.failed),
                static_cast<unsigned long long>(r.withheld),
                static_cast<unsigned long long>(r.refused),
                static_cast<unsigned long long>(r.missed),
                static_cast<unsigned long long>(r.imported),
                static_cast<unsigned long long>(r.surfaces),
                static_cast<unsigned long long>(r.fused),
                static_cast<unsigned long long>(r.skipped));
  line += counts;
  if (!r.thermal.empty()) line += " | thermal " + r.thermal;
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
  line += std::string(" | mode ") + to_string(r.mode);
  if (!r.session_error.empty()) {
    line += " | session failed: " + r.session_error;
  }
  return line;
}

std::string stream_report_line(const StreamReport& r) {
  char head[64];
  std::snprintf(head, sizeof head, "st%s %.1f s: ", r.stop ? " stop" : "",
                r.seconds);
  std::string line = head;
  if (!r.on) {
    return line + "off, " + (r.error.empty() ? "no sender" : r.error);
  }
  char counts[256];
  std::snprintf(counts, sizeof counts,
                "%s | taken %llu busy %llu coded %llu (keys %llu) failed %llu "
                "written %llu refused %llu | map steps %llu",
                r.session ? "session" : "dry",
                static_cast<unsigned long long>(r.taken),
                static_cast<unsigned long long>(r.busy),
                static_cast<unsigned long long>(r.coded),
                static_cast<unsigned long long>(r.keys),
                static_cast<unsigned long long>(r.failed),
                static_cast<unsigned long long>(r.written),
                static_cast<unsigned long long>(r.refused),
                static_cast<unsigned long long>(r.map_steps));
  line += counts;
  char rates[96];
  std::snprintf(rates, sizeof rates, " | video %.2f depth %.2f Mbit/s",
                r.video_bits_per_s * 1e-6, r.depth_bits_per_s * 1e-6);
  line += rates;
  line += " | send ms " + spread(r.send_ms, 2);
  line += " | coded ms " + spread(r.coded_ms, 2);
  if (r.session) {
    line += " | viewers " + std::to_string(r.viewers) + " dial failures " +
            std::to_string(r.dial_failures);
  }
  if (!r.error.empty()) line += " | last failure: " + r.error;
  return line;
}

std::string without_path(std::string text, const std::string& path) {
  if (path.empty()) return text;
  static const std::string kSaid = "the session file";
  for (std::size_t at = text.find(path); at != std::string::npos;
       at = text.find(path, at + kSaid.size())) {
    text.replace(at, path.size(), kSaid);
  }
  return text;
}

}  // namespace volumetric_kit::ios_app

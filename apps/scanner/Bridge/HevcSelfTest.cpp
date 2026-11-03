// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "HevcSelfTest.hpp"

#include <CoreVideo/CoreVideo.h>
#include <sys/sysctl.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "SelfTest.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/testing/hevc_conformance.hpp"
#include "volumetric_kit/sensor/testing/nv12_pattern.hpp"
#include "volumetric_kit/sensor/testing/nv12_pixel_buffer.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder.hpp"
#include "volumetric_kit/sensor/video/hevc_encoder_apple.hpp"

namespace volumetric_kit::ios_app {
namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace testing = volumetric_kit::sensor::testing;
using Clock = std::chrono::steady_clock;
using Line = std::function<void(const std::string&)>;

// ARKit's capturedImage size, the picture the stream sends.
constexpr std::uint32_t kWidth = 1920;
constexpr std::uint32_t kHeight = 1440;
constexpr int kQp = 22;
// The timing run: pictures before the timed ones, pictures timed, a key
// every this many (2 s at 30 Hz), and distinct pictures sent in turn.
constexpr std::uint32_t kWarmUp = 10;
constexpr std::uint32_t kTimed = 300;
constexpr std::uint32_t kKeyEvery = 60;
constexpr std::uint32_t kRing = 8;

// What an ARKit picture with no colour tags is (ARFrame.h).
constexpr sensor::ColorCodes kBt601Full{6, 1, 6, true, 0};

struct ReleasePixels {
  void operator()(CVPixelBufferRef p) const { CVPixelBufferRelease(p); }
};
using Pixels =
    std::unique_ptr<std::remove_pointer_t<CVPixelBufferRef>, ReleasePixels>;

std::string said(const vkc::Status& s) {
  return std::string(vkc::to_string(s.domain())) + ": " + s.message();
}

double ms(Clock::duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

std::string sysctl_text(const char* name) {
  char text[128] = {};
  std::size_t size = sizeof text - 1;
  if (sysctlbyname(name, text, &size, nullptr, 0) != 0) return "unknown";
  return text;
}

std::string codes_text(const sensor::ColorCodes& c) {
  return "matrix " + std::to_string(c.matrix) + " transfer " +
         std::to_string(c.transfer) + " primaries " +
         std::to_string(c.primaries) + (c.full_range ? " full" : " video") +
         " siting " + std::to_string(c.chroma_location);
}

sensor::HevcEncoder::Options options(const sensor::ColorCodes& color,
                                     int qp = kQp,
                                     std::optional<int> key_qp = {}) {
  sensor::HevcEncoder::Options o;
  o.width = kWidth;
  o.height = kHeight;
  o.qp = qp;
  o.key_qp = key_qp;
  o.color = color;
  return o;
}

struct Case {
  std::string name;
  sensor::HevcEncoder::Options options;
  std::uint32_t pictures = 30;
};

// The colours a camera hands over, a stated siting, the QP ends at both
// picture kinds, then every code in the encoder's set (normalization
// included) and every siting it writes, each a short run.
std::vector<Case> cases() {
  const std::string size =
      std::to_string(kWidth) + "x" + std::to_string(kHeight) + " ";
  std::vector<Case> all = {
      {size + "BT.601 full, QP 22", options(kBt601Full)},
      {size + "BT.601 video, QP 22", options({6, 1, 6, false, 0})},
      {size + "BT.709 full centre-sited, QP 22", options({1, 1, 1, true, 1})},
      {size + "BT.601 full, QP 1, key QP 51", options(kBt601Full, 1, 51)},
      {size + "BT.601 full, QP 51, key QP 1", options(kBt601Full, 51, 1)},
  };
  std::vector<sensor::ColorCodes> set;
  for (const int m : {1, 5, 6, 7, 9}) set.push_back({m, 1, 1, true, 0});
  for (const int t : {2, 6, 7, 8, 13, 14, 15, 16, 17, 18}) {
    set.push_back({1, t, 1, true, 0});  // transfer 1 is the matrices'
  }
  for (const int p : {1, 2, 5, 6, 9, 11, 12, 22}) {
    set.push_back({1, 1, p, false, 0});
  }
  for (int s = 1; s <= 5; ++s) set.push_back({1, 1, 1, true, s});
  for (const sensor::ColorCodes& c : set) {
    all.push_back({size + codes_text(c) + ", 4 pictures", options(c), 4});
  }
  return all;
}

// One run of the rules on @p c, decoded on @p device when it is not null.
RuleCase run_rules(const Case& c, const vkc::Device* device,
                   vkc::Allocator* allocator) {
  testing::HevcEncoderCase run;
  run.options = c.options;
  run.device = device;
  run.allocator = allocator;
  run.pixel_buffers = true;
  run.pictures = c.pictures;
  RuleCase out;
  out.name = c.name;
  const auto start = Clock::now();
  const auto broken = testing::hevc_encoder_breach(run);
  out.seconds = ms(Clock::now() - start) / 1000.0;
  if (!broken) {
    // Not run only where the session itself does not open; any other
    // status (the decoder's, a send's or a receive's) is a failure.
    const auto opens = sensor::HevcEncoder::create(c.options);
    out.result = opens ? RuleCase::Result::Failed : RuleCase::Result::NotRun;
    out.detail = said(opens ? broken.status() : opens.status());
  } else if (broken->empty()) {
    out.result = RuleCase::Result::Kept;
  } else {
    out.result = RuleCase::Result::Broken;
    out.detail = *broken;
  }
  return out;
}

// @p c's case, and where the decode may be what failed, the same case again
// without it, so the encoder's own rules are answered either way.
RuleCase run_case(const Case& c, const vkc::Device* device,
                  vkc::Allocator* allocator) {
  RuleCase out = run_rules(c, device, allocator);
  if (device != nullptr && rerun_without_decode(out)) {
    const RuleCase alone = run_rules(c, nullptr, nullptr);
    out.without_decode = alone.result;
    out.without_decode_detail = alone.detail;
  }
  return out;
}

// One session, one picture in flight: each picture sent, then received
// before the next, so send to access unit bounds the encode's own time from
// above (it includes the hand-off to the coding thread and the receive's
// wake-up).
EncodeTiming time_encoder() {
  EncodeTiming t;
  t.what = std::to_string(kWidth) + "x" + std::to_string(kHeight) +
           " BT.601 full, QP 22, one picture in flight";
  auto made = sensor::HevcEncoder::create(options(kBt601Full));
  if (!made) {
    t.failure = said(made.status());
    return t;
  }
  sensor::HevcEncoder& encoder = *made;
  std::vector<Pixels> ring;
  for (std::uint32_t i = 0; i < kRing; ++i) {
    auto pixels = testing::pixel_buffer(
        testing::test_pattern(kWidth, kHeight, i), kBt601Full);
    if (!pixels) {
      t.failure = said(pixels.status());
      return t;
    }
    ring.emplace_back(*pixels);
  }
  t.send_ms.reserve(kTimed);
  t.to_unit_ms.reserve(kTimed);
  t.bytes.reserve(kTimed);
  const auto fail = [&t](std::string why) {
    if (t.failure.empty()) t.failure = std::move(why);
  };
  sensor::HevcEncoder::CodedPicture coded;
  for (std::uint32_t i = 0; i < kWarmUp + kTimed; ++i) {
    const bool timed = i >= kWarmUp;
    const bool key = i % kKeyEvery == 0 || encoder.key_owed();
    const std::int64_t pts = std::int64_t{i} * 1'000'000'000 / 30;
    const auto start = Clock::now();
    const vkc::Status sent =
        sensor::send_pixel_buffer(encoder, ring[i % kRing].get(), key, pts);
    const auto returned = Clock::now();
    if (timed) ++t.sent;
    if (!sent.ok()) {
      fail("send " + std::to_string(i) + ": " + said(sent));
      continue;
    }
    const auto got = encoder.receive(coded);
    const auto back = Clock::now();
    if (!got || !*got) {
      fail("picture " + std::to_string(i) + ": " +
           (got ? std::string("no access unit") : said(got.status())));
      continue;
    }
    if (!timed) continue;
    ++t.coded;
    if (coded.key) ++t.keys;
    t.send_ms.push_back(ms(returned - start));
    t.to_unit_ms.push_back(ms(back - start));
    t.bytes.push_back(coded.access_unit.size());
  }
  return t;
}

}  // namespace

bool run_hevc_self_test(const Line& line,
                        const std::function<std::string()>& thermal) {
  line(
      "self-test hevc: sensor's HEVC encoder rules on synthetic 1920x1440 "
      "pixel buffers; no camera, no file, no network");
  line("device: " + sysctl_text("hw.machine") + ", OS " +
       sysctl_text("kern.osproductversion"));
  line("thermal: " + thermal());

  // The GPU the decode check (C6) runs on, with what sensor's decoder needs.
  // Declared in order, so they go in reverse.
  std::optional<vkc::Instance> instance;
  std::optional<vkc::Device> device;
  std::optional<vkc::Allocator> allocator;
  const vkc::Status gpu = [&]() -> vkc::Status {
    vkc::InstanceConfig config;
#ifdef NDEBUG
    config.request_debug_utils = false;
#endif
    VKC_ASSIGN(vkc::Instance made, vkc::Instance::create(config));
    instance.emplace(std::move(made));
    const vkc::DeviceRequirements reqs = sensor::device_requirements();
    VKC_ASSIGN(const vkc::PhysicalDeviceInfo physical,
               instance->select_physical_device(reqs));
    VKC_ASSIGN(vkc::Device made_device,
               vkc::Device::create(*instance, physical, reqs));
    device.emplace(std::move(made_device));
    VKC_ASSIGN(vkc::Allocator made_allocator,
               vkc::Allocator::create(instance->handle(), *device));
    allocator.emplace(std::move(made_allocator));
    line(std::string("GPU: ") + physical.properties().deviceName +
         ", VK_EXT_metal_objects " +
         (device->extension_enabled("VK_EXT_metal_objects") ? "on" : "off"));
    return vkc::Status{};
  }();
  // Without it the cases still run, so the encoder's own rules are answered,
  // and the verdict fails.
  const bool decoded = gpu.ok();
  if (!decoded) {
    allocator.reset();
    device.reset();
    line("GPU: none, so no case decodes (C6 left out): " + said(gpu));
  }

  // The session's own facts: its encoder, and the configuration it reports,
  // which says whether it takes a QP a picture. Or why it does not open.
  if (auto probe = sensor::HevcEncoder::create(options(kBt601Full))) {
    line("encoder: " + probe->facts().encoder);
    line("settings: " + probe->facts().settings);
  } else {
    line("encoder: does not open: " + said(probe.status()));
  }

  std::vector<RuleCase> results;
  for (Case c : cases()) {
    if (!decoded) c.name += ", no decode";
    results.push_back(run_case(c, device ? &*device : nullptr,
                               allocator ? &*allocator : nullptr));
    line(rule_line(results.back()));
  }

  const EncodeTiming timing = time_encoder();
  for (const std::string& l : timing_lines(timing)) line(l);
  line("thermal at the end: " + thermal());
  line(verdict_line(results, timing, decoded));
  return self_test_passed(results, timing, decoded);
}

}  // namespace volumetric_kit::ios_app

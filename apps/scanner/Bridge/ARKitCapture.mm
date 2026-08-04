// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#import "ARKitCapture.h"

#import <Accelerate/Accelerate.h>
#import <CoreVideo/CoreVideo.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"
#include "volumetric_kit/recon/sensor/color_conventions.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// ARKit's near/far working range for scene depth. Outside it the LiDAR return
// is unreliable, and recon skips samples outside the camera's range anyway.
constexpr float kMinDepth = 0.15f;
constexpr float kMaxDepth = 5.0f;

// Keep only ARConfidenceLevelMedium and above. ARKit's low-confidence samples
// are frequently metres wrong at depth discontinuities, and a TSDF carves free
// space along every ray it believes -- so a bad sample does lasting damage to
// the volume, not just to one frame.
constexpr uint8_t kMinConfidence = ARConfidenceLevelMedium;

/// Holds a CVPixelBuffer's base-address lock for a scope.
///
/// The lock can fail, and when it does `CVPixelBufferGetBaseAddress` returns
/// null -- so the result has to be checked before any pixel is touched, and the
/// unlock has to be skipped on the path that never locked. Both are easy to get
/// wrong across the early returns in the converters below, so they are a type
/// rather than a rule.
class PixelBufferLock {
 public:
  explicit PixelBufferLock(CVPixelBufferRef _Nullable buffer)
      : buffer_(buffer) {
    locked_ = buffer != nullptr &&
              CVPixelBufferLockBaseAddress(
                  buffer, kCVPixelBufferLock_ReadOnly) == kCVReturnSuccess;
  }

  ~PixelBufferLock() {
    if (locked_) {
      CVPixelBufferUnlockBaseAddress(buffer_, kCVPixelBufferLock_ReadOnly);
    }
  }

  PixelBufferLock(const PixelBufferLock&) = delete;
  PixelBufferLock& operator=(const PixelBufferLock&) = delete;

  bool locked() const { return locked_; }

 private:
  CVPixelBufferRef _Nullable buffer_;
  bool locked_ = false;
};

/// One frame's converted pixels. Owned, because ARKit recycles its
/// CVPixelBuffers as soon as the delegate returns, while `CapturedFrame` is a
/// non-owning view that must stay valid until the *next* poll.
struct FrameBuffers {
  std::vector<float> depth;
  std::vector<std::uint32_t> color;
  vr::DepthCameraParams depth_camera{};
  vr::ColorCameraParams color_camera{};
  /// What the frame carries once staged: always canonical, because
  /// `convert_color` brings a non-canonical source across before publishing.
  vr::ColorEncoding color_encoding{};
  /// What the buffer declared, for the read-out -- kept as the strings rather
  /// than as a `ColorEncoding`, because a tag this driver cannot represent has
  /// no enumerator to name it and is exactly the case worth showing. Retained
  /// when a frame's colour is refused, so the read-out names the culprit
  /// instead of merely losing colour.
  const char* color_matrix = "(none)";
  const char* color_transfer = "(none)";
  const char* color_primaries = "(none)";
  /// `true` when a `to_canonical` pass ran, i.e. the declaration was not
  /// already canonical.
  bool color_converted = false;
  /// `true` when the declaration could not be honoured and the colour was
  /// dropped for it.
  bool color_refused = false;
  float color_convert_ms = 0.0f;
  std::uint64_t timestamp_ns = 0;
  bool has_color = false;
  float confidence_kept = 0.0f;
};

/// Copy a `DepthFloat32` plane out row by row.
///
/// A CVPixelBuffer's rows are padded to an alignment, so `bytesPerRow` is
/// generally NOT `width * sizeof(float)`. Treating the base address as tightly
/// packed is the classic mistake here: it does not crash, it shears the depth
/// image diagonally, which reads as a tracking failure rather than a bug.
///
/// Confidence gating happens in the same pass: a sample below the threshold
/// becomes 0, which the fusion kernels already skip -- the mechanism
/// `CapturedFrame::depth` documents for forwarding a confidence mask.
///
/// @return The fraction of samples kept, in [0, 1]; or nullopt if the map could
///         not be read -- an unlockable buffer, or a format that is not
///         `DepthFloat32`, which reinterpreted as `float` would produce
///         plausible garbage rather than an error.
std::optional<float> copy_depth(CVPixelBufferRef depth_buffer,
                                CVPixelBufferRef _Nullable confidence_buffer,
                                std::vector<float>& out) {
  if (CVPixelBufferGetPixelFormatType(depth_buffer) !=
      kCVPixelFormatType_DepthFloat32) {
    return std::nullopt;
  }
  const std::size_t width = CVPixelBufferGetWidth(depth_buffer);
  const std::size_t height = CVPixelBufferGetHeight(depth_buffer);
  if (width == 0 || height == 0) {
    return std::nullopt;
  }

  const PixelBufferLock depth_lock(depth_buffer);
  if (!depth_lock.locked()) {
    return std::nullopt;
  }
  const auto* src = static_cast<const std::uint8_t*>(
      CVPixelBufferGetBaseAddress(depth_buffer));
  if (src == nullptr) {
    return std::nullopt;
  }
  const std::size_t src_stride = CVPixelBufferGetBytesPerRow(depth_buffer);

  // Gate on confidence only when the map covers the same grid. `ARDepthData`
  // documents one confidence value per depth value but does not guarantee the
  // sizes in the type, and a smaller map would be read past its end -- for the
  // sake of a check that costs two comparisons a frame.
  //
  // The format is checked for the same reason the depth buffer's is: the loop
  // below indexes `conf_row` as bytes, so a map that was not
  // `OneComponent8` would be read at the wrong stride and gate on values that
  // are not confidence levels at all.
  const bool confidence_matches =
      confidence_buffer != nullptr &&
      CVPixelBufferGetPixelFormatType(confidence_buffer) ==
          kCVPixelFormatType_OneComponent8 &&
      CVPixelBufferGetWidth(confidence_buffer) == width &&
      CVPixelBufferGetHeight(confidence_buffer) == height;
  const PixelBufferLock confidence_lock(confidence_matches ? confidence_buffer
                                                           : nullptr);

  const std::uint8_t* conf = nullptr;
  std::size_t conf_stride = 0;
  // Committed to gating the moment the map matched, so a failure to actually
  // read it refuses the frame rather than falling through to an ungated copy.
  // Failing open here is the one failure mode nothing downstream can see:
  // `conf_row` would be null, `trusted` unconditionally true, and every
  // ARConfidenceLevelLow sample -- the ones this file's header calls
  // "frequently metres wrong at depth discontinuities" -- would carve free
  // space into the volume permanently, while `confidence_kept` still reported
  // 100% because `kept` counts exactly the samples that were let through.
  // `CVPixelBufferGetBaseAddress` returns NULL for IOSurface-backed buffers
  // even after a successful lock, so the null check is not theoretical.
  if (confidence_matches) {
    if (!confidence_lock.locked()) {
      return std::nullopt;
    }
    conf = static_cast<const std::uint8_t*>(
        CVPixelBufferGetBaseAddress(confidence_buffer));
    if (conf == nullptr) {
      return std::nullopt;
    }
    conf_stride = CVPixelBufferGetBytesPerRow(confidence_buffer);
  }

  out.resize(width * height);
  std::size_t kept = 0;
  for (std::size_t y = 0; y < height; ++y) {
    const auto* row = reinterpret_cast<const float*>(src + y * src_stride);
    const std::uint8_t* conf_row =
        conf != nullptr ? conf + y * conf_stride : nullptr;
    float* dst = out.data() + y * width;
    for (std::size_t x = 0; x < width; ++x) {
      const bool trusted = conf_row == nullptr || conf_row[x] >= kMinConfidence;
      const float d = row[x];
      // NaN and non-finite returns exist in the raw map; reject them here
      // rather than letting them poison a voxel's running average.
      const bool usable = trusted && std::isfinite(d) && d > 0.0f;
      dst[x] = usable ? d : 0.0f;
      kept += usable ? 1u : 0u;
    }
  }

  return static_cast<float>(kept) / static_cast<float>(width * height);
}

/// What a CVPixelBuffer says about its own colour: the matrix needed to get
/// R'G'B' out of its YCbCr planes, and what those R'G'B' values then *are*.
///
/// The two are independent, and conflating them is a silent error in its own
/// right -- the matrix reconstructs chroma, the transfer and primaries describe
/// the result. A frame can carry a BT.601 matrix and BT.709 primaries at once,
/// and iOS ones routinely do.
struct SourceColor {
  const vImage_YpCbCrToARGBMatrix* matrix;
  const char* matrix_name;
  const char* transfer_name;
  const char* primaries_name;
  vr::ColorEncoding encoding;
  /// `false` when an attachment named something this driver cannot represent,
  /// so the frame's colour must be dropped rather than declared.
  bool understood;
};

/// Read @p image's colour attachments rather than assuming them.
///
/// Assuming is what this replaces: the matrix was pinned to BT.601 and the
/// encoding was never stated at all, so a device tagging its capture BT.709 had
/// its chroma reconstructed through the wrong matrix, and a wide-gamut one had
/// Display P3 values fused as though they were BT.709 -- an oversaturation with
/// no error attached to it.
///
/// Every branch is explicit and the fall-through is a *refusal*, not the
/// default. A value with no enumerator -- an HLG transfer, a BT.2020 matrix
/// vImage ships no conversion for -- would otherwise land on `ColorEncoding{}`,
/// which `is_canonical` accepts, so the frame would be fused through the wrong
/// curve and reported on screen as observed. That is this file's own failure
/// mode one level down, and `to_canonical` already sets the precedent by
/// refusing PQ rather than approximating it.
///
/// Falls back to the canonical declaration only when an attachment is
/// *missing*, which is what an untagged buffer most likely is, and what the
/// previous code assumed unconditionally.
SourceColor source_color(CVPixelBufferRef image) {
  // The fallback is BT.709 where the old code assumed BT.601 unconditionally,
  // which is a real change on this path -- so it is named "assumed" and shows
  // up that way on screen. An ARKit buffer is tagged in practice, making this
  // nearly dead code; a silently different guess in nearly dead code is exactly
  // the kind of thing that surfaces once, years later, on one device.
  SourceColor out{kvImage_YpCbCrToARGBMatrix_ITU_R_709_2,
                  "BT.709 (assumed)",
                  "sRGB (assumed)",
                  "BT.709 (assumed)",
                  vr::ColorEncoding{},
                  true};

  const auto matches = [](CFTypeRef value, CFStringRef expected) {
    return value != nullptr && CFGetTypeID(value) == CFStringGetTypeID() &&
           CFStringCompare(static_cast<CFStringRef>(value), expected, 0) ==
               kCFCompareEqualTo;
  };
  // CFTypeRef, released on scope exit: CVBufferCopyAttachment returns +1.
  const auto attachment = [image](CFStringRef key) {
    return CVBufferCopyAttachment(image, key, nullptr);
  };

  if (CFTypeRef matrix = attachment(kCVImageBufferYCbCrMatrixKey)) {
    if (matches(matrix, kCVImageBufferYCbCrMatrix_ITU_R_601_4)) {
      out.matrix = kvImage_YpCbCrToARGBMatrix_ITU_R_601_4;
      out.matrix_name = "BT.601";
    } else if (matches(matrix, kCVImageBufferYCbCrMatrix_ITU_R_709_2)) {
      out.matrix_name = "BT.709";
    } else {
      // Accelerate ships conversions for these two matrices and no others, so
      // there is no correct value to fall back to. Reconstructing BT.2020 or
      // SMPTE-240M chroma through the 709 3x3 is a hue error, and one that used
      // to print as a flat "BT.709" -- a wrong reading is worse than a missing
      // one precisely because this line exists to be trusted.
      out.matrix_name = matches(matrix, kCVImageBufferYCbCrMatrix_ITU_R_2020)
                            ? "BT.2020 (unsupported)"
                            : "unrecognized";
      out.understood = false;
    }
    CFRelease(matrix);
  }

  if (CFTypeRef primaries = attachment(kCVImageBufferColorPrimariesKey)) {
    if (matches(primaries, kCVImageBufferColorPrimaries_ITU_R_709_2)) {
      out.primaries_name = "BT.709";
    } else if (matches(primaries, kCVImageBufferColorPrimaries_P3_D65)) {
      out.encoding.primaries = vr::ColorEncoding::Primaries::DisplayP3;
      out.primaries_name = "Display P3";
    } else if (matches(primaries, kCVImageBufferColorPrimaries_ITU_R_2020)) {
      out.encoding.primaries = vr::ColorEncoding::Primaries::Bt2020;
      out.primaries_name = "BT.2020";
    } else {
      // SMPTE-C, EBU 3213, P22, DCI-P3: `Primaries` has no enumerator for any
      // of them, so `primaries_to_working` has no basis to rotate them from and
      // calling them BT.709 would be a claim rather than a reading.
      out.primaries_name = "unrecognized";
      out.understood = false;
    }
    CFRelease(primaries);
  }

  if (CFTypeRef transfer = attachment(kCVImageBufferTransferFunctionKey)) {
    // BT.709 and sRGB are both canonical, and recon accepts them as one: they
    // differ by a couple of codes in the toe, a bounded and stated error.
    if (matches(transfer, kCVImageBufferTransferFunction_ITU_R_709_2)) {
      out.encoding.transfer = vr::ColorEncoding::Transfer::Bt709;
      out.transfer_name = "BT.709";
    } else if (matches(transfer, kCVImageBufferTransferFunction_sRGB)) {
      out.encoding.transfer = vr::ColorEncoding::Transfer::Srgb;
      out.transfer_name = "sRGB";
    } else if (matches(transfer, kCVImageBufferTransferFunction_Linear)) {
      out.encoding.transfer = vr::ColorEncoding::Transfer::Linear;
      out.transfer_name = "linear";
    } else if (matches(transfer,
                       kCVImageBufferTransferFunction_SMPTE_ST_2084_PQ)) {
      // Declared rather than refused here, so that `to_canonical` stays the one
      // place deciding what a transfer can become; it reports PQ as unsupported
      // rather than tone-mapping it into something quietly wrong.
      out.encoding.transfer = vr::ColorEncoding::Transfer::Bt2020Pq;
      out.transfer_name = "BT.2020 PQ";
    } else if (matches(transfer,
                       kCVImageBufferTransferFunction_ITU_R_2100_HLG)) {
      // PQ's sibling, and the case this used to get silently wrong: `Transfer`
      // has no HLG enumerator, so the old fall-through declared it sRGB --
      // canonical, no conversion, no error, an HDR curve decoded as a display
      // one. Named rather than lumped in with "unrecognized" because it is the
      // reachable one: ARKit 6 captures HDR video on the iOS 16 floor this app
      // already builds against.
      out.transfer_name = "BT.2100 HLG (unsupported)";
      out.understood = false;
    } else {
      // SMPTE-240M, ST 428-1, UseGamma, EBU 3213.
      out.transfer_name = "unrecognized";
      out.understood = false;
    }
    CFRelease(transfer);
  }
  return out;
}

/// The vImage conversion for @p matrix, built once per matrix.
///
/// Both are built on first use rather than one being chosen at compile time,
/// because the matrix now comes from the buffer: generating a conversion costs
/// more than applying it, so it cannot be done per frame, and a session that
/// switched formats must not silently keep the first one.
///
/// The range is full ("f") 420 bi-planar, which is what ARKit delivers; the
/// video-range matrix against full-range data would clip highlights.
const vImage_YpCbCrToARGB* conversion_for(
    const vImage_YpCbCrToARGBMatrix* matrix) {
  struct Conversions {
    vImage_YpCbCrToARGB bt601{};
    vImage_YpCbCrToARGB bt709{};
    bool bt601_ok = false;
    bool bt709_ok = false;
  };
  static const Conversions built = [] {
    Conversions c;
    vImage_YpCbCrPixelRange range{0, 128, 255, 255, 255, 1, 255, 0};
    c.bt601_ok = vImageConvert_YpCbCrToARGB_GenerateConversion(
                     kvImage_YpCbCrToARGBMatrix_ITU_R_601_4, &range, &c.bt601,
                     kvImage420Yp8_CbCr8, kvImageARGB8888,
                     kvImageNoFlags) == kvImageNoError;
    c.bt709_ok = vImageConvert_YpCbCrToARGB_GenerateConversion(
                     kvImage_YpCbCrToARGBMatrix_ITU_R_709_2, &range, &c.bt709,
                     kvImage420Yp8_CbCr8, kvImageARGB8888,
                     kvImageNoFlags) == kvImageNoError;
    return c;
  }();
  // Tracked per matrix rather than behind one flag: an all-or-nothing gate lets
  // a failure to build either conversion disable the other one that built fine,
  // dropping colour entirely on a device where half the path still worked.
  if (matrix == kvImage_YpCbCrToARGBMatrix_ITU_R_601_4) {
    return built.bt601_ok ? &built.bt601 : nullptr;
  }
  return built.bt709_ok ? &built.bt709 : nullptr;
}

/// Convert ARKit's bi-planar full-range YCbCr `capturedImage` to the packed-RGB
/// `uint32` the tsdf/mesh tiers use, in the canonical colour encoding.
///
/// vImage rather than a hand-rolled loop: this is 2.7 M pixels per frame at
/// 1920x1440, so a scalar YCbCr->RGB would dominate the capture path.
/// Accelerate is a system framework, so it costs no dependency.
///
/// @param expected_width   The size the frame will advertise in its
/// @param expected_height  `ColorCameraParams`, which comes from
///                         `ARCamera.imageResolution` while the buffer is sized
///                         from the luma plane. A consumer reads
///                         `color_camera.width * height` pixels, so if the two
///                         ever disagreed it would read past the end of this
///                         vector. They match on every device this runs on;
///                         checking makes that an assumption the code states
///                         rather than one it silently depends on.
/// @param out              Receives the pixels and everything the read-out
///                         reports about them. Its declaration strings are
///                         cleared on entry and filled the moment the
///                         attachments are read, so a refused frame still names
///                         why, while one that failed before any attachment was
///                         read reads `"(none)"` rather than carrying a stale
///                         declaration from whichever frame last used this
///                         rotating buffer.
/// @return `true` if @p out holds a full frame of packed RGB.
bool convert_color(CVPixelBufferRef image, std::size_t expected_width,
                   std::size_t expected_height, FrameBuffers& out) {
  // Cleared up front, all of it: these buffers rotate, so anything not written
  // on an early return is not empty, it is whatever the frame three frames ago
  // left behind. `CapturedFrame` says the encoding is meaningful only when
  // colour is set, but a stale non-canonical declaration surviving next to a
  // null colour pointer is the kind of thing that stays harmless only until a
  // consumer checks the declaration first.
  out.color_encoding = vr::ColorEncoding{};
  out.color_matrix = "(none)";
  out.color_transfer = "(none)";
  out.color_primaries = "(none)";
  out.color_converted = false;
  out.color_refused = false;

  if (CVPixelBufferGetPlaneCount(image) < 2) {
    return false;
  }
  // The conversion is generated for 8-bit *full-range* 420 bi-planar and
  // nothing else (see `conversion_for`), so the buffer's format has to be
  // checked rather than assumed from its plane count. Two reachable formats
  // carry two planes and would otherwise sail straight through the gate above:
  //
  //   - `420YpCbCr8BiPlanarVideoRange` -- luma 16-235 expanded as 0-255,
  //     crushing blacks and clipping highlights into the volume permanently.
  //   - `420YpCbCr10BiPlanarFullRange` -- what ARKit delivers once
  //     `videoHDRAllowed` is set, read as 8-bit and so simply scrambled.
  //
  // Neither errors; both fuse plausible garbage while the read-out reports a
  // clean canonical frame. `copy_depth` checks its own buffer's format for
  // exactly this reason, and says so in the same words.
  //
  // Refused rather than silently dropped, so an unexpected format shows up on
  // the read-out as a named refusal instead of colour going quietly missing.
  if (CVPixelBufferGetPixelFormatType(image) !=
      kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) {
    out.color_matrix = "unsupported pixel format";
    out.color_refused = true;
    return false;
  }
  // Plane geometry does not need the base-address lock; only the pixels do.
  const std::size_t width = CVPixelBufferGetWidthOfPlane(image, 0);
  const std::size_t height = CVPixelBufferGetHeightOfPlane(image, 0);
  if (width != expected_width || height != expected_height || width == 0 ||
      height == 0) {
    return false;
  }

  const SourceColor source = source_color(image);
  out.color_matrix = source.matrix_name;
  out.color_transfer = source.transfer_name;
  out.color_primaries = source.primaries_name;
  if (!source.understood) {
    // Refused ahead of the conversion rather than after it: these pixels would
    // be thrown away either way, and there is no reason to spend a pass over
    // 2.7 M of them first.
    out.color_refused = true;
    return false;
  }
  const vImage_YpCbCrToARGB* info = conversion_for(source.matrix);
  if (info == nullptr) {
    return false;
  }

  out.color.resize(width * height);
  vImage_Buffer argb{out.color.data(), height, width, width * 4};

  {
    // Scoped: nothing after this reads the CVPixelBuffer, and ARKit wants its
    // buffers back, so the read lock is dropped as soon as the read ends.
    const PixelBufferLock lock(image);
    if (!lock.locked()) {
      return false;
    }
    void* luma_base = CVPixelBufferGetBaseAddressOfPlane(image, 0);
    void* chroma_base = CVPixelBufferGetBaseAddressOfPlane(image, 1);
    if (luma_base == nullptr || chroma_base == nullptr) {
      return false;
    }

    vImage_Buffer luma{luma_base, height, width,
                       CVPixelBufferGetBytesPerRowOfPlane(image, 0)};
    vImage_Buffer chroma{chroma_base, CVPixelBufferGetHeightOfPlane(image, 1),
                         CVPixelBufferGetWidthOfPlane(image, 1),
                         CVPixelBufferGetBytesPerRowOfPlane(image, 1)};

    // vImage writes A,R,G,B in memory order; recon reads RGB from the low three
    // bytes of a little-endian uint32, i.e. R,G,B,A. That is the permutation
    // [1, 2, 3, 0] -- passed *here* rather than run afterwards as a second
    // vImagePermuteChannels_ARGB8888 pass over 2.7 M pixels.
    //
    // Measured, because moving less memory and being faster are different
    // claims and the first does not imply the second. Alternating the two
    // strategies frame by frame on one scene, n=783 each on an iPad Pro M5:
    //
    //   folded    0.224 ms  (0.177 - 0.542)
    //   separate  0.342 ms  (0.271 - 0.873)
    //
    // 0.118 ms a frame, and it checks out against the hardware rather than
    // just against expectation: the pass skipped is 11 MB read plus 11 MB
    // written, so 22 MB in 0.118 ms is ~186 GB/s -- unified-memory bandwidth,
    // which is what a pure copy should be bound by.
    const std::uint8_t permute[4] = {1, 2, 3, 0};
    if (vImageConvert_420Yp8_CbCr8ToARGB8888(&luma, &chroma, &argb, info,
                                             permute, 255, kvImageNoFlags) !=
        kvImageNoError) {
      return false;
    }
  }

  // The bytes are now full-range R'G'B' in whatever the buffer declared, which
  // is the form the capture contract wants -- the YCbCr matrix and the range
  // expansion are the driver's job, which is why ColorEncoding carries no
  // `Range` member to describe them.
  //
  // Canonical is the overwhelmingly common case here (ARKit's own declaration),
  // and `to_canonical` would walk it verbatim anyway; skipping the call keeps
  // the fast path free of a function that would touch 11 MB to change nothing
  // but the alpha byte vImage already wrote as 255.
  out.color_encoding = source.encoding;
  if (vr::is_canonical(out.color_encoding)) {
    return true;
  }
  // Wide gamut, most likely: converting in place, since the source is our own
  // buffer and to_canonical documents exact aliasing as supported.
  //
  // The cost here is *booked, not measured*, and it is the one claim in this
  // file standing on arithmetic rather than a stopwatch -- no device in hand
  // reaches the path. `to_canonical`'s non-canonical branch is scalar and runs
  // srgb_to_linear then linear_to_srgb per channel, so 1920x1440 is ~16.6 M
  // std::pow calls on the session queue: orders above the 0.224 ms the vImage
  // pass costs, and easily enough to turn a 60 Hz capture into a drop loop.
  // Correct, in other words, but very likely not usable. @ref color_convert_ms
  // covers this pass precisely so the first real wide-gamut device says so
  // plainly instead of presenting as a mysterious frame-rate collapse. The fix,
  // if that day comes, is to fold the primaries 3x3 into the vImage matrix
  // rather than walking pixels on the CPU -- only the transfer decode genuinely
  // needs the curve, and ARKit's is already canonical.
  const vr::Status brought = sensor::to_canonical(
      out.color.data(), out.color.size(), out.color_encoding, out.color.data());
  if (!brought) {
    // PQ lands here, reported by `to_canonical` rather than judged above, and a
    // frame we cannot bring across is dropped rather than declared canonical --
    // fusing P3 values through the BT.709 basis is precisely the silent
    // oversaturation the declaration exists to prevent.
    out.color_refused = true;
    return false;
  }
  out.color_encoding = vr::ColorEncoding{};
  out.color_converted = true;
  return true;
}

/// The stats a session starts from, and returns to on @ref ARKitCapture::reset.
///
/// One function rather than two spellings of the same thing, because `{}` is
/// *wrong* here and silently so: it zero-initializes, the three `const char*`
/// members import into Swift as implicitly-unwrapped pointers, and
/// `String(cString:)` on a null traps. The read-out runs from the first
/// display-link tick -- before any ARFrame has been submitted -- so a null is
/// reached on every launch rather than on some edge case, and assigning a
/// `{}`-initialised temporary anywhere reopens exactly that crash. `reset` did.
VolumetricCaptureStats initial_stats() {
  VolumetricCaptureStats s{};
  s.color_matrix = "(none)";
  s.color_transfer = "(none)";
  s.color_primaries = "(none)";
  s.color_was_canonical = true;
  s.color_declaration_refused = false;
  return s;
}

/// The `ICameraCapture` recon consumes. Staging rotates three `FrameBuffers`:
/// the session queue converts into the caller's scratch, @ref stage swaps that
/// with `back_`, and a poll swaps `back_` to `front_`. The view handed out
/// points into `front_`, which only a poll ever touches -- so it stays valid
/// exactly as long as the contract promises (until the next poll) even while
/// ARKit keeps delivering into the other two.
class ARKitCapture final : public sensor::ICameraCapture {
 public:
  // Nothing to start: Swift owns the ARSession and pushes frames in. Present so
  // a consumer can drive any ICameraCapture uniformly.
  vr::Status start() override { return vr::Status(); }
  void stop() noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    staged_ = false;
  }

  vr::Result<std::optional<sensor::CapturedFrame>> poll() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!staged_) {
      return no_frame();
    }
    std::swap(front_, back_);
    staged_ = false;
    ++stats_.frames_polled;

    sensor::CapturedFrame frame{};
    frame.depth = front_.depth.data();
    frame.color = front_.has_color ? front_.color.data() : nullptr;
    frame.depth_camera = front_.depth_camera;
    frame.color_camera = front_.color_camera;
    // Always the canonical form: convert_color has already brought a
    // non-canonical source across. Declared rather than left defaulted, because
    // the default *is* a declaration -- so a source we failed to convert would
    // be fused through the wrong curve rather than refused.
    frame.color_encoding = front_.color_encoding;
    frame.timestamp_ns = front_.timestamp_ns;
    return some_frame(frame);
  }

  /// Hand over a freshly converted frame, superseding any unpolled one, and
  /// record what it held for the read-out.
  ///
  /// Swaps rather than move-assigns: @p buffers comes back owning the vectors
  /// this call retired, and the caller refills *those* next frame. The three
  /// `FrameBuffers` therefore rotate, and the 11 MB colour vector is allocated
  /// once per session instead of once per frame. Move-assigning from a fresh
  /// local -- the obvious shape -- means a fresh mapping every frame, so vImage
  /// paid a first-touch fault on each of its ~690 pages while writing.
  void stage(FrameBuffers& buffers, float convert_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (staged_) {
      ++stats_.frames_dropped;  // nobody took the previous one
    }
    std::swap(back_, buffers);
    staged_ = true;
    ++stats_.frames_submitted;

    const vr::DepthCameraParams& cam = back_.depth_camera;
    stats_.depth_width = cam.width;
    stats_.depth_height = cam.height;
    stats_.color_width = back_.has_color ? back_.color_camera.width : 0;
    stats_.color_height = back_.has_color ? back_.color_camera.height : 0;
    stats_.confidence_kept = back_.confidence_kept;
    stats_.depth_fx = cam.fx;
    stats_.depth_fy = cam.fy;
    stats_.depth_cx = cam.cx;
    stats_.depth_cy = cam.cy;
    stats_.position_x = cam.cam_to_world[3].x;
    stats_.position_y = cam.cam_to_world[3].y;
    stats_.position_z = cam.cam_to_world[3].z;
    stats_.convert_ms = convert_ms;
    stats_.color_convert_ms = back_.color_convert_ms;
    // Copied unconditionally, unlike the sizes above: `convert_color` already
    // spells "no colour buffer was read" as "(none)", and a *refused* frame
    // carries no colour yet still has a declaration worth showing. Gating these
    // on `has_color` would blank out precisely the case they exist to surface.
    stats_.color_matrix = back_.color_matrix;
    stats_.color_transfer = back_.color_transfer;
    stats_.color_primaries = back_.color_primaries;
    stats_.color_was_canonical = !back_.color_converted;
    stats_.color_declaration_refused = back_.color_refused;
  }

  /// Count a frame that carried depth but could not be converted.
  ///
  /// Without a counter of its own a persistent conversion failure is invisible:
  /// the submitted count simply stays at zero, which is the same "looks like a
  /// silent hang" symptom the LiDAR capability check exists to avoid.
  void reject() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.frames_rejected;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    staged_ = false;
    stats_ = initial_stats();
  }

  /// Counters and scalars only, deliberately. An earlier version also returned
  /// the staged `FrameBuffers` for the read-out, which copied ~11 MB of depth +
  /// colour vectors on every submit *and* every poll -- most of a 20 ms
  /// conversion budget, spent to display a handful of scalars.
  ///
  /// Under `mutex_` like the buffers: @ref stage runs on the session queue
  /// while this runs on the render loop, so an unguarded struct would tear.
  VolumetricCaptureStats stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

 private:
  std::mutex mutex_;
  FrameBuffers front_;
  FrameBuffers back_;
  bool staged_ = false;
  VolumetricCaptureStats stats_ = initial_stats();
};

}  // namespace

@implementation VolumetricCapture {
  std::unique_ptr<ARKitCapture> _capture;
  /// Refilled then swapped into the capture every frame, so the frame buffers
  /// rotate rather than being reallocated. Touched only from the session queue,
  /// via `submitFrame:`.
  FrameBuffers _scratch;
}

- (instancetype)init {
  self = [super init];
  if (self != nil) {
    _capture = std::make_unique<ARKitCapture>();
  }
  return self;
}

- (void)submitFrame:(ARFrame*)frame {
  // `smoothedSceneDepth` when the session asked for it. ARKit temporally
  // filters that map against previous frames, which suppresses the per-frame
  // flicker a TSDF would otherwise average into the volume -- but it is a
  // *separate property* from `sceneDepth` (ARFrame.h), not a filter applied in
  // place, so reading the wrong one silently discards smoothing the session is
  // already paying to compute. The fallback covers the frames early in a
  // session where the filter has no history yet.
  ARDepthData* depth_data = frame.smoothedSceneDepth ?: frame.sceneDepth;
  if (depth_data == nil) {
    // Ordinary at session start: tracking begins before the depth sensor has a
    // result. Not an error, so it is not counted as a rejection either.
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();

  const std::optional<float> kept =
      copy_depth(depth_data.depthMap, depth_data.confidenceMap, _scratch.depth);
  if (!kept) {
    _capture->reject();
    return;
  }
  _scratch.confidence_kept = *kept;

  // ARKit reports one set of intrinsics, for capturedImage. The colour camera
  // is described directly by them; the depth camera is *derived* from it,
  // because sceneDepth is registered to colour -- one physical camera, so the
  // two share a pose and differ only by an image-size scale. Letting recon do
  // that derivation is the point: it is the arithmetic its host tests pin.
  const simd_float3x3 k = frame.camera.intrinsics;
  const CGSize image_size = frame.camera.imageResolution;

  vr::ColorCameraParams color{};
  color.fx = k.columns[0][0];
  color.fy = k.columns[1][1];
  color.cx = k.columns[2][0];
  color.cy = k.columns[2][1];
  color.width = static_cast<std::uint32_t>(image_size.width);
  color.height = static_cast<std::uint32_t>(image_size.height);

  // ARKit's camera looks down -Z with +Y up; recon projects +Z forward with +Y
  // down. cv_from_gl_camera is the conversion, and getting it wrong smears the
  // reconstruction rather than raising anything.
  const simd_float4x4 t = frame.camera.transform;
  vr::Mat4f arkit_pose(1.0f);
  for (int c = 0; c < 4; ++c) {
    arkit_pose[c] = vr::Vec4f(t.columns[c][0], t.columns[c][1], t.columns[c][2],
                              t.columns[c][3]);
  }
  color.cam_to_world = sensor::cv_from_gl_camera(arkit_pose);

  vr::Result<vr::DepthCameraParams> derived =
      sensor::depth_from_registered_color(
          color,
          static_cast<std::uint32_t>(
              CVPixelBufferGetWidth(depth_data.depthMap)),
          static_cast<std::uint32_t>(
              CVPixelBufferGetHeight(depth_data.depthMap)),
          kMinDepth, kMaxDepth);
  if (!derived) {
    _capture->reject();
    return;
  }
  _scratch.depth_camera = derived.value();
  _scratch.color_camera = color;
  _scratch.timestamp_ns = static_cast<std::uint64_t>(frame.timestamp * 1e9);
  // A colour failure is not a frame failure: depth alone still fuses, and
  // `CapturedFrame` spells null colour as "no colour this frame".
  //
  // Timed around the whole call rather than inside it, so the figure covers the
  // attachment read and any `to_canonical` pass as well as the vImage
  // conversion -- a colour number that omitted the conversion would repeat, one
  // level down, the mistake that splitting it out of `convert_ms` fixed.
  const auto t_color = std::chrono::steady_clock::now();
  _scratch.has_color =
      convert_color(frame.capturedImage, color.width, color.height, _scratch);
  _scratch.color_convert_ms = std::chrono::duration<float, std::milli>(
                                  std::chrono::steady_clock::now() - t_color)
                                  .count();

  const float convert_ms = std::chrono::duration<float, std::milli>(
                               std::chrono::steady_clock::now() - t0)
                               .count();
  _capture->stage(_scratch, convert_ms);
}

- (BOOL)pollLatest {
  vr::Result<std::optional<sensor::CapturedFrame>> got = _capture->poll();
  return got && got.value() ? YES : NO;
}

- (void)reset {
  _capture->reset();
}

- (VolumetricCaptureStats)stats {
  return _capture->stats();
}

- (void*)captureHandle {
  return static_cast<sensor::ICameraCapture*>(_capture.get());
}

@end

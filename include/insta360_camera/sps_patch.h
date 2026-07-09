// Patch H.264 SPS max_num_reorder_frames to 0 so libavcodec doesn't add
// reorder-buffer latency. The Insta360 X3 preview stream is I/P-only but its
// SPS advertises max_num_reorder_frames=6, which makes any spec-compliant
// decoder hold back 6 frames (~200ms at 30fps).
//
// max_num_reorder_frames / max_dec_frame_buffering are the final fields of
// the SPS RBSP, so we can parse up to them and rewrite the tail in place
// without re-serializing the whole structure.
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace sps_patch {

// Hand-tuned bitstream code below: the grouped field-skips
// (`r.ue(); r.ue();  // pic_width/height`) read better than one statement per
// line, so this header opts out of clang-format.
// clang-format off

class BitReader {
 public:
  explicit BitReader(const std::vector<uint8_t>& b) : b_(b) {}
  int bit() {
    if (pos_ >= b_.size() * 8) { ok_ = false; return 0; }
    int v = (b_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1;
    pos_++;
    return v;
  }
  uint32_t bits(int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | bit();
    return v;
  }
  uint32_t ue() {
    int zeros = 0;
    while (bit() == 0 && ok_) {
      if (++zeros > 31) { ok_ = false; return 0; }  // malformed: avoid shift UB
    }
    if (!ok_ || zeros == 0) return 0;
    return (1u << zeros) - 1 + bits(zeros);
  }
  int32_t se() {
    uint32_t k = ue();
    return (k % 2) ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
  }
  size_t pos() const { return pos_; }
  bool ok() const { return ok_; }
  void fail() { ok_ = false; }

 private:
  const std::vector<uint8_t>& b_;
  size_t pos_ = 0;
  bool ok_ = true;
};

class BitWriter {
 public:
  void bit(int v) {
    if ((pos_ & 7) == 0) b_.push_back(0);
    if (v) b_[pos_ >> 3] |= 1 << (7 - (pos_ & 7));
    pos_++;
  }
  void bits(uint32_t v, int n) {
    for (int i = n - 1; i >= 0; i--) bit((v >> i) & 1);
  }
  void ue(uint32_t v) {
    if (v >= 0x7FFFFFFF) v = 0;  // we only ever emit small values; avoid UB
    int len = 0;
    uint32_t x = v + 1;
    while (x >> (len + 1)) len++;
    bits(0, len);
    bits(x, len + 1);
  }
  void rbspTrailing() {
    bit(1);
    while (pos_ & 7) bit(0);
  }
  const std::vector<uint8_t>& bytes() const { return b_; }

 private:
  std::vector<uint8_t> b_;
  size_t pos_ = 0;
};

inline std::vector<uint8_t> stripEmulation(const uint8_t* p, size_t n) {
  std::vector<uint8_t> out;
  out.reserve(n);
  int zeros = 0;
  for (size_t i = 0; i < n; i++) {
    if (zeros >= 2 && p[i] == 3) { zeros = 0; continue; }
    zeros = (p[i] == 0) ? zeros + 1 : 0;
    out.push_back(p[i]);
  }
  return out;
}

inline std::vector<uint8_t> addEmulation(const std::vector<uint8_t>& in) {
  std::vector<uint8_t> out;
  out.reserve(in.size() + 8);
  int zeros = 0;
  for (uint8_t c : in) {
    if (zeros >= 2 && c <= 3) {
      out.push_back(3);
      zeros = 0;
    }
    out.push_back(c);
    zeros = (c == 0) ? zeros + 1 : 0;
  }
  return out;
}

inline void skipHrd(BitReader& r) {
  uint32_t cpb_cnt = r.ue() + 1;
  if (cpb_cnt > 32) {  // spec max: cpb_cnt_minus1 <= 31; corrupt input.
    r.fail();          // poison the reader: continuing would misalign the parse
    return;
  }
  r.bits(4);  // bit_rate_scale
  r.bits(4);  // cpb_size_scale
  for (uint32_t i = 0; i < cpb_cnt && r.ok(); i++) {
    r.ue();  // bit_rate_value_minus1
    r.ue();  // cpb_size_value_minus1
    r.bit(); // cbr_flag
  }
  r.bits(5); r.bits(5); r.bits(5); r.bits(5);
}

// Patch one SPS NAL (payload starts with the NAL header byte, no start code).
// Returns the patched NAL, or empty on parse failure / nothing to do.
inline std::vector<uint8_t> patchSpsNal(const uint8_t* nal, size_t size) {
  if (size < 4 || (nal[0] & 0x1f) != 7) return {};
  std::vector<uint8_t> rbsp = stripEmulation(nal + 1, size - 1);
  BitReader r(rbsp);

  uint32_t profile = r.bits(8);
  r.bits(8);  // constraint flags
  r.bits(8);  // level
  r.ue();     // sps_id
  if (profile == 100 || profile == 110 || profile == 122 || profile == 244 ||
      profile == 44 || profile == 83 || profile == 86 || profile == 118 ||
      profile == 128 || profile == 138 || profile == 139 || profile == 134) {
    uint32_t chroma = r.ue();
    if (chroma == 3) r.bit();
    r.ue(); r.ue(); r.bit();
    if (r.bit()) {  // seq_scaling_matrix_present: bail, rare for cameras
      return {};
    }
  }
  r.ue();  // log2_max_frame_num_minus4
  uint32_t poc_type = r.ue();
  if (poc_type == 0) {
    r.ue();
  } else if (poc_type == 1) {
    r.bit(); r.se(); r.se();
    uint32_t n = r.ue();
    if (n > 255) return {};  // spec max: num_ref_frames_in_pic_order_cnt_cycle
    for (uint32_t i = 0; i < n && r.ok(); i++) r.se();
  }
  if (!r.ok()) return {};
  uint32_t max_num_ref_frames = r.ue();
  r.bit();          // gaps_in_frame_num
  r.ue(); r.ue();   // pic_width/height in mbs
  if (!r.bit()) r.bit();  // frame_mbs_only -> mb_adaptive
  r.bit();          // direct_8x8
  if (r.bit()) { r.ue(); r.ue(); r.ue(); r.ue(); }  // crop
  if (!r.bit()) return {};  // no VUI: nothing advertises reordering; skip

  // --- VUI ---
  if (r.bit()) {  // aspect_ratio_info
    uint32_t ar = r.bits(8);
    if (ar == 255) r.bits(32);
  }
  if (r.bit()) r.bit();  // overscan
  if (r.bit()) {          // video_signal_type
    r.bits(3); r.bit();
    if (r.bit()) r.bits(24);
  }
  if (r.bit()) { r.ue(); r.ue(); }  // chroma_loc
  if (r.bit()) { r.bits(32); r.bits(32); r.bit(); }  // timing
  int nal_hrd = r.bit();
  if (nal_hrd) skipHrd(r);
  int vcl_hrd = r.bit();
  if (vcl_hrd) skipHrd(r);
  if (nal_hrd || vcl_hrd) r.bit();  // low_delay_hrd
  r.bit();  // pic_struct_present
  size_t restriction_flag_pos = r.pos();
  int has_restriction = r.bit();
  // bitstream_restriction defaults, used only when we synthesize the block:
  // mv_over_pic default 1; 16/16 are the spec log2_max_mv_length_h/v defaults.
  uint32_t mv_over_pic = 1, max_bytes_denom = 0, max_bits_denom = 0;
  uint32_t mv_h = 16, mv_v = 16;
  // Where the rewrite starts: at the reorder field if the block is present,
  // else at the restriction flag itself (we insert a whole block there).
  size_t reorder_pos = restriction_flag_pos;
  if (has_restriction) {
    mv_over_pic = r.bit();
    max_bytes_denom = r.ue();
    max_bits_denom = r.ue();
    mv_h = r.ue();
    mv_v = r.ue();
    reorder_pos = r.pos();
    uint32_t reorder_frames = r.ue();
    r.ue();  // max_dec_frame_buffering
    if (!r.ok()) return {};
    if (reorder_frames == 0) return {};  // already low-delay
  }
  if (!r.ok()) return {};

  // Rebuild: copy bits up to the rewrite point, then emit restriction with
  // max_num_reorder_frames=0.
  BitReader src(rbsp);
  BitWriter w;
  for (size_t i = 0; i < reorder_pos; i++) w.bit(src.bit());
  if (!has_restriction) {
    w.bit(1);  // bitstream_restriction_flag
    w.bit(mv_over_pic);
    w.ue(max_bytes_denom);
    w.ue(max_bits_denom);
    w.ue(mv_h);
    w.ue(mv_v);
  }
  w.ue(0);  // max_num_reorder_frames
  w.ue(max_num_ref_frames > 0 ? max_num_ref_frames : 1);  // max_dec_frame_buffering
  w.rbspTrailing();

  std::vector<uint8_t> out;
  out.push_back(nal[0]);
  std::vector<uint8_t> body = addEmulation(w.bytes());
  out.insert(out.end(), body.begin(), body.end());
  return out;
}

// Scan an Annex-B buffer; if it contains an SPS advertising reordering,
// return a copy with the SPS patched. Otherwise return empty (use original).
inline std::vector<uint8_t> patchAnnexb(const uint8_t* data, size_t size) {
  // find NAL boundaries (both 3- and 4-byte start codes)
  struct Nal { size_t start, payload, end; };
  std::vector<Nal> nals;
  size_t i = 0;
  while (i + 3 < size) {
    if (data[i] == 0 && data[i + 1] == 0 &&
        (data[i + 2] == 1 ||
         (data[i + 2] == 0 && i + 4 <= size && data[i + 3] == 1))) {
      size_t payload = i + (data[i + 2] == 1 ? 3 : 4);
      if (!nals.empty()) nals.back().end = i;
      nals.push_back({i, payload, size});
      i = payload;
    } else {
      i++;
    }
  }
  auto is_sps = [&](const Nal& n) {
    return n.payload < size && (data[n.payload] & 0x1f) == 7;
  };
  // Fast path: no SPS NAL in this AU (the common case — SPS only rides on
  // keyframes). Avoids building a full copy just to throw it away.
  bool has_sps = false;
  for (const Nal& n : nals)
    if (is_sps(n)) { has_sps = true; break; }
  if (!has_sps) return {};

  bool patched_any = false;
  std::vector<uint8_t> out;
  out.reserve(size + 16);
  for (const Nal& n : nals) {
    if (is_sps(n)) {
      std::vector<uint8_t> patched =
          patchSpsNal(data + n.payload, n.end - n.payload);
      if (!patched.empty()) {
        out.insert(out.end(), data + n.start, data + n.payload);
        out.insert(out.end(), patched.begin(), patched.end());
        patched_any = true;
        continue;
      }
    }
    out.insert(out.end(), data + n.start, data + n.end);
  }
  if (!patched_any) return {};
  return out;
}

// clang-format on
}  // namespace sps_patch

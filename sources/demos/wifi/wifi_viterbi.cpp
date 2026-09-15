#include "wifi_viterbi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <zlib.h>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace quadrf_wifi {

namespace {

// Encoder lookup table: [state | (bit << 6)][0=output_bits (A<<1 | B), 1=next_state]
struct EncLut {
  uint8_t table[128][2];
  constexpr EncLut() : table{} {
    for (int s = 0; s < 64; ++s) {
      for (int b = 0; b < 2; ++b) {
        int word = s | (b << 6);
        int out_a = 0, out_b = 0;
        int m_a = word & 0133;
        while (m_a) { out_a ^= (m_a & 1); m_a >>= 1; }
        int m_b = word & 0171;
        while (m_b) { out_b ^= (m_b & 1); m_b >>= 1; }
        table[word][0] = static_cast<uint8_t>((out_a << 1) | out_b);
        table[word][1] = static_cast<uint8_t>(((s >> 1) | (b << 5)) & 0x3F);
      }
    }
  }
};

inline constexpr EncLut kEncLut;

}  // namespace

std::vector<uint8_t> WifiEncoder::encode(std::span<const uint8_t> data_bits,
                                         int rate_num, int rate_den) {
  if (data_bits.empty()) return {};

  std::vector<uint8_t> mother;
  mother.reserve(data_bits.size() * 2);

  uint8_t state = 0;
  for (uint8_t bit : data_bits) {
    const uint8_t idx = static_cast<uint8_t>(state | ((bit & 1) << 6));
    const uint8_t entry = kEncLut.table[idx][0];
    mother.push_back((entry >> 1) & 1);
    mother.push_back(entry & 1);
    state = kEncLut.table[idx][1];
  }

  return puncture(mother, rate_num, rate_den);
}

std::vector<uint8_t> WifiEncoder::puncture(std::span<const uint8_t> enc,
                                           int rate_num, int rate_den) {
  if (rate_num == 1 && rate_den == 2) {
    return std::vector<uint8_t>(enc.begin(), enc.end());
  }

  std::vector<uint8_t> out;
  out.reserve((enc.size() * static_cast<size_t>(rate_num)) /
              static_cast<size_t>(rate_den));

  if (rate_num == 2 && rate_den == 3) {
    for (size_t i = 0; i < enc.size(); i += 4) {
      const size_t rem = enc.size() - i;
      if (rem >= 1) out.push_back(enc[i]);
      if (rem >= 2) out.push_back(enc[i + 1]);
      if (rem >= 3) out.push_back(enc[i + 2]);
    }
  } else if (rate_num == 3 && rate_den == 4) {
    for (size_t i = 0; i < enc.size(); i += 6) {
      const size_t rem = enc.size() - i;
      if (rem >= 1) out.push_back(enc[i]);
      if (rem >= 2) out.push_back(enc[i + 1]);
      if (rem >= 3) out.push_back(enc[i + 2]);
      if (rem >= 6) out.push_back(enc[i + 5]);
    }
  } else {
    return std::vector<uint8_t>(enc.begin(), enc.end());
  }

  return out;
}

WifiViterbi::WifiViterbi() {
  float outputs[64][2][2];
  for (int s = 0; s < 64; ++s) {
    const int S1 = (s >> 0) & 1;
    const int S2 = (s >> 1) & 1;
    const int S3 = (s >> 2) & 1;
    const int S5 = (s >> 4) & 1;
    const int S6 = (s >> 5) & 1;
    for (int b = 0; b < 2; ++b) {
      const int S0 = b;
      const int out_A = (S0 + S2 + S3 + S5 + S6) % 2;
      const int out_B = (S0 + S1 + S2 + S3 + S6) % 2;
      outputs[s][b][0] = (out_A == 0) ? 1.f : -1.f;
      outputs[s][b][1] = (out_B == 0) ? 1.f : -1.f;
    }
  }
  for (int ns = 0; ns < 64; ++ns) {
    const int p0 = ns >> 1;
    const int p1 = (ns >> 1) | 0x20;
    prev_states_[ns][0] = p0;
    prev_states_[ns][1] = p1;
    const int input_b = ns & 1;
    prev_outputs_[ns][0][0] = outputs[p0][input_b][0];
    prev_outputs_[ns][0][1] = outputs[p0][input_b][1];
    prev_outputs_[ns][1][0] = outputs[p1][input_b][0];
    prev_outputs_[ns][1][1] = outputs[p1][input_b][1];
  }
}

std::vector<float> WifiViterbi::depuncture(std::span<const float> in_llrs,
                                           int rate_num, int rate_den) {
  if (rate_num == 1 && rate_den == 2) {
    return std::vector<float>(in_llrs.begin(), in_llrs.end());
  }

  std::vector<float> out;
  out.reserve((in_llrs.size() * static_cast<size_t>(rate_den)) /
              static_cast<size_t>(rate_num) + 16);

  if (rate_num == 2 && rate_den == 3) {
    for (size_t i = 0; i < in_llrs.size(); i += 3) {
      const size_t rem = std::min<size_t>(3, in_llrs.size() - i);
      if (rem == 3) {
        out.push_back(in_llrs[i]);
        out.push_back(in_llrs[i + 1]);
        out.push_back(in_llrs[i + 2]);
        out.push_back(0.f);
      } else {
        for (size_t j = 0; j < rem; ++j) out.push_back(in_llrs[i + j]);
      }
    }
  } else if (rate_num == 3 && rate_den == 4) {
    for (size_t i = 0; i < in_llrs.size(); i += 4) {
      const size_t rem = std::min<size_t>(4, in_llrs.size() - i);
      if (rem == 4) {
        out.push_back(in_llrs[i]);
        out.push_back(in_llrs[i + 1]);
        out.push_back(in_llrs[i + 2]);
        out.push_back(0.f);
        out.push_back(0.f);
        out.push_back(in_llrs[i + 3]);
      } else {
        for (size_t j = 0; j < rem; ++j) out.push_back(in_llrs[i + j]);
      }
    }
  } else {
    return std::vector<float>(in_llrs.begin(), in_llrs.end());
  }

  return out;
}

std::vector<uint8_t> WifiViterbi::decode(std::span<const float> soft_llrs,
                                         bool force_state_zero) const {
  const size_t n_pairs = soft_llrs.size() / 2;
  if (n_pairs == 0) return {};

  alignas(16) float pm_buf[2][64];
  for (int s = 0; s < 64; ++s) pm_buf[0][s] = 1e6f;
  pm_buf[0][0] = 0.f;

  std::vector<uint8_t> history(n_pairs * 64);
  int cur_buf = 0;

  for (size_t t = 0; t < n_pairs; ++t) {
    const float rx_a = soft_llrs[2 * t];
    const float rx_b = soft_llrs[2 * t + 1];
    const int nxt_buf = 1 - cur_buf;
    const float* curr_pm = pm_buf[cur_buf];
    float* next_pm = pm_buf[nxt_buf];
    uint8_t* hist = &history[t * 64];

#if defined(__ARM_NEON) || defined(__aarch64__)
    const float32x4_t v_rxa = vdupq_n_f32(rx_a);
    const float32x4_t v_rxb = vdupq_n_f32(rx_b);

    for (int ns = 0; ns < 64; ns += 4) {
      alignas(16) float pm0[4], pm1[4], o0a[4], o0b[4], o1a[4], o1b[4];
      int p0[4], p1[4];
      for (int k = 0; k < 4; ++k) {
        const int s = ns + k;
        p0[k] = prev_states_[s][0];
        p1[k] = prev_states_[s][1];
        pm0[k] = curr_pm[p0[k]];
        pm1[k] = curr_pm[p1[k]];
        o0a[k] = prev_outputs_[s][0][0];
        o0b[k] = prev_outputs_[s][0][1];
        o1a[k] = prev_outputs_[s][1][0];
        o1b[k] = prev_outputs_[s][1][1];
      }

      const float32x4_t m0 = vaddq_f32(
          vld1q_f32(pm0),
          vnegq_f32(vfmaq_f32(vmulq_f32(v_rxa, vld1q_f32(o0a)), v_rxb, vld1q_f32(o0b))));
      const float32x4_t m1 = vaddq_f32(
          vld1q_f32(pm1),
          vnegq_f32(vfmaq_f32(vmulq_f32(v_rxa, vld1q_f32(o1a)), v_rxb, vld1q_f32(o1b))));

      vst1q_f32(&next_pm[ns], vminq_f32(m0, m1));

      for (int k = 0; k < 4; ++k) {
        const float a = vgetq_lane_f32(m0, k);
        const float b = vgetq_lane_f32(m1, k);
        hist[ns + k] = static_cast<uint8_t>((b < a) ? p1[k] : p0[k]);
      }
    }
#else
    for (int ns = 0; ns < 64; ++ns) {
      const int p0 = prev_states_[ns][0];
      const int p1 = prev_states_[ns][1];
      const float m0 = curr_pm[p0] - (rx_a * prev_outputs_[ns][0][0] + rx_b * prev_outputs_[ns][0][1]);
      const float m1 = curr_pm[p1] - (rx_a * prev_outputs_[ns][1][0] + rx_b * prev_outputs_[ns][1][1]);
      next_pm[ns] = std::min(m0, m1);
      hist[ns] = static_cast<uint8_t>((m1 < m0) ? p1 : p0);
    }
#endif
    cur_buf = nxt_buf;
  }

  int curr = 0;
  if (!force_state_zero) {
    float best = pm_buf[cur_buf][0];
    for (int s = 1; s < 64; ++s) {
      if (pm_buf[cur_buf][s] < best) {
        best = pm_buf[cur_buf][s];
        curr = s;
      }
    }
  }

  std::vector<uint8_t> bits(n_pairs);
  for (int t = static_cast<int>(n_pairs) - 1; t >= 0; --t) {
    bits[static_cast<size_t>(t)] = static_cast<uint8_t>(curr & 1);
    curr = history[static_cast<size_t>(t) * 64 + static_cast<size_t>(curr)];
  }

  return bits;
}

std::vector<uint8_t> scramble(std::span<const uint8_t> bits, int seed) {
  int state = seed & 0x7F;
  std::vector<uint8_t> out(bits.size());
  for (size_t i = 0; i < bits.size(); ++i) {
    const int fb = ((state >> 6) ^ (state >> 3)) & 1;
    state = ((state << 1) | fb) & 0x7F;
    out[i] = static_cast<uint8_t>(bits[i] ^ fb);
  }
  return out;
}

std::vector<uint8_t> descramble(std::span<const uint8_t> bits) {
  if (bits.size() < 7) return std::vector<uint8_t>(bits.begin(), bits.end());

  // In 802.11, the first 7 bits of the SERVICE field are transmitted as zeros.
  // Therefore, the first 7 received bits are the first 7 outputs of the scrambler LFSR:
  // fb0, fb1, fb2, fb3, fb4, fb5, fb6.
  // After these 7 shifts, the scrambler LFSR shift register state is identically:
  int state = 0;
  for (int i = 0; i < 7; ++i) {
    state |= (int(bits[static_cast<size_t>(i)]) << (6 - i));
  }

  std::vector<uint8_t> out(bits.size(), 0);
  // Bits 0..6 were zeros in the transmitter
  for (size_t i = 7; i < bits.size(); ++i) {
    const int fb = ((state >> 6) ^ (state >> 3)) & 1;
    state = ((state << 1) | fb) & 0x7F;
    out[i] = static_cast<uint8_t>(bits[i] ^ fb);
  }
  return out;
}

std::vector<uint8_t> descramble(std::span<const uint8_t> bits, int seed) {
  if (seed < 0) return descramble(bits);
  return scramble(bits, seed);  // Involution if initial seed is known
}

uint32_t wifi_crc32(const uint8_t* data, size_t len) {
  return crc32(0u, data, static_cast<uInt>(len)) & 0xFFFFFFFFu;
}

}  // namespace quadrf_wifi

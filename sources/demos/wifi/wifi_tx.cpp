#include "wifi_tx.hpp"
#include "wifi_dsp.hpp"

#include <fftw3.h>


#include <algorithm>
#include <cmath>
#include <cstring>

namespace quadrf_wifi {

namespace {

// Standard 802.11a STS frequency domain subcarriers (-26 to +26)
inline constexpr float STS_NORM = 1.471960144f;  // sqrt(13/6)
inline constexpr c32 S_STD[53] = {
    c32(0, 0), c32(0, 0), c32(1, 1), c32(0, 0), c32(0, 0), c32(0, 0),
    c32(-1, -1), c32(0, 0), c32(0, 0), c32(0, 0), c32(1, 1), c32(0, 0),
    c32(0, 0), c32(0, 0), c32(-1, -1), c32(0, 0), c32(0, 0), c32(0, 0),
    c32(-1, -1), c32(0, 0), c32(0, 0), c32(0, 0), c32(1, 1), c32(0, 0),
    c32(0, 0), c32(0, 0), c32(0, 0),  // DC
    c32(0, 0), c32(0, 0), c32(0, 0), c32(-1, -1), c32(0, 0), c32(0, 0),
    c32(0, 0), c32(-1, -1), c32(0, 0), c32(0, 0), c32(0, 0), c32(1, 1),
    c32(0, 0), c32(0, 0), c32(0, 0), c32(1, 1), c32(0, 0), c32(0, 0),
    c32(0, 0), c32(1, 1), c32(0, 0), c32(0, 0), c32(0, 0), c32(1, 1),
    c32(0, 0), c32(0, 0)};

}  // namespace

WifiTx::WifiTx(TxConfig cfg) : cfg_(cfg) {
  init_fftw();
  init_preambles();
}

WifiTx::~WifiTx() {
  std::lock_guard<std::mutex> lock(g_fftw_planner_mutex);
  if (fft_plan_bwd_) fftwf_destroy_plan(static_cast<fftwf_plan>(fft_plan_bwd_));
  if (fft_in_) fftwf_free(fft_in_);
  if (fft_out_) fftwf_free(fft_out_);
}

void WifiTx::init_fftw() {
  std::lock_guard<std::mutex> lock(g_fftw_planner_mutex);
  fft_in_ = fftwf_malloc(sizeof(fftwf_complex) * 64);
  fft_out_ = fftwf_malloc(sizeof(fftwf_complex) * 64);
  fft_plan_bwd_ = fftwf_plan_dft_1d(
      64, static_cast<fftwf_complex*>(fft_in_),
      static_cast<fftwf_complex*>(fft_out_), FFTW_BACKWARD, FFTW_ESTIMATE);
}

void WifiTx::ifft64(const c32* in, c32* out) {
  auto* fin = static_cast<fftwf_complex*>(fft_in_);
  auto* fout = static_cast<fftwf_complex*>(fft_out_);
  for (int i = 0; i < 64; ++i) {
    fin[i][0] = in[i].real();
    fin[i][1] = in[i].imag();
  }
  fftwf_execute(static_cast<fftwf_plan>(fft_plan_bwd_));
  constexpr float inv_n = 1.f / 64.f;
  for (int i = 0; i < 64; ++i) {
    out[i] = c32(fout[i][0] * inv_n, fout[i][1] * inv_n);
  }
}

void WifiTx::init_preambles() {
  // 1. Generate Short Training Sequence (STS): 10 repetitions of 16 samples = 160 samples
  c32 S_fft[64]{};
  for (int i = 0; i < 26; ++i) {
    S_fft[38 + i] = S_STD[i] * STS_NORM;
  }
  for (int i = 0; i < 26; ++i) {
    S_fft[1 + i] = S_STD[27 + i] * STS_NORM;
  }
  c32 s_time[64];
  ifft64(S_fft, s_time);

  sts_preamble_.resize(160);
  for (int rep = 0; rep < 10; ++rep) {
    for (int k = 0; k < 16; ++k) {
      sts_preamble_[static_cast<size_t>(rep * 16 + k)] = s_time[k] * cfg_.digital_scale;
    }
  }

  // 2. Generate Long Training Sequence (LTF): 32-sample CP + 2x 64-sample LTS = 160 samples
  c32 L_fft[64]{};
  for (int i = 0; i < 26; ++i) {
    L_fft[38 + i] = c32(L_STD[i], 0.f);
  }
  for (int i = 0; i < 26; ++i) {
    L_fft[1 + i] = c32(L_STD[27 + i], 0.f);
  }
  c32 l_time[64];
  ifft64(L_fft, l_time);

  ltf_preamble_.resize(160);
  // 32-sample cyclic prefix (last 32 samples of LTS)
  for (int k = 0; k < 32; ++k) {
    ltf_preamble_[static_cast<size_t>(k)] = l_time[32 + k] * cfg_.digital_scale;
  }
  // Two 64-sample LTS symbols
  for (int rep = 0; rep < 2; ++rep) {
    for (int k = 0; k < 64; ++k) {
      ltf_preamble_[static_cast<size_t>(32 + rep * 64 + k)] = l_time[k] * cfg_.digital_scale;
    }
  }
}

c32 WifiTx::map_subcarrier(const uint8_t* bits, int n_bpsc) {
  switch (n_bpsc) {
    case 1:  // BPSK: 0 -> +1, 1 -> -1
      return (bits[0] == 0) ? c32(1.f, 0.f) : c32(-1.f, 0.f);

    case 2: {  // QPSK Gray coded
      constexpr float kNorm = 0.70710678f;  // 1 / sqrt(2)
      const float i = (bits[0] == 0) ? kNorm : -kNorm;
      const float q = (bits[1] == 0) ? kNorm : -kNorm;
      return c32(i, q);
    }

    case 4: {  // 16-QAM Gray coded: {-3, -1, +1, +3} / sqrt(10)
      constexpr float kNorm = 0.316227766f;  // 1 / sqrt(10)
      // Standard 802.11: b0=0 -> positive, b1=0 -> smaller magnitude

      // 00 -> -3, 01 -> -1, 11 -> +1, 10 -> +3
      auto map_2bits = [](uint8_t b0, uint8_t b1) -> float {
        if (b0 == 0 && b1 == 0) return -3.f;
        if (b0 == 0 && b1 == 1) return -1.f;
        if (b0 == 1 && b1 == 1) return 1.f;
        return 3.f;
      };
      const float r = map_2bits(bits[0], bits[1]) * kNorm;
      const float im = map_2bits(bits[2], bits[3]) * kNorm;
      return c32(r, im);
    }

    case 6: {  // 64-QAM Gray coded: {-7, -5, -3, -1, +1, +3, +5, +7} / sqrt(42)
      constexpr float kNorm = 0.15430335f;  // 1 / sqrt(42)
      auto map_3bits = [](uint8_t b0, uint8_t b1, uint8_t b2) -> float {
        if (b0 == 0) {
          if (b1 == 0 && b2 == 0) return -7.f;
          if (b1 == 0 && b2 == 1) return -5.f;
          if (b1 == 1 && b2 == 1) return -3.f;
          return -1.f;
        } else {
          if (b1 == 1 && b2 == 0) return 1.f;
          if (b1 == 1 && b2 == 1) return 3.f;
          if (b1 == 0 && b2 == 1) return 5.f;
          return 7.f;
        }
      };
      const float r = map_3bits(bits[0], bits[1], bits[2]) * kNorm;
      const float im = map_3bits(bits[3], bits[4], bits[5]) * kNorm;
      return c32(r, im);
    }

    default:
      return c32(0.f, 0.f);
  }
}

std::vector<c32> WifiTx::modulate_frame(std::span<const uint8_t> mpdu, const RateInfo& rate) {
  if (mpdu.empty()) return {};

  const int pkt_len = static_cast<int>(mpdu.size());
  const int n_bpsc = rate.n_bpsc;
  const int n_cbps = rate.n_cbps;
  const int n_dbps = rate.n_dbps;

  const int total_data_bits = 16 + 8 * pkt_len + 6;
  const int n_syms = (total_data_bits + n_dbps - 1) / n_dbps;
  const int padded_bits = n_syms * n_dbps;

  // Prepare raw bits: 16 service bits (0) + payload bits + 6 tail bits (0) + pad bits (0)
  std::vector<uint8_t> unencoded_bits(static_cast<size_t>(padded_bits), 0);
  for (int i = 0; i < pkt_len; ++i) {
    const uint8_t byte = mpdu[static_cast<size_t>(i)];
    for (int b = 0; b < 8; ++b) {
      unencoded_bits[static_cast<size_t>(16 + i * 8 + b)] = (byte >> b) & 1;
    }
  }

  // Scramble unencoded payload bits (service bits + data bits; tail bits remain 0)
  auto scrambled_bits = scramble(unencoded_bits, cfg_.seed);
  // Re-zero tail bits (last 6 bits of packet before padding)
  for (int i = 0; i < 6; ++i) {
    scrambled_bits[static_cast<size_t>(16 + 8 * pkt_len + i)] = 0;
  }

  // Convolutional encode + puncture
  const auto coded_bits = WifiEncoder::encode(scrambled_bits, rate.rate_num, rate.rate_den);

  const auto fwd_interleave = WifiDsp::make_interleaver_map(n_cbps, n_bpsc);

  // Reserve output: STS (160) + LTF (160) + SIGNAL (80) + Data (n_syms * 80) + Guard (trailing)
  std::vector<c32> tx_samples;
  tx_samples.reserve(static_cast<size_t>(400 + n_syms * 80 + cfg_.trailing_guard_samples));

  // 1. Preamble: STS
  tx_samples.insert(tx_samples.end(), sts_preamble_.begin(), sts_preamble_.end());

  // 2. Preamble: LTF
  tx_samples.insert(tx_samples.end(), ltf_preamble_.begin(), ltf_preamble_.end());

  // 3. SIGNAL Symbol
  std::vector<uint8_t> sig_bits(24, 0);
  // RATE (bits 0..3)
  for (int b = 0; b < 4; ++b) {
    sig_bits[static_cast<size_t>(b)] = (rate.rate_val >> b) & 1;
  }
  // LENGTH (bits 5..16)
  for (int b = 0; b < 12; ++b) {
    sig_bits[static_cast<size_t>(5 + b)] = (pkt_len >> b) & 1;
  }
  // Parity (bit 17: even parity over bits 0..16)
  int par = 0;
  for (int b = 0; b < 17; ++b) par += sig_bits[static_cast<size_t>(b)];
  sig_bits[17] = par & 1;
  // Tail bits 18..23 are 0

  const auto sig_coded = WifiEncoder::encode(sig_bits, 1, 2);  // Rate 1/2 -> 48 bits
  const auto sig_deint_map = WifiDsp::make_interleaver_map(48, 1);
  std::vector<uint8_t> sig_interleaved(48);
  for (int k = 0; k < 48; ++k) {
    sig_interleaved[static_cast<size_t>(sig_deint_map[static_cast<size_t>(k)])] = sig_coded[static_cast<size_t>(k)];
  }

  // Map SIGNAL to 64 FFT subcarriers
  c32 sig_fft[64]{};
  for (int k = 0; k < 48; ++k) {
    sig_fft[DATA_SC[k]] = (sig_interleaved[static_cast<size_t>(k)] == 0) ? c32(1.f, 0.f) : c32(-1.f, 0.f);
  }
  for (int p = 0; p < 4; ++p) {
    sig_fft[PILOT_SC[p]] = c32(BASE_PILOT_POLARITY[p], 0.f);
  }
  c32 sig_time[64];
  ifft64(sig_fft, sig_time);

  // Add 16 CP + 64 symbol
  for (int k = 48; k < 64; ++k) tx_samples.push_back(sig_time[k] * cfg_.digital_scale);
  for (int k = 0; k < 64; ++k) tx_samples.push_back(sig_time[k] * cfg_.digital_scale);

  // 4. Data Symbols
  for (int s = 0; s < n_syms; ++s) {
    // Interleave
    std::vector<uint8_t> sym_interleaved(static_cast<size_t>(n_cbps));
    for (int k = 0; k < n_cbps; ++k) {
      sym_interleaved[static_cast<size_t>(fwd_interleave[static_cast<size_t>(k)])] =
          coded_bits[static_cast<size_t>(s * n_cbps + k)];
    }

    c32 data_fft[64]{};
    for (int k = 0; k < 48; ++k) {
      data_fft[DATA_SC[k]] = map_subcarrier(&sym_interleaved[static_cast<size_t>(k * n_bpsc)], n_bpsc);
    }

    // Pilot subcarriers
    const float pilot_pol = static_cast<float>(PILOT_SEQ[(s + 1) % 127]);
    for (int p = 0; p < 4; ++p) {
      data_fft[PILOT_SC[p]] = c32(BASE_PILOT_POLARITY[p] * pilot_pol, 0.f);
    }

    c32 data_time[64];
    ifft64(data_fft, data_time);

    // Add 16 CP + 64 symbol
    for (int k = 48; k < 64; ++k) tx_samples.push_back(data_time[k] * cfg_.digital_scale);
    for (int k = 0; k < 64; ++k) tx_samples.push_back(data_time[k] * cfg_.digital_scale);
  }

  // 5. Trailing Guard Silence (minimal zero padding to allow filter tail decay without holding channel)
  for (int k = 0; k < cfg_.trailing_guard_samples; ++k) {
    tx_samples.emplace_back(0.f, 0.f);
  }

  return tx_samples;
}

std::vector<int8_t> WifiTx::modulate_frame_cs8(std::span<const uint8_t> mpdu, const RateInfo& rate) {
  const auto cf32 = modulate_frame(mpdu, rate);
  if (cf32.empty()) return {};

  float peak = 1e-6f;
  for (const auto& s : cf32) {
    peak = std::max(peak, std::abs(s));
  }
  const float scale = 100.f / peak;

  std::vector<int8_t> cs8(cf32.size() * 2);
  for (size_t i = 0; i < cf32.size(); ++i) {
    float r = std::clamp(cf32[i].real() * scale, -127.f, 127.f);
    float im = std::clamp(cf32[i].imag() * scale, -127.f, 127.f);
    cs8[2 * i] = static_cast<int8_t>(std::lrintf(r));
    cs8[2 * i + 1] = static_cast<int8_t>(std::lrintf(im));
  }
  return cs8;
}

}  // namespace quadrf_wifi

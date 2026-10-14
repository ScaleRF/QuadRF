#include "wifi_dsp.hpp"

#include <fftw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace quadrf_wifi {

namespace {

inline float fft_bin_k(int bin) {
  return (bin < 32) ? static_cast<float>(bin) : static_cast<float>(bin - 64);
}

void mmse_equalize(const c32* Y, const c32* H, c32* eq, float mmse_reg) {
  for (int k = 0; k < 64; ++k) {
    const float h2 = std::norm(H[k]);
    if (h2 > 1e-12f) {
      eq[k] = (Y[k] * std::conj(H[k])) / (h2 + mmse_reg);
    } else {
      eq[k] = c32(0.f, 0.f);
    }
  }
}

void apply_phase_correction(c32* eq, float cpe, float slope) {
  for (int k = 0; k < 64; ++k) {
    const float ph = -(cpe + slope * fft_bin_k(k));
    eq[k] *= c32(std::cos(ph), std::sin(ph));
  }
}

inline float snap_1d(float val, const float* levels, int n) {
  float best = levels[0];
  float min_d = std::abs(val - best);
  for (int i = 1; i < n; ++i) {
    float d = std::abs(val - levels[i]);
    if (d < min_d) { min_d = d; best = levels[i]; }
  }
  return best;
}

inline constexpr float qam16_levels[4] = {-3.f * 0.316227766f, -1.f * 0.316227766f, 1.f * 0.316227766f, 3.f * 0.316227766f};
inline constexpr float qam64_levels[8] = {-7.f * 0.15430335f, -5.f * 0.15430335f, -3.f * 0.15430335f, -1.f * 0.15430335f,
                                          1.f * 0.15430335f, 3.f * 0.15430335f, 5.f * 0.15430335f, 7.f * 0.15430335f};
inline constexpr float qpsk_levels[2] = {-0.70710678f, 0.70710678f};

float compute_evm_db(const std::vector<c32>& syms, Modulation mod) {
  if (syms.empty()) return 0.f;
  double err_pwr = 0.0;
  double ref_pwr = 0.0;

  for (const auto& s : syms) {
    c32 ideal(0.f, 0.f);
    switch (mod) {
      case Modulation::BPSK:
        ideal = c32((s.real() >= 0.f) ? 1.f : -1.f, 0.f);
        break;
      case Modulation::QPSK:
        ideal = c32(snap_1d(s.real(), qpsk_levels, 2), snap_1d(s.imag(), qpsk_levels, 2));
        break;
      case Modulation::QAM16:
        ideal = c32(snap_1d(s.real(), qam16_levels, 4), snap_1d(s.imag(), qam16_levels, 4));
        break;
      case Modulation::QAM64:
        ideal = c32(snap_1d(s.real(), qam64_levels, 8), snap_1d(s.imag(), qam64_levels, 8));
        break;
      default:
        ideal = c32((s.real() >= 0.f) ? 1.f : -1.f, 0.f);
        break;
    }
    const c32 diff = s - ideal;
    err_pwr += std::norm(diff);
    ref_pwr += std::norm(ideal);
  }

  if (ref_pwr <= 1e-12 || err_pwr <= 1e-12) return -50.f;
  return static_cast<float>(10.0 * std::log10(err_pwr / ref_pwr));
}

}  // namespace

WifiDsp::WifiDsp(DspConfig cfg) : cfg_(cfg), ltf_ref_(make_ltf_ref()) {
  init_fftw();
}

WifiDsp::~WifiDsp() {
  std::lock_guard<std::mutex> lock(g_fftw_planner_mutex);
  if (fft_plan_fwd_) fftwf_destroy_plan(static_cast<fftwf_plan>(fft_plan_fwd_));
  if (fft_plan_bwd_) fftwf_destroy_plan(static_cast<fftwf_plan>(fft_plan_bwd_));
  if (fft_in_) fftwf_free(fft_in_);
  if (fft_out_) fftwf_free(fft_out_);
}

void WifiDsp::init_fftw() {
  std::lock_guard<std::mutex> lock(g_fftw_planner_mutex);
  fft_in_ = fftwf_malloc(sizeof(fftwf_complex) * 64);
  fft_out_ = fftwf_malloc(sizeof(fftwf_complex) * 64);
  fft_plan_fwd_ = fftwf_plan_dft_1d(
      64, static_cast<fftwf_complex*>(fft_in_),
      static_cast<fftwf_complex*>(fft_out_), FFTW_FORWARD, FFTW_ESTIMATE);
  fft_plan_bwd_ = fftwf_plan_dft_1d(
      64, static_cast<fftwf_complex*>(fft_in_),
      static_cast<fftwf_complex*>(fft_out_), FFTW_BACKWARD, FFTW_ESTIMATE);
}

void WifiDsp::fft64(const c32* in, c32* out) {
  auto* fin = static_cast<fftwf_complex*>(fft_in_);
  auto* fout = static_cast<fftwf_complex*>(fft_out_);
  for (int i = 0; i < 64; ++i) {
    fin[i][0] = in[i].real();
    fin[i][1] = in[i].imag();
  }
  fftwf_execute(static_cast<fftwf_plan>(fft_plan_fwd_));
  for (int i = 0; i < 64; ++i) {
    out[i] = c32(fout[i][0], fout[i][1]);
  }
}

void WifiDsp::ifft64(const c32* in, c32* out) {
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

void WifiDsp::derotate(c32* iq, size_t len, float rad_per_sample, float start_sample) {
  if (len == 0 || std::abs(rad_per_sample) < 1e-9f) return;

#if defined(__ARM_NEON) || defined(__aarch64__)
  // Vectorized block derotation: phase at sample i is -rad_per_sample * (i - start_sample)
  float cur_phase = rad_per_sample * start_sample;
  const float d_phase = -rad_per_sample;
  for (size_t i = 0; i < len; ++i) {
    const float c = std::cos(cur_phase);
    const float s = std::sin(cur_phase);
    const c32 z = iq[i];
    iq[i] = c32(z.real() * c - z.imag() * s, z.real() * s + z.imag() * c);
    cur_phase += d_phase;
    if (cur_phase > kPi) cur_phase -= kTwoPi;
    else if (cur_phase < -kPi) cur_phase += kTwoPi;
  }
#else
  float cur_phase = rad_per_sample * start_sample;
  const float d_phase = -rad_per_sample;
  for (size_t i = 0; i < len; ++i) {
    const float c = std::cos(cur_phase);
    const float s = std::sin(cur_phase);
    const c32 z = iq[i];
    iq[i] = c32(z.real() * c - z.imag() * s, z.real() * s + z.imag() * c);
    cur_phase += d_phase;
  }
#endif
}

StsResult WifiDsp::detect_sts(const c32* iq, size_t len) const {
  StsResult res;
  if (len < 64) return res;

  // Schmidl & Cox Delay-16 autocorrelation: P[d] = sum_{m=0..15} r*[d+m] * r[d+m+16]
  c32 P(0.f, 0.f);
  c32 S1(0.f, 0.f), S2(0.f, 0.f);
  float R1 = 0.f, R2 = 0.f;
  for (size_t m = 0; m < 16; ++m) {
    P += std::conj(iq[m]) * iq[m + 16];
    S1 += iq[m];
    S2 += iq[m + 16];
    R1 += std::norm(iq[m]);
    R2 += std::norm(iq[m + 16]);
  }

  int consec = 0;
  for (size_t d = 0; d + 32 < len; ++d) {
    if ((d & 63) == 0) {
      P = c32(0.f, 0.f);
      S1 = c32(0.f, 0.f);
      S2 = c32(0.f, 0.f);
      R1 = 0.f;
      R2 = 0.f;
      for (size_t m = 0; m < 16; ++m) {
        P += std::conj(iq[d + m]) * iq[d + m + 16];
        S1 += iq[d + m];
        S2 += iq[d + m + 16];
        R1 += std::norm(iq[d + m]);
        R2 += std::norm(iq[d + m + 16]);
      }
    }

    // Zero-mean covariance and variance (DC offset cancellation)
    constexpr float inv16 = 1.f / 16.f;
    const c32 P_zm = P - std::conj(S1) * S2 * inv16;
    const float R1_zm = std::max(0.f, R1 - std::norm(S1) * inv16);
    const float R2_zm = std::max(0.f, R2 - std::norm(S2) * inv16);

    const float pwr_avg = 0.5f * (R1_zm + R2_zm) * inv16;
    if (pwr_avg > cfg_.cca_threshold) {
      const float p_mag2 = std::norm(P_zm);
      const float denom = R1_zm * R2_zm;
      const float m = (denom > 1e-12f) ? (p_mag2 / denom) : 0.f;

      if (m > cfg_.sts_threshold && m <= 1.05f) {
        ++consec;
        if (consec >= cfg_.sts_consecutive) {
          // Reject narrowband CW / LO-leakage transients: a 12-tone 802.11a STS spans
          // +-7.5 MHz so its lag-1 autocorrelation magnitude-squared is ~0.01..0.20,
          // whereas CW / LO leakage within +-1.5 MHz has m_lag1 > 0.70.
          const c32 mu = (S1 + S2) * (0.5f * inv16);
          c32 p1(0.f, 0.f);
          float r0 = 0.f;
          for (size_t k = 0; k < 31; ++k) {
            const c32 a = iq[d + k] - mu;
            const c32 b = iq[d + k + 1] - mu;
            p1 += std::conj(a) * b;
            r0 += std::norm(a);
          }
          const float m_lag1 = (r0 > 1e-12f) ? (std::norm(p1) / (r0 * r0)) : 1.f;
          if (m_lag1 < 0.50f) {
            res.trig_idx = static_cast<int>(d);
            res.P_at_trig = P_zm;
            res.R_at_trig = pwr_avg * 16.f;
            res.metric_at_trig = m;
            return res;
          }
        }
      } else {
        consec = 0;
      }
    } else {
      consec = 0;
    }

    // Sliding update O(1)
    P += std::conj(iq[d + 16]) * iq[d + 32] - std::conj(iq[d]) * iq[d + 16];
    S1 += iq[d + 16] - iq[d];
    S2 += iq[d + 32] - iq[d + 16];
    R1 += std::norm(iq[d + 16]) - std::norm(iq[d]);
    R2 += std::norm(iq[d + 32]) - std::norm(iq[d + 16]);
    if (R1 < 0.f) R1 = 0.f;
    if (R2 < 0.f) R2 = 0.f;
  }

  return res;
}

int WifiDsp::find_dual_ltf(const c32* iq, size_t len, float* out_peak) const {
  if (len < 160) {
    if (out_peak) *out_peak = 0.f;
    return -1;
  }

  float max_score = 0.f;
  int max_i = -1;

  // Search window: LTF starts within 320 samples of STS trigger
  const size_t max_search = std::min<size_t>(len - 160, 320);

  for (size_t i = 0; i < max_search; ++i) {
    // 1. Dual-symbol repetition autocorrelation (immune to CFO)
    c32 p_rep(0.f, 0.f);
    float pwr1 = 0.f, pwr2 = 0.f;
    for (int k = 0; k < 64; ++k) {
      const c32 s1 = iq[i + 32 + k];       // LTF1 (skipping 32 CP)
      const c32 s2 = iq[i + 32 + 64 + k];  // LTF2
      p_rep += std::conj(s1) * s2;
      pwr1 += std::norm(s1);
      pwr2 += std::norm(s2);
    }
    const float rep_denom = 0.5f * (pwr1 + pwr2);
    if (rep_denom < 1e-9f) continue;
    const float rep_metric = std::abs(p_rep) / rep_denom;

    // 2. Coherent matched filter derotated by repetition phase
    const float fine_phase_step = std::arg(p_rep) / 64.f;
    c32 sum1(0.f, 0.f), sum2(0.f, 0.f);
    for (int k = 0; k < 64; ++k) {
      const float derot_ang1 = -fine_phase_step * static_cast<float>(k);
      const float derot_ang2 = -fine_phase_step * static_cast<float>(k + 64);
      const c32 s1_c = iq[i + 32 + k] * c32(std::cos(derot_ang1), std::sin(derot_ang1));
      const c32 s2_c = iq[i + 32 + 64 + k] * c32(std::cos(derot_ang2), std::sin(derot_ang2));
      const c32 ref = ltf_ref_.td[static_cast<size_t>(k)];
      sum1 += s1_c * std::conj(ref);
      sum2 += s2_c * std::conj(ref);
    }
    const float mf_score = (std::norm(sum1) + std::norm(sum2)) / (pwr1 + pwr2 + 1e-9f);

    // Combined score: repetition metric * matched filter score
    const float score = rep_metric * mf_score;
    if (score > max_score) {
      max_score = score;
      max_i = static_cast<int>(i);
    }
  }

  if (out_peak) *out_peak = max_score;
  if (max_score < cfg_.min_ltf_peak || max_i < 0) return -1;

  return max_i + 32;
}

std::vector<int> WifiDsp::make_interleaver_map(int n_cbps, int n_bpsc) {
  std::vector<int> map(static_cast<size_t>(n_cbps));
  const int s = std::max(1, n_bpsc / 2);
  for (int k = 0; k < n_cbps; ++k) {
    const int i = (n_cbps / 16) * (k % 16) + (k / 16);
    const int j = s * (i / s) + (i + n_cbps - (16 * i) / n_cbps) % s;
    map[static_cast<size_t>(k)] = j;
  }
  return map;
}

std::vector<int> WifiDsp::make_deinterleaver_map(int n_cbps, int n_bpsc) {
  // To deinterleave where deint_llrs[k] = raw_llrs[map[k]],
  // the map is directly the forward interleaver permutation:
  // since TX placed coded bit k at subcarrier bit index fwd[k],
  // the soft bit for coded bit k is gathered from raw_llrs[fwd[k]].
  return make_interleaver_map(n_cbps, n_bpsc);
}

void WifiDsp::soft_demap(const c32* eq_data48, const float* h_weight, int n_bpsc, float* llr_out) {
  constexpr float kMaxLlr = 8.0f;
  switch (n_bpsc) {
    case 1: {  // BPSK
      for (int i = 0; i < 48; ++i) {
        llr_out[i] = std::clamp(8.f * h_weight[i] * eq_data48[i].real(), -kMaxLlr, kMaxLlr);
      }
      break;
    }
    case 2: {  // QPSK
      constexpr float kNorm = 1.41421356f;  // sqrt(2)
      for (int i = 0; i < 48; ++i) {
        const float w = 6.f * kNorm * h_weight[i];
        llr_out[2 * i + 0] = std::clamp(w * eq_data48[i].real(), -kMaxLlr, kMaxLlr);
        llr_out[2 * i + 1] = std::clamp(w * eq_data48[i].imag(), -kMaxLlr, kMaxLlr);
      }
      break;
    }
    case 4: {  // 16-QAM
      constexpr float th = 0.63245553f;  // 2 / sqrt(10)
      for (int i = 0; i < 48; ++i) {
        const float w = 6.f * h_weight[i];
        const float r = eq_data48[i].real();
        const float im = eq_data48[i].imag();
        llr_out[4 * i + 0] = std::clamp(-w * r, -kMaxLlr, kMaxLlr);
        llr_out[4 * i + 1] = std::clamp(w * (std::abs(r) - th), -kMaxLlr, kMaxLlr);
        llr_out[4 * i + 2] = std::clamp(-w * im, -kMaxLlr, kMaxLlr);
        llr_out[4 * i + 3] = std::clamp(w * (std::abs(im) - th), -kMaxLlr, kMaxLlr);
      }
      break;
    }
    case 6: {  // 64-QAM
      constexpr float th4 = 0.6172134f;  // 4 / sqrt(42)
      constexpr float th2 = 0.3086067f;  // 2 / sqrt(42)
      for (int i = 0; i < 48; ++i) {
        const float w = 6.f * h_weight[i];
        const float r = eq_data48[i].real();
        const float im = eq_data48[i].imag();
        llr_out[6 * i + 0] = std::clamp(-w * r, -kMaxLlr, kMaxLlr);
        llr_out[6 * i + 1] = std::clamp(w * (std::abs(r) - th4), -kMaxLlr, kMaxLlr);
        llr_out[6 * i + 2] = std::clamp(w * (std::abs(std::abs(r) - th4) - th2), -kMaxLlr, kMaxLlr);
        llr_out[6 * i + 3] = std::clamp(-w * im, -kMaxLlr, kMaxLlr);
        llr_out[6 * i + 4] = std::clamp(w * (std::abs(im) - th4), -kMaxLlr, kMaxLlr);
        llr_out[6 * i + 5] = std::clamp(w * (std::abs(std::abs(im) - th4) - th2), -kMaxLlr, kMaxLlr);
      }
      break;
    }
    default:
      break;
  }
}

FrameResult WifiDsp::decode_burst(std::span<const c32> raw_iq, const StsResult* known_sts) {
  FrameResult fr;
  if (raw_iq.size() < 400) return fr;

  std::vector<c32> iq(raw_iq.begin(), raw_iq.end());

  float coarse_rad = 0.f;
  if (known_sts && known_sts->trig_idx >= 0) {
    fr.sts_idx = known_sts->trig_idx;
    // Multi-sample correlation over steady STS starting at the trigger anchor with DC cancellation
    const size_t trig_in_burst = (known_sts->trig_idx >= 32) ? 32 : static_cast<size_t>(known_sts->trig_idx);
    c32 p_coarse(0.f, 0.f), sum_s1(0.f, 0.f), sum_s2(0.f, 0.f);
    const size_t c_end = std::min(iq.size() - 16, trig_in_burst + 48);
    int cnt = 0;
    for (size_t m = trig_in_burst; m < c_end; ++m) {
      p_coarse += std::conj(iq[m]) * iq[m + 16];
      sum_s1 += iq[m];
      sum_s2 += iq[m + 16];
      ++cnt;
    }
    if (cnt > 0) {
      p_coarse -= std::conj(sum_s1) * sum_s2 * (1.f / static_cast<float>(cnt));
      if (std::norm(p_coarse) > 1e-12f) {
        coarse_rad = std::arg(p_coarse) / 16.f;
      }
    }
  } else {
    // STS Detection & Coarse CFO for standalone burst buffers
    const StsResult sts = detect_sts(iq.data(), iq.size());
    if (sts.trig_idx < 0) return fr;
    fr.sts_idx = sts.trig_idx;
    c32 p_coarse(0.f, 0.f), sum_s1(0.f, 0.f), sum_s2(0.f, 0.f);
    const size_t s_start = static_cast<size_t>(sts.trig_idx);
    const size_t s_end = std::min(iq.size() - 16, s_start + 48);
    int cnt = 0;
    for (size_t m = s_start; m < s_end; ++m) {
      p_coarse += std::conj(iq[m]) * iq[m + 16];
      sum_s1 += iq[m];
      sum_s2 += iq[m + 16];
      ++cnt;
    }
    if (cnt > 0) {
      p_coarse -= std::conj(sum_s1) * sum_s2 * (1.f / static_cast<float>(cnt));
      if (std::norm(p_coarse) > 1e-12f) {
        coarse_rad = std::arg(p_coarse) / 16.f;
      }
    }
  }
  // Dual LTF has an unambiguous range of +/-156.25 kHz. If coarse CFO is within +/-150 kHz,
  // bypass STS coarse CFO to eliminate STS boundary jitter and let LTF measure CFO cleanly.
  if (std::abs(coarse_rad * FS / kTwoPi) < 150000.f) {
    coarse_rad = 0.f;
  }
  fr.coarse_cfo_hz = coarse_rad * FS / kTwoPi;
  derotate(iq.data(), iq.size(), coarse_rad, 0.f);

  // Dual LTF Timing & Fine CFO
  float ltf_peak = 0.f;
  const int ltf1 = find_dual_ltf(iq.data(), iq.size(), &ltf_peak);
  fr.ltf_peak = ltf_peak;
  if (ltf1 < 0 || ltf1 + 208 >= static_cast<int>(iq.size())) return fr;

  constexpr int kFftAdvance = 4;

  auto try_decode = [&](int cur_ltf1) -> FrameResult {
    FrameResult fr;
    std::vector<c32> iq_local = iq;
    fr.sts_idx = (known_sts && known_sts->trig_idx >= 0) ? known_sts->trig_idx : -1;
    fr.ltf1_idx = cur_ltf1;
    fr.ltf_peak = ltf_peak;
    fr.coarse_cfo_hz = coarse_rad * FS / kTwoPi;

    // Fine CFO from LTF1 vs LTF2 for this specific timing hypothesis
    c32 p_fine(0.f, 0.f);
    for (int i = 0; i < 64; ++i) {
      p_fine += std::conj(iq_local[static_cast<size_t>(cur_ltf1 + i)]) *
                iq_local[static_cast<size_t>(cur_ltf1 + 64 + i)];
    }
    const float fine_rad = std::arg(p_fine) / 64.f;
    fr.fine_cfo_hz = fine_rad * FS / kTwoPi;
    derotate(iq_local.data(), iq_local.size(), fine_rad, static_cast<float>(cur_ltf1 + 64 - kFftAdvance));

    auto compute_channel_estimate = [&](c32* out_H_smooth, float& out_avg_h2, float& out_mmse_reg, float& out_snr_db) {
      c32 ltf1_fft[64], ltf2_fft[64];
      fft64(&iq_local[static_cast<size_t>(cur_ltf1 - kFftAdvance)], ltf1_fft);
      fft64(&iq_local[static_cast<size_t>(cur_ltf1 + 64 - kFftAdvance)], ltf2_fft);
      ltf1_fft[0] = c32(0.f, 0.f);
      ltf2_fft[0] = c32(0.f, 0.f);

      c32 H_raw[64]{};
      for (int i = 0; i < 53; ++i) {
        const int k = i - 26;
        if (k == 0) continue;
        const int bin = (k + 64) % 64;
        const c32 avg_y = 0.5f * (ltf1_fft[bin] + ltf2_fft[bin]);
        H_raw[bin] = avg_y / L_STD[i];
      }

      if (cfg_.enable_cir_smoothing) {
        // Extrapolate DC and 5 virtual guard subcarriers on each band edge with a raised-cosine
        // taper and phase slope continuity so 64-pt IFFT CIR windowing does not suffer Gibbs ringing.
        c32 H_ext[64]{};
        std::memcpy(H_ext, H_raw, sizeof(H_raw));
        H_ext[0] = 0.5f * (H_raw[1] + H_raw[63]);
        const float dphi_pos = std::arg(H_raw[26] * std::conj(H_raw[25]));
        const float dphi_neg = std::arg(H_raw[38] * std::conj(H_raw[39]));
        for (int d = 1; d <= 5; ++d) {
          const float w = 0.5f * (1.f + std::cos(kPi * static_cast<float>(d) / 6.f));
          const float ang_pos = dphi_pos * static_cast<float>(d);
          const float ang_neg = dphi_neg * static_cast<float>(d);
          H_ext[26 + d] = w * H_raw[26] * c32(std::cos(ang_pos), std::sin(ang_pos));
          H_ext[38 - d] = w * H_raw[38] * c32(std::cos(ang_neg), std::sin(ang_neg));
        }
        H_ext[32] = c32(0.f, 0.f);

        c32 cir[64], cir_w[64]{};
        ifft64(H_ext, cir);
        for (int i = 0; i <= 8; ++i) cir_w[i] = cir[i];
        cir_w[9]  = cir[9] * 0.85f;
        cir_w[10] = cir[10] * 0.60f;
        cir_w[11] = cir[11] * 0.30f;
        cir_w[12] = cir[12] * 0.10f;
        cir_w[61] = cir[61] * 0.30f;
        cir_w[62] = cir[62] * 0.60f;
        cir_w[63] = cir[63] * 0.85f;
        fft64(cir_w, out_H_smooth);
        out_H_smooth[0] = c32(0.f, 0.f);
      } else {
        std::memcpy(out_H_smooth, H_raw, sizeof(H_raw));
      }

      float pwr_sig = 0.f, pwr_noise = 0.f;
      for (int k = -26; k <= 26; ++k) {
        if (k == 0) continue;
        const int bin = (k + 64) % 64;
        pwr_sig += std::norm(out_H_smooth[bin]);
        const c32 diff = (ltf1_fft[bin] - ltf2_fft[bin]) * 0.70710678f;
        pwr_noise += std::norm(diff);
      }
      out_snr_db = 10.f * std::log10((pwr_sig + 1e-9f) / (pwr_noise + 1e-9f));
      out_avg_h2 = (pwr_sig + 1e-6f) / 52.f;
      const float noise_var = (pwr_noise + 1e-9f) / 52.f;
      out_mmse_reg = std::clamp(noise_var, 1e-4f * out_avg_h2, 0.15f * out_avg_h2);
    };

    c32 H_smooth[64]{};
    float avg_h2 = 1.f, mmse_reg = 1e-3f;
    compute_channel_estimate(H_smooth, avg_h2, mmse_reg, fr.snr_db);

    const size_t td_len = std::min<size_t>(256, iq_local.size());
    fr.time_iq.assign(iq_local.begin(), iq_local.begin() + td_len);

    float max_peak = 0.f;
    for (const auto& s : fr.time_iq) {
      max_peak = std::max(max_peak, std::max(std::abs(s.real()), std::abs(s.imag())));
    }
    fr.peak_cs8 = static_cast<int>(std::round(max_peak * 127.f));

    // SIGNAL Symbol (starts at cur_ltf1 + 144 - kFftAdvance)
    const int sig_start = cur_ltf1 + 144 - kFftAdvance;
    if (sig_start + 64 > static_cast<int>(iq_local.size())) return fr;
    c32 sig_fft[64], sig_eq[64];
    fft64(&iq_local[static_cast<size_t>(sig_start)], sig_fft);
    sig_fft[0] = c32(0.f, 0.f);
    mmse_equalize(sig_fft, H_smooth, sig_eq, mmse_reg);

    // Pilot correction on SIGNAL symbol: pure CPE without artificial slope (symbol immediately follows LTF)
    c32 sum_p_sig(0.f, 0.f);
    for (int i = 0; i < 4; ++i) {
      sum_p_sig += sig_fft[PILOT_SC[i]] * std::conj(H_smooth[PILOT_SC[i]]) * BASE_PILOT_POLARITY[i];
    }
    const float cpe_sig = std::arg(sum_p_sig);
    apply_phase_correction(sig_eq, cpe_sig, 0.f);

    // Demap SIGNAL symbol (BPSK) with MMSE channel weights
    float sig_weight[48];
    c32 sig_data[48];
    for (int i = 0; i < 48; ++i) {
      const float h2 = std::norm(H_smooth[DATA_SC[i]]);
      sig_weight[i] = h2 / (h2 + mmse_reg);
      sig_data[i] = sig_eq[DATA_SC[i]];
    }
    float sig_llrs[48];
    soft_demap(sig_data, sig_weight, 1, sig_llrs);

    const auto deint48 = make_deinterleaver_map(48, 1);
    float sig_deint[48];
    for (int i = 0; i < 48; ++i) {
      sig_deint[i] = sig_llrs[deint48[static_cast<size_t>(i)]];
    }

    const auto sig_bits = viterbi_.decode(sig_deint, true);
    if (sig_bits.size() < 24) return fr;

    // Parse RATE (bits 0..3) and LENGTH (bits 5..16)
    int rate_val = 0;
    for (int b = 0; b < 4; ++b) rate_val |= (int(sig_bits[static_cast<size_t>(b)]) << b);
    int pkt_len = 0;
    for (int b = 0; b < 12; ++b) pkt_len |= (int(sig_bits[static_cast<size_t>(5 + b)]) << b);

    // Parity check
    int par = 0;
    for (int b = 0; b < 17; ++b) par += sig_bits[static_cast<size_t>(b)];
    if ((par & 1) != sig_bits[17]) return fr;

    // Reserved bit 4 must be 0, tail bits 18..23 must be 0
    if (sig_bits[4] != 0) return fr;
    for (int b = 18; b < 24; ++b) {
      if (sig_bits[static_cast<size_t>(b)] != 0) return fr;
    }

    const RateInfo* rate_info = rate_info_for(rate_val);
    if (!rate_info || pkt_len <= 0 || pkt_len > 2048) return fr;

    fr.signal_ok = true;
    fr.rate_val = rate_val;
    fr.rate_name = rate_info->name;
    fr.pkt_len = pkt_len;

    const int n_cbps = rate_info->n_cbps;
    const int n_bpsc = rate_info->n_bpsc;
    const int n_dbps = rate_info->n_dbps;

    // Calculate required data OFDM symbols
    const int total_data_bits = 16 + 8 * pkt_len + 6;
    const int n_syms = (total_data_bits + n_dbps - 1) / n_dbps;
    const int data_start = cur_ltf1 + 224 - kFftAdvance;
    fr.needed_samples = data_start + n_syms * 80;
    if (static_cast<size_t>(fr.needed_samples) > iq_local.size()) return fr;

    // Pass 1: Measure initial pilot phases across symbols and fit linear residual CFO slope
    std::vector<float> pilot_phases(static_cast<size_t>(n_syms), 0.f);
    for (int si = 0; si < n_syms; ++si) {
      const int ss = data_start + si * 80;
      c32 Y[64];
      fft64(&iq_local[static_cast<size_t>(ss)], Y);
      const float pilot_pol = static_cast<float>(PILOT_SEQ[(si + 1) % 127]);
      c32 sum_p(0.f, 0.f);
      for (int i = 0; i < 4; ++i) {
        sum_p += Y[PILOT_SC[i]] * std::conj(H_smooth[PILOT_SC[i]]) * (BASE_PILOT_POLARITY[i] * pilot_pol);
      }
      pilot_phases[static_cast<size_t>(si)] = std::arg(sum_p);
    }

    float resid_slope_per_sym = 0.f;
    if (n_syms >= 2) {
      std::vector<float> unwrap_ph(static_cast<size_t>(n_syms));
      unwrap_ph[0] = pilot_phases[0];
      for (int i = 1; i < n_syms; ++i) {
        const float d = std::remainder(pilot_phases[static_cast<size_t>(i)] - unwrap_ph[static_cast<size_t>(i - 1)], kTwoPi);
        unwrap_ph[static_cast<size_t>(i)] = unwrap_ph[static_cast<size_t>(i - 1)] + d;
      }
      const float x_mean = 0.5f * static_cast<float>(n_syms - 1);
      float y_mean = 0.f;
      for (int i = 0; i < n_syms; ++i) y_mean += unwrap_ph[static_cast<size_t>(i)];
      y_mean /= static_cast<float>(n_syms);
      float num = 0.f, den = 0.f;
      for (int i = 0; i < n_syms; ++i) {
        const float dx = static_cast<float>(i) - x_mean;
        num += dx * (unwrap_ph[static_cast<size_t>(i)] - y_mean);
        den += dx * dx;
      }
      if (den > 1e-9f) resid_slope_per_sym = num / den;
    }

    const float resid_rad = resid_slope_per_sym / 80.f;
    fr.resid_cfo_hz = resid_rad * FS / kTwoPi;
    if (std::abs(resid_rad) > 1e-7f) {
      // Remove residual CFO in the time domain before final LTF channel estimation and Data FFTs
      // to eliminate Inter-Carrier Interference (ICI) and residual phase ramp.
      derotate(iq_local.data(), iq_local.size(), resid_rad, static_cast<float>(cur_ltf1 + 64 - kFftAdvance));
      compute_channel_estimate(H_smooth, avg_h2, mmse_reg, fr.snr_db);
      fr.fine_cfo_hz += fr.resid_cfo_hz;
    }

    fr.H_mag.resize(52);
    fr.H_phase.resize(52);
    for (int i = 0; i < 52; ++i) {
      const int k = (i < 26) ? (i - 26) : (i - 25);
      const int bin = (k + 64) % 64;
      const float mag = std::abs(H_smooth[bin]);
      const float ph = std::arg(H_smooth[bin]) * (180.f / float(M_PI));
      fr.H_mag[i] = 20.f * std::log10(mag + 1e-6f);
      fr.H_phase[i] = ph;
    }

    auto snap_ideal = [&](const c32& d) -> c32 {
      if (rate_info->mod == Modulation::BPSK) {
        return c32((d.real() >= 0.f) ? 1.f : -1.f, 0.f);
      } else if (rate_info->mod == Modulation::QPSK) {
        return c32((d.real() >= 0.f) ? 0.70710678f : -0.70710678f,
                   (d.imag() >= 0.f) ? 0.70710678f : -0.70710678f);
      } else if (rate_info->mod == Modulation::QAM16) {
        return c32(snap_1d(d.real(), qam16_levels, 4), snap_1d(d.imag(), qam16_levels, 4));
      } else if (rate_info->mod == Modulation::QAM64) {
        return c32(snap_1d(d.real(), qam64_levels, 8), snap_1d(d.imag(), qam64_levels, 8));
      }
      return c32((d.real() >= 0.f) ? 1.f : -1.f, 0.f);
    };

    // Pass 2: Equalize all symbols, track pilot + decision-directed CPE and timing slope (SRO)
    std::vector<std::array<c32, 48>> eq_matrix(static_cast<size_t>(n_syms));
    fr.pilot_cpe.resize(static_cast<size_t>(n_syms), 0.f);
    c32 sum_dd_sc[48]{};
    float sum_ref2_sc[48]{};

    for (int si = 0; si < n_syms; ++si) {
      const int ss = data_start + si * 80;
      c32 Y[64], eq[64];
      fft64(&iq_local[static_cast<size_t>(ss)], Y);
      Y[0] = c32(0.f, 0.f);
      mmse_equalize(Y, H_smooth, eq, mmse_reg);

      const float pilot_pol = static_cast<float>(PILOT_SEQ[(si + 1) % 127]);
      c32 sum_p(0.f, 0.f);
      for (int i = 0; i < 4; ++i) {
        sum_p += Y[PILOT_SC[i]] * std::conj(H_smooth[PILOT_SC[i]]) * (BASE_PILOT_POLARITY[i] * pilot_pol);
      }
      const float cpe0 = std::arg(sum_p);
      apply_phase_correction(eq, cpe0, 0.f);

      // Decision-directed CPE & SRO timing-slope refinement weighted by subcarrier channel power
      c32 dd_acc(0.f, 0.f);
      float k_num = 0.f, k_den = 0.f;
      for (int i = 0; i < 4; ++i) {
        const int pb = PILOT_SC[i];
        const float w_b = std::norm(H_smooth[pb]) / (avg_h2 + 1e-9f);
        const c32 z_err = eq[pb] * (BASE_PILOT_POLARITY[i] * pilot_pol);
        dd_acc += 2.f * w_b * z_err;
        const float ks = fft_bin_k(pb);
        k_num += 2.f * w_b * ks * std::arg(z_err);
        k_den += 2.f * w_b * ks * ks;
      }
      for (int i = 0; i < 48; ++i) {
        const int db = DATA_SC[i];
        const float w_b = std::norm(H_smooth[db]) / (avg_h2 + 1e-9f);
        const c32 ideal = snap_ideal(eq[db]);
        const c32 z_err = eq[db] * std::conj(ideal);
        dd_acc += w_b * z_err;
        const float ks = fft_bin_k(db);
        k_num += w_b * ks * std::arg(z_err);
        k_den += w_b * ks * ks;
      }
      const float cpe_dd = std::clamp(std::arg(dd_acc), -0.35f, 0.35f);
      const float sro_slope = (k_den > 1e-6f) ? std::clamp(k_num / k_den, -0.025f, 0.025f) : 0.f;
      apply_phase_correction(eq, cpe_dd, sro_slope);
      fr.pilot_cpe[static_cast<size_t>(si)] = std::remainder(cpe0 + cpe_dd, kTwoPi);

      for (int i = 0; i < 48; ++i) {
        const c32 d = eq[DATA_SC[i]];
        eq_matrix[static_cast<size_t>(si)][static_cast<size_t>(i)] = d;
        const c32 ideal = snap_ideal(d);
        sum_dd_sc[i] += d * std::conj(ideal);
        sum_ref2_sc[i] += std::norm(ideal);
      }
    }

    // Per-subcarrier residual complex channel refinement across the packet (damped to prevent overfitting)
    c32 corr_sc[48];
    for (int i = 0; i < 48; ++i) {
      if (n_syms >= 4 && sum_ref2_sc[i] > 1e-6f) {
        const c32 g_hat = sum_dd_sc[i] / sum_ref2_sc[i];
        const c32 g_blend = c32(0.45f, 0.f) + 0.55f * g_hat;
        const float mag = std::clamp(std::abs(g_blend), 0.70f, 1.45f);
        const float ph = std::clamp(std::arg(g_blend), -0.30f, 0.30f);
        const float inv_mag = 1.f / mag;
        corr_sc[i] = c32(inv_mag * std::cos(-ph), inv_mag * std::sin(-ph));
      } else {
        corr_sc[i] = c32(1.f, 0.f);
      }
    }

    std::vector<float> raw_llrs;
    raw_llrs.reserve(static_cast<size_t>(n_syms * n_cbps));
    fr.eq_symbols.reserve(static_cast<size_t>(n_syms * 48));

    float h_sq[48], h_weight[48];
    for (int i = 0; i < 48; ++i) {
      h_sq[i] = std::norm(H_smooth[DATA_SC[i]]);
      h_weight[i] = h_sq[i] / (h_sq[i] + mmse_reg);
    }

    for (int si = 0; si < n_syms; ++si) {
      c32 sym_data[48];
      for (int i = 0; i < 48; ++i) {
        sym_data[i] = eq_matrix[static_cast<size_t>(si)][static_cast<size_t>(i)] * corr_sc[i];
        fr.eq_symbols.push_back(sym_data[i]);
      }

      std::vector<float> sym_llrs(static_cast<size_t>(n_cbps));
      soft_demap(sym_data, h_weight, n_bpsc, sym_llrs.data());
      raw_llrs.insert(raw_llrs.end(), sym_llrs.begin(), sym_llrs.end());
    }

    // Compute EVM across all equalized symbols
    fr.evm_db = compute_evm_db(fr.eq_symbols, rate_info->mod);

    // Deinterleave
    const auto deint_map = make_deinterleaver_map(n_cbps, n_bpsc);
    std::vector<float> deint_llrs(raw_llrs.size());
    for (int s = 0; s < n_syms; ++s) {
      for (int j = 0; j < n_cbps; ++j) {
        deint_llrs[static_cast<size_t>(s * n_cbps + j)] =
            raw_llrs[static_cast<size_t>(s * n_cbps + deint_map[static_cast<size_t>(j)])];
      }
    }

    // Depuncture
    const auto depunct = WifiViterbi::depuncture(deint_llrs, rate_info->rate_num, rate_info->rate_den);

    // Multi-stage Viterbi decode and verification
    const size_t useful_pairs = static_cast<size_t>(16 + pkt_len * 8 + 6);
    std::vector<uint8_t> decoded_bits_z0;
    if (depunct.size() >= useful_pairs * 2) {
      decoded_bits_z0 = viterbi_.decode(std::span<const float>(depunct.data(), useful_pairs * 2), true);
    } else {
      decoded_bits_z0 = viterbi_.decode(depunct, true);
    }
    if (decoded_bits_z0.size() < 16) return fr;

    // Descramble payload & verify FCS with seed fallback search
    auto check_payload = [&](const std::vector<uint8_t>& descrambled_bits, std::vector<uint8_t>& out_payload) -> bool {
      if (descrambled_bits.size() < 16 + static_cast<size_t>(pkt_len * 8)) return false;
      out_payload.resize(static_cast<size_t>(pkt_len));
      for (int i = 0; i < pkt_len; ++i) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; ++b) {
          byte |= (descrambled_bits[static_cast<size_t>(16 + i * 8 + b)] << b);
        }
        out_payload[static_cast<size_t>(i)] = byte;
      }
      if (out_payload.size() < 4) return false;
      const size_t body_len = out_payload.size() - 4;
      const uint32_t calc_crc = wifi_crc32(out_payload.data(), body_len);
      uint32_t rx_crc = 0;
      std::memcpy(&rx_crc, out_payload.data() + body_len, 4);
      return (calc_crc == rx_crc);
    };

    auto find_best_seeds = [](const std::vector<uint8_t>& dec_bits) -> std::vector<int> {
      std::vector<std::pair<int, int>> scored;
      scored.reserve(127);
      for (int s = 1; s <= 127; ++s) {
        int errs = 0;
        int state = s & 0x7F;
        for (size_t i = 0; i < 7 && i < dec_bits.size(); ++i) {
          const int fb = ((state >> 6) ^ (state >> 3)) & 1;
          state = ((state << 1) | fb) & 0x7F;
          if ((dec_bits[i] ^ fb) != 0) ++errs;
        }
        scored.push_back({errs, s});
      }
      std::sort(scored.begin(), scored.end());
      std::vector<int> seeds;
      for (const auto& p : scored) {
        if (p.first <= 2) seeds.push_back(p.second);
      }
      if (seeds.empty() && !scored.empty()) seeds.push_back(scored[0].second);
      return seeds;
    };

    auto try_verify = [&](const std::vector<uint8_t>& dec_bits) -> bool {
      // 1. Standard self-synchronizing descrambler
      const auto desc_self = descramble(dec_bits);
      if (check_payload(desc_self, fr.payload)) return true;

      // 2. Test seeds ranked by SERVICE field zero-error matching
      const auto ranked_seeds = find_best_seeds(dec_bits);
      for (int s : ranked_seeds) {
        const auto desc_s = descramble(dec_bits, s);
        if (check_payload(desc_s, fr.payload)) return true;
      }

      // 3. Fallback: test all remaining non-zero 7-bit seeds
      for (int s = 1; s <= 127; ++s) {
        const auto desc_s = descramble(dec_bits, s);
        if (check_payload(desc_s, fr.payload)) return true;
      }

      // 4. Chase Combining on the highest-confidence recovered seed payload
      int best_seed = ranked_seeds.empty() ? 0x5D : ranked_seeds[0];
      std::vector<uint8_t> payload;
      check_payload(descramble(dec_bits, best_seed), payload);
      if (payload.empty()) {
        check_payload(desc_self, payload);
      }
      if (!payload.empty() && payload.size() >= 4) {
        const size_t body_len = payload.size() - 4;

        // 4a. Single-bit flip across entire payload and CRC
        for (size_t byte_idx = 0; byte_idx < payload.size(); ++byte_idx) {
          for (int bit = 0; bit < 8; ++bit) {
            payload[byte_idx] ^= (1 << bit);
            const uint32_t calc_crc = wifi_crc32(payload.data(), body_len);
            uint32_t rx_crc = 0;
            std::memcpy(&rx_crc, payload.data() + body_len, 4);
            if (calc_crc == rx_crc) {
              fr.payload = payload;
              return true;
            }
            payload[byte_idx] ^= (1 << bit);
          }
        }

        // 4b. 2-bit flips on low-confidence bits
        const size_t total_uncoded = std::min<size_t>(dec_bits.size(), static_cast<size_t>(16 + pkt_len * 8));
        if (total_uncoded > 16 && !depunct.empty()) {
          std::vector<std::pair<float, size_t>> conf;
          conf.reserve(total_uncoded - 16);
          for (size_t b = 16; b < total_uncoded; ++b) {
            float c = 100.f;
            if (2 * b + 1 < depunct.size()) {
              c = std::abs(depunct[2 * b]) + std::abs(depunct[2 * b + 1]);
            }
            conf.push_back({c, b - 16});
          }
          std::sort(conf.begin(), conf.end());

          const size_t n_search2 = std::min<size_t>(conf.size(), 36);
          for (size_t i = 0; i < n_search2; ++i) {
            const size_t b1 = conf[i].second;
            if (b1 / 8 >= payload.size()) continue;
            payload[b1 / 8] ^= (1 << (b1 % 8));
            for (size_t j = i + 1; j < n_search2; ++j) {
              const size_t b2 = conf[j].second;
              if (b2 / 8 >= payload.size()) continue;
              payload[b2 / 8] ^= (1 << (b2 % 8));
              const uint32_t calc_crc = wifi_crc32(payload.data(), body_len);
              uint32_t rx_crc = 0;
              std::memcpy(&rx_crc, payload.data() + body_len, 4);
              if (calc_crc == rx_crc) {
                fr.payload = payload;
                return true;
              }
              payload[b2 / 8] ^= (1 << (b2 % 8));
            }
            payload[b1 / 8] ^= (1 << (b1 % 8));
          }

          // 4c. 3-bit flips on top 14 lowest-confidence bits
          const size_t n_search3 = std::min<size_t>(conf.size(), 14);
          for (size_t i = 0; i < n_search3; ++i) {
            const size_t b1 = conf[i].second;
            if (b1 / 8 >= payload.size()) continue;
            payload[b1 / 8] ^= (1 << (b1 % 8));
            for (size_t j = i + 1; j < n_search3; ++j) {
              const size_t b2 = conf[j].second;
              if (b2 / 8 >= payload.size()) continue;
              payload[b2 / 8] ^= (1 << (b2 % 8));
              for (size_t k = j + 1; k < n_search3; ++k) {
                const size_t b3 = conf[k].second;
                if (b3 / 8 >= payload.size()) continue;
                payload[b3 / 8] ^= (1 << (b3 % 8));
                const uint32_t calc_crc = wifi_crc32(payload.data(), body_len);
                uint32_t rx_crc = 0;
                std::memcpy(&rx_crc, payload.data() + body_len, 4);
                if (calc_crc == rx_crc) {
                  fr.payload = payload;
                  return true;
                }
                payload[b3 / 8] ^= (1 << (b3 % 8));
              }
              payload[b2 / 8] ^= (1 << (b2 % 8));
            }
            payload[b1 / 8] ^= (1 << (b1 % 8));
          }
        }
      }

      return false;
    };

    // Stage A: Constrained Viterbi (force state 0 at tail)
    if (try_verify(decoded_bits_z0)) {
      fr.fcs_ok = true;
      return fr;
    }

    // Stage B: Unconstrained Viterbi traceback (fallback if tail bit was damaged)
    std::vector<uint8_t> decoded_bits_free;
    if (depunct.size() >= useful_pairs * 2) {
      decoded_bits_free = viterbi_.decode(std::span<const float>(depunct.data(), useful_pairs * 2), false);
    } else {
      decoded_bits_free = viterbi_.decode(depunct, false);
    }
    if (!decoded_bits_free.empty() && try_verify(decoded_bits_free)) {
      fr.fcs_ok = true;
      return fr;
    }

    // Retain payload from recovered seed attempt for telemetry
    const auto best_seeds = find_best_seeds(decoded_bits_z0);
    const int rep_seed = best_seeds.empty() ? 0x5D : best_seeds[0];
    const auto desc_rep = descramble(decoded_bits_z0, rep_seed);
    if (!check_payload(desc_rep, fr.payload)) {
      check_payload(descramble(decoded_bits_z0), fr.payload);
    }

    fr.fcs_ok = false;
    return fr;
  };

  const int shifts[] = {0, -1, 1, -2, 2, -3, 3, -4, 4};
  FrameResult best_fr;
  for (int shift : shifts) {
    const int cur_ltf1 = ltf1 + shift;
    if (cur_ltf1 < kFftAdvance || cur_ltf1 + 208 >= static_cast<int>(iq.size())) continue;
    FrameResult res = try_decode(cur_ltf1);
    if (res.fcs_ok) {
      return res;
    }
    if (res.signal_ok && res.needed_samples > static_cast<int>(iq.size())) {
      return res;
    }
    if (shift == 0 || (!best_fr.signal_ok && res.signal_ok)) {
      best_fr = res;
    }
  }
  return best_fr;
}

}  // namespace quadrf_wifi

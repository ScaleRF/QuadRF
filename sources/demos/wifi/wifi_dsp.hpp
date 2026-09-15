#pragma once

#include "wifi_constants.hpp"
#include "wifi_viterbi.hpp"

#include <complex>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace quadrf_wifi {

struct FrameResult {
  bool signal_ok = false;
  bool fcs_ok = false;
  int rate_val = 0;
  int pkt_len = 0;
  std::string rate_name;
  float snr_db = 0.f;
  float coarse_cfo_hz = 0.f;
  float fine_cfo_hz = 0.f;
  float resid_cfo_hz = 0.f;
  float ltf_peak = 0.f;
  int sts_idx = -1;
  int ltf1_idx = -1;
  int peak_cs8 = 0;
  std::vector<uint8_t> payload;
  std::vector<c32> eq_symbols;
  float evm_db = 0.f;
  int needed_samples = 0;
};

struct StsResult {
  int trig_idx = -1;
  c32 P_at_trig{0.f, 0.f};
  float R_at_trig = 0.f;
  float metric_at_trig = 0.f;
};

struct DspConfig {
  float sts_threshold = 0.70f;
  float cca_threshold = 0.00008f;
  int sts_consecutive = 8;
  bool enable_cir_smoothing = false;
  float pll_alpha = 0.40f;
  float pll_beta = 0.04f;
  float pll_gamma = 0.08f;
};

class WifiDsp {
 public:
  explicit WifiDsp(DspConfig cfg = {});
  ~WifiDsp();

  WifiDsp(const WifiDsp&) = delete;
  WifiDsp& operator=(const WifiDsp&) = delete;

  FrameResult decode_burst(std::span<const c32> burst_iq, const StsResult* known_sts = nullptr);
  StsResult detect_sts(const c32* iq, size_t len) const;
  int find_dual_ltf(const c32* iq, size_t len, float* out_peak) const;
  static void derotate(c32* iq, size_t len, float rad_per_sample, float start_sample = 0.f);
  void fft64(const c32* in, c32* out);
  void ifft64(const c32* in, c32* out);
  static void soft_demap(const c32* eq_data48, const float* h_weight, int n_bpsc, float* llr_out);
  static std::vector<int> make_interleaver_map(int n_cbps, int n_bpsc);
  static std::vector<int> make_deinterleaver_map(int n_cbps, int n_bpsc);

 private:
  DspConfig cfg_;
  LtfRef ltf_ref_;
  WifiViterbi viterbi_;

  void* fft_plan_fwd_ = nullptr;
  void* fft_plan_bwd_ = nullptr;
  void* fft_in_ = nullptr;
  void* fft_out_ = nullptr;
  void init_fftw();
};

}  // namespace quadrf_wifi

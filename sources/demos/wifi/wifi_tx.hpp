#pragma once

#include "wifi_constants.hpp"
#include "wifi_viterbi.hpp"

#include <complex>
#include <cstdint>
#include <span>
#include <vector>

namespace quadrf_wifi {

struct TxConfig {
  float digital_scale = 0.50f;  // Digital backoff scaling to prevent DAC / IFFT clipping
  int seed = 0x5D;              // Scrambler initial seed
  int trailing_guard_samples = 160;  // 8 us guard silence at end of burst
};

class WifiTx {
 public:
  explicit WifiTx(TxConfig cfg = {});
  ~WifiTx();

  WifiTx(const WifiTx&) = delete;
  WifiTx& operator=(const WifiTx&) = delete;

  std::vector<c32> modulate_frame(std::span<const uint8_t> mpdu, const RateInfo& rate);
  std::vector<int8_t> modulate_frame_cs8(std::span<const uint8_t> mpdu, const RateInfo& rate);

 private:
  TxConfig cfg_;
  void* fft_plan_bwd_ = nullptr;
  void* fft_in_ = nullptr;
  void* fft_out_ = nullptr;

  std::vector<c32> sts_preamble_;
  std::vector<c32> ltf_preamble_;

  void init_fftw();
  void init_preambles();
  void ifft64(const c32* in, c32* out);

  static c32 map_subcarrier(const uint8_t* bits, int n_bpsc);
};

}  // namespace quadrf_wifi

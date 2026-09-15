#pragma once

#include <cstdint>
#include <vector>
#include <span>

namespace quadrf_wifi {

// IEEE 802.11 Convolutional Encoder & Puncturer (K=7, r=1/2, 2/3, 3/4)
class WifiEncoder {
 public:
  static std::vector<uint8_t> encode(std::span<const uint8_t> data_bits,
                                     int rate_num, int rate_den);

 private:
  static std::vector<uint8_t> puncture(std::span<const uint8_t> coded_bits,
                                       int rate_num, int rate_den);
};

class WifiViterbi {
 public:
  WifiViterbi();

  // force_state_zero: last 6 tail bits terminate at state 0
  std::vector<uint8_t> decode(std::span<const float> soft_llrs,
                              bool force_state_zero = true) const;

  static std::vector<float> depuncture(std::span<const float> in_llrs,
                                       int rate_num, int rate_den);

 private:
  int prev_states_[64][2];
  float prev_outputs_[64][2][2];
};

// 127-bit IEEE 802.11 LFSR Scrambler / Descrambler: S(x) = x^7 + x^4 + 1
std::vector<uint8_t> scramble(std::span<const uint8_t> bits, int seed = 0x5D);
std::vector<uint8_t> descramble(std::span<const uint8_t> bits);
std::vector<uint8_t> descramble(std::span<const uint8_t> bits, int seed);

// IEEE 802.3 CRC32 (FCS)
uint32_t wifi_crc32(const uint8_t* data, size_t len);

}  // namespace quadrf_wifi

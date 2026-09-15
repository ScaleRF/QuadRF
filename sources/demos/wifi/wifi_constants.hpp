#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace quadrf_wifi {

inline std::mutex g_fftw_planner_mutex;

using c32 = std::complex<float>;

inline constexpr float FS = 20e6f;
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;

// IEEE 802.11a Long Training Sequence in frequency domain (subcarriers -26 to +26, DC=0)
inline constexpr float L_STD[53] = {
    1,  1, -1, -1, 1,  1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1,
    1, -1, 1, -1, 1, 1, 1, 1, 0, 1, -1, -1, 1, 1, -1, 1, -1, 1,
    -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1};

// 48 Data subcarrier FFT bin indices in 64-point FFT
inline constexpr int DATA_SC[48] = {
    38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54,
    55, 56, 58, 59, 60, 61, 62, 63, 1,  2,  3,  4,  5,  6,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26};

// 4 Pilot subcarrier FFT bin indices
inline constexpr int PILOT_SC[4] = {43, 57, 7, 21};
inline constexpr float PILOT_SC_K[4] = {-21.f, -7.f, 7.f, 21.f};
inline constexpr float BASE_PILOT_POLARITY[4] = {1.f, 1.f, 1.f, -1.f};

// IEEE 802.11 127-bit pseudorandom pilot sequence
inline constexpr int8_t PILOT_SEQ[127] = {
    1,  1,  1,  1,  -1, -1, -1, 1,  -1, -1, -1, -1, 1,  1,  -1, 1,  -1, -1, 1,
    1,  -1, 1,  1,  -1, 1,  1,  1,  1,  1,  1,  -1, 1,  1,  1,  -1, 1,  1,  -1,
    -1, 1,  1,  1,  -1, 1,  -1, -1, -1, 1,  -1, 1,  -1, -1, 1,  -1, -1, 1,  1,
    1,  1,  1,  -1, -1, 1,  1,  -1, -1, 1,  -1, 1,  -1, 1,  1,  -1, -1, -1, 1,
    1,  -1, -1, -1, -1, 1,  -1, -1, 1,  -1, 1,  1,  1,  1,  -1, 1,  -1, 1,  -1,
    1,  -1, -1, -1, -1, -1, 1,  -1, 1,  1,  -1, 1,  -1, 1,  1,  1,  -1, -1, 1,
    -1, -1, -1, 1,  1,  1,  -1, -1, -1, -1, -1, -1, -1};

enum class Modulation {
  BPSK,
  QPSK,
  QAM16,
  QAM64
};

struct RateInfo {
  const char* name;
  Modulation mod;
  int rate_val;  // 4-bit SIGNAL field rate code
  int n_bpsc;    // Coded bits per subcarrier
  int n_cbps;    // Coded bits per OFDM symbol (48 * n_bpsc)
  int n_dbps;    // Data bits per OFDM symbol
  int rate_num;  // Coding rate numerator
  int rate_den;  // Coding rate denominator
};

inline constexpr RateInfo kRate6 {"6 Mbps (BPSK 1/2)",   Modulation::BPSK,  11, 1, 48,  24,  1, 2};
inline constexpr RateInfo kRate9 {"9 Mbps (BPSK 3/4)",   Modulation::BPSK,  15, 1, 48,  36,  3, 4};
inline constexpr RateInfo kRate12{"12 Mbps (QPSK 1/2)",  Modulation::QPSK,  10, 2, 96,  48,  1, 2};
inline constexpr RateInfo kRate18{"18 Mbps (QPSK 3/4)",  Modulation::QPSK,  14, 2, 96,  72,  3, 4};
inline constexpr RateInfo kRate24{"24 Mbps (16-QAM 1/2)",Modulation::QAM16, 9, 4, 192, 96,  1, 2};
inline constexpr RateInfo kRate36{"36 Mbps (16-QAM 3/4)",Modulation::QAM16, 13, 4, 192, 144, 3, 4};
inline constexpr RateInfo kRate48{"48 Mbps (64-QAM 2/3)",Modulation::QAM64, 8, 6, 288, 192, 2, 3};
inline constexpr RateInfo kRate54{"54 Mbps (64-QAM 3/4)",Modulation::QAM64, 12, 6, 288, 216, 3, 4};

// GUI rate stepper. CLI --rate still accepts the full 802.11a set.
// Append kRate24, kRate36, kRate48, kRate54 to expose 16-QAM / 64-QAM again.
inline constexpr const RateInfo* const kGuiRates[] = {
    &kRate6, &kRate9, &kRate12, &kRate18};

inline const RateInfo* rate_info_for(int rate_val) {
  switch (rate_val) {
    case 11: return &kRate6;
    case 15: return &kRate9;
    case 10: return &kRate12;
    case 14: return &kRate18;
    case 9:  return &kRate24;
    case 13: return &kRate36;
    case 8:  return &kRate48;
    case 12: return &kRate54;
    default: return nullptr;
  }
}

inline const RateInfo* rate_info_by_name(const std::string& name) {
  if (name == "6" || name == "6m" || name == "bpsk1/2" || name == "bpsk") return &kRate6;
  if (name == "9" || name == "9m" || name == "bpsk3/4") return &kRate9;
  if (name == "12" || name == "12m" || name == "qpsk1/2" || name == "qpsk") return &kRate12;
  if (name == "18" || name == "18m" || name == "qpsk3/4") return &kRate18;
  if (name == "24" || name == "24m" || name == "16qam1/2" || name == "16qam") return &kRate24;
  if (name == "36" || name == "36m" || name == "16qam3/4") return &kRate36;
  if (name == "48" || name == "48m" || name == "64qam2/3") return &kRate48;
  if (name == "54" || name == "54m" || name == "64qam3/4" || name == "64qam") return &kRate54;
  return &kRate6;
}

struct LtfRef {
  std::array<c32, 64> td{};
  float norm = 1.f;
};

// Compute standard unit-norm time-domain LTF reference
inline LtfRef make_ltf_ref() {
  std::array<c32, 64> L{};
  for (int i = 0; i < 26; ++i)
    L[static_cast<size_t>(38 + i)] = c32(L_STD[i], 0.f);
  for (int i = 0; i < 26; ++i)
    L[static_cast<size_t>(1 + i)] = c32(L_STD[27 + i], 0.f);

  LtfRef ref;
  constexpr float scale = 1.f / 64.f;
  for (int n = 0; n < 64; ++n) {
    c32 sum(0.f, 0.f);
    for (int k = 0; k < 64; ++k) {
      const float ang = kTwoPi * static_cast<float>(k * n) / 64.f;
      sum += L[static_cast<size_t>(k)] * c32(std::cos(ang), std::sin(ang));
    }
    ref.td[static_cast<size_t>(n)] = sum * scale;
  }
  float acc = 0.f;
  for (const auto& z : ref.td)
    acc += std::norm(z);
  const float nrm = std::sqrt(acc) + 1e-12f;
  for (auto& z : ref.td)
    z /= nrm;
  ref.norm = 1.f;
  return ref;
}

}  // namespace quadrf_wifi

/**
 * EllipticIIR.hpp
 * Header-only Elliptic Filter Design (Order 4 SOS) and IIR Implementation.
 * Optimized for interleaved stereo (IQ) processing on ARM NEON.
 * * Update: Uses float16 (FP16) arithmetic for IIR Core for max throughput on Cortex-A76.
 */

#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <arm_neon.h>

namespace DSP {

using Complex = std::complex<double>;
static constexpr double PI = 3.14159265358979323846;
// -----------------------------------------------------------------------------
// Farrow Resampler (NEON 64-bit Optimized)
// -----------------------------------------------------------------------------
class FarrowResampler {
private:
    std::vector<float> history_; 
    double mu_;                 
    bool enabled_;

public:
    FarrowResampler() : history_(8, 0.0f), mu_(0.0), enabled_(false) {}

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool en) { enabled_ = en; }
    void reset() {
        std::fill(history_.begin(), history_.end(), 0.0f);
        mu_ = 0.0;
    }

    // Toggle this for max efficiency vs. quality.
    // 1 = 2-Tap Linear Farrow (Lowest CPU, good for heavily oversampled signals)
    // 0 = 4-Tap Cubic Farrow (Higher quality, optimized 64-bit NEON)
    #define USE_LINEAR_FARROW 0

    int process(const float* in, int inCount, float* out, int outLimit, double ratio) {
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            std::memcpy(out, in, toCopy * 2 * sizeof(float));
            return toCopy;
        }
        int outProduced = 0;
        int inIndex = 0;

        #if !USE_LINEAR_FARROW
        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);
        #endif

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx; 
                if (h < 0) h = 0; if (h > 3) h = 3;
                return vld1_f32(&history_[2*h]);
            }
            return vld1_f32(&in[2*idx]);
        };

        while (outProduced < outLimit) {
            #if USE_LINEAR_FARROW
            if (inIndex + 1 >= inCount) break;
            
            float32x2_t s1 = (inIndex >= 0) ? vld1_f32(&in[2*inIndex]) : loadPair(inIndex);
            float32x2_t s2 = vld1_f32(&in[2*(inIndex+1)]);
            float32x2_t vMu = vdup_n_f32((float)mu_);

            // 2-Tap Linear: res = s1 + mu * (s2 - s1)
            float32x2_t res = vmla_f32(s1, vsub_f32(s2, s1), vMu);
            
            #else
            if (inIndex + 2 >= inCount) break;

            float32x2_t s0 = (inIndex >= 1) ? vld1_f32(&in[2*(inIndex-1)]) : loadPair(inIndex-1);
            float32x2_t s1 = vld1_f32(&in[2*inIndex]);
            float32x2_t s2 = vld1_f32(&in[2*(inIndex+1)]);
            float32x2_t s3 = vld1_f32(&in[2*(inIndex+2)]);

            float32x2_t c0 = s1;
            float32x2_t c2 = vmla_f32(vneg_f32(s1), vHalf, vadd_f32(s0, s2));
            float32x2_t term1 = vmul_f32(vsub_f32(s3, s0), vSixth);
            float32x2_t term2 = vmul_f32(vsub_f32(s1, s2), vHalf);
            float32x2_t c3 = vadd_f32(term1, term2);
            float32x2_t c1 = vsub_f32(s2, vmul_f32(s1, vHalf));
            c1 = vmls_f32(c1, s0, vThird);
            c1 = vmls_f32(c1, s3, vSixth);

            float32x2_t vMu = vdup_n_f32((float)mu_);
            float32x2_t res = vmla_f32(c2, c3, vMu); 
            res = vmla_f32(c1, res, vMu);            
            res = vmla_f32(c0, res, vMu);            
            #endif

            vst1_f32(&out[2*outProduced], res);
            outProduced++;
            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
        }

        if (inCount >= 4) {
            std::memcpy(history_.data(), &in[2*(inCount-4)], 4 * 2 * sizeof(float));
        }
        return outProduced;
    }

    int processConsume(const float* in, int inCount, float* out, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            std::memcpy(out, in, toCopy * 2 * sizeof(float));
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

        int outProduced = 0;
        int inIndex = 0; 
        
        #if !USE_LINEAR_FARROW
        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);
        #endif

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx; 
                if (h < 0) h = 0; if (h > 3) h = 3;
                return vld1_f32(&history_[2*h]);
            }
            return vld1_f32(&in[2*idx]);
        };

        while (outProduced < outLimit) {
            #if USE_LINEAR_FARROW
            if (inIndex + 1 >= inCount) break;
            
            float32x2_t s1 = (inIndex >= 0) ? vld1_f32(&in[2*inIndex]) : loadPair(inIndex);
            float32x2_t s2 = vld1_f32(&in[2*(inIndex+1)]);
            float32x2_t vMu = vdup_n_f32((float)mu_);

            float32x2_t res = vmla_f32(s1, vsub_f32(s2, s1), vMu);
            
            #else
            if (inIndex + 2 >= inCount) break;

            float32x2_t s0 = (inIndex >= 1) ? vld1_f32(&in[2*(inIndex-1)]) : loadPair(inIndex-1);
            float32x2_t s1 = vld1_f32(&in[2*inIndex]);
            float32x2_t s2 = vld1_f32(&in[2*(inIndex+1)]);
            float32x2_t s3 = vld1_f32(&in[2*(inIndex+2)]);

            float32x2_t c0 = s1;
            float32x2_t c2 = vmla_f32(vneg_f32(s1), vHalf, vadd_f32(s0, s2));
            float32x2_t term1 = vmul_f32(vsub_f32(s3, s0), vSixth);
            float32x2_t term2 = vmul_f32(vsub_f32(s1, s2), vHalf);
            float32x2_t c3 = vadd_f32(term1, term2);
            float32x2_t c1 = vsub_f32(s2, vmul_f32(s1, vHalf));
            c1 = vmls_f32(c1, s0, vThird);
            c1 = vmls_f32(c1, s3, vSixth);

            float32x2_t vMu = vdup_n_f32((float)mu_);
            float32x2_t res = vmla_f32(c2, c3, vMu);
            res = vmla_f32(c1, res, vMu);
            res = vmla_f32(c0, res, vMu);
            #endif

            vst1_f32(&out[2*outProduced], res);
            outProduced++;
            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
        }

        inConsumed = std::min(inIndex, inCount);
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

private:
    inline void updateHistory(const float* in, int inCount, int inConsumed) {
        for (int k = 0; k < 4; k++) {
            int idx = inConsumed - 4 + k;
            if (idx < 0) {
                int h = 4 + idx; 
                history_[2*k+0] = history_[2*h+0];
                history_[2*k+1] = history_[2*h+1];
            } else if (idx < inCount) {
                history_[2*k+0] = in[2*idx+0];
                history_[2*k+1] = in[2*idx+1];
            } else {
                history_[2*k+0] = 0.0f; history_[2*k+1] = 0.0f;
            }
        }
    }
};

} // namespace DSP
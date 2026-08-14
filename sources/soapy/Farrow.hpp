/**
 * Farrow.hpp
 * * Implements Farrow Resampling
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
// ratio = F_in / F_out (input samples consumed per output sample).
//
// Two kernels:
//   ratio < 0.5  -> processUpsample: many outputs share one input interval
//                   (TX host -> DSI line). Coeffs once, NEON-batched eval.
//   ratio >= 0.5 -> generic per-output path (RX line -> host, mild TX).
//
// Chunk-boundary rule: never clamp inConsumed below the phase advance.
// If the last mu step would walk past the current input buffer, park the
// remainder in skip_samples_ and consume it at the start of the next call.
// Clamping without that carry drops ~ratio/2 samples per chunk and shows up
// as ~1k ppm at 1 MSps RX (ratio ~80).
// -----------------------------------------------------------------------------
class FarrowResampler {
private:
    std::vector<float> history_;
    double mu_;
    int skip_samples_;
    bool enabled_;

public:
    FarrowResampler()
        : history_(8, 0.0f), mu_(0.0), skip_samples_(0), enabled_(false) {}

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool en) { enabled_ = en; }
    void reset() {
        std::fill(history_.begin(), history_.end(), 0.0f);
        mu_ = 0.0;
        skip_samples_ = 0;
    }

    // Renamed from processConsume to process, and removed the non-consuming version
    int process(const float* in, int inCount, float* out, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            std::memcpy(out, in, toCopy * 2 * sizeof(float));
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

        // Drain a previous overshoot before producing anything. Keeps mu_
        // consistent with the samples that actually leave the buffer.
        int inIndex = 0;
        if (skip_samples_ > 0) {
            const int skip = std::min(skip_samples_, inCount);
            inIndex += skip;
            skip_samples_ -= skip;
            if (skip_samples_ > 0) {
                inConsumed = inIndex;
                return 0;
            }
        }

        // Large upsampling ratios (TX: 1 MSps host -> ~86 MSps line) spend
        // many consecutive outputs inside the same input interval, so the
        // per-output coefficient recomputation of the generic loop dominates
        // and cannot keep up with the DSI scanout. Batch those runs instead.
        if (ratio < 0.5) {
            return processUpsample(in, inCount, out, outLimit, ratio, inConsumed, inIndex);
        }

        int outProduced = 0;

        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return vld1_f32(&history_[2*h]);
            }
            return vld1_f32(&in[2*idx]);
        };

        while (outProduced < outLimit) {
            // Need s1,s2,s3 in-buffer; s0 may come from history.
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

            vst1_f32(&out[2*outProduced], res);
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                // Phase walked past this chunk; carry the remainder.
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

private:
    // Upsampling path: coefficients are computed once per input interval,
    // then the polynomial is evaluated for the whole run of mu values with
    // 4-wide NEON (two complex outputs per iteration).
    // inIndex may already be past 0 if skip_samples_ was drained in process().
    int processUpsample(const float* in, int inCount, float* out, int outLimit,
                        double ratio, int &inConsumed, int inIndex) {
        int outProduced = 0;

        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return vld1_f32(&history_[2*h]);
            }
            return vld1_f32(&in[2*idx]);
        };

        while (outProduced < outLimit && inIndex + 2 < inCount) {
            float32x2_t s0 = (inIndex >= 1) ? vld1_f32(&in[2*(inIndex-1)]) : loadPair(inIndex-1);
            float32x2_t s1 = vld1_f32(&in[2*inIndex]);
            float32x2_t s2 = vld1_f32(&in[2*(inIndex+1)]);
            float32x2_t s3 = vld1_f32(&in[2*(inIndex+2)]);

            float32x2_t c0 = s1;
            float32x2_t c2 = vmla_f32(vneg_f32(s1), vHalf, vadd_f32(s0, s2));
            float32x2_t c3 = vadd_f32(vmul_f32(vsub_f32(s3, s0), vSixth),
                                      vmul_f32(vsub_f32(s1, s2), vHalf));
            float32x2_t c1 = vsub_f32(s2, vmul_f32(s1, vHalf));
            c1 = vmls_f32(c1, s0, vThird);
            c1 = vmls_f32(c1, s3, vSixth);

            // Largest n with mu + (n-1)*ratio < 1. The old
            // (int)(span/ratio)+1 can land last_mu >= 1 for some remainders;
            // floor keeps the run inside the interval.
            int runLen = (int)std::floor((1.0 - mu_) / ratio - 1e-12) + 1;
            if (runLen < 1) runLen = 1;
            if (runLen > outLimit - outProduced) runLen = outLimit - outProduced;

            const float32x4_t qc0 = vcombine_f32(c0, c0);
            const float32x4_t qc1 = vcombine_f32(c1, c1);
            const float32x4_t qc2 = vcombine_f32(c2, c2);
            const float32x4_t qc3 = vcombine_f32(c3, c3);
            // Float for the NEON eval (hot path); mu_ itself still advances in double.
            const float rf = (float)ratio;
            const float mu0f = (float)mu_;

            int k = 0;
            for (; k + 1 < runLen; k += 2) {
                const float mua = mu0f + k * rf;
                const float mub = mua + rf;
                float32x4_t vMu = vcombine_f32(vdup_n_f32(mua), vdup_n_f32(mub));
                float32x4_t res = vmlaq_f32(qc2, qc3, vMu);
                res = vmlaq_f32(qc1, res, vMu);
                res = vmlaq_f32(qc0, res, vMu);
                vst1q_f32(&out[2*(outProduced + k)], res);
            }
            if (k < runLen) {
                float32x2_t vMu = vdup_n_f32(mu0f + k * rf);
                float32x2_t res = vmla_f32(c2, c3, vMu);
                res = vmla_f32(c1, res, vMu);
                res = vmla_f32(c0, res, vMu);
                vst1_f32(&out[2*(outProduced + k)], res);
                ++k;
            }

            outProduced += runLen;
            mu_ += (double)runLen * ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

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
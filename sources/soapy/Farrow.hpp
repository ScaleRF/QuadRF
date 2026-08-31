/**
 * Farrow.hpp
 *
 * Implements 4-tap cubic Lagrange Farrow resampling for SDR RX downsampling
 * and TX upsampling.
 *
 * Optimized SIMD implementations:
 *   - FarrowResampler: Standard CF32 input resampler with fused CS8 quantization
 *   - FarrowResamplerCS8: Direct on-the-fly CS8 input resampler (1-channel RX)
 *   - FarrowResampler4Ch: Unified 128-bit SIMD 4-channel interleaved CS8 resampler
 *
 * Mathematical equivalence to 4-tap cubic Lagrange:
 *   c0 = s1
 *   c1 = -1/3 s0 - 1/2 s1 + s2 - 1/6 s3
 *   c2 = 1/2 s0 - s1 + 1/2 s2
 *   c3 = -1/6 s0 + 1/2 s1 - 1/2 s2 + 1/6 s3
 *   y(mu) = c0 + mu * (c1 + mu * (c2 + mu * c3))
 */

#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace DSP {

using Complex = std::complex<double>;
static constexpr double PI = 3.14159265358979323846;

// =============================================================================
// 1. Standard Farrow Resampler (Float IQ input)
// =============================================================================
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

    inline int processConsume(const float* in, int inCount, float* out, int outLimit, double ratio, int &inConsumed) {
        return process(in, inCount, out, outLimit, ratio, inConsumed);
    }

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

        if (ratio < 0.5) {
            return processUpsample(in, inCount, out, outLimit, ratio, inConsumed, inIndex);
        }

        int outProduced = 0;

#if defined(__ARM_NEON)
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
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return history_[2*h + c];
            }
            return in[2*idx + c];
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? in[2*(inIndex-1) + c] : loadSample(inIndex-1, c);
                float s1 = in[2*inIndex + c];
                float s2 = in[2*(inIndex+1) + c];
                float s3 = in[2*(inIndex+2) + c];

                float c0 = s1;
                float c2 = -s1 + 0.5f * (s0 + s2);
                float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                out[2*outProduced + c] = c0 + mu * (c1 + mu * (c2 + mu * c3));
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    // Fused TX path: Float input directly to CS8 (int8_t) output with zero float scratch buffer
    int processToCS8(const float* in, int inCount, int8_t* out, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            for (int i = 0; i < toCopy; ++i) {
                out[2*i+0] = (int8_t)std::max(-128.0f, std::min(127.0f, in[2*i+0] * 127.0f));
                out[2*i+1] = (int8_t)std::max(-128.0f, std::min(127.0f, in[2*i+1] * 127.0f));
            }
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

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

        if (ratio < 0.5) {
            return processUpsampleToCS8(in, inCount, out, outLimit, ratio, inConsumed, inIndex);
        }

        int outProduced = 0;
#if defined(__ARM_NEON)
        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);
        const float32x2_t vScale = vdup_n_f32(127.0f);

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return vld1_f32(&history_[2*h]);
            }
            return vld1_f32(&in[2*idx]);
        };

        while (outProduced < outLimit) {
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

            float32x2_t scaled = vmul_f32(res, vScale);
            int32x2_t int32_val = vcvt_s32_f32(scaled);
            int16x8_t int16_val = vcombine_s16(vqmovn_s32(vcombine_s32(int32_val, int32_val)), vdup_n_s16(0));
            int8x8_t int8_val = vqmovn_s16(int16_val);
            *reinterpret_cast<int16_t*>(&out[2 * outProduced]) = vget_lane_s16(vreinterpret_s16_s8(int8_val), 0);
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return history_[2*h + c];
            }
            return in[2*idx + c];
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? in[2*(inIndex-1) + c] : loadSample(inIndex-1, c);
                float s1 = in[2*inIndex + c];
                float s2 = in[2*(inIndex+1) + c];
                float s3 = in[2*(inIndex+2) + c];

                float c0 = s1;
                float c2 = -s1 + 0.5f * (s0 + s2);
                float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                float val = c0 + mu * (c1 + mu * (c2 + mu * c3));
                out[2*outProduced + c] = (int8_t)std::max(-128.0f, std::min(127.0f, val * 127.0f));
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif
        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

private:
    int processUpsample(const float* in, int inCount, float* out, int outLimit,
                        double ratio, int &inConsumed, int inIndex) {
        int outProduced = 0;

#if defined(__ARM_NEON)
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

            int runLen = (int)std::floor((1.0 - mu_) / ratio - 1e-12) + 1;
            if (runLen < 1) runLen = 1;
            if (runLen > outLimit - outProduced) runLen = outLimit - outProduced;

            const float32x4_t qc0 = vcombine_f32(c0, c0);
            const float32x4_t qc1 = vcombine_f32(c1, c1);
            const float32x4_t qc2 = vcombine_f32(c2, c2);
            const float32x4_t qc3 = vcombine_f32(c3, c3);
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
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return history_[2*h + c];
            }
            return in[2*idx + c];
        };

        while (outProduced < outLimit && inIndex + 2 < inCount) {
            float c0[2], c1[2], c2[2], c3[2];
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? in[2*(inIndex-1) + c] : loadSample(inIndex-1, c);
                float s1 = in[2*inIndex + c];
                float s2 = in[2*(inIndex+1) + c];
                float s3 = in[2*(inIndex+2) + c];

                c0[c] = s1;
                c2[c] = -s1 + 0.5f * (s0 + s2);
                c3[c] = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                c1[c] = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);
            }

            int runLen = (int)std::floor((1.0 - mu_) / ratio - 1e-12) + 1;
            if (runLen < 1) runLen = 1;
            if (runLen > outLimit - outProduced) runLen = outLimit - outProduced;

            const float rf = (float)ratio;
            const float mu0f = (float)mu_;

            for (int k = 0; k < runLen; ++k) {
                const float mu = mu0f + k * rf;
                for (int c = 0; c < 2; ++c) {
                    out[2*(outProduced + k) + c] = c0[c] + mu * (c1[c] + mu * (c2[c] + mu * c3[c]));
                }
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
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    int processUpsampleToCS8(const float* in, int inCount, int8_t* out, int outLimit,
                             double ratio, int &inConsumed, int inIndex) {
        int outProduced = 0;

#if defined(__ARM_NEON)
        const float32x2_t vHalf  = vdup_n_f32(0.5f);
        const float32x2_t vSixth = vdup_n_f32(1.0f/6.0f);
        const float32x2_t vThird = vdup_n_f32(1.0f/3.0f);
        const float32x4_t vScale4 = vdupq_n_f32(127.0f);
        const float32x2_t vScale2 = vdup_n_f32(127.0f);

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

            int runLen = (int)std::floor((1.0 - mu_) / ratio - 1e-12) + 1;
            if (runLen < 1) runLen = 1;
            if (runLen > outLimit - outProduced) runLen = outLimit - outProduced;

            const float32x4_t qc0 = vcombine_f32(c0, c0);
            const float32x4_t qc1 = vcombine_f32(c1, c1);
            const float32x4_t qc2 = vcombine_f32(c2, c2);
            const float32x4_t qc3 = vcombine_f32(c3, c3);
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

                float32x4_t scaled = vmulq_f32(res, vScale4);
                int32x4_t int32_val = vcvtq_s32_f32(scaled);
                int16x4_t int16_val = vqmovn_s32(int32_val);
                int8x8_t int8_val = vqmovn_s16(vcombine_s16(int16_val, vdup_n_s16(0)));
                *reinterpret_cast<int32_t*>(&out[2 * (outProduced + k)]) = vget_lane_s32(vreinterpret_s32_s8(int8_val), 0);
            }
            if (k < runLen) {
                float32x2_t vMu = vdup_n_f32(mu0f + k * rf);
                float32x2_t res = vmla_f32(c2, c3, vMu);
                res = vmla_f32(c1, res, vMu);
                res = vmla_f32(c0, res, vMu);

                float32x2_t scaled = vmul_f32(res, vScale2);
                int32x2_t int32_val = vcvt_s32_f32(scaled);
                int16x8_t int16_val = vcombine_s16(vqmovn_s32(vcombine_s32(int32_val, int32_val)), vdup_n_s16(0));
                int8x8_t int8_val = vqmovn_s16(int16_val);
                *reinterpret_cast<int16_t*>(&out[2 * (outProduced + k)]) = vget_lane_s16(vreinterpret_s16_s8(int8_val), 0);
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
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return history_[2*h + c];
            }
            return in[2*idx + c];
        };

        while (outProduced < outLimit && inIndex + 2 < inCount) {
            float c0[2], c1[2], c2[2], c3[2];
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? in[2*(inIndex-1) + c] : loadSample(inIndex-1, c);
                float s1 = in[2*inIndex + c];
                float s2 = in[2*(inIndex+1) + c];
                float s3 = in[2*(inIndex+2) + c];

                c0[c] = s1;
                c2[c] = -s1 + 0.5f * (s0 + s2);
                c3[c] = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                c1[c] = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);
            }

            int runLen = (int)std::floor((1.0 - mu_) / ratio - 1e-12) + 1;
            if (runLen < 1) runLen = 1;
            if (runLen > outLimit - outProduced) runLen = outLimit - outProduced;

            const float rf = (float)ratio;
            const float mu0f = (float)mu_;

            for (int k = 0; k < runLen; ++k) {
                const float mu = mu0f + k * rf;
                for (int c = 0; c < 2; ++c) {
                    float val = c0[c] + mu * (c1[c] + mu * (c2[c] + mu * c3[c]));
                    out[2*(outProduced + k) + c] = (int8_t)std::max(-128.0f, std::min(127.0f, val * 127.0f));
                }
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
#endif

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

// =============================================================================
// 2. Single-Channel On-The-Fly CS8 Input Farrow Resampler (RX 1-Channel)
// =============================================================================
class FarrowResamplerCS8 {
private:
    int8_t history_[8]; // 4 complex pairs: [I0, Q0, I1, Q1, I2, Q2, I3, Q3]
    double mu_;
    int skip_samples_;
    bool enabled_;

public:
    FarrowResamplerCS8()
        : mu_(0.0), skip_samples_(0), enabled_(false) {
        std::memset(history_, 0, sizeof(history_));
    }

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool en) { enabled_ = en; }
    void reset() {
        std::memset(history_, 0, sizeof(history_));
        mu_ = 0.0;
        skip_samples_ = 0;
    }

    int processToCF32(const int8_t* in, int inCount, float* out, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            for (int i = 0; i < toCopy; ++i) {
                out[2*i+0] = float(in[2*i+0]) / 127.0f;
                out[2*i+1] = float(in[2*i+1]) / 127.0f;
            }
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

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

        int outProduced = 0;

#if defined(__ARM_NEON)
        const float32x4_t vscale4 = vdupq_n_f32(1.0f / 127.0f);
        const float32x2_t vscale2 = vdup_n_f32(1.0f / 127.0f);
        const float32x2_t vHalf   = vdup_n_f32(0.5f);
        const float32x2_t vSixth  = vdup_n_f32(1.0f / 6.0f);
        const float32x2_t vThird  = vdup_n_f32(1.0f / 3.0f);

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                int16_t val16 = *reinterpret_cast<const int16_t*>(&history_[2*h]);
                int8x8_t s8 = vset_lane_s8(static_cast<int8_t>(val16 & 0xFF), vdup_n_s8(0), 0);
                s8 = vset_lane_s8(static_cast<int8_t>((val16 >> 8) & 0xFF), s8, 1);
                int16x4_t s16 = vget_low_s16(vmovl_s8(s8));
                int32x2_t s32 = vget_low_s32(vmovl_s16(s16));
                return vmul_f32(vcvt_f32_s32(s32), vscale2);
            }
            int16_t val16 = *reinterpret_cast<const int16_t*>(&in[2*idx]);
            int8x8_t s8 = vset_lane_s8(static_cast<int8_t>(val16 & 0xFF), vdup_n_s8(0), 0);
            s8 = vset_lane_s8(static_cast<int8_t>((val16 >> 8) & 0xFF), s8, 1);
            int16x4_t s16 = vget_low_s16(vmovl_s8(s8));
            int32x2_t s32 = vget_low_s32(vmovl_s16(s16));
            return vmul_f32(vcvt_f32_s32(s32), vscale2);
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            float32x2_t s0, s1, s2, s3;
            if (inIndex >= 1) {
                int8x8_t raw8 = vld1_s8(&in[2 * (inIndex - 1)]);
                int16x8_t raw16 = vmovl_s8(raw8);
                int32x4_t s01_32 = vmovl_s16(vget_low_s16(raw16));
                int32x4_t s23_32 = vmovl_s16(vget_high_s16(raw16));
                float32x4_t s01_f = vmulq_f32(vcvtq_f32_s32(s01_32), vscale4);
                float32x4_t s23_f = vmulq_f32(vcvtq_f32_s32(s23_32), vscale4);
                s0 = vget_low_f32(s01_f);
                s1 = vget_high_f32(s01_f);
                s2 = vget_low_f32(s23_f);
                s3 = vget_high_f32(s23_f);
            } else {
                s0 = loadPair(inIndex - 1);
                int8x8_t raw8 = vld1_s8(&in[2 * inIndex]);
                int16x8_t raw16 = vmovl_s8(raw8);
                int32x4_t s12_32 = vmovl_s16(vget_low_s16(raw16));
                float32x4_t s12_f = vmulq_f32(vcvtq_f32_s32(s12_32), vscale4);
                s1 = vget_low_f32(s12_f);
                s2 = vget_high_f32(s12_f);
                s3 = loadPair(inIndex + 2);
            }

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

            vst1_f32(&out[2 * outProduced], res);
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return float(history_[2*h + c]) / 127.0f;
            }
            return float(in[2*idx + c]) / 127.0f;
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? (float(in[2*(inIndex-1) + c]) / 127.0f) : loadSample(inIndex-1, c);
                float s1 = float(in[2*inIndex + c]) / 127.0f;
                float s2 = float(in[2*(inIndex+1) + c]) / 127.0f;
                float s3 = float(in[2*(inIndex+2) + c]) / 127.0f;

                float c0 = s1;
                float c2 = -s1 + 0.5f * (s0 + s2);
                float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                out[2*outProduced + c] = c0 + mu * (c1 + mu * (c2 + mu * c3));
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    int processToCS8(const int8_t* in, int inCount, int8_t* out, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;
        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            std::memcpy(out, in, toCopy * 2 * sizeof(int8_t));
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

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

        int outProduced = 0;

#if defined(__ARM_NEON)
        const float32x4_t vscale4 = vdupq_n_f32(1.0f / 127.0f);
        const float32x2_t vscale2 = vdup_n_f32(1.0f / 127.0f);
        const float32x2_t vHalf   = vdup_n_f32(0.5f);
        const float32x2_t vSixth  = vdup_n_f32(1.0f / 6.0f);
        const float32x2_t vThird  = vdup_n_f32(1.0f / 3.0f);
        const float32x2_t vScale2 = vdup_n_f32(127.0f);

        auto loadPair = [&](int idx) -> float32x2_t {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                int16_t val16 = *reinterpret_cast<const int16_t*>(&history_[2*h]);
                int8x8_t s8 = vset_lane_s8(static_cast<int8_t>(val16 & 0xFF), vdup_n_s8(0), 0);
                s8 = vset_lane_s8(static_cast<int8_t>((val16 >> 8) & 0xFF), s8, 1);
                int16x4_t s16 = vget_low_s16(vmovl_s8(s8));
                int32x2_t s32 = vget_low_s32(vmovl_s16(s16));
                return vmul_f32(vcvt_f32_s32(s32), vscale2);
            }
            int16_t val16 = *reinterpret_cast<const int16_t*>(&in[2*idx]);
            int8x8_t s8 = vset_lane_s8(static_cast<int8_t>(val16 & 0xFF), vdup_n_s8(0), 0);
            s8 = vset_lane_s8(static_cast<int8_t>((val16 >> 8) & 0xFF), s8, 1);
            int16x4_t s16 = vget_low_s16(vmovl_s8(s8));
            int32x2_t s32 = vget_low_s32(vmovl_s16(s16));
            return vmul_f32(vcvt_f32_s32(s32), vscale2);
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            float32x2_t s0, s1, s2, s3;
            if (inIndex >= 1) {
                int8x8_t raw8 = vld1_s8(&in[2 * (inIndex - 1)]);
                int16x8_t raw16 = vmovl_s8(raw8);
                int32x4_t s01_32 = vmovl_s16(vget_low_s16(raw16));
                int32x4_t s23_32 = vmovl_s16(vget_high_s16(raw16));
                float32x4_t s01_f = vmulq_f32(vcvtq_f32_s32(s01_32), vscale4);
                float32x4_t s23_f = vmulq_f32(vcvtq_f32_s32(s23_32), vscale4);
                s0 = vget_low_f32(s01_f);
                s1 = vget_high_f32(s01_f);
                s2 = vget_low_f32(s23_f);
                s3 = vget_high_f32(s23_f);
            } else {
                s0 = loadPair(inIndex - 1);
                int8x8_t raw8 = vld1_s8(&in[2 * inIndex]);
                int16x8_t raw16 = vmovl_s8(raw8);
                int32x4_t s12_32 = vmovl_s16(vget_low_s16(raw16));
                float32x4_t s12_f = vmulq_f32(vcvtq_f32_s32(s12_32), vscale4);
                s1 = vget_low_f32(s12_f);
                s2 = vget_high_f32(s12_f);
                s3 = loadPair(inIndex + 2);
            }

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

            float32x2_t scaled = vmul_f32(res, vScale2);
            int32x2_t int32_val = vcvt_s32_f32(scaled);
            int16x8_t int16_val = vcombine_s16(vqmovn_s32(vcombine_s32(int32_val, int32_val)), vdup_n_s16(0));
            int8x8_t int8_val = vqmovn_s16(int16_val);
            *reinterpret_cast<int16_t*>(&out[2 * outProduced]) = vget_lane_s16(vreinterpret_s16_s8(int8_val), 0);
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return float(history_[2*h + c]) / 127.0f;
            }
            return float(in[2*idx + c]) / 127.0f;
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (int c = 0; c < 2; ++c) {
                float s0 = (inIndex >= 1) ? (float(in[2*(inIndex-1) + c]) / 127.0f) : loadSample(inIndex-1, c);
                float s1 = float(in[2*inIndex + c]) / 127.0f;
                float s2 = float(in[2*(inIndex+1) + c]) / 127.0f;
                float s3 = float(in[2*(inIndex+2) + c]) / 127.0f;

                float c0 = s1;
                float c2 = -s1 + 0.5f * (s0 + s2);
                float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                float val = c0 + mu * (c1 + mu * (c2 + mu * c3));
                out[2*outProduced + c] = (int8_t)std::max(-128.0f, std::min(127.0f, val * 127.0f));
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    inline void updateHistory(const int8_t* in, int inCount, int inConsumed) {
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
                history_[2*k+0] = 0;
                history_[2*k+1] = 0;
            }
        }
    }
};

// =============================================================================
// 3. Unified 4-Channel Interleaved CS8 Resampler (RX Phased Array)
// =============================================================================
class FarrowResampler4Ch {
private:
    int8_t history_[32]; // 4 time steps * 8 bytes (4 channels * 2 bytes IQ)
    double mu_;
    int skip_samples_;
    bool enabled_;

public:
    FarrowResampler4Ch()
        : mu_(0.0), skip_samples_(0), enabled_(false) {
        std::memset(history_, 0, sizeof(history_));
    }

    bool isEnabled() const { return enabled_; }
    void setEnabled(bool en) { enabled_ = en; }
    void reset() {
        std::memset(history_, 0, sizeof(history_));
        mu_ = 0.0;
        skip_samples_ = 0;
    }

    int processToCF32(const int8_t* in, int inCount, void* const* buffs,
                      const std::vector<size_t>& channels, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;

        float* f0 = (channels.size() > 0 && buffs && buffs[0]) ? static_cast<float*>(buffs[0]) : nullptr;
        float* f1 = (channels.size() > 1 && buffs && buffs[1]) ? static_cast<float*>(buffs[1]) : nullptr;
        float* f2 = (channels.size() > 2 && buffs && buffs[2]) ? static_cast<float*>(buffs[2]) : nullptr;
        float* f3 = (channels.size() > 3 && buffs && buffs[3]) ? static_cast<float*>(buffs[3]) : nullptr;

        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            for (int i = 0; i < toCopy; ++i) {
                if (f0) { f0[2*i+0] = float(in[8*i+0])/127.0f; f0[2*i+1] = float(in[8*i+1])/127.0f; }
                if (f1) { f1[2*i+0] = float(in[8*i+2])/127.0f; f1[2*i+1] = float(in[8*i+3])/127.0f; }
                if (f2) { f2[2*i+0] = float(in[8*i+4])/127.0f; f2[2*i+1] = float(in[8*i+5])/127.0f; }
                if (f3) { f3[2*i+0] = float(in[8*i+6])/127.0f; f3[2*i+1] = float(in[8*i+7])/127.0f; }
            }
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

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

        int outProduced = 0;

#if defined(__ARM_NEON)
        const float32x4_t vscale4 = vdupq_n_f32(1.0f / 127.0f);
        const float32x4_t vHalf4  = vdupq_n_f32(0.5f);
        const float32x4_t vSixth4 = vdupq_n_f32(1.0f / 6.0f);
        const float32x4_t vThird4 = vdupq_n_f32(1.0f / 3.0f);

        auto load4ChStep = [&](int idx, float32x4_t& out_I, float32x4_t& out_Q) {
            const int8_t* src = (idx < 0) ? &history_[8 * (4 + idx >= 0 ? (4 + idx <= 3 ? 4 + idx : 3) : 0)]
                                          : &in[8 * idx];
            int8x8_t raw = vld1_s8(src);
            int16x8_t raw16 = vmovl_s8(raw);
            int16x4x2_t deint16 = vuzp_s16(vget_low_s16(raw16), vget_high_s16(raw16));
            out_I = vmulq_f32(vcvtq_f32_s32(vmovl_s16(deint16.val[0])), vscale4);
            out_Q = vmulq_f32(vcvtq_f32_s32(vmovl_s16(deint16.val[1])), vscale4);
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            float32x4_t s0_I, s0_Q, s1_I, s1_Q, s2_I, s2_Q, s3_I, s3_Q;
            load4ChStep(inIndex - 1, s0_I, s0_Q);
            load4ChStep(inIndex,     s1_I, s1_Q);
            load4ChStep(inIndex + 1, s2_I, s2_Q);
            load4ChStep(inIndex + 2, s3_I, s3_Q);

            // In-phase cubic interpolation
            float32x4_t c0_I = s1_I;
            float32x4_t c2_I = vmlaq_f32(vnegq_f32(s1_I), vHalf4, vaddq_f32(s0_I, s2_I));
            float32x4_t term1_I = vmulq_f32(vsubq_f32(s3_I, s0_I), vSixth4);
            float32x4_t term2_I = vmulq_f32(vsubq_f32(s1_I, s2_I), vHalf4);
            float32x4_t c3_I = vaddq_f32(term1_I, term2_I);
            float32x4_t c1_I = vsubq_f32(s2_I, vmulq_f32(s1_I, vHalf4));
            c1_I = vmlsq_f32(c1_I, s0_I, vThird4);
            c1_I = vmlsq_f32(c1_I, s3_I, vSixth4);

            float32x4_t vMu4 = vdupq_n_f32((float)mu_);
            float32x4_t res_I = vmlaq_f32(c2_I, c3_I, vMu4);
            res_I = vmlaq_f32(c1_I, res_I, vMu4);
            res_I = vmlaq_f32(c0_I, res_I, vMu4);

            // Quadrature cubic interpolation
            float32x4_t c0_Q = s1_Q;
            float32x4_t c2_Q = vmlaq_f32(vnegq_f32(s1_Q), vHalf4, vaddq_f32(s0_Q, s2_Q));
            float32x4_t term1_Q = vmulq_f32(vsubq_f32(s3_Q, s0_Q), vSixth4);
            float32x4_t term2_Q = vmulq_f32(vsubq_f32(s1_Q, s2_Q), vHalf4);
            float32x4_t c3_Q = vaddq_f32(term1_Q, term2_Q);
            float32x4_t c1_Q = vsubq_f32(s2_Q, vmulq_f32(s1_Q, vHalf4));
            c1_Q = vmlsq_f32(c1_Q, s0_Q, vThird4);
            c1_Q = vmlsq_f32(c1_Q, s3_Q, vSixth4);

            float32x4_t res_Q = vmlaq_f32(c2_Q, c3_Q, vMu4);
            res_Q = vmlaq_f32(c1_Q, res_Q, vMu4);
            res_Q = vmlaq_f32(c0_Q, res_Q, vMu4);

            float32x4x2_t zip01 = vzipq_f32(res_I, res_Q);
            if (f0) vst1_f32(f0 + 2*outProduced, vget_low_f32(zip01.val[0]));
            if (f1) vst1_f32(f1 + 2*outProduced, vget_high_f32(zip01.val[0]));
            if (f2) vst1_f32(f2 + 2*outProduced, vget_low_f32(zip01.val[1]));
            if (f3) vst1_f32(f3 + 2*outProduced, vget_high_f32(zip01.val[1]));

            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int ch, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return float(history_[8*h + 2*ch + c]) / 127.0f;
            }
            return float(in[8*idx + 2*ch + c]) / 127.0f;
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (size_t i = 0; i < channels.size(); ++i) {
                const size_t ch = channels[i];
                float* dst = (buffs && buffs[i]) ? static_cast<float*>(buffs[i]) : nullptr;
                if (!dst) continue;

                for (int c = 0; c < 2; ++c) {
                    float s0 = (inIndex >= 1) ? (float(in[8*(inIndex-1) + 2*ch + c]) / 127.0f) : loadSample(inIndex-1, int(ch), c);
                    float s1 = float(in[8*inIndex + 2*ch + c]) / 127.0f;
                    float s2 = float(in[8*(inIndex+1) + 2*ch + c]) / 127.0f;
                    float s3 = float(in[8*(inIndex+2) + 2*ch + c]) / 127.0f;

                    float c0 = s1;
                    float c2 = -s1 + 0.5f * (s0 + s2);
                    float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                    float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                    dst[2*outProduced + c] = c0 + mu * (c1 + mu * (c2 + mu * c3));
                }
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    int processToCS8(const int8_t* in, int inCount, void* const* buffs,
                     const std::vector<size_t>& channels, int outLimit, double ratio, int &inConsumed) {
        inConsumed = 0;
        if (inCount <= 0 || outLimit <= 0) return 0;

        int8_t* d0 = (channels.size() > 0 && buffs && buffs[0]) ? static_cast<int8_t*>(buffs[0]) : nullptr;
        int8_t* d1 = (channels.size() > 1 && buffs && buffs[1]) ? static_cast<int8_t*>(buffs[1]) : nullptr;
        int8_t* d2 = (channels.size() > 2 && buffs && buffs[2]) ? static_cast<int8_t*>(buffs[2]) : nullptr;
        int8_t* d3 = (channels.size() > 3 && buffs && buffs[3]) ? static_cast<int8_t*>(buffs[3]) : nullptr;

        if (!enabled_) {
            int toCopy = std::min(inCount, outLimit);
            for (int i = 0; i < toCopy; ++i) {
                if (d0) { d0[2*i+0] = in[8*i+0]; d0[2*i+1] = in[8*i+1]; }
                if (d1) { d1[2*i+0] = in[8*i+2]; d1[2*i+1] = in[8*i+3]; }
                if (d2) { d2[2*i+0] = in[8*i+4]; d2[2*i+1] = in[8*i+5]; }
                if (d3) { d3[2*i+0] = in[8*i+6]; d3[2*i+1] = in[8*i+7]; }
            }
            inConsumed = toCopy;
            updateHistory(in, inCount, inConsumed);
            return toCopy;
        }

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

        int outProduced = 0;

#if defined(__ARM_NEON)
        const float32x4_t vscale4 = vdupq_n_f32(1.0f / 127.0f);
        const float32x4_t vHalf4  = vdupq_n_f32(0.5f);
        const float32x4_t vSixth4 = vdupq_n_f32(1.0f / 6.0f);
        const float32x4_t vThird4 = vdupq_n_f32(1.0f / 3.0f);
        const float32x4_t v127    = vdupq_n_f32(127.0f);

        auto load4ChStep = [&](int idx, float32x4_t& out_I, float32x4_t& out_Q) {
            const int8_t* src = (idx < 0) ? &history_[8 * (4 + idx >= 0 ? (4 + idx <= 3 ? 4 + idx : 3) : 0)]
                                          : &in[8 * idx];
            int8x8_t raw = vld1_s8(src);
            int16x8_t raw16 = vmovl_s8(raw);
            int16x4x2_t deint16 = vuzp_s16(vget_low_s16(raw16), vget_high_s16(raw16));
            out_I = vmulq_f32(vcvtq_f32_s32(vmovl_s16(deint16.val[0])), vscale4);
            out_Q = vmulq_f32(vcvtq_f32_s32(vmovl_s16(deint16.val[1])), vscale4);
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            float32x4_t s0_I, s0_Q, s1_I, s1_Q, s2_I, s2_Q, s3_I, s3_Q;
            load4ChStep(inIndex - 1, s0_I, s0_Q);
            load4ChStep(inIndex,     s1_I, s1_Q);
            load4ChStep(inIndex + 1, s2_I, s2_Q);
            load4ChStep(inIndex + 2, s3_I, s3_Q);

            float32x4_t c0_I = s1_I;
            float32x4_t c2_I = vmlaq_f32(vnegq_f32(s1_I), vHalf4, vaddq_f32(s0_I, s2_I));
            float32x4_t term1_I = vmulq_f32(vsubq_f32(s3_I, s0_I), vSixth4);
            float32x4_t term2_I = vmulq_f32(vsubq_f32(s1_I, s2_I), vHalf4);
            float32x4_t c3_I = vaddq_f32(term1_I, term2_I);
            float32x4_t c1_I = vsubq_f32(s2_I, vmulq_f32(s1_I, vHalf4));
            c1_I = vmlsq_f32(c1_I, s0_I, vThird4);
            c1_I = vmlsq_f32(c1_I, s3_I, vSixth4);

            float32x4_t vMu4 = vdupq_n_f32((float)mu_);
            float32x4_t res_I = vmlaq_f32(c2_I, c3_I, vMu4);
            res_I = vmlaq_f32(c1_I, res_I, vMu4);
            res_I = vmlaq_f32(c0_I, res_I, vMu4);

            float32x4_t c0_Q = s1_Q;
            float32x4_t c2_Q = vmlaq_f32(vnegq_f32(s1_Q), vHalf4, vaddq_f32(s0_Q, s2_Q));
            float32x4_t term1_Q = vmulq_f32(vsubq_f32(s3_Q, s0_Q), vSixth4);
            float32x4_t term2_Q = vmulq_f32(vsubq_f32(s1_Q, s2_Q), vHalf4);
            float32x4_t c3_Q = vaddq_f32(term1_Q, term2_Q);
            float32x4_t c1_Q = vsubq_f32(s2_Q, vmulq_f32(s1_Q, vHalf4));
            c1_Q = vmlsq_f32(c1_Q, s0_Q, vThird4);
            c1_Q = vmlsq_f32(c1_Q, s3_Q, vSixth4);

            float32x4_t res_Q = vmlaq_f32(c2_Q, c3_Q, vMu4);
            res_Q = vmlaq_f32(c1_Q, res_Q, vMu4);
            res_Q = vmlaq_f32(c0_Q, res_Q, vMu4);

            int32x4_t i_s32 = vcvtq_s32_f32(vmulq_f32(res_I, v127));
            int32x4_t q_s32 = vcvtq_s32_f32(vmulq_f32(res_Q, v127));
            int16x4_t i_s16 = vqmovn_s32(i_s32);
            int16x4_t q_s16 = vqmovn_s32(q_s32);
            int16x4x2_t zip_iq = vzip_s16(i_s16, q_s16);
            int8x8_t s8_01 = vqmovn_s16(vcombine_s16(zip_iq.val[0], vdup_n_s16(0)));
            int8x8_t s8_23 = vqmovn_s16(vcombine_s16(zip_iq.val[1], vdup_n_s16(0)));

            if (d0) *reinterpret_cast<int16_t*>(d0 + 2*outProduced) = vget_lane_s16(vreinterpret_s16_s8(s8_01), 0);
            if (d1) *reinterpret_cast<int16_t*>(d1 + 2*outProduced) = vget_lane_s16(vreinterpret_s16_s8(s8_01), 1);
            if (d2) *reinterpret_cast<int16_t*>(d2 + 2*outProduced) = vget_lane_s16(vreinterpret_s16_s8(s8_23), 0);
            if (d3) *reinterpret_cast<int16_t*>(d3 + 2*outProduced) = vget_lane_s16(vreinterpret_s16_s8(s8_23), 1);

            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#else
        auto loadSample = [&](int idx, int ch, int c) -> float {
            if (idx < 0) {
                int h = 4 + idx;
                if (h < 0) h = 0; if (h > 3) h = 3;
                return float(history_[8*h + 2*ch + c]) / 127.0f;
            }
            return float(in[8*idx + 2*ch + c]) / 127.0f;
        };

        while (outProduced < outLimit) {
            if (inIndex + 2 >= inCount) break;

            const float mu = (float)mu_;
            for (size_t i = 0; i < channels.size(); ++i) {
                const size_t ch = channels[i];
                int8_t* dst = (buffs && buffs[i]) ? static_cast<int8_t*>(buffs[i]) : nullptr;
                if (!dst) continue;

                for (int c = 0; c < 2; ++c) {
                    float s0 = (inIndex >= 1) ? (float(in[8*(inIndex-1) + 2*ch + c]) / 127.0f) : loadSample(inIndex-1, int(ch), c);
                    float s1 = float(in[8*inIndex + 2*ch + c]) / 127.0f;
                    float s2 = float(in[8*(inIndex+1) + 2*ch + c]) / 127.0f;
                    float s3 = float(in[8*(inIndex+2) + 2*ch + c]) / 127.0f;

                    float c0 = s1;
                    float c2 = -s1 + 0.5f * (s0 + s2);
                    float c3 = (s3 - s0) * (1.0f/6.0f) + (s1 - s2) * 0.5f;
                    float c1 = s2 - s1 * 0.5f - s0 * (1.0f/3.0f) - s3 * (1.0f/6.0f);

                    float val = c0 + mu * (c1 + mu * (c2 + mu * c3));
                    dst[2*outProduced + c] = (int8_t)std::max(-128.0f, std::min(127.0f, val * 127.0f));
                }
            }
            outProduced++;

            mu_ += ratio;
            int advance = (int)mu_;
            mu_ -= advance;
            inIndex += advance;
            if (inIndex > inCount) {
                skip_samples_ += inIndex - inCount;
                inIndex = inCount;
                break;
            }
        }
#endif

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    inline void updateHistory(const int8_t* in, int inCount, int inConsumed) {
        for (int k = 0; k < 4; k++) {
            int idx = inConsumed - 4 + k;
            if (idx < 0) {
                int h = 4 + idx;
                std::memcpy(&history_[8*k], &history_[8*h], 8);
            } else if (idx < inCount) {
                std::memcpy(&history_[8*k], &in[8*idx], 8);
            } else {
                std::memset(&history_[8*k], 0, 8);
            }
        }
    }
};

} // namespace DSP

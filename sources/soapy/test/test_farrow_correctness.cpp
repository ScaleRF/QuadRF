#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <random>
#include <chrono>
#if __has_include("../Farrow.hpp")
#include "../Farrow.hpp"
#else
#include "Farrow.hpp"
#endif

// Reference standard float Farrow for comparison
class ReferenceFarrowResampler {
private:
    std::vector<float> history_;
    double mu_;
    int skip_samples_;
    bool enabled_;

public:
    ReferenceFarrowResampler()
        : history_(8, 0.0f), mu_(0.0), skip_samples_(0), enabled_(false) {}

    void setEnabled(bool en) { enabled_ = en; }
    void reset() {
        std::fill(history_.begin(), history_.end(), 0.0f);
        mu_ = 0.0;
        skip_samples_ = 0;
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

        int outProduced = 0;
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

        inConsumed = inIndex;
        updateHistory(in, inCount, inConsumed);
        return outProduced;
    }

    void updateHistory(const float* in, int inCount, int inConsumed) {
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

int main() {
    std::cout << "=== Running Farrow Resampler Mathematical Equivalence Tests ===\n";

    const size_t NUM_SAMPLES = 100000;
    std::vector<int8_t> cs8_input(NUM_SAMPLES * 2);
    std::vector<float> cf32_input(NUM_SAMPLES * 2);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(-127, 127);
    for (size_t i = 0; i < NUM_SAMPLES * 2; ++i) {
        int8_t val = static_cast<int8_t>(dist(rng));
        cs8_input[i] = val;
        cf32_input[i] = float(val) / 127.0f;
    }

    // Test ratios (RX downsample rates)
    std::vector<double> rx_ratios = {1.868759, 3.737518, 7.475037, 14.950074, 37.375185, 74.750371};

    for (double ratio : rx_ratios) {
        std::cout << "\nTesting Downsampling Ratio: " << ratio << " (Fin/Fout)...\n";

        // 1. Reference standard float resampler
        ReferenceFarrowResampler ref_resamp;
        ref_resamp.setEnabled(true);
        std::vector<float> ref_out(NUM_SAMPLES * 2);
        int ref_consumed = 0;
        int ref_produced = ref_resamp.process(cf32_input.data(), (int)NUM_SAMPLES, ref_out.data(), (int)NUM_SAMPLES, ratio, ref_consumed);

        // 2. FarrowResamplerCS8
        DSP::FarrowResamplerCS8 cs8_resamp;
        cs8_resamp.setEnabled(true);
        std::vector<float> cs8_out(NUM_SAMPLES * 2);
        int cs8_consumed = 0;
        int cs8_produced = cs8_resamp.processToCF32(cs8_input.data(), (int)NUM_SAMPLES, cs8_out.data(), (int)NUM_SAMPLES, ratio, cs8_consumed);

        assert(ref_produced == cs8_produced);
        assert(ref_consumed == cs8_consumed);

        float max_diff = 0.0f;
        for (int i = 0; i < ref_produced * 2; ++i) {
            float diff = std::abs(ref_out[i] - cs8_out[i]);
            if (diff > max_diff) max_diff = diff;
        }

        std::cout << "  [FarrowResamplerCS8 -> CF32] Max Diff: " << max_diff
                  << " (Produced: " << cs8_produced << ", Consumed: " << cs8_consumed << ")\n";
        assert(max_diff < 1e-5f);
    }

    // Test 4-Channel Interleaved
    std::cout << "\n=== Testing 4-Channel Interleaved SIMD vs 4 Reference Instances ===\n";
    std::vector<int8_t> cs8_4ch(NUM_SAMPLES * 8);
    std::vector<std::vector<float>> cf32_4ch_in(4, std::vector<float>(NUM_SAMPLES * 2));
    for (size_t i = 0; i < NUM_SAMPLES; ++i) {
        for (int ch = 0; ch < 4; ++ch) {
            int8_t I = static_cast<int8_t>(dist(rng));
            int8_t Q = static_cast<int8_t>(dist(rng));
            cs8_4ch[8*i + 2*ch + 0] = I;
            cs8_4ch[8*i + 2*ch + 1] = Q;
            cf32_4ch_in[ch][2*i + 0] = float(I) / 127.0f;
            cf32_4ch_in[ch][2*i + 1] = float(Q) / 127.0f;
        }
    }

    for (double ratio : rx_ratios) {
        // Run 4 independent reference resamplers
        std::vector<std::vector<float>> ref_4ch_out(4, std::vector<float>(NUM_SAMPLES * 2));
        int ref_p[4] = {0}, ref_c[4] = {0};
        for (int ch = 0; ch < 4; ++ch) {
            ReferenceFarrowResampler r;
            r.setEnabled(true);
            ref_p[ch] = r.process(cf32_4ch_in[ch].data(), (int)NUM_SAMPLES, ref_4ch_out[ch].data(), (int)NUM_SAMPLES, ratio, ref_c[ch]);
        }

        // Run unified 4-channel resampler
        DSP::FarrowResampler4Ch resamp4ch;
        resamp4ch.setEnabled(true);
        std::vector<std::vector<float>> out_4ch(4, std::vector<float>(NUM_SAMPLES * 2));
        void* buffs[4] = {out_4ch[0].data(), out_4ch[1].data(), out_4ch[2].data(), out_4ch[3].data()};
        std::vector<size_t> channels = {0, 1, 2, 3};
        int consumed_4ch = 0;
        int produced_4ch = resamp4ch.processToCF32(cs8_4ch.data(), (int)NUM_SAMPLES, buffs, channels, (int)NUM_SAMPLES, ratio, consumed_4ch);

        assert(produced_4ch == ref_p[0]);
        assert(consumed_4ch == ref_c[0]);

        float max_diff = 0.0f;
        for (int ch = 0; ch < 4; ++ch) {
            for (int i = 0; i < produced_4ch * 2; ++i) {
                float diff = std::abs(ref_4ch_out[ch][i] - out_4ch[ch][i]);
                if (diff > max_diff) max_diff = diff;
            }
        }
        std::cout << "  [FarrowResampler4Ch Ratio " << ratio << "] Max Diff across all 4 channels: " << max_diff << "\n";
        assert(max_diff < 1e-5f);
    }

    // Test TX Upsampling Fused CS8
    std::cout << "\n=== Testing TX Upsampling Fused CS8 Quantization ===\n";
    std::vector<double> tx_ratios = {0.46468, 0.23234, 0.11617, 0.05, 0.0116};
    for (double ratio : tx_ratios) {
        DSP::FarrowResampler tx_standard;
        tx_standard.setEnabled(true);
        std::vector<float> tx_float_out(NUM_SAMPLES * 2);
        int tx_c1 = 0;
        int tx_p1 = tx_standard.process(cf32_input.data(), (int)(NUM_SAMPLES / 10), tx_float_out.data(), (int)NUM_SAMPLES, ratio, tx_c1);

        std::vector<int8_t> tx_cs8_expected(tx_p1 * 2);
        for (int i = 0; i < tx_p1 * 2; ++i) {
            tx_cs8_expected[i] = (int8_t)std::max(-128.0f, std::min(127.0f, tx_float_out[i] * 127.0f));
        }

        DSP::FarrowResampler tx_fused;
        tx_fused.setEnabled(true);
        std::vector<int8_t> tx_cs8_actual(tx_p1 * 2);
        int tx_c2 = 0;
        int tx_p2 = tx_fused.processToCS8(cf32_input.data(), (int)(NUM_SAMPLES / 10), tx_cs8_actual.data(), (int)NUM_SAMPLES, ratio, tx_c2);

        assert(tx_p1 == tx_p2);
        assert(tx_c1 == tx_c2);

        int max_err = 0;
        for (int i = 0; i < tx_p1 * 2; ++i) {
            int err = std::abs(int(tx_cs8_expected[i]) - int(tx_cs8_actual[i]));
            if (err > max_err) max_err = err;
        }
        std::cout << "  [TX processToCS8 Ratio " << ratio << "] Max Quantization Difference: " << max_err << " LSB\n";
        assert(max_err <= 1); // Exact within 1 LSB roundoff
    }

    std::cout << "\n>>> ALL MATHEMATICAL EQUIVALENCE TESTS PASSED SUCCESSFULLY! <<<\n";
    return 0;
}

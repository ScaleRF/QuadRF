#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <iomanip>
#if __has_include("../Farrow.hpp")
#include "../Farrow.hpp"
#else
#include "Farrow.hpp"
#endif

// Old bulk conversion helper
void convert_CS8_to_CF32(const int8_t *input, float *output, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        output[2*i+0] = float(input[2*i+0]) / 127.0f;
        output[2*i+1] = float(input[2*i+1]) / 127.0f;
    }
}

void convert_CF32_to_CS8(const float *input, int8_t *output, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        output[2*i+0] = (int8_t)std::max(-128.0f, std::min(127.0f, input[2*i+0] * 127.0f));
        output[2*i+1] = (int8_t)std::max(-128.0f, std::min(127.0f, input[2*i+1] * 127.0f));
    }
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "     Farrow Resampler Benchmark: Old vs Zero-Loss Optimized      \n";
    std::cout << "=================================================================\n\n";

    const size_t NUM_SAMPLES = 2000000; // 2 Million samples
    std::vector<int8_t> cs8_in(NUM_SAMPLES * 2);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(-127, 127);
    for (size_t i = 0; i < NUM_SAMPLES * 2; ++i) {
        cs8_in[i] = static_cast<int8_t>(dist(rng));
    }

    // Benchmark 1: 1-Channel RX Downsampling (Ratio = 14.95, e.g. 150 Msps -> 10 Msps)
    {
        double ratio = 14.950074;
        std::cout << "--- Test 1: 1-Channel RX Downsampling (Fin=149.5 Msps -> Fout=10 Msps, Ratio=" << ratio << ") ---\n";

        // Old approach: Bulk CS8 -> CF32, then Float Farrow -> CF32
        std::vector<float> float_in(NUM_SAMPLES * 2);
        std::vector<float> float_out(NUM_SAMPLES * 2);

        auto t0 = std::chrono::high_resolution_clock::now();
        convert_CS8_to_CF32(cs8_in.data(), float_in.data(), NUM_SAMPLES);
        DSP::FarrowResampler old_resamp;
        old_resamp.setEnabled(true);
        int old_c = 0;
        int old_p = old_resamp.process(float_in.data(), (int)NUM_SAMPLES, float_out.data(), (int)NUM_SAMPLES, ratio, old_c);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms_old = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double msps_old = (double(NUM_SAMPLES) / (ms_old / 1000.0)) / 1e6;

        // New approach: Direct on-the-fly CS8 -> CF32
        DSP::FarrowResamplerCS8 new_resamp;
        new_resamp.setEnabled(true);
        std::vector<float> new_out(NUM_SAMPLES * 2);
        auto t2 = std::chrono::high_resolution_clock::now();
        int new_c = 0;
        int new_p = new_resamp.processToCF32(cs8_in.data(), (int)NUM_SAMPLES, new_out.data(), (int)NUM_SAMPLES, ratio, new_c);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms_new = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double msps_new = (double(NUM_SAMPLES) / (ms_new / 1000.0)) / 1e6;

        std::cout << "  Old (Bulk Convert + Float Farrow): " << std::fixed << std::setprecision(2)
                  << ms_old << " ms (" << msps_old << " Msps)\n";
        std::cout << "  New (Direct On-The-Fly CS8 Tap):   " << std::fixed << std::setprecision(2)
                  << ms_new << " ms (" << msps_new << " Msps)\n";
        std::cout << "  >>> SPEEDUP: " << (ms_old / ms_new) << "x faster <<<\n\n";
    }

    // Benchmark 2: 4-Channel Interleaved RX Downsampling (Ratio = 3.7375, e.g. 37.4 Msps -> 10 Msps)
    {
        double ratio = 3.737518;
        std::cout << "--- Test 2: 4-Channel RX Downsampling (Fin=37.4 Msps -> Fout=10 Msps, Ratio=" << ratio << ") ---\n";
        std::vector<int8_t> cs8_4ch(NUM_SAMPLES * 8);
        for (size_t i = 0; i < NUM_SAMPLES * 8; ++i) cs8_4ch[i] = static_cast<int8_t>(dist(rng));

        // Old approach: 4-channel deinterleave to 4 float buffers, then 4 separate Farrow instances
        std::vector<std::vector<float>> float_4ch_in(4, std::vector<float>(NUM_SAMPLES * 2));
        std::vector<std::vector<float>> float_4ch_out(4, std::vector<float>(NUM_SAMPLES * 2));
        auto t0 = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < NUM_SAMPLES; ++i) {
            for (int ch = 0; ch < 4; ++ch) {
                float_4ch_in[ch][2*i+0] = float(cs8_4ch[8*i + 2*ch + 0]) / 127.0f;
                float_4ch_in[ch][2*i+1] = float(cs8_4ch[8*i + 2*ch + 1]) / 127.0f;
            }
        }
        for (int ch = 0; ch < 4; ++ch) {
            DSP::FarrowResampler r;
            r.setEnabled(true);
            int c = 0;
            r.process(float_4ch_in[ch].data(), (int)NUM_SAMPLES, float_4ch_out[ch].data(), (int)NUM_SAMPLES, ratio, c);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms_old = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double msps_old = (double(NUM_SAMPLES * 4) / (ms_old / 1000.0)) / 1e6;

        // New approach: Unified 4-channel 128-bit SIMD kernel
        DSP::FarrowResampler4Ch resamp4ch;
        resamp4ch.setEnabled(true);
        std::vector<std::vector<float>> new_4ch_out(4, std::vector<float>(NUM_SAMPLES * 2));
        void* buffs[4] = {new_4ch_out[0].data(), new_4ch_out[1].data(), new_4ch_out[2].data(), new_4ch_out[3].data()};
        std::vector<size_t> channels = {0, 1, 2, 3};
        auto t2 = std::chrono::high_resolution_clock::now();
        int new_c = 0;
        resamp4ch.processToCF32(cs8_4ch.data(), (int)NUM_SAMPLES, buffs, channels, (int)NUM_SAMPLES, ratio, new_c);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms_new = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double msps_new = (double(NUM_SAMPLES * 4) / (ms_new / 1000.0)) / 1e6;

        std::cout << "  Old (4x Float Buffers + 4x Farrow): " << std::fixed << std::setprecision(2)
                  << ms_old << " ms (" << msps_old << " total Msps across 4 ch)\n";
        std::cout << "  New (Unified 128-bit 4-Ch SIMD):     " << std::fixed << std::setprecision(2)
                  << ms_new << " ms (" << msps_new << " total Msps across 4 ch)\n";
        std::cout << "  >>> SPEEDUP: " << (ms_old / ms_new) << "x faster <<<\n\n";
    }

    // Benchmark 3: TX Upsampling (Ratio = 0.23234, e.g. 20 Msps host -> 86 Msps line)
    {
        double ratio = 0.232340;
        size_t host_samples = 200000;
        std::cout << "--- Test 3: TX Upsampling (Fhost=20 Msps -> Fline=86 Msps, Ratio=" << ratio << ") ---\n";
        std::vector<float> host_float_in(host_samples * 2);
        for (size_t i = 0; i < host_samples * 2; ++i) host_float_in[i] = float(dist(rng)) / 127.0f;

        // Old approach: Float Farrow to Float Scratch Buffer, then Float -> CS8
        std::vector<float> scratch_float(host_samples * 10 * 2);
        std::vector<int8_t> scratch_cs8(host_samples * 10 * 2);
        auto t0 = std::chrono::high_resolution_clock::now();
        DSP::FarrowResampler tx_old;
        tx_old.setEnabled(true);
        int c_old = 0;
        int p_old = tx_old.process(host_float_in.data(), (int)host_samples, scratch_float.data(), (int)(host_samples * 10), ratio, c_old);
        convert_CF32_to_CS8(scratch_float.data(), scratch_cs8.data(), p_old);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms_old = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double msps_old = (double(p_old) / (ms_old / 1000.0)) / 1e6;

        // New approach: Fused TX Upsample + Direct int8 Quantization
        DSP::FarrowResampler tx_new;
        tx_new.setEnabled(true);
        std::vector<int8_t> new_tx_cs8(host_samples * 10 * 2);
        auto t2 = std::chrono::high_resolution_clock::now();
        int c_new = 0;
        int p_new = tx_new.processToCS8(host_float_in.data(), (int)host_samples, new_tx_cs8.data(), (int)(host_samples * 10), ratio, c_new);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms_new = std::chrono::duration<double, std::milli>(t3 - t2).count();
        double msps_new = (double(p_new) / (ms_new / 1000.0)) / 1e6;

        std::cout << "  Old (Float Upsample + Convert Pass): " << std::fixed << std::setprecision(2)
                  << ms_old << " ms (" << msps_old << " output Msps)\n";
        std::cout << "  New (Fused SIMD Upsample to CS8):    " << std::fixed << std::setprecision(2)
                  << ms_new << " ms (" << msps_new << " output Msps)\n";
        std::cout << "  >>> SPEEDUP: " << (ms_old / ms_new) << "x faster <<<\n\n";
    }

    return 0;
}

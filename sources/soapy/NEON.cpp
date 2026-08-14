#include "MipiDevice.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <limits>
#include <atomic>


// ---------- RX NEON Helpers ----------

// 1 Channel: Convert contiguous CS8 to CF32
void MipiDevice::convert_CS8_to_CF32_NEON(const int8_t *input, float *output, size_t count)
{
    size_t i = 0;
    float32x4_t vscale = vdupq_n_f32(1.0f / 127.0f);

    for (; i + 8 <= count; i += 8) {
        int8x16_t in_s8 = vld1q_s8(input + 2 * i);

        int16x8_t in_s16_lo = vmovl_s8(vget_low_s8(in_s8));
        int16x8_t in_s16_hi = vmovl_s8(vget_high_s8(in_s8));

        int32x4_t in_s32_0 = vmovl_s16(vget_low_s16(in_s16_lo));
        int32x4_t in_s32_1 = vmovl_s16(vget_high_s16(in_s16_lo));
        int32x4_t in_s32_2 = vmovl_s16(vget_low_s16(in_s16_hi));
        int32x4_t in_s32_3 = vmovl_s16(vget_high_s16(in_s16_hi));

        float32x4_t f32_0 = vmulq_f32(vcvtq_f32_s32(in_s32_0), vscale);
        float32x4_t f32_1 = vmulq_f32(vcvtq_f32_s32(in_s32_1), vscale);
        float32x4_t f32_2 = vmulq_f32(vcvtq_f32_s32(in_s32_2), vscale);
        float32x4_t f32_3 = vmulq_f32(vcvtq_f32_s32(in_s32_3), vscale);

        vst1q_f32(output + 2 * i,      f32_0);
        vst1q_f32(output + 2 * i + 4,  f32_1);
        vst1q_f32(output + 2 * i + 8,  f32_2);
        vst1q_f32(output + 2 * i + 12, f32_3);
    }
    for (; i < count; ++i) {
        output[2*i+0] = float(input[2*i+0]) / 127.0f;
        output[2*i+1] = float(input[2*i+1]) / 127.0f;
    }
}

// 4 Channel: Deinterleave CS8 -> CS8
// Input: I0 Q0 I1 Q1 I2 Q2 I3 Q3 ...
void MipiDevice::deinterleave_CS8_NEON(const int8_t *input, void * const *buffs, size_t numElems)
{
    int8_t* d0 = (int8_t*)buffs[0];
    int8_t* d1 = (int8_t*)buffs[1];
    int8_t* d2 = (int8_t*)buffs[2];
    int8_t* d3 = (int8_t*)buffs[3];
    
    const int16_t* src16 = (const int16_t*)input; // Treat IQ pair as one int16 element
    int16_t* d0_16 = (int16_t*)d0;
    int16_t* d1_16 = (int16_t*)d1;
    int16_t* d2_16 = (int16_t*)d2;
    int16_t* d3_16 = (int16_t*)d3;

    size_t i = 0;
    // Process 8 elements per channel per loop (32 elements total read)
    for (; i + 8 <= numElems; i += 8) {
        // Load 4 interleaved vectors of 8 x int16 (I+Q)
        // lane 0 = ch0, lane 1 = ch1, etc.
        int16x8x4_t v = vld4q_s16(src16);
        src16 += 32; 

        vst1q_s16(d0_16, v.val[0]); d0_16 += 8;
        vst1q_s16(d1_16, v.val[1]); d1_16 += 8;
        vst1q_s16(d2_16, v.val[2]); d2_16 += 8;
        vst1q_s16(d3_16, v.val[3]); d3_16 += 8;
    }
    
    // Fallback for remaining samples
    const int8_t* rem_src = (const int8_t*)src16;
    for (; i < numElems; ++i) {
        // each step reads 4 channels * 2 bytes = 8 bytes
        if (d0) { d0[2*i] = rem_src[0]; d0[2*i+1] = rem_src[1]; }
        if (d1) { d1[2*i] = rem_src[2]; d1[2*i+1] = rem_src[3]; }
        if (d2) { d2[2*i] = rem_src[4]; d2[2*i+1] = rem_src[5]; }
        if (d3) { d3[2*i] = rem_src[6]; d3[2*i+1] = rem_src[7]; }
        rem_src += 8;
    }
}

// 4 Channel: Deinterleave CS8 -> CF32
void MipiDevice::deinterleave_CS8_to_CF32_NEON(const int8_t *input, void * const *buffs, size_t numElems)
{
    float* f0 = (float*)buffs[0];
    float* f1 = (float*)buffs[1];
    float* f2 = (float*)buffs[2];
    float* f3 = (float*)buffs[3];
    
    const int16_t* src16 = (const int16_t*)input;
    float32x4_t vscale = vdupq_n_f32(1.0f / 127.0f);

    size_t i = 0;
    for (; i + 8 <= numElems; i += 8) {
        // 1. Load interleaved IQ pairs
        int16x8x4_t v = vld4q_s16(src16);
        src16 += 32;

        // Helper to expand int16x8 (8 packed IQ pairs) to 16 floats
        auto expand_and_store = [&](int16x8_t packed_iq, float* dst) {
            int8x16_t s8 = vreinterpretq_s8_s16(packed_iq);
            
            // Expand low 8 int8s -> 8 floats
            int16x8_t s16_lo = vmovl_s8(vget_low_s8(s8));
            int32x4_t s32_0  = vmovl_s16(vget_low_s16(s16_lo));
            int32x4_t s32_1  = vmovl_s16(vget_high_s16(s16_lo));
            
            // Expand high 8 int8s -> 8 floats
            int16x8_t s16_hi = vmovl_s8(vget_high_s8(s8));
            int32x4_t s32_2  = vmovl_s16(vget_low_s16(s16_hi));
            int32x4_t s32_3  = vmovl_s16(vget_high_s16(s16_hi));
            
            // Convert and scale
            vst1q_f32(dst,    vmulq_f32(vcvtq_f32_s32(s32_0), vscale));
            vst1q_f32(dst+4,  vmulq_f32(vcvtq_f32_s32(s32_1), vscale));
            vst1q_f32(dst+8,  vmulq_f32(vcvtq_f32_s32(s32_2), vscale));
            vst1q_f32(dst+12, vmulq_f32(vcvtq_f32_s32(s32_3), vscale));
        };

        if (f0) expand_and_store(v.val[0], f0 + 2*i);
        if (f1) expand_and_store(v.val[1], f1 + 2*i);
        if (f2) expand_and_store(v.val[2], f2 + 2*i);
        if (f3) expand_and_store(v.val[3], f3 + 2*i);
    }
    
    // Fallback
    const int8_t* rem_src = (const int8_t*)src16;
    for (; i < numElems; ++i) {
        if (f0) { f0[2*i] = float(rem_src[0])/127.0f; f0[2*i+1] = float(rem_src[1])/127.0f; }
        if (f1) { f1[2*i] = float(rem_src[2])/127.0f; f1[2*i+1] = float(rem_src[3])/127.0f; }
        if (f2) { f2[2*i] = float(rem_src[4])/127.0f; f2[2*i+1] = float(rem_src[5])/127.0f; }
        if (f3) { f3[2*i] = float(rem_src[6])/127.0f; f3[2*i+1] = float(rem_src[7])/127.0f; }
        rem_src += 8;
    }
}


// ---------- Tx NEON Functions ----------
void MipiDevice::convert_CF32_to_CS8_NEON(const float *input, int8_t *output, size_t count)
{
    size_t i = 0;
    
    // Scale factor: 127.0 to utilize full range -127 to 127
    float32x4_t vscale = vdupq_n_f32(127.0f); 

    for (; i + 8 <= count; i += 8) {
        float32x4_t in0 = vld1q_f32(input + 2*i);      
        float32x4_t in1 = vld1q_f32(input + 2*i + 4);  
        float32x4_t in2 = vld1q_f32(input + 2*i + 8);  
        float32x4_t in3 = vld1q_f32(input + 2*i + 12); 

        in0 = vmulq_f32(in0, vscale);
        in1 = vmulq_f32(in1, vscale);
        in2 = vmulq_f32(in2, vscale);
        in3 = vmulq_f32(in3, vscale);

        int32x4_t int0 = vcvtq_s32_f32(in0);
        int32x4_t int1 = vcvtq_s32_f32(in1);
        int32x4_t int2 = vcvtq_s32_f32(in2);
        int32x4_t int3 = vcvtq_s32_f32(in3);

        int16x8_t packed_16_0 = vcombine_s16(vqmovn_s32(int0), vqmovn_s32(int1));
        int16x8_t packed_16_1 = vcombine_s16(vqmovn_s32(int2), vqmovn_s32(int3));

        int8x8_t result0 = vqmovn_s16(packed_16_0);
        int8x8_t result1 = vqmovn_s16(packed_16_1);

        vst1_s8(output + 2*i, result0);
        vst1_s8(output + 2*i + 8, result1);
    }

    for (; i < count; ++i) {
        output[2*i]     = (int8_t)(std::max(-128.0f, std::min(127.0f, input[2*i] * 127.0f)));     
        output[2*i + 1] = (int8_t)(std::max(-128.0f, std::min(127.0f, input[2*i + 1] * 127.0f))); 
    }
}


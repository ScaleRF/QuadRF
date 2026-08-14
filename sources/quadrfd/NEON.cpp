#include "NEON.hpp"
#include <arm_neon.h>
#include <algorithm>

void convert_CS8_to_CF32_NEON(const int8_t *input, float *output, size_t count) {
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

void convert_CF32_to_CS8_NEON(const float *input, int8_t *output, size_t count) {
    size_t i = 0;
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

// 4-Channel placeholders
void deinterleave_CS8_NEON(const int8_t *input, void * const *buffs, size_t numElems) {}
void deinterleave_CS8_to_CF32_NEON(const int8_t *input, void * const *buffs, size_t numElems) {}

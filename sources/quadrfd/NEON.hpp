#pragma once
#include <cstdint>
#include <cstddef>

// Standard Stereo IQ
void convert_CS8_to_CF32_NEON(const int8_t *input, float *output, size_t count);
void convert_CF32_to_CS8_NEON(const float *input, int8_t *output, size_t count);

// Placeholders for QuadRF 4-channel phased array mode
void deinterleave_CS8_NEON(const int8_t *input, void * const *buffs, size_t numElems);
void deinterleave_CS8_to_CF32_NEON(const int8_t *input, void * const *buffs, size_t numElems);

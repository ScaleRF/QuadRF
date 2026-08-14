// rendering.c
// Rendering functions: accumulator decay and pixel conversion

#include "rendering.h"
#include "config.h"
#include <math.h>

void decay_accum(uint16_t *acc)
{
    size_t n = (size_t)CANVAS_W * CANVAS_H * 3;
    for (size_t i = 0; i < n; ++i)
        acc[i] = (uint16_t)((float)acc[i] * DECAY_FACTOR);
}

void decay_accum_advanced(uint16_t *acc, float decay_factor, float decay_power)
{
    size_t n = (size_t)CANVAS_W * CANVAS_H * 3;
    float decay = powf(decay_factor, decay_power);
    for (size_t i = 0; i < n; ++i)
        acc[i] = (uint16_t)((float)acc[i] * decay);
}

void accum_to_pixels(const uint16_t *acc, uint32_t *pix)
{
    for (size_t i = 0; i < (size_t)CANVAS_W * CANVAS_H; ++i)
    {
        uint32_t r = acc[i*3+0] >> 6;
        uint32_t g = acc[i*3+1] >> 6;
        uint32_t b = acc[i*3+2] >> 6;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;
        pix[i] = 0xFF000000u | (r<<16) | (g<<8) | b;
    }
}


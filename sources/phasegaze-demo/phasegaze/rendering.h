// rendering.h
// Rendering functions: accumulator decay, pixel/disk stamping

#ifndef RENDERING_H
#define RENDERING_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "config.h"

// ------------------------------------------------------------
// Inline Stamp Functions (hot path - must be inline for performance)
// ------------------------------------------------------------

// Single pixel add with saturation (fast).
static inline void stamp_pixel_add(uint16_t *acc, int x, int y,
                                   float cr, float cg, float cb, float strength)
{
    if ((unsigned)x >= (unsigned)CANVAS_W)  return;
    if ((unsigned)y >= (unsigned)CANVAS_H)  return;

    size_t idx = ((size_t)y * CANVAS_W + (size_t)x) * 3;

    uint32_t ir = (uint32_t)(cr * strength);
    uint32_t ig = (uint32_t)(cg * strength);
    uint32_t ib = (uint32_t)(cb * strength);

    uint32_t nr = acc[idx+0] + ir;
    uint32_t ng = acc[idx+1] + ig;
    uint32_t nb = acc[idx+2] + ib;

    acc[idx+0] = (nr > 65535u) ? 65535u : (uint16_t)nr;
    acc[idx+1] = (ng > 65535u) ? 65535u : (uint16_t)ng;
    acc[idx+2] = (nb > 65535u) ? 65535u : (uint16_t)nb;
}

// Disk stamping with intensity falloff (efficient for small radii).
static inline void stamp_disk_add(uint16_t *acc, int x, int y, int radius,
                                  float cr, float cg, float cb, float strength)
{
    if (radius <= 0)
    {
        stamp_pixel_add(acc, x, y, cr, cg, cb, strength);
        return;
    }

    int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            int dist2 = dx*dx + dy*dy;
            if (dist2 > r2) continue;

            int px = x + dx;
            int py = y + dy;
            if ((unsigned)px >= (unsigned)CANVAS_W) continue;
            if ((unsigned)py >= (unsigned)CANVAS_H) continue;

            // Linear falloff from center
            float dist = sqrtf((float)dist2);
            float falloff = 1.0f - (dist / (float)radius);
            if (falloff < 0.0f) falloff = 0.0f;

            size_t idx = ((size_t)py * CANVAS_W + (size_t)px) * 3;
            float s = strength * falloff;

            uint32_t ir = (uint32_t)(cr * s);
            uint32_t ig = (uint32_t)(cg * s);
            uint32_t ib = (uint32_t)(cb * s);

            uint32_t nr = acc[idx+0] + ir;
            uint32_t ng = acc[idx+1] + ig;
            uint32_t nb = acc[idx+2] + ib;

            acc[idx+0] = (nr > 65535u) ? 65535u : (uint16_t)nr;
            acc[idx+1] = (ng > 65535u) ? 65535u : (uint16_t)ng;
            acc[idx+2] = (nb > 65535u) ? 65535u : (uint16_t)nb;
        }
    }
}

// ------------------------------------------------------------
// Accumulator Functions (implemented in rendering.c)
// ------------------------------------------------------------

void decay_accum(uint16_t *acc);
void decay_accum_advanced(uint16_t *acc, float decay_factor, float decay_power);
void accum_to_pixels(const uint16_t *acc, uint32_t *pix);

#endif // RENDERING_H


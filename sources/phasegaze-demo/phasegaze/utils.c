// utils.c
// Implementation of non-inline utility functions

#include "utils.h"
#include <math.h>

void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b)
{
    if (s <= 0.0f) { *r = *g = *b = v; return; }
    h = fmodf(h, 1.0f); if (h < 0) h += 1.0f;

    float hf = h * 6.0f;
    int i = (int)hf;
    float f = hf - i;

    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));

    switch (i % 6)
    {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default:*r = v; *g = p; *b = q; break;
    }
}

bool is_deterministic(float gx, float gy, float d_lambda)
{
    float pi = (float)M_PI;
    
    // 1. Primary Hexagon Check
    if (gy < -pi || gy > pi) return false;
    float ch1 = -sqrtf(3.0f)/2.0f * gx + 0.5f * gy;
    if (ch1 < -pi || ch1 > pi) return false;
    float ch3 = sqrtf(3.0f)/2.0f * gx + 0.5f * gy;
    if (ch3 < -pi || ch3 > pi) return false;

    // 2. Neighboring Circles Check
    // Reject if (gx,gy) is within radius 2*pi*d_lambda of any reciprocal lattice neighbor.
    // That radius is the hemisphere's extent in gradient space at this d/lambda.
    float r1x = 4.0f * pi / sqrtf(3.0f);
    float r1y = 0.0f;
    float r2x = 2.0f * pi / sqrtf(3.0f);
    float r2y = 2.0f * pi;

    float neighbors[6][2] = {
        { r1x, r1y },
        { -r1x, -r1y },
        { r2x, r2y },
        { -r2x, -r2y },
        { r1x - r2x, r1y - r2y },
        { -r1x + r2x, -r1y + r2y }
    };

    float R = 2.0f * pi * d_lambda;
    float R_squared = R * R;

    for (int i = 0; i < 6; i++) {
        float dx = gx - neighbors[i][0];
        float dy = gy - neighbors[i][1];
        float dist_squared = (dx * dx) + (dy * dy);
        
        if (dist_squared <= R_squared) {
            return false;
        }
    }
    
    return true;
}


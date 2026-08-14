// utils.h
// Utility functions for timing, math, and color conversion

#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

// ------------------------------------------------------------
// Error Handling
// ------------------------------------------------------------

static inline void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

// ------------------------------------------------------------
// Timing Utilities (inline for performance)
// ------------------------------------------------------------

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static inline double ns_to_ms(uint64_t ns)
{
    return (double)ns / 1e6;
}

// ------------------------------------------------------------
// Math Utilities (inline for performance)
// ------------------------------------------------------------

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float wrap_pi_f(float a)
{
    // Wrap to [-pi, pi]
    while (a > (float)M_PI)  a -= (float)(2.0*M_PI);
    while (a < (float)-M_PI) a += (float)(2.0*M_PI);
    return a;
}

/* Circular mean of two angles (radians), result in (-pi, pi] */
static inline float circular_mean2_f(float a, float b)
{
    return atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b));
}

// ------------------------------------------------------------
// CPU Hint (inline for performance)
// ------------------------------------------------------------

static inline void cpu_relax(void)
{
#if defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
    __asm__ __volatile__("pause" ::: "memory");
#else
    __asm__ __volatile__("" ::: "memory");
#endif
}

// ------------------------------------------------------------
// Color Conversion (declaration - implemented in utils.c)
// ------------------------------------------------------------

void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b);
bool is_deterministic(float gx, float gy, float d_lambda);

#endif // UTILS_H


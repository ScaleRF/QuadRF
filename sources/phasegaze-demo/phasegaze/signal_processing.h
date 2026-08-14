// signal_processing.h
// Signal processing: FFT conversion, LO programming, and top-K heap selection

#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>
#include <fftw3.h>
#include "config.h"

// ------------------------------------------------------------
// Heap Item for Top-K Selection
// ------------------------------------------------------------

typedef struct {
    float v;
    int   k;
} heap_item_t;

// ------------------------------------------------------------
// Inline Heap Operations (hot path - must be inline for performance)
// ------------------------------------------------------------

static inline void heap_swap(heap_item_t *a, heap_item_t *b)
{
    heap_item_t t = *a; *a = *b; *b = t;
}

static inline void heap_sift_up(heap_item_t *h, int idx)
{
    while (idx > 0)
    {
        int p = (idx - 1) >> 1;
        if (h[p].v <= h[idx].v) break;
        heap_swap(&h[p], &h[idx]);
        idx = p;
    }
}

static inline void heap_sift_down(heap_item_t *h, int n, int idx)
{
    while (1)
    {
        int l = idx * 2 + 1;
        int r = l + 1;
        int m = idx;

        if (l < n && h[l].v < h[m].v) m = l;
        if (r < n && h[r].v < h[m].v) m = r;
        if (m == idx) break;
        heap_swap(&h[m], &h[idx]);
        idx = m;
    }
}

static inline int heap_push_topk(heap_item_t *h, int size, int cap, float v, int k)
{
    if (size < cap)
    {
        h[size].v = v;
        h[size].k = k;
        heap_sift_up(h, size);
        return size + 1;
    }
    if (v <= h[0].v) return size;
    h[0].v = v;
    h[0].k = k;
    heap_sift_down(h, cap, 0);
    return size;
}

// ------------------------------------------------------------
// Signal Processing Functions (implemented in signal_processing.c)
// ------------------------------------------------------------

// Convert CS8 interleaved samples to FFTW complex format for a specific channel
void cs8_to_fftw_ch(const int8_t *src, int ch, fftwf_complex *dst);

// Program the LO frequency via JTAG
int program_set_freq(int fd, double freq_mhz);

#endif // SIGNAL_PROCESSING_H


// mask.c
// Index-based spur mask implementation

#include "mask.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

int spur_mask_init(spur_mask_t *m)
{
    memset(m, 0, sizeof(*m));
    m->mask  = (float *)calloc(MASK_TOTAL_BINS, sizeof(float));
    m->accum = (float *)calloc(MASK_TOTAL_BINS, sizeof(float));
    if (!m->mask || !m->accum) {
        free(m->mask);
        free(m->accum);
        memset(m, 0, sizeof(*m));
        return -1;
    }
    return 0;
}

void spur_mask_destroy(spur_mask_t *m)
{
    free(m->mask);
    free(m->accum);
    memset(m, 0, sizeof(*m));
}

void spur_mask_cal_start(spur_mask_t *m, int nsteps)
{
    memset(m->accum, 0, MASK_TOTAL_BINS * sizeof(float));
    m->cal_frames = 0;
    m->nlo = nsteps;
}

void spur_mask_cal_accumulate(spur_mask_t *m, int lo_idx,
                              const heap_item_t *topk, int hsz,
                              const float *Vraw, float vmax)
{
    if (lo_idx < 0 || lo_idx >= MAX_LO_STEPS) return;
    if (vmax < 1e-9f) return;

    float inv_vmax = 1.0f / vmax;
    int base = lo_idx * FFT_SIZE;

    for (int t = 0; t < hsz; t++) {
        int k = topk[t].k;
        float v = Vraw[k] * inv_vmax;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        m->accum[base + k] += v;
    }
}

void spur_mask_cal_end_frame(spur_mask_t *m)
{
    m->cal_frames++;
}

void spur_mask_finalize(spur_mask_t *m)
{
    if (m->cal_frames < 1) return;

    float inv_n = 1.0f / (float)m->cal_frames;
    for (int i = 0; i < MASK_TOTAL_BINS; i++)
        m->mask[i] = m->accum[i] * inv_n;

    m->active = 1;
    fprintf(stdout, "[mask] finalized from %d frames, %d LO steps\n",
            m->cal_frames, m->nlo);
}

void spur_mask_clear(spur_mask_t *m)
{
    memset(m->mask, 0, MASK_TOTAL_BINS * sizeof(float));
    memset(m->accum, 0, MASK_TOTAL_BINS * sizeof(float));
    m->cal_frames = 0;
    m->nlo = 0;
    m->active = 0;
}

// mask.h
// Index-based spur mask: identifies and removes hardware artifacts
// (LO leakage, clock harmonics, DC offsets) by their (LO_step, FFT_bin) indices.

#ifndef MASK_H
#define MASK_H

#include "config.h"
#include "signal_processing.h"

#define MASK_TOTAL_BINS  (MAX_LO_STEPS * FFT_SIZE)

typedef struct {
    float *mask;        // finalized mask [MAX_LO_STEPS * FFT_SIZE]
    float *accum;       // calibration accumulator, same shape
    int    cal_frames;  // sweep frames accumulated so far
    int    nlo;         // nsteps when calibration started
    int    active;      // nonzero = mask is loaded and should be applied
} spur_mask_t;

// Allocate mask and accumulator arrays. Returns 0 on success.
int  spur_mask_init(spur_mask_t *m);

// Free all allocations.
void spur_mask_destroy(spur_mask_t *m);

// Begin a new calibration pass: zero the accumulator, record nsteps.
void spur_mask_cal_start(spur_mask_t *m, int nsteps);

// Accumulate one LO step's top-k bins into the calibration accumulator.
// lo_idx: current LO step index within the sweep.
// topk/hsz: the top-k heap items for this step.
// Vraw: per-bin log-power array, vmax: normalizer for this step.
void spur_mask_cal_accumulate(spur_mask_t *m, int lo_idx,
                              const heap_item_t *topk, int hsz,
                              const float *Vraw, float vmax);

// Increment the calibration frame counter (call once per sweep frame).
void spur_mask_cal_end_frame(spur_mask_t *m);

// Finalize: divide accumulator by frame count, store into mask, set active.
void spur_mask_finalize(spur_mask_t *m);

// Deactivate the mask and zero both arrays.
void spur_mask_clear(spur_mask_t *m);

// Return the mask value for a given (lo_idx, fft_bin). Inline for hot path.
static inline float spur_mask_value(const spur_mask_t *m, int lo_idx, int k)
{
    return m->mask[lo_idx * FFT_SIZE + k];
}

#endif // MASK_H

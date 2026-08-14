// worker.h
// Worker thread context and function declarations

#ifndef WORKER_H
#define WORKER_H

#include <stdint.h>
#include <pthread.h>
#include <fftw3.h>
#include "config.h"
#include "mask.h"
#include "phase_cal.h"
#include "imu_worker.h"

// ------------------------------------------------------------
// Point Data (worker output, one per detected signal)
// ------------------------------------------------------------

typedef struct {
    float gx, gy;       // raw phase gradients (kept for future mirroring)
    float r, g, b;       // frequency-mapped color
    float intensity;     // normalized signal magnitude [0,1]
} point_data_t;

// ------------------------------------------------------------
// Telemetry Structure
// ------------------------------------------------------------

typedef struct {
    uint64_t frame_idx;

    double t_read_ms;
    double t_lo_ms;
    double t_fft_ms;
    double t_proc_ms;
    double t_frame_ms;

    // Read breakdown
    double t_read_getinfo_ms;
    double t_read_copy_ms;
    double t_read_consume_ms;
    uint64_t read_calls_getinfo;
    uint64_t read_calls_consume;
    uint64_t read_wraps;

    uint32_t blocks;
    uint32_t points;

    double fps;

    int mask_cal_frames;
    int mask_active;
} telemetry_t;

// ------------------------------------------------------------
// Worker Context Structure
// ------------------------------------------------------------

typedef struct {
    int fd;
    void *ring;
    uint64_t ring_size;

    fftwf_complex *fft_in;
    fftwf_complex *fft_out[CHANNELS_USED];
    fftwf_plan plan[CHANNELS_USED];

    // Double-buffered point output (worker writes back, renderer reads front)
    point_data_t *points_front;
    point_data_t *points_back;
    int npoints_front;
    int npoints_back;
    pthread_mutex_t mtx;

    volatile int quit;
    volatile int camera_mode;
    volatile int calibrate_mode;
    spur_mask_t mask;

    // Phase-offset calibration. Worker reads eps10/eps20/eps30 each frame to
    // apply the trace-phase bias correction, and publishes the live centroid
    // for the boresight UI.
    phase_cal_ctx_t *phase_cal;

    telemetry_t telem;

    // Range control
    double lo_start_mhz;
    double lo_end_mhz;
    double lo_start_new;
    double lo_end_new;
    volatile int range_changed;
    pthread_mutex_t range_mtx;

} ctx_t;

// ------------------------------------------------------------
// Worker Thread Function
// ------------------------------------------------------------

void *worker(void *arg);

#endif // WORKER_H

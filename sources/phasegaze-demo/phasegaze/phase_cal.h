// phase_cal.h
// Phase-offset calibration for the four antenna/receiver paths.
//
// During cal mode, the worker thread pushes one (phi10, phi20, phi30, IMU
// quaternion, intensity) sample per sweep frame into a shared ring. When the
// user stops the cal mode (or hits Phase Save), main.c runs a Gauss-Newton
// fit that simultaneously estimates the three per-baseline trace offsets
// (eps10, eps20, eps30) and the unknown world-frame source direction D.
//
// The offsets are then applied each sweep frame by being subtracted from the
// raw phi_ij values in worker.c, before the ambiguity search.

#ifndef PHASE_CAL_H
#define PHASE_CAL_H

#include <pthread.h>
#include <stdint.h>
#include <stddef.h>

// One stored data point from a sweep frame. Captured at the strongest
// detection (highest normalized magnitude) of that frame. phi10/phi20/phi30
// are the raw differential phases before any bias subtraction; the IMU
// quaternion is the device-to-world rotation at the time of capture.
typedef struct {
    float phi10, phi20, phi30;
    float qw, qx, qy, qz;
    float intensity;
    double rf_mhz;
    double t_unix;
} phase_cal_sample_t;

#define DELAY_CAL_SLOTS 5

typedef struct {
    int    slot;
    double freq_mhz;
    float  eps10, eps20, eps30;
} delay_cal_sample_t;

typedef struct {
    float m10, b10, m20, b20, m30, b30;
    delay_cal_sample_t samples[DELAY_CAL_SLOTS];
    int n_samples;
    int fitted;
} delay_cal_t;

typedef struct {
    pthread_mutex_t lock;

    // ---- Sample ring (written by worker, read by solver) ----
    phase_cal_sample_t *ring;
    int cap;            // capacity in samples
    int head;           // next write position
    int count;          // number of valid samples (saturates at cap)

    // ---- Live state for the GUI ----
    // Most-recent device-frame centroid (intensity-weighted mean (gx, gy))
    // for the on-screen marker. valid=0 means there's no usable centroid
    // this frame (e.g. no points above threshold).
    float live_centroid_gx;
    float live_centroid_gy;
    int   live_centroid_valid;

    // Coverage estimate updated by the solver. orientation_coverage is the
    // number of strongly-separated IMU pose clusters present in the ring:
    // 1 = single sweep, 2 = +flip around one axis, 3 = +flip around both.
    // Updated whenever phase_cal_solve runs.
    int orientation_coverage;

    // Whether a calibration is currently loaded and being applied each frame.
    int loaded;

    // Currently-applied per-baseline offsets (radians). Worker reads these
    // every frame; main writes them after a solve or load. Protected by lock.
    float eps10_rad;
    float eps20_rad;
    float eps30_rad;

    // True while the cal mode is active and samples are being recorded.
    // Worker reads this; main flips it on the button toggle.
    volatile int active;

    // Multi-frequency delay matching (eps_i(f) = m_i * f + b_i).
    delay_cal_t delay_cal;

    // When set, worker skips applying any loaded calibration so boresight
    // capture sees raw phases.
    volatile int capture_active;
} phase_cal_ctx_t;

// ---- Lifecycle ----

// ring_capacity is the maximum number of samples kept in the buffer. Older
// samples are overwritten when the ring wraps.
int  phase_cal_init(phase_cal_ctx_t *ctx, int ring_capacity);
void phase_cal_destroy(phase_cal_ctx_t *ctx);

// ---- Worker-side push ----

// Append one sample to the ring (overwrites oldest if full). Cheap; safe to
// call from the worker thread. No-op if ctx is NULL.
void phase_cal_push_sample(phase_cal_ctx_t *ctx,
                           float phi10, float phi20, float phi30,
                           float qw, float qx, float qy, float qz,
                           float intensity, double rf_mhz);

// Publish the device-frame centroid of the current sweep frame. Worker calls
// this once per frame; the renderer reads it for the overlay crosshair.
void phase_cal_set_live_centroid(phase_cal_ctx_t *ctx,
                                 float gx, float gy, int valid);

// ---- Main-side query ----

// Copy out the live centroid under lock.
void phase_cal_get_live_centroid(const phase_cal_ctx_t *ctx,
                                 float *gx_out, float *gy_out, int *valid_out);

// Return the current sample count (snapshot, no lock needed - int read).
int  phase_cal_sample_count(const phase_cal_ctx_t *ctx);

// Reset the ring to empty. Doesn't touch the currently-applied calibration.
void phase_cal_reset_samples(phase_cal_ctx_t *ctx);

// Apply a new calibration immediately. Worker picks it up on the next frame.
void phase_cal_apply(phase_cal_ctx_t *ctx,
                     float eps10, float eps20, float eps30);

// ---- Solver and persistence ----

typedef struct {
    float eps10_rad, eps20_rad, eps30_rad;
    float Dx, Dy, Dz;             // unit vector, estimated source direction
    float rms_residual_rad;       // RMS residual across all samples/baselines
    int   iterations;             // Gauss-Newton iterations actually run
    int   n_samples;              // samples actually used (after filtering)
    int   orientation_coverage;   // 1..3, see ctx field above
    int   converged;              // 1 if step size dropped below tol
    int   version;
    delay_cal_t delay_cal;
} phase_cal_result_t;

// ---- Delay matching (multi-frequency boresight cal) ----

void phase_cal_delay_reset(phase_cal_ctx_t *ctx);

// Clear the in-progress capture buffer without discarding a fitted calibration.
// Call when starting a new multi-slot capture session.
void phase_cal_delay_begin_capture(phase_cal_ctx_t *ctx);

int phase_cal_delay_push_sample(phase_cal_ctx_t *ctx,
                                int slot, double freq_mhz,
                                float eps10, float eps20, float eps30);

int phase_cal_delay_fit(phase_cal_ctx_t *ctx);

void phase_cal_delay_eval(const phase_cal_ctx_t *ctx, double rf_mhz,
                          float *eps10_out, float *eps20_out,
                          float *eps30_out);

void phase_cal_set_capture_active(phase_cal_ctx_t *ctx, int active);

// Copy delay_cal from a loaded result into ctx and mark loaded.
void phase_cal_apply_result(phase_cal_ctx_t *ctx,
                            const phase_cal_result_t *r);

// Run a Gauss-Newton solve over the samples currently in the ring. Returns 0
// on success, -1 if there are not enough samples or coverage is insufficient.
// sf_for_solver is the phase-gradient scale factor at the LO band center
// (i.e. SCALE_FACTOR_AT_MHZ((lo_start + lo_end) * 0.5)). It maps the linear
// phase model the worker uses into the IMU-rotated source-direction model.
int phase_cal_solve(const phase_cal_ctx_t *ctx,
                    float sf_for_solver,
                    phase_cal_result_t *result_out);

// Write the calibration to a small JSON file (atomic temp+rename).
int phase_cal_save_json(const phase_cal_result_t *r, const char *path);

// Load a calibration JSON. eps10/eps20/eps30 fields are required; others
// optional. Returns 0 on success.
int phase_cal_load_json(phase_cal_result_t *r, const char *path);

// Dump the current sample ring to a CSV (one row per sample). Useful for
// offline analysis or reproducing the solve in Python.
int phase_cal_dump_csv(const phase_cal_ctx_t *ctx, const char *path);

// Build the default paths beside the running executable. Mirrors the
// settings.json helper in settings.c.
int phase_cal_default_json_path(char *buf, size_t buf_sz);
int phase_cal_default_csv_path(char *buf, size_t buf_sz);   // timestamped

#endif // PHASE_CAL_H

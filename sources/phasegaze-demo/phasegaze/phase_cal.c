// phase_cal.c
// Phase-offset calibration. See header for the high-level model.
//
// Math summary
// ------------
// For a Wi-Fi source at world-frame unit direction D and a device pose
// described by quaternion q_k (device->world), the source direction in the
// device frame is d_k = R_k^T D, where R_k is the rotation matrix built
// from q_k. The (gx, gy) phase-gradient produced by the worker satisfies
//
//   gx = sf * d_k.x   gy = sf * d_k.y
//
// (where sf is the array scale factor at the band-center frequency: see
// SCALE_FACTOR_AT_MHZ in config.h). The three differential-phase observables
// the worker measures relate to (gx, gy) through the linear model from
// worker.c:
//
//   psi_1(gx, gy) = -sqrt(3)/2 * gx + 1/2 * gy
//   psi_2(gx, gy) =                       gy
//   psi_3(gx, gy) =  sqrt(3)/2 * gx + 1/2 * gy
//
// Per baseline, the measurement model is
//
//   phi_i_measured = wrap_pi( psi_i(sf*d_k.x, sf*d_k.y) + eps_i + 2*pi*n )
//
// where eps_i (i = 10, 20, 30) is the constant trace-phase offset we want to
// estimate and n is an unknown integer ambiguity. The residual we feed into
// Gauss-Newton wraps to (-pi, pi], which automatically handles n.
//
// Unknowns: 6 -> (eps10, eps20, eps30, Dx, Dy, Dz) with constraint |D| = 1.
// We enforce |D| = 1 by renormalizing after each step. Initial D is the
// (intensity-weighted) average of R_k * (gx_k/sf, gy_k/sf, w_k).
//
// The fit is small (3*N residuals, 6 unknowns) so we solve the 6x6 normal
// equations directly. The whole solve is a couple of milliseconds for a few
// thousand samples on a Pi.

#include "phase_cal.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// Small math helpers (kept local; we don't want to depend on the worker)
// ------------------------------------------------------------

static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= (float)(2.0 * M_PI);
    while (a < -(float)M_PI) a += (float)(2.0 * M_PI);
    return a;
}

// Quaternion (w, x, y, z) -> 3x3 rotation matrix, row-major.
// Identical convention to the rest of the codebase (see main.c).
static void quat_to_R(float qw, float qx, float qy, float qz,
                      float R[3][3])
{
    float xx = qx*qx, yy = qy*qy, zz = qz*qz;
    float xy = qx*qy, xz = qx*qz, yz = qy*qz;
    float wx = qw*qx, wy = qw*qy, wz = qw*qz;

    R[0][0] = 1.0f - 2.0f*(yy + zz);
    R[0][1] =        2.0f*(xy - wz);
    R[0][2] =        2.0f*(xz + wy);

    R[1][0] =        2.0f*(xy + wz);
    R[1][1] = 1.0f - 2.0f*(xx + zz);
    R[1][2] =        2.0f*(yz - wx);

    R[2][0] =        2.0f*(xz - wy);
    R[2][1] =        2.0f*(yz + wx);
    R[2][2] = 1.0f - 2.0f*(xx + yy);
}

// 6x6 linear solve via gaussian elimination with partial pivoting. Used to
// solve normal equations (J^T J) * step = J^T r. n is fixed at 6 here but
// kept general so the routine stays readable.
static int solve_dense(int n, double *A, double *b, double *x)
{
    // Augmented row reduction in place.
    for (int col = 0; col < n; ++col) {
        // Pivot
        int piv = col;
        double pivval = fabs(A[col * n + col]);
        for (int r = col + 1; r < n; ++r) {
            double v = fabs(A[r * n + col]);
            if (v > pivval) { pivval = v; piv = r; }
        }
        if (pivval < 1e-14) return -1;
        if (piv != col) {
            for (int c = 0; c < n; ++c) {
                double t = A[col * n + c];
                A[col * n + c] = A[piv * n + c];
                A[piv * n + c] = t;
            }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        // Eliminate
        double inv = 1.0 / A[col * n + col];
        for (int r = col + 1; r < n; ++r) {
            double f = A[r * n + col] * inv;
            if (f == 0.0) continue;
            for (int c = col; c < n; ++c)
                A[r * n + c] -= f * A[col * n + c];
            b[r] -= f * b[col];
        }
    }
    // Back-substitute
    for (int r = n - 1; r >= 0; --r) {
        double s = b[r];
        for (int c = r + 1; c < n; ++c) s -= A[r * n + c] * x[c];
        x[r] = s / A[r * n + r];
    }
    return 0;
}

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

int phase_cal_init(phase_cal_ctx_t *ctx, int ring_capacity)
{
    if (!ctx) return -1;
    memset(ctx, 0, sizeof(*ctx));
    if (ring_capacity < 64) ring_capacity = 64;
    ctx->ring = (phase_cal_sample_t*)calloc((size_t)ring_capacity,
                                            sizeof(phase_cal_sample_t));
    if (!ctx->ring) return -1;
    ctx->cap   = ring_capacity;
    ctx->head  = 0;
    ctx->count = 0;
    pthread_mutex_init(&ctx->lock, NULL);
    return 0;
}

void phase_cal_destroy(phase_cal_ctx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_destroy(&ctx->lock);
    free(ctx->ring);
    memset(ctx, 0, sizeof(*ctx));
}

// ------------------------------------------------------------
// Worker-side push
// ------------------------------------------------------------

void phase_cal_push_sample(phase_cal_ctx_t *ctx,
                           float phi10, float phi20, float phi30,
                           float qw, float qx, float qy, float qz,
                           float intensity, double rf_mhz)
{
    if (!ctx || !ctx->ring) return;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    double t_unix = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;

    pthread_mutex_lock(&ctx->lock);
    phase_cal_sample_t *s = &ctx->ring[ctx->head];
    s->phi10 = phi10; s->phi20 = phi20; s->phi30 = phi30;
    s->qw = qw; s->qx = qx; s->qy = qy; s->qz = qz;
    s->intensity = intensity;
    s->rf_mhz    = rf_mhz;
    s->t_unix    = t_unix;
    ctx->head = (ctx->head + 1) % ctx->cap;
    if (ctx->count < ctx->cap) ctx->count++;
    pthread_mutex_unlock(&ctx->lock);
}

void phase_cal_set_live_centroid(phase_cal_ctx_t *ctx,
                                 float gx, float gy, int valid)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    ctx->live_centroid_gx = gx;
    ctx->live_centroid_gy = gy;
    ctx->live_centroid_valid = valid;
    pthread_mutex_unlock(&ctx->lock);
}

// ------------------------------------------------------------
// Main-side query / control
// ------------------------------------------------------------

void phase_cal_get_live_centroid(const phase_cal_ctx_t *ctx,
                                 float *gx_out, float *gy_out, int *valid_out)
{
    if (!ctx) { if (valid_out) *valid_out = 0; return; }
    pthread_mutex_lock((pthread_mutex_t*)&ctx->lock);
    if (gx_out)    *gx_out    = ctx->live_centroid_gx;
    if (gy_out)    *gy_out    = ctx->live_centroid_gy;
    if (valid_out) *valid_out = ctx->live_centroid_valid;
    pthread_mutex_unlock((pthread_mutex_t*)&ctx->lock);
}

int phase_cal_sample_count(const phase_cal_ctx_t *ctx)
{
    if (!ctx) return 0;
    return ctx->count;
}

void phase_cal_reset_samples(phase_cal_ctx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    ctx->head  = 0;
    ctx->count = 0;
    pthread_mutex_unlock(&ctx->lock);
}

void phase_cal_apply(phase_cal_ctx_t *ctx,
                     float eps10, float eps20, float eps30)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    ctx->eps10_rad = eps10;
    ctx->eps20_rad = eps20;
    ctx->eps30_rad = eps30;
    ctx->loaded = 1;
    pthread_mutex_unlock(&ctx->lock);
}

// Least-squares y = m*x + b for n points.
static int linear_fit(const double *x, const double *y, int n,
                      float *m_out, float *b_out)
{
    if (n < 2 || !x || !y || !m_out || !b_out) return -1;

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int i = 0; i < n; ++i) {
        sx  += x[i];
        sy  += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    double dn = (double)n;
    double denom = dn * sxx - sx * sx;
    if (fabs(denom) < 1e-12) return -1;

    double m = (dn * sxy - sx * sy) / denom;
    double b = (sy - m * sx) / dn;
    *m_out = (float)m;
    *b_out = (float)b;
    return 0;
}

void phase_cal_delay_reset(phase_cal_ctx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    memset(&ctx->delay_cal, 0, sizeof(ctx->delay_cal));
    ctx->loaded = 0;
    ctx->eps10_rad = 0.0f;
    ctx->eps20_rad = 0.0f;
    ctx->eps30_rad = 0.0f;
    pthread_mutex_unlock(&ctx->lock);
}

void phase_cal_delay_begin_capture(phase_cal_ctx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    ctx->delay_cal.n_samples = 0;
    memset(ctx->delay_cal.samples, 0, sizeof(ctx->delay_cal.samples));
    pthread_mutex_unlock(&ctx->lock);
}

int phase_cal_delay_push_sample(phase_cal_ctx_t *ctx,
                                int slot, double freq_mhz,
                                float eps10, float eps20, float eps30)
{
    if (!ctx) return -1;
    pthread_mutex_lock(&ctx->lock);
    if (ctx->delay_cal.n_samples >= DELAY_CAL_SLOTS) {
        pthread_mutex_unlock(&ctx->lock);
        return -1;
    }
    delay_cal_sample_t *s = &ctx->delay_cal.samples[ctx->delay_cal.n_samples];
    s->slot     = slot;
    s->freq_mhz = freq_mhz;
    s->eps10    = eps10;
    s->eps20    = eps20;
    s->eps30    = eps30;
    ctx->delay_cal.n_samples++;
    ctx->delay_cal.fitted = 0;
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

int phase_cal_delay_fit(phase_cal_ctx_t *ctx)
{
    if (!ctx) return -1;

    pthread_mutex_lock(&ctx->lock);
    int n = ctx->delay_cal.n_samples;
    if (n < DELAY_CAL_SLOTS) {
        pthread_mutex_unlock(&ctx->lock);
        return -1;
    }

    double fx[DELAY_CAL_SLOTS];
    double y10[DELAY_CAL_SLOTS], y20[DELAY_CAL_SLOTS], y30[DELAY_CAL_SLOTS];
    for (int i = 0; i < n; ++i) {
        fx[i]  = ctx->delay_cal.samples[i].freq_mhz;
        y10[i] = (double)ctx->delay_cal.samples[i].eps10;
        y20[i] = (double)ctx->delay_cal.samples[i].eps20;
        y30[i] = (double)ctx->delay_cal.samples[i].eps30;
    }

    float m10, b10, m20, b20, m30, b30;
    if (linear_fit(fx, y10, n, &m10, &b10) != 0 ||
        linear_fit(fx, y20, n, &m20, &b20) != 0 ||
        linear_fit(fx, y30, n, &m30, &b30) != 0)
    {
        pthread_mutex_unlock(&ctx->lock);
        return -1;
    }

    ctx->delay_cal.m10 = m10;
    ctx->delay_cal.b10 = b10;
    ctx->delay_cal.m20 = m20;
    ctx->delay_cal.b20 = b20;
    ctx->delay_cal.m30 = m30;
    ctx->delay_cal.b30 = b30;
    ctx->delay_cal.fitted = 1;
    ctx->loaded = 1;
    ctx->eps10_rad = 0.0f;
    ctx->eps20_rad = 0.0f;
    ctx->eps30_rad = 0.0f;
    pthread_mutex_unlock(&ctx->lock);
    return 0;
}

void phase_cal_delay_eval(const phase_cal_ctx_t *ctx, double rf_mhz,
                          float *eps10_out, float *eps20_out,
                          float *eps30_out)
{
    if (!ctx || !ctx->delay_cal.fitted) {
        if (eps10_out) *eps10_out = 0.0f;
        if (eps20_out) *eps20_out = 0.0f;
        if (eps30_out) *eps30_out = 0.0f;
        return;
    }
    float f = (float)rf_mhz;
    if (eps10_out) *eps10_out = ctx->delay_cal.m10 * f + ctx->delay_cal.b10;
    if (eps20_out) *eps20_out = ctx->delay_cal.m20 * f + ctx->delay_cal.b20;
    if (eps30_out) *eps30_out = ctx->delay_cal.m30 * f + ctx->delay_cal.b30;
}

void phase_cal_set_capture_active(phase_cal_ctx_t *ctx, int active)
{
    if (!ctx) return;
    ctx->capture_active = active ? 1 : 0;
}

void phase_cal_apply_result(phase_cal_ctx_t *ctx,
                            const phase_cal_result_t *r)
{
    if (!ctx || !r) return;
    pthread_mutex_lock(&ctx->lock);
    ctx->eps10_rad = r->eps10_rad;
    ctx->eps20_rad = r->eps20_rad;
    ctx->eps30_rad = r->eps30_rad;
    ctx->delay_cal = r->delay_cal;
    // Persisted samples are archival; don't block a new capture session.
    ctx->delay_cal.n_samples = 0;
    memset(ctx->delay_cal.samples, 0, sizeof(ctx->delay_cal.samples));
    if (r->delay_cal.fitted) {
        ctx->loaded = 1;
        ctx->eps10_rad = 0.0f;
        ctx->eps20_rad = 0.0f;
        ctx->eps30_rad = 0.0f;
    } else if (r->version >= 1) {
        // Legacy v1: constant offset at all frequencies (m=0, b=eps).
        ctx->delay_cal.m10 = 0.0f;
        ctx->delay_cal.b10 = r->eps10_rad;
        ctx->delay_cal.m20 = 0.0f;
        ctx->delay_cal.b20 = r->eps20_rad;
        ctx->delay_cal.m30 = 0.0f;
        ctx->delay_cal.b30 = r->eps30_rad;
        ctx->delay_cal.fitted = 1;
        ctx->delay_cal.n_samples = 0;
        ctx->loaded = 1;
    } else {
        ctx->loaded = 0;
    }
    pthread_mutex_unlock(&ctx->lock);
}

// ------------------------------------------------------------
// Solver
// ------------------------------------------------------------
//
// Layout of unknowns x (length 6):
//   x[0] = eps10
//   x[1] = eps20
//   x[2] = eps30
//   x[3] = Dx
//   x[4] = Dy
//   x[5] = Dz
//
// For each sample k, three residual rows (one per baseline). Letting
//   d = R_k^T D
//   ux = sf * d.x, uy = sf * d.y
//   psi_1 = -SQ32*ux + 0.5*uy
//   psi_2 =               uy
//   psi_3 =  SQ32*ux + 0.5*uy
// (SQ32 = sqrt(3)/2)
//
// Residual r_i = wrap_pi( phi_i - psi_i - eps_i )
// dpsi_i/dDx, dpsi_i/dDy, dpsi_i/dDz are linear functions of (R_k columns).
//
// Jacobian row for baseline i, sample k:
//   J[col eps_j]  = -1 if j == i else 0
//   J[col Dx]     = -dpsi_i/dDx
//   J[col Dy]     = -dpsi_i/dDy
//   J[col Dz]     = -dpsi_i/dDz
//
// (dpsi_i/dD = -dpsi_i/dD_world inverse? - careful: psi_i depends on d = R^T D
//  so dd/dD = R^T, and dpsi_i/dD = (dpsi_i/dd) * R^T.)

static const float SQ32 = 0.8660254037844386f;

// Returns 1 if the sample is geometrically valid (|R^T D|=1 ensured by
// normalization elsewhere; here we just guard against bad quaternions).
static int build_residual_jacobian_row(
    const phase_cal_sample_t *s,
    int baseline_idx,                // 0,1,2 for phi10/phi20/phi30
    float sf,
    const float D[3],
    const float eps[3],
    double r_out[1],                 // single residual value
    double J_out[6])                 // 6-element row of the jacobian
{
    float R[3][3];
    quat_to_R(s->qw, s->qx, s->qy, s->qz, R);

    // d = R^T * D
    float dx = R[0][0]*D[0] + R[1][0]*D[1] + R[2][0]*D[2];
    float dy = R[0][1]*D[0] + R[1][1]*D[1] + R[2][1]*D[2];

    float ux = sf * dx;
    float uy = sf * dy;

    // Predicted phase + observation
    float psi[3] = {
        -SQ32*ux + 0.5f*uy,
                       uy,
         SQ32*ux + 0.5f*uy,
    };
    float phi_obs[3] = { s->phi10, s->phi20, s->phi30 };

    float resid = wrap_pi(phi_obs[baseline_idx] - psi[baseline_idx]
                          - eps[baseline_idx]);
    r_out[0] = (double)resid;

    // Jacobian w.r.t. eps_j -> -delta_{ij}
    for (int j = 0; j < 3; ++j) J_out[j] = 0.0;
    J_out[baseline_idx] = -1.0;

    // dpsi/dux, dpsi/duy
    float dpsi_dux, dpsi_duy;
    switch (baseline_idx) {
        case 0: dpsi_dux = -SQ32; dpsi_duy = 0.5f; break;
        case 1: dpsi_dux = 0.0f;  dpsi_duy = 1.0f; break;
        default:dpsi_dux =  SQ32; dpsi_duy = 0.5f; break;
    }

    // dux/dD_world = sf * R^T row 0 = sf * (R[0][0], R[1][0], R[2][0])
    // duy/dD_world = sf * R^T row 1 = sf * (R[0][1], R[1][1], R[2][1])
    float dux_dDx = sf * R[0][0], dux_dDy = sf * R[1][0], dux_dDz = sf * R[2][0];
    float duy_dDx = sf * R[0][1], duy_dDy = sf * R[1][1], duy_dDz = sf * R[2][1];

    float dpsi_dDx = dpsi_dux*dux_dDx + dpsi_duy*duy_dDx;
    float dpsi_dDy = dpsi_dux*dux_dDy + dpsi_duy*duy_dDy;
    float dpsi_dDz = dpsi_dux*dux_dDz + dpsi_duy*duy_dDz;

    // r = phi - psi - eps, so dr/dD = -dpsi/dD
    J_out[3] = -(double)dpsi_dDx;
    J_out[4] = -(double)dpsi_dDy;
    J_out[5] = -(double)dpsi_dDz;

    return 1;
}

// Estimate orientation coverage by counting how many distinct boresight
// directions the samples span. Boresight in world frame at sample k is
// R_k * (0, 0, 1) = third column of R_k. We bucket those onto +Z, -Z and
// "sideways" (|z| < 0.5); the count is 1 for sweep-only, 2 if a flip was
// done, 3 if multiple axes were flipped.
static int estimate_coverage(const phase_cal_sample_t *ring, int count)
{
    if (count <= 0) return 0;
    int has_up = 0, has_down = 0, has_side_x = 0, has_side_y = 0;
    for (int i = 0; i < count; ++i) {
        const phase_cal_sample_t *s = &ring[i];
        float R[3][3];
        quat_to_R(s->qw, s->qx, s->qy, s->qz, R);
        // Boresight (device +Z) in world: third column of R.
        float bz = R[2][2];
        float bx = R[0][2];
        float by = R[1][2];
        if      (bz >  0.5f) has_up = 1;
        else if (bz < -0.5f) has_down = 1;
        else if (fabsf(bx) > 0.5f) has_side_x = 1;
        else if (fabsf(by) > 0.5f) has_side_y = 1;
    }
    int n = has_up + has_down + has_side_x + has_side_y;
    if (n < 1) n = 1;
    if (n > 3) n = 3;
    return n;
}

int phase_cal_solve(const phase_cal_ctx_t *ctx,
                    float sf_for_solver,
                    phase_cal_result_t *result_out)
{
    if (!ctx || !result_out) return -1;
    if (sf_for_solver <= 0.0f) return -1;

    // Snapshot the ring so we don't have to hold the lock during the solve.
    pthread_mutex_lock((pthread_mutex_t*)&ctx->lock);
    int n = ctx->count;
    if (n < 16) {
        pthread_mutex_unlock((pthread_mutex_t*)&ctx->lock);
        return -1;
    }
    phase_cal_sample_t *samp = (phase_cal_sample_t*)malloc(
        (size_t)n * sizeof(phase_cal_sample_t));
    if (!samp) {
        pthread_mutex_unlock((pthread_mutex_t*)&ctx->lock);
        return -1;
    }
    // Linearize the ring: oldest first. head is next-write index, count==cap
    // means we've wrapped. If not wrapped, samples are [0, count).
    if (ctx->count < ctx->cap) {
        memcpy(samp, ctx->ring, (size_t)n * sizeof(phase_cal_sample_t));
    } else {
        int tail = ctx->head;
        int first_n = ctx->cap - tail;
        memcpy(samp, &ctx->ring[tail],
               (size_t)first_n * sizeof(phase_cal_sample_t));
        memcpy(samp + first_n, ctx->ring,
               (size_t)tail * sizeof(phase_cal_sample_t));
    }
    pthread_mutex_unlock((pthread_mutex_t*)&ctx->lock);

    int coverage = estimate_coverage(samp, n);
    if (coverage < 2) {
        // The constant-bias direction is degenerate without at least one
        // flipped pose. Warn the caller; they should ask the user to flip.
        free(samp);
        return -1;
    }

    // ---- Initial guess ----
    // eps = 0; D = normalized intensity-weighted mean of R_k * (ux/sf, uy/sf, w)
    // estimated from the worker's existing (gx, gy). We don't have gx/gy here
    // directly, but we can recover an approximation: solve the per-sample
    // ambiguity-free system for (ux, uy) from phi_i, then project. For an
    // initial guess that's good enough.
    float eps[3] = {0.0f, 0.0f, 0.0f};
    float D[3] = {0.0f, 0.0f, 1.0f};
    {
        double sx = 0.0, sy = 0.0, sz = 0.0;
        double wsum = 0.0;
        for (int i = 0; i < n; ++i) {
            // Least-squares (ux, uy) from psi model. Closed form:
            //   gx = (phi30 - phi10) / sqrt(3)
            //   gy = (phi10 + 2*phi20 + phi30) / 3
            // (matches the existing predicted-vs-measured layout)
            float gx = (samp[i].phi30 - samp[i].phi10) / 1.7320508f;
            float gy = (samp[i].phi10 + 2.0f*samp[i].phi20 + samp[i].phi30)
                       / 3.0f;
            float u = gx / sf_for_solver;
            float v = gy / sf_for_solver;
            float r2 = u*u + v*v;
            if (r2 > 0.95f) continue;
            float w = sqrtf(1.0f - r2);

            // d_dev = (u, v, w); D = R * d_dev
            float R[3][3];
            quat_to_R(samp[i].qw, samp[i].qx, samp[i].qy, samp[i].qz, R);
            float Dx = R[0][0]*u + R[0][1]*v + R[0][2]*w;
            float Dy = R[1][0]*u + R[1][1]*v + R[1][2]*w;
            float Dz = R[2][0]*u + R[2][1]*v + R[2][2]*w;

            float wt = samp[i].intensity > 0.0f ? samp[i].intensity : 1.0f;
            sx += wt * Dx; sy += wt * Dy; sz += wt * Dz;
            wsum += wt;
        }
        if (wsum > 0.0) {
            float nx = (float)(sx / wsum);
            float ny = (float)(sy / wsum);
            float nz = (float)(sz / wsum);
            float nm = sqrtf(nx*nx + ny*ny + nz*nz);
            if (nm > 1e-6f) {
                D[0] = nx / nm; D[1] = ny / nm; D[2] = nz / nm;
            }
        }
    }

    // ---- Gauss-Newton iterations ----
    const int max_iter = 30;
    const double tol_step = 1e-6;
    int iter = 0;
    int converged = 0;
    double last_rms = 0.0;

    double JtJ[36], Jtr[6], step[6];

    for (iter = 0; iter < max_iter; ++iter) {
        memset(JtJ, 0, sizeof(JtJ));
        memset(Jtr, 0, sizeof(Jtr));

        double sse = 0.0;
        int nres = 0;

        for (int k = 0; k < n; ++k) {
            for (int i = 0; i < 3; ++i) {
                double r;
                double J[6];
                if (!build_residual_jacobian_row(&samp[k], i, sf_for_solver,
                                                 D, eps, &r, J))
                    continue;
                sse += r * r;
                nres++;

                // JtJ += J^T J ; Jtr += J^T r
                for (int a = 0; a < 6; ++a) {
                    Jtr[a] += J[a] * r;
                    for (int b = 0; b < 6; ++b)
                        JtJ[a*6 + b] += J[a] * J[b];
                }
            }
        }
        if (nres == 0) {
            free(samp);
            return -1;
        }

        last_rms = sqrt(sse / (double)nres);

        // Regularize D-block lightly to keep things well-conditioned when
        // the orientations don't fully cover the sphere.
        for (int a = 3; a < 6; ++a) JtJ[a*6 + a] += 1e-4;

        // Solve JtJ * step = Jtr  -> Gauss-Newton step = -(JtJ)^-1 * Jtr,
        // but we already put r (not -r) in Jtr, so step is in the direction
        // that *reduces* SSE: x_new = x_old - step.
        if (solve_dense(6, JtJ, Jtr, step) != 0) {
            free(samp);
            return -1;
        }

        // Apply step
        eps[0] -= (float)step[0];
        eps[1] -= (float)step[1];
        eps[2] -= (float)step[2];
        D[0]   -= (float)step[3];
        D[1]   -= (float)step[4];
        D[2]   -= (float)step[5];
        // Wrap epses to (-pi, pi]
        eps[0] = wrap_pi(eps[0]);
        eps[1] = wrap_pi(eps[1]);
        eps[2] = wrap_pi(eps[2]);
        // Renormalize D
        float nm = sqrtf(D[0]*D[0] + D[1]*D[1] + D[2]*D[2]);
        if (nm > 1e-6f) { D[0] /= nm; D[1] /= nm; D[2] /= nm; }

        double snorm = 0.0;
        for (int a = 0; a < 6; ++a) snorm += step[a] * step[a];
        if (sqrt(snorm) < tol_step) { converged = 1; iter++; break; }
    }

    result_out->eps10_rad = eps[0];
    result_out->eps20_rad = eps[1];
    result_out->eps30_rad = eps[2];
    result_out->Dx = D[0];
    result_out->Dy = D[1];
    result_out->Dz = D[2];
    result_out->rms_residual_rad = (float)last_rms;
    result_out->iterations = iter;
    result_out->n_samples = n;
    result_out->orientation_coverage = coverage;
    result_out->converged = converged;

    free(samp);
    return 0;
}

// ------------------------------------------------------------
// JSON / CSV
// ------------------------------------------------------------
//
// Minimal hand-rolled JSON to match the style used by settings.c. No external
// dependency, no quoting of strings other than keys.

static int find_key(const char *buf, const char *key, const char **val_out)
{
    char q[80];
    snprintf(q, sizeof(q), "\"%s\"", key);
    const char *p = strstr(buf, q);
    if (!p) return -1;
    p = strchr(p + strlen(q), ':');
    if (!p) return -1;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    *val_out = p;
    return 0;
}

static int parse_double_field(const char *buf, const char *key, double *out)
{
    const char *v;
    if (find_key(buf, key, &v) != 0) return -1;
    char *end = NULL;
    double x = strtod(v, &end);
    if (end == v) return -1;
    *out = x;
    return 0;
}

int phase_cal_save_json(const phase_cal_result_t *r, const char *path)
{
    if (!r || !path) return -1;

    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *fp = fopen(tmp, "wb");
    if (!fp) return -1;

    long ts = (long)time(NULL);
    int ver = r->delay_cal.fitted ? 2 : 1;

    int n = fprintf(fp,
        "{\n"
        "  \"version\": %d,\n"
        "  \"eps10_rad\": %.10g,\n"
        "  \"eps20_rad\": %.10g,\n"
        "  \"eps30_rad\": %.10g,\n"
        "  \"estimated_source_dir\": [%.6f, %.6f, %.6f],\n"
        "  \"rms_residual_rad\": %.6g,\n"
        "  \"samples\": %d,\n"
        "  \"orientation_coverage\": %d,\n"
        "  \"iterations\": %d,\n"
        "  \"converged\": %d,\n"
        "  \"created_unix\": %ld",
        ver,
        (double)r->eps10_rad,
        (double)r->eps20_rad,
        (double)r->eps30_rad,
        (double)r->Dx, (double)r->Dy, (double)r->Dz,
        (double)r->rms_residual_rad,
        r->n_samples,
        r->orientation_coverage,
        r->iterations,
        r->converged,
        ts);

    if (n < 0) { fclose(fp); unlink(tmp); return -1; }

    if (r->delay_cal.fitted) {
        const delay_cal_t *d = &r->delay_cal;
        n = fprintf(fp,
            ",\n"
            "  \"delay_cal\": {\n"
            "    \"m10_rad_per_mhz\": %.10g,\n"
            "    \"b10_rad\": %.10g,\n"
            "    \"m20_rad_per_mhz\": %.10g,\n"
            "    \"b20_rad\": %.10g,\n"
            "    \"m30_rad_per_mhz\": %.10g,\n"
            "    \"b30_rad\": %.10g,\n"
            "    \"samples\": [\n",
            (double)d->m10, (double)d->b10,
            (double)d->m20, (double)d->b20,
            (double)d->m30, (double)d->b30);
        if (n < 0) { fclose(fp); unlink(tmp); return -1; }

        for (int i = 0; i < d->n_samples; ++i) {
            const delay_cal_sample_t *s = &d->samples[i];
            n = fprintf(fp,
                "      {\"slot\": %d, \"freq_mhz\": %.1f, "
                "\"eps10_rad\": %.10g, \"eps20_rad\": %.10g, "
                "\"eps30_rad\": %.10g}%s\n",
                s->slot, s->freq_mhz,
                (double)s->eps10, (double)s->eps20, (double)s->eps30,
                (i + 1 < d->n_samples) ? "," : "");
            if (n < 0) { fclose(fp); unlink(tmp); return -1; }
        }
        n = fprintf(fp, "    ]\n  }\n");
        if (n < 0) { fclose(fp); unlink(tmp); return -1; }
    }

    n = fprintf(fp, "}\n");
    if (n < 0 || fclose(fp) != 0) { unlink(tmp); return -1; }
    if (rename(tmp, path) != 0) { unlink(tmp); return -1; }
    return 0;
}

int phase_cal_load_json(phase_cal_result_t *r, const char *path)
{
    if (!r || !path) return -1;
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return -1; }
    long sz = ftell(fp);
    if (sz < 0 || sz > 65536) { fclose(fp); return -1; }
    rewind(fp);
    char *buf = (char*)malloc((size_t)sz + 1);
    if (!buf) { fclose(fp); return -1; }
    if (fread(buf, 1, (size_t)sz, fp) != (size_t)sz) {
        free(buf); fclose(fp); return -1;
    }
    fclose(fp);
    buf[sz] = '\0';

    memset(r, 0, sizeof(*r));

    double v;
    if (parse_double_field(buf, "version", &v) == 0)
        r->version = (int)v;
    else
        r->version = 1;

    if (parse_double_field(buf, "eps10_rad", &v) != 0) { free(buf); return -1; }
    r->eps10_rad = (float)v;
    if (parse_double_field(buf, "eps20_rad", &v) != 0) { free(buf); return -1; }
    r->eps20_rad = (float)v;
    if (parse_double_field(buf, "eps30_rad", &v) != 0) { free(buf); return -1; }
    r->eps30_rad = (float)v;

    if (parse_double_field(buf, "rms_residual_rad", &v) == 0)
        r->rms_residual_rad = (float)v;
    if (parse_double_field(buf, "samples", &v) == 0)
        r->n_samples = (int)v;
    if (parse_double_field(buf, "orientation_coverage", &v) == 0)
        r->orientation_coverage = (int)v;
    if (parse_double_field(buf, "iterations", &v) == 0)
        r->iterations = (int)v;
    if (parse_double_field(buf, "converged", &v) == 0)
        r->converged = (int)v;

    // estimated_source_dir is a JSON array; cheap & sufficient: look for the
    // key, skip to '[', parse three doubles.
    const char *p = NULL;
    if (find_key(buf, "estimated_source_dir", &p) == 0) {
        p = strchr(p, '[');
        if (p) {
            char *end = NULL;
            r->Dx = (float)strtod(p + 1, &end);
            if (end && *end == ',') {
                r->Dy = (float)strtod(end + 1, &end);
                if (end && *end == ',') {
                    r->Dz = (float)strtod(end + 1, &end);
                }
            }
        }
    }

    // Version 2: delay_cal block with m,b and per-slot samples.
    const char *dc = strstr(buf, "\"delay_cal\"");
    if (dc) {
        delay_cal_t *d = &r->delay_cal;
        if (parse_double_field(buf, "m10_rad_per_mhz", &v) == 0) d->m10 = (float)v;
        if (parse_double_field(buf, "b10_rad", &v) == 0) d->b10 = (float)v;
        if (parse_double_field(buf, "m20_rad_per_mhz", &v) == 0) d->m20 = (float)v;
        if (parse_double_field(buf, "b20_rad", &v) == 0) d->b20 = (float)v;
        if (parse_double_field(buf, "m30_rad_per_mhz", &v) == 0) d->m30 = (float)v;
        if (parse_double_field(buf, "b30_rad", &v) == 0) d->b30 = (float)v;
        d->fitted = 1;

        const char *sp = strstr(dc, "\"samples\"");
        if (sp) {
            sp = strchr(sp, '[');
            if (sp) {
                sp++;
                while (*sp && d->n_samples < DELAY_CAL_SLOTS) {
                    while (*sp == ' ' || *sp == '\t' || *sp == '\n' ||
                           *sp == '\r' || *sp == ',')
                        sp++;
                    if (*sp == ']' || *sp == '\0') break;
                    if (*sp != '{') break;

                    delay_cal_sample_t *s = &d->samples[d->n_samples];
                    const char *slotp = strstr(sp, "\"slot\"");
                    const char *freqp = strstr(sp, "\"freq_mhz\"");
                    const char *e10p  = strstr(sp, "\"eps10_rad\"");
                    const char *e20p  = strstr(sp, "\"eps20_rad\"");
                    const char *e30p  = strstr(sp, "\"eps30_rad\"");
                    if (!slotp || !freqp || !e10p || !e20p || !e30p) break;

                    slotp = strchr(slotp, ':');
                    freqp = strchr(freqp, ':');
                    e10p  = strchr(e10p, ':');
                    e20p  = strchr(e20p, ':');
                    e30p  = strchr(e30p, ':');
                    if (!slotp || !freqp || !e10p || !e20p || !e30p) break;

                    s->slot     = (int)strtol(slotp + 1, NULL, 10);
                    s->freq_mhz = strtod(freqp + 1, NULL);
                    s->eps10    = (float)strtod(e10p + 1, NULL);
                    s->eps20    = (float)strtod(e20p + 1, NULL);
                    s->eps30    = (float)strtod(e30p + 1, NULL);
                    d->n_samples++;

                    sp = strchr(sp, '}');
                    if (!sp) break;
                    sp++;
                }
            }
        }
    }

    free(buf);
    return 0;
}

int phase_cal_dump_csv(const phase_cal_ctx_t *ctx, const char *path)
{
    if (!ctx || !path) return -1;

    pthread_mutex_lock((pthread_mutex_t*)&ctx->lock);
    int n = ctx->count;
    phase_cal_sample_t *snapshot = NULL;
    if (n > 0) {
        snapshot = (phase_cal_sample_t*)malloc(
            (size_t)n * sizeof(phase_cal_sample_t));
        if (snapshot) {
            if (ctx->count < ctx->cap) {
                memcpy(snapshot, ctx->ring,
                       (size_t)n * sizeof(phase_cal_sample_t));
            } else {
                int tail = ctx->head;
                int first_n = ctx->cap - tail;
                memcpy(snapshot, &ctx->ring[tail],
                       (size_t)first_n * sizeof(phase_cal_sample_t));
                memcpy(snapshot + first_n, ctx->ring,
                       (size_t)tail * sizeof(phase_cal_sample_t));
            }
        } else {
            n = 0;
        }
    }
    pthread_mutex_unlock((pthread_mutex_t*)&ctx->lock);

    FILE *fp = fopen(path, "wb");
    if (!fp) { free(snapshot); return -1; }
    fprintf(fp, "t_unix,phi10,phi20,phi30,qw,qx,qy,qz,intensity,rf_mhz\n");
    for (int i = 0; i < n; ++i) {
        const phase_cal_sample_t *s = &snapshot[i];
        fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.3f\n",
                s->t_unix,
                (double)s->phi10, (double)s->phi20, (double)s->phi30,
                (double)s->qw, (double)s->qx, (double)s->qy, (double)s->qz,
                (double)s->intensity, s->rf_mhz);
    }
    fclose(fp);
    free(snapshot);
    return 0;
}

// ------------------------------------------------------------
// Default file paths (mirror settings.c style)
// ------------------------------------------------------------

static int build_exe_dir(char *buf, size_t buf_sz)
{
    ssize_t nread = readlink("/proc/self/exe", buf, buf_sz - 1);
    if (nread < 0) {
        if ((size_t)snprintf(buf, buf_sz, ".") >= buf_sz) return -1;
        return 0;
    }
    buf[nread] = '\0';
    char *slash = strrchr(buf, '/');
    if (!slash) return -1;
    *slash = '\0';
    return 0;
}

int phase_cal_default_json_path(char *buf, size_t buf_sz)
{
    if (buf_sz < 64) return -1;
    if (build_exe_dir(buf, buf_sz) != 0) return -1;
    size_t plen = strlen(buf);
    const char *fname = "/phase_calibration.json";
    if (plen + strlen(fname) + 1 > buf_sz) return -1;
    snprintf(buf + plen, buf_sz - plen, "%s", fname);
    return 0;
}

int phase_cal_default_csv_path(char *buf, size_t buf_sz)
{
    if (buf_sz < 64) return -1;
    if (build_exe_dir(buf, buf_sz) != 0) return -1;
    size_t plen = strlen(buf);

    time_t t = time(NULL);
    struct tm *lt = localtime(&t);
    char stamp[32];
    if (lt)
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", lt);
    else
        snprintf(stamp, sizeof(stamp), "%ld", (long)t);

    int need = snprintf(NULL, 0, "/phase_cal_samples_%s.csv", stamp);
    if (plen + (size_t)need + 1 > buf_sz) return -1;
    snprintf(buf + plen, buf_sz - plen, "/phase_cal_samples_%s.csv", stamp);
    return 0;
}

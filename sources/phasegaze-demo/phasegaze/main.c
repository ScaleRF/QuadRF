// main.c
// CSI sweep with interactive 3D hemisphere + SDL2 control panel
//
// Worker thread: CSI → FFT → phase gradients → point_data_t buffer
// Main thread:   GLFW/OpenGL 3D view + SDL2 control window

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

#include <sys/mman.h>
#include <sys/ioctl.h>

#include <pthread.h>
#include <fftw3.h>

#define GL_GLEXT_PROTOTYPES
#include <GLFW/glfw3.h>

#include "config.h"
#include "utils.h"
#include "worker.h"
#include "signal_processing.h"
#include "sphere_render.h"
#include "camera.h"
#include "control_window.h"
#include "external_controls.h"
#include "imu_worker.h"
#include "snapshot.h"
#include "shutter_stream.h"
#include "phase_cal.h"
#include "tx_slots.h"

#include <linux/types.h>
#include "fpga_csi.h"

// Global camera for GLFW callbacks
static camera_t g_cam;

// ---- IMU mount calibration (antenna ↔ sensor frame) -----------------------
//
// q_mount is the quaternion that takes a vector expressed in antenna-frame
// coordinates and re-expresses it in sensor-frame coordinates:
//     v_sensor = q_mount · v_antenna · conj(q_mount)
//
// The full antenna → world transform is then  q_cur · q_mount.
//
// CalDown cycles four steps (press again after clear restarts at face):
//   0 = next: face down  -> z_hat = -sensor_up  (antenna +Z along gravity)
//   1 = next: bottom down -> y_hat = +sensor_up  (antenna -Y along gravity)
//   2 = next: top down    -> refine y_hat from -sensor_up (average w/ bottom)
//   3 = next: clear all, then back to 0
//
static float z_hat_sensor[3]      = { 0.0f, 0.0f, 1.0f };
static int   z_hat_valid          = 0;

static float y_hat_committed[3]   = { 0.0f, 1.0f, 0.0f };
static int   y_hat_committed_valid = 0;

// What the next CalDown press will do (see enum above).
static int   mount_cal_step       = 0;

static float q_mount_w = 1.0f, q_mount_x = 0.0f, q_mount_y = 0.0f, q_mount_z = 0.0f;
static int   q_mount_valid = 0;

// ---- Quaternion helpers ----

// Rotate vector (vx,vy,vz) by quaternion (qw,qx,qy,qz): v' = q · v · conj(q).
// Uses the Rodrigues form so we don't pay for a full quat-vec-quat triple.
static void quat_rotate_vec(float qw, float qx, float qy, float qz,
                             float vx, float vy, float vz,
                             float *ox, float *oy, float *oz)
{
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);
    *ox = vx + qw * tx + (qy * tz - qz * ty);
    *oy = vy + qw * ty + (qz * tx - qx * tz);
    *oz = vz + qw * tz + (qx * ty - qy * tx);
}

// Hamilton quaternion product: out = a * b.
static void quat_mul(float aw, float ax, float ay, float az,
                     float bw, float bx, float by, float bz,
                     float *ow, float *ox, float *oy, float *oz)
{
    *ow = aw*bw - ax*bx - ay*by - az*bz;
    *ox = aw*bx + ax*bw + ay*bz - az*by;
    *oy = aw*by - ax*bz + ay*bw + az*bx;
    *oz = aw*bz + ax*by - ay*bx + az*bw;
}

// Build q_mount from z_hat (antenna +Z in sensor) and y_hat (antenna +Y in
// sensor). y_hat is Gram-Schmidt-orthogonalized against z_hat so a sloppy
// user pose doesn't make the rotation matrix non-orthogonal. Returns 1 on
// success, 0 if the two axes are degenerate (parallel) and no valid mount
// can be derived.
static int compute_q_mount_from_axes(const float zh[3], const float yh[3],
                                     float *qw, float *qx, float *qy, float *qz)
{
    float zx = zh[0], zy = zh[1], zz = zh[2];
    float zn = sqrtf(zx*zx + zy*zy + zz*zz);
    if (zn < 1e-6f) return 0;
    zx /= zn; zy /= zn; zz /= zn;

    float yx = yh[0], yy = yh[1], yz = yh[2];
    float dot = yx*zx + yy*zy + yz*zz;
    yx -= dot * zx;
    yy -= dot * zy;
    yz -= dot * zz;
    float yn = sqrtf(yx*yx + yy*yy + yz*yz);
    if (yn < 1e-3f) return 0;  // antenna +Y too close to ±antenna +Z
    yx /= yn; yy /= yn; yz /= yn;

    // x_hat = y_hat × z_hat (right-handed: +X × +Y = +Z).
    float xx = yy*zz - yz*zy;
    float xy = yz*zx - yx*zz;
    float xz = yx*zy - yy*zx;
    float xn = sqrtf(xx*xx + xy*xy + xz*xz);
    if (xn < 1e-6f) return 0;
    xx /= xn; xy /= xn; xz /= xn;

    // R_as = [ x_hat | y_hat | z_hat ] (columns), then convert to quaternion
    // using the standard numerically-stable branched form.
    float r00 = xx, r01 = yx, r02 = zx;
    float r10 = xy, r11 = yy, r12 = zy;
    float r20 = xz, r21 = yz, r22 = zz;

    float trace = r00 + r11 + r22;
    float w, x, y, z;
    if (trace > 0.0f) {
        float S = sqrtf(trace + 1.0f) * 2.0f;
        w = 0.25f * S;
        x = (r21 - r12) / S;
        y = (r02 - r20) / S;
        z = (r10 - r01) / S;
    } else if (r00 > r11 && r00 > r22) {
        float S = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
        w = (r21 - r12) / S;
        x = 0.25f * S;
        y = (r01 + r10) / S;
        z = (r02 + r20) / S;
    } else if (r11 > r22) {
        float S = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
        w = (r02 - r20) / S;
        x = (r01 + r10) / S;
        y = 0.25f * S;
        z = (r12 + r21) / S;
    } else {
        float S = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
        w = (r10 - r01) / S;
        x = (r02 + r20) / S;
        y = (r12 + r21) / S;
        z = 0.25f * S;
    }
    float qn = sqrtf(w*w + x*x + y*y + z*z);
    if (qn < 1e-9f) return 0;
    *qw = w / qn; *qx = x / qn; *qy = y / qn; *qz = z / qn;
    return 1;
}

// Rebuild q_mount from the currently-committed axes. Pass back validity
// so callers can keep the UI mirror in sync.
static void recompute_q_mount(void)
{
    if (z_hat_valid && y_hat_committed_valid &&
        compute_q_mount_from_axes(z_hat_sensor, y_hat_committed,
                                  &q_mount_w, &q_mount_x,
                                  &q_mount_y, &q_mount_z))
    {
        q_mount_valid = 1;
    } else {
        q_mount_valid = 0;
        q_mount_w = 1.0f; q_mount_x = 0.0f; q_mount_y = 0.0f; q_mount_z = 0.0f;
    }
}

// Antenna → north-referenced world: q_aw = q_cur · q_mount.
static void get_antenna_world_orientation(imu_ctx_t *imu,
                                          float *qw, float *qx, float *qy, float *qz)
{
    float cur_qw, cur_qx, cur_qy, cur_qz;
    imu_worker_get_orientation(imu, &cur_qw, &cur_qx, &cur_qy, &cur_qz);
    if (q_mount_valid) {
        quat_mul(cur_qw, cur_qx, cur_qy, cur_qz,
                 q_mount_w, q_mount_x, q_mount_y, q_mount_z,
                 qw, qx, qy, qz);
    } else {
        *qw = cur_qw; *qx = cur_qx; *qy = cur_qy; *qz = cur_qz;
    }
}

static void fill_snap_orientation(imu_ctx_t *imu, snapshot_orientation_t *out)
{
    memset(out, 0, sizeof(*out));
    if (imu_worker_get_status(imu, NULL, 0) != IMU_STATUS_CONNECTED)
        return;
    out->imu_valid = 1;
    get_antenna_world_orientation(imu, &out->qw, &out->qx, &out->qy, &out->qz);
    imu_cal_status_t cal;
    imu_worker_get_cal_status(imu, &cal);
    out->accuracy_rad = cal.accuracy_rad;
    out->cal_accel    = cal.cal_accel;
    out->cal_gyro     = cal.cal_gyro;
    out->cal_mag      = cal.cal_mag;
}

// Copy src -> dst and rotate each point from antenna frame to absolute
// north-referenced world frame via q_aw = q_cur · q_mount.
static void imu_world_rotate_points(const point_data_t *src, point_data_t *dst,
                                      int n, imu_ctx_t *imu,
                                      double lo_start_mhz, double lo_end_mhz)
{
    if (n <= 0) return;
    memcpy(dst, src, (size_t)n * sizeof(point_data_t));

    float aw_w, aw_x, aw_y, aw_z;
    get_antenna_world_orientation(imu, &aw_w, &aw_x, &aw_y, &aw_z);

    float sf = SCALE_FACTOR_AT_MHZ((float)((lo_start_mhz + lo_end_mhz) * 0.5));
    float sf_inv = (sf > 1e-9f) ? 1.0f / sf : 1.0f;

    for (int i = 0; i < n; i++)
    {
        float u = dst[i].gx * sf_inv;
        float v = dst[i].gy * sf_inv;
        float r2 = u*u + v*v;
        if (r2 >= 1.0f) continue;
        float w = sqrtf(1.0f - r2);

        float ou, ov, ow;
        quat_rotate_vec(aw_w, aw_x, aw_y, aw_z,
                        u, v, w, &ou, &ov, &ow);

        dst[i].gx = ou * sf;
        dst[i].gy = ov * sf;
        if (ow < 0.0f)
            dst[i].intensity = -dst[i].intensity;
    }
}

// World +Z (up) expressed in sensor coords from the current fusion quaternion.
static int read_sensor_up(imu_ctx_t *imu, float *ux, float *uy, float *uz)
{
    float cw, cx, cy, cz;
    imu_worker_get_orientation(imu, &cw, &cx, &cy, &cz);
    quat_rotate_vec(cw, -cx, -cy, -cz, 0.0f, 0.0f, 1.0f, ux, uy, uz);
    float n = sqrtf((*ux)*(*ux) + (*uy)*(*uy) + (*uz)*(*uz));
    if (n < 1e-3f) return 0;
    *ux /= n; *uy /= n; *uz /= n;
    return 1;
}

static void mount_cal_step_from_loaded(void)
{
    if (z_hat_valid && y_hat_committed_valid)
        mount_cal_step = 3;
    else if (z_hat_valid)
        mount_cal_step = 1;
    else
        mount_cal_step = 0;
}

// Push current mount cal state out to the control window so the UI label
// updates and so the next persist saves the right values.
static void sync_mount_cal_to_ui(control_window_t *ctrl)
{
    ctrl->mount_z_valid = z_hat_valid ? 1 : 0;
    ctrl->mount_z_sx    = z_hat_sensor[0];
    ctrl->mount_z_sy    = z_hat_sensor[1];
    ctrl->mount_z_sz    = z_hat_sensor[2];
    ctrl->mount_y_valid = y_hat_committed_valid ? 1 : 0;
    ctrl->mount_y_samples = 0;
    ctrl->mount_y_sx    = y_hat_committed[0];
    ctrl->mount_y_sy    = y_hat_committed[1];
    ctrl->mount_y_sz    = y_hat_committed[2];
    ctrl->mount_cal_step = mount_cal_step;
}

static int filter_pts_by_threshold(point_data_t *pts, int n, float threshold)
{
    int out = 0;
    for (int i = 0; i < n; ++i) {
        if (fabsf(pts[i].intensity) >= threshold)
            pts[out++] = pts[i];
    }
    return out;
}

static void glfw_error_cb(int err, const char *desc)
{
    fprintf(stderr, "GLFW error %d: %s\n", err, desc);
}

static void glfw_key_cb(GLFWwindow *win, int key, int scancode,
                        int action, int mods)
{
    (void)scancode; (void)mods;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(win, GLFW_TRUE);
}

static void glfw_mouse_button_cb(GLFWwindow *win, int button,
                                 int action, int mods)
{
    (void)mods;
    double mx, my;
    glfwGetCursorPos(win, &mx, &my);
    camera_on_mouse_button(&g_cam, button, action, mx, my);
}

static void glfw_cursor_pos_cb(GLFWwindow *win, double mx, double my)
{
    (void)win;
    camera_on_cursor_move(&g_cam, mx, my);
}

static void glfw_scroll_cb(GLFWwindow *win, double xoff, double yoff)
{
    (void)win; (void)xoff;
    camera_on_scroll(&g_cam, yoff);
}

int main(int argc, char **argv)
{
    int headless = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--headless") == 0) {
            headless = 1;
        }
    }

    // ---- CSI device ----
    int fd = open(DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open");

    ioctl(fd, CSI_IOC_JTAG_SETUP);

    struct csi_ring_info ri;
    if (ioctl(fd, CSI_IOC_GET_RING_INFO, &ri) < 0)
        die("CSI_IOC_GET_RING_INFO");
    if (!ri.ring_size) { fprintf(stderr, "ring_size=0\n"); return 1; }

    long page = sysconf(_SC_PAGESIZE);
    size_t map_len = (size_t)((ri.ring_size + page - 1) &
                              ~((uint64_t)page - 1));
    void *ring = mmap(NULL, map_len, PROT_READ, MAP_SHARED, fd, 0);
    if (ring == MAP_FAILED) die("mmap");

    // ---- SDL2 control window (init before GLFW so it gets its own X conn) ----
    control_window_t ctrl;
    if (!headless) {
        if (control_init(&ctrl) != 0) die("control_init");
    } else {
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.lo_start_mhz = LO_START_MHZ;
        ctrl.lo_end_mhz = LO_END_MHZ;
        ctrl.rf_gain_db  = 30;
        ctrl.dig_bw_k    = 15;  // 240/15 = 16 MHz
    }

    // Pin the radio into a known-good state for CSI capture: AGC off (implicit
    // via manual gain), TX off, 4-antenna interleaved. One-shot — if the web
    // UI later changes any of these, csi_sweep does not fight back.
    external_controls_lockdown(ctrl.rf_gain_db, ctrl.dig_bw_k);

    // ---- IMU worker thread ----
    imu_ctx_t imu_ctx;
    imu_worker_init(&imu_ctx);
    if (imu_worker_start(&imu_ctx) != 0)
        fprintf(stderr, "Warning: IMU worker thread failed to start\n");

    // ---- Phase-offset calibration context ----
    // 8192 samples = ~2 minutes of sweeps at 60 fps, plenty for any plausible
    // calibration session.
    phase_cal_ctx_t phase_cal;
    if (phase_cal_init(&phase_cal, 8192) != 0)
        die("phase_cal_init");

    // Auto-load any previously-saved calibration so the user doesn't have to
    // click Load on every startup.
    {
        char cal_path[640];
        if (phase_cal_default_json_path(cal_path, sizeof(cal_path)) == 0) {
            phase_cal_result_t loaded;
            if (phase_cal_load_json(&loaded, cal_path) == 0) {
                phase_cal_apply_result(&phase_cal, &loaded);
                fprintf(stdout,
                    "[phase_cal] loaded from %s: eps=(%.3f, %.3f, %.3f) rad, "
                    "delay_fitted=%d, rms=%.3f rad, n=%d\n",
                    cal_path,
                    (double)loaded.eps10_rad,
                    (double)loaded.eps20_rad,
                    (double)loaded.eps30_rad,
                    loaded.delay_cal.fitted,
                    (double)loaded.rms_residual_rad,
                    loaded.n_samples);
            }
        }
    }

    // ---- GLFW / OpenGL ----
    GLFWwindow *win = NULL;
    sphere_state_t sphere;
    if (!headless) {
        glfwSetErrorCallback(glfw_error_cb);
        if (!glfwInit()) die("glfwInit");

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_SAMPLES, 0);

        win = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT,
                                            "CSI Hemisphere", NULL, NULL);
        if (!win) die("glfwCreateWindow");

        glfwMakeContextCurrent(win);
        glfwSwapInterval(1);

        glfwSetKeyCallback(win, glfw_key_cb);
        glfwSetMouseButtonCallback(win, glfw_mouse_button_cb);
        glfwSetCursorPosCallback(win, glfw_cursor_pos_cb);
        glfwSetScrollCallback(win, glfw_scroll_cb);

        fprintf(stdout, "OpenGL: %s\n", glGetString(GL_VERSION));
        fprintf(stdout, "Renderer: %s\n", glGetString(GL_RENDERER));

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        // ---- Camera ----
        camera_init(&g_cam);
        {
            int w, h;
            glfwGetFramebufferSize(win, &w, &h);
            float aspect = (h > 0) ? (float)w / (float)h : 1.0f;
            camera_set_projection(&g_cam, aspect, 45.0f, 0.1f, 100.0f);
        }
        camera_update(&g_cam);

        // ---- Sphere renderer ----
        sphere_init(&sphere);
    }

    // ---- FFT plans & worker context ----
    fftwf_complex *fft_in = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
    if (!fft_in) die("fftwf_malloc in");

    fftwf_complex *fft_out[CHANNELS_USED];
    fftwf_plan plan[CHANNELS_USED];
    for (int u = 0; u < CHANNELS_USED; ++u)
    {
        fft_out[u] = fftwf_malloc(sizeof(fftwf_complex) * FFT_SIZE);
        if (!fft_out[u]) die("fftwf_malloc out");
        plan[u] = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out[u],
                                     FFTW_FORWARD, FFTW_MEASURE);
        if (!plan[u]) die("fftw plan");
    }

    point_data_t *pts_a = (point_data_t*)calloc(MAX_POINTS_PER_FRAME,
                                                  sizeof(point_data_t));
    point_data_t *pts_b = (point_data_t*)calloc(MAX_POINTS_PER_FRAME,
                                                  sizeof(point_data_t));
    if (!pts_a || !pts_b) die("alloc point buffers");

    ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.fd        = fd;
    ctx.ring      = ring;
    ctx.ring_size = ri.ring_size;
    ctx.fft_in    = fft_in;
    for (int u = 0; u < CHANNELS_USED; ++u)
    {
        ctx.fft_out[u] = fft_out[u];
        ctx.plan[u]    = plan[u];
    }
    ctx.points_front = pts_a;
    ctx.points_back  = pts_b;
    ctx.npoints_front = 0;
    ctx.npoints_back  = 0;
    pthread_mutex_init(&ctx.mtx, NULL);

    ctx.lo_start_mhz = ctrl.lo_start_mhz;
    ctx.lo_end_mhz   = ctrl.lo_end_mhz;
    ctx.lo_start_new  = ctrl.lo_start_mhz;
    ctx.lo_end_new    = ctrl.lo_end_mhz;
    ctx.range_changed = 0;
    pthread_mutex_init(&ctx.range_mtx, NULL);

    if (spur_mask_init(&ctx.mask) != 0)
        die("spur_mask_init");

    ctx.phase_cal      = &phase_cal;

    pthread_t worker_th;
    if (pthread_create(&worker_th, NULL, worker, &ctx) != 0)
        die("pthread_create");

    // Local copy of points for the render side
    point_data_t *local_pts = (point_data_t*)malloc(
        (size_t)MAX_POINTS_PER_FRAME * sizeof(point_data_t));
    point_data_t *imu_pts = (point_data_t*)malloc(
        (size_t)MAX_POINTS_PER_FRAME * sizeof(point_data_t));
    if (!local_pts || !imu_pts) die("alloc local_pts");
    int local_npts = 0;
    uint64_t last_frame_idx = 0;
    int prev_camera_mode = 0;

    // Load IMU mount calibration from settings (populated in control_init).
    // If both halves are valid, q_mount is in effect from the very first
    // rendered frame -- the user shouldn't have to recalibrate at every
    // launch.
    if (!headless && ctrl.mount_z_valid) {
        z_hat_sensor[0] = ctrl.mount_z_sx;
        z_hat_sensor[1] = ctrl.mount_z_sy;
        z_hat_sensor[2] = ctrl.mount_z_sz;
        float n = sqrtf(z_hat_sensor[0]*z_hat_sensor[0] +
                        z_hat_sensor[1]*z_hat_sensor[1] +
                        z_hat_sensor[2]*z_hat_sensor[2]);
        if (n > 1e-6f) {
            z_hat_sensor[0] /= n;
            z_hat_sensor[1] /= n;
            z_hat_sensor[2] /= n;
            z_hat_valid = 1;
        }
    }
    if (!headless && ctrl.mount_y_valid) {
        y_hat_committed[0] = ctrl.mount_y_sx;
        y_hat_committed[1] = ctrl.mount_y_sy;
        y_hat_committed[2] = ctrl.mount_y_sz;
        float n = sqrtf(y_hat_committed[0]*y_hat_committed[0] +
                        y_hat_committed[1]*y_hat_committed[1] +
                        y_hat_committed[2]*y_hat_committed[2]);
        if (n > 1e-6f) {
            y_hat_committed[0] /= n;
            y_hat_committed[1] /= n;
            y_hat_committed[2] /= n;
            y_hat_committed_valid = 1;
        }
    }
    mount_cal_step_from_loaded();
    recompute_q_mount();
    if (!headless) {
        sync_mount_cal_to_ui(&ctrl);
        if (q_mount_valid)
            fprintf(stdout,
                "[mount] loaded calibration: z=(%.3f, %.3f, %.3f) "
                "y=(%.3f, %.3f, %.3f) step=%d\n",
                (double)z_hat_sensor[0], (double)z_hat_sensor[1],
                (double)z_hat_sensor[2],
                (double)y_hat_committed[0], (double)y_hat_committed[1],
                (double)y_hat_committed[2], mount_cal_step);
    }

    // Phase-cal state tracking: remember what viewfinder_mode was before cal
    // started so we can restore it when the user leaves cal mode. -1 = no
    // saved value yet.
    int prev_phase_cal_mode = 0;
    int saved_viewfinder_mode = -1;

    // Reciprocal lattice basis vectors for Voronoi cell folding
    const float r1x = 4.0f * (float)M_PI / sqrtf(3.0f);
    const float r1y = 0.0f;
    const float r2x = 2.0f * (float)M_PI / sqrtf(3.0f);
    const float r2y = 2.0f * (float)M_PI;

    // ---- Main loop ----
    if (headless) {
        printf("Running in headless mode.\n");
    }

    while ((headless || !glfwWindowShouldClose(win)) && !ctrl.quit_requested)
    {
        if (!headless) {
            glfwPollEvents();
            control_process_events(&ctrl);
        }

        // Apply frequency changes from the control window
        if (!headless && ctrl.freq_changed)
        {
            pthread_mutex_lock(&ctx.range_mtx);
            ctx.lo_start_new  = ctrl.lo_start_mhz;
            ctx.lo_end_new    = ctrl.lo_end_mhz;
            ctx.range_changed = 1;
            pthread_mutex_unlock(&ctx.range_mtx);
            ctrl.freq_changed = 0;
        }

        // If camera mode is being turned off while shutter is rolling, save
        // and stop the shutter first so the user doesn't lose the recording.
        if (!headless && prev_camera_mode && !ctrl.camera_mode && ctrl.shutter_active)
        {
            snapshot_orientation_t snap_orient = {0};
            fill_snap_orientation(&imu_ctx, &snap_orient);
            // Auto-save: no label dialog, just finalize the streamed file.
            uint32_t final_count = 0;
            int res = shutter_stream_end(&snap_orient, &final_count);
            if (res == -2) {
                ctrl.snapshot_error_time = SDL_GetTicks();
            } else if (res == 0) {
                ctrl.snapshot_flash_time = SDL_GetTicks();
                ctrl.snapshot_count++;
            }
            sphere_shutter_end(&sphere);
            sphere.shutter_total_points = 0;
            sphere.shutter_full = 0;
            ctrl.shutter_active = 0;
        }

        // Camera mode transitions
        if (!headless && ctrl.camera_mode != prev_camera_mode)
        {
            ctx.camera_mode = ctrl.camera_mode;
            sphere.history_count = 0;
            prev_camera_mode = ctrl.camera_mode;
        }

        // Shutter toggle (start / stop+save). Only operative while camera mode
        // is on; the UI already gates the button, this is a belt-and-suspenders
        // check on the main side.
        if (!headless && ctrl.shutter_toggle_requested)
        {
            ctrl.shutter_toggle_requested = 0;
            if (ctrl.camera_mode) {
                if (!ctrl.shutter_active) {
                    char saved_path[640] = {0};
                    int rc = shutter_stream_begin(ctrl.lo_start_mhz,
                                                   ctrl.lo_end_mhz,
                                                   ctrl.camera_mode,
                                                   saved_path,
                                                   (int)sizeof(saved_path));
                    if (rc == 0) {
                        sphere_shutter_begin(&sphere);
                        ctrl.shutter_active = 1;
                        // Remember the final path so the label dialog at stop
                        // can rename the right file.
                        snprintf(ctrl.label_pending_path,
                                 sizeof(ctrl.label_pending_path),
                                 "%s", saved_path);
                    } else {
                        ctrl.snapshot_error_time = SDL_GetTicks();
                    }
                } else {
                    snapshot_orientation_t snap_orient = {0};
                    fill_snap_orientation(&imu_ctx, &snap_orient);
                    uint32_t final_count = 0;
                    int res = shutter_stream_end(&snap_orient, &final_count);
                    if (res == -2) {
                        ctrl.snapshot_error_time = SDL_GetTicks();
                    } else if (res == 0) {
                        ctrl.snapshot_flash_time = SDL_GetTicks();
                        ctrl.snapshot_count++;
                        // label_pending_path was set at begin; show the
                        // rename dialog now.
                        ctrl.label_text[0]          = '\0';
                        ctrl.label_dialog_confirmed = 0;
                        ctrl.label_dialog_active    = 1;
                    } else {
                        ctrl.snapshot_error_time = SDL_GetTicks();
                    }
                    sphere_shutter_end(&sphere);
                    sphere.shutter_total_points = 0;
                    sphere.shutter_full = 0;
                    ctrl.shutter_active = 0;
                }
            }
        }

        // Calibrate mode: pass UI toggle to worker
        if (!headless)
            ctx.calibrate_mode = ctrl.calibrate_mode;

        // Phase calibration mode: enter/exit transitions. When entering, force
        // viewfinder display and restore the previous mode on exit.
        if (!headless && ctrl.phase_cal_mode != prev_phase_cal_mode)
        {
            if (ctrl.phase_cal_mode) {
                saved_viewfinder_mode = ctrl.viewfinder_mode;
                ctrl.viewfinder_mode  = 1;
                ctrl.phase_cal_saved_lo_start = ctrl.lo_start_mhz;
                ctrl.phase_cal_saved_lo_end   = ctrl.lo_end_mhz;
                ctrl.boresight_state    = BS_CLEARED;
                ctrl.boresight_slot_idx = 0;
                phase_cal_set_capture_active(&phase_cal, 0);
            } else {
                if (saved_viewfinder_mode >= 0)
                    ctrl.viewfinder_mode = saved_viewfinder_mode;
                saved_viewfinder_mode = -1;
                ctrl.boresight_state    = BS_CLEARED;
                ctrl.boresight_slot_idx = 0;
                phase_cal_set_capture_active(&phase_cal, 0);
                control_set_freq_range(&ctrl,
                    ctrl.phase_cal_saved_lo_start,
                    ctrl.phase_cal_saved_lo_end);
            }
            prev_phase_cal_mode = ctrl.phase_cal_mode;
        }
        // Keep viewfinder pinned while cal is active.
        if (!headless && ctrl.phase_cal_mode)
            ctrl.viewfinder_mode = 1;

        // Snapshot request — grab absolute orientation at capture time
        if (!headless && ctrl.snapshot_requested) {
            ctrl.snapshot_requested = 0;

            snapshot_orientation_t snap_orient = {0};
            fill_snap_orientation(&imu_ctx, &snap_orient);

            char saved_path[640] = {0};
            int res = snapshot_save(sphere.history, sphere.history_count,
                          ctrl.lo_start_mhz, ctrl.lo_end_mhz,
                          ctrl.camera_mode, &snap_orient,
                          saved_path, (int)sizeof(saved_path));
            if (res == -2) {
                ctrl.snapshot_error_time = SDL_GetTicks();
            } else if (res == 0) {
                // Show label dialog so the user can name the file.
                snprintf(ctrl.label_pending_path,
                         sizeof(ctrl.label_pending_path), "%s", saved_path);
                ctrl.label_text[0]          = '\0';
                ctrl.label_dialog_confirmed = 0;
                ctrl.label_dialog_active    = 1;
            }
        }

        // Label dialog confirmed — rename the file if user entered a name.
        if (!headless && ctrl.label_dialog_confirmed != 0)
        {
            if (ctrl.label_dialog_confirmed == 1 && ctrl.label_text[0])
                snapshot_apply_label(ctrl.label_pending_path, ctrl.label_text);
            ctrl.label_dialog_active    = 0;
            ctrl.label_dialog_confirmed = 0;
            ctrl.label_pending_path[0]  = '\0';
        }

        // Boresight button (phase-cal mode only): delay-matching capture FSM.
        if (!headless && ctrl.center_requested)
        {
            ctrl.center_requested = 0;

            if (!ctrl.phase_cal_mode)
                continue;

            switch (ctrl.boresight_state) {
            case BS_CLEARED:
                ctrl.boresight_state = BS_CAL_ARMED;
                break;

            case BS_CAL_ARMED:
                phase_cal_delay_begin_capture(&phase_cal);
                ctrl.boresight_state    = BS_CAPTURE;
                ctrl.boresight_slot_idx = 0;
                phase_cal_set_capture_active(&phase_cal, 1);
                {
                    int slot_id = tx_capture_order[0];
                    const tx_slot_info_t *sl = tx_slot_by_id(slot_id);
                    if (sl) {
                        control_set_freq_range(&ctrl,
                            sl->center_mhz - TX_FREQ_MARGIN_MHZ,
                            sl->center_mhz + TX_FREQ_MARGIN_MHZ);
                    }
                }
                break;

            case BS_CAPTURE: {
                float cgx = 0.0f, cgy = 0.0f;
                int cvalid = 0;
                phase_cal_get_live_centroid(&phase_cal, &cgx, &cgy, &cvalid);
                if (!cvalid) {
                    fprintf(stderr,
                        "[phase_cal] no centroid — aim at the beacon and wait "
                        "for points before recording\n");
                    break;
                }

                float rot_rad = CANVAS_ROTATE_DEG * (float)M_PI / 180.0f;
                float cs = cosf(rot_rad), sn = sinf(rot_rad);
                float gx_ant =  cgx * cs + cgy * sn;
                float gy_ant = -cgx * sn + cgy * cs;

                static const float SQ32 = 0.8660254037844386f;
                float d1 = -SQ32 * gx_ant + 0.5f * gy_ant;
                float d2 =                          gy_ant;
                float d3 =  SQ32 * gx_ant + 0.5f * gy_ant;

                int cap_idx = ctrl.boresight_slot_idx;
                int slot_id = tx_capture_order[cap_idx];
                const tx_slot_info_t *sl = tx_slot_by_id(slot_id);
                if (!sl)
                    break;

                if (phase_cal_delay_push_sample(&phase_cal, slot_id,
                        sl->center_mhz, d1, d2, d3) != 0) {
                    fprintf(stderr,
                        "[phase_cal] sample buffer full — click Clear Cal "
                        "and start again\n");
                    break;
                }

                fprintf(stdout,
                    "[phase_cal] slot %d @ %.0f MHz: eps=(%.3f, %.3f, %.3f) rad\n",
                    slot_id, sl->center_mhz,
                    (double)d1, (double)d2, (double)d3);

                if (cap_idx + 1 < TX_SLOT_COUNT) {
                    ctrl.boresight_slot_idx = cap_idx + 1;
                    int next_id = tx_capture_order[cap_idx + 1];
                    const tx_slot_info_t *ns = tx_slot_by_id(next_id);
                    if (ns) {
                        control_set_freq_range(&ctrl,
                            ns->center_mhz - TX_FREQ_MARGIN_MHZ,
                            ns->center_mhz + TX_FREQ_MARGIN_MHZ);
                    }
                } else {
                    if (phase_cal_delay_fit(&phase_cal) == 0) {
                        phase_cal_result_t saved = {0};
                        saved.version = 2;
                        saved.n_samples = TX_SLOT_COUNT;
                        pthread_mutex_lock(&phase_cal.lock);
                        saved.delay_cal = phase_cal.delay_cal;
                        pthread_mutex_unlock(&phase_cal.lock);

                        char json_path[640];
                        if (phase_cal_default_json_path(json_path,
                                sizeof(json_path)) == 0)
                            phase_cal_save_json(&saved, json_path);

                        fprintf(stdout,
                            "[phase_cal] delay fit: "
                            "m10=%.6g b10=%.3f m20=%.6g b20=%.3f "
                            "m30=%.6g b30=%.3f\n",
                            (double)saved.delay_cal.m10,
                            (double)saved.delay_cal.b10,
                            (double)saved.delay_cal.m20,
                            (double)saved.delay_cal.b20,
                            (double)saved.delay_cal.m30,
                            (double)saved.delay_cal.b30);
                    } else {
                        fprintf(stderr,
                            "[phase_cal] delay fit failed\n");
                    }
                    phase_cal_set_capture_active(&phase_cal, 0);
                    ctrl.boresight_state    = BS_COMPLETE;
                    ctrl.boresight_slot_idx = 0;
                    sphere.history_count = 0;
                }
                break;
            }

            case BS_COMPLETE:
                phase_cal_delay_reset(&phase_cal);
                phase_cal_set_capture_active(&phase_cal, 0);
                ctrl.boresight_state    = BS_CLEARED;
                ctrl.boresight_slot_idx = 0;
                sphere.history_count = 0;
                fprintf(stdout, "[phase_cal] delay cal cleared\n");
                break;
            }
        }

        // CalDown cycles face / bottom / top / clear (see mount_cal_step).
        if (!headless && ctrl.cal_down_requested)
        {
            ctrl.cal_down_requested = 0;
            if (ctrl.camera_mode &&
                imu_worker_get_status(&imu_ctx, NULL, 0) == IMU_STATUS_CONNECTED)
            {
                int changed = 0;
                float ux, uy, uz;

                if (mount_cal_step == 3) {
                    z_hat_valid = 0;
                    y_hat_committed_valid = 0;
                    mount_cal_step = 0;
                    changed = 1;
                    fprintf(stdout, "[mount] CalDown: cleared mount calibration\n");
                } else if (read_sensor_up(&imu_ctx, &ux, &uy, &uz)) {
                    if (mount_cal_step == 0) {
                        z_hat_sensor[0] = -ux;
                        z_hat_sensor[1] = -uy;
                        z_hat_sensor[2] = -uz;
                        z_hat_valid = 1;
                        y_hat_committed_valid = 0;
                        mount_cal_step = 1;
                        changed = 1;
                        fprintf(stdout,
                            "[mount] CalDown face: z_hat=(%.3f, %.3f, %.3f)\n",
                            (double)z_hat_sensor[0], (double)z_hat_sensor[1],
                            (double)z_hat_sensor[2]);
                    } else if (mount_cal_step == 1) {
                        y_hat_committed[0] = ux;
                        y_hat_committed[1] = uy;
                        y_hat_committed[2] = uz;
                        y_hat_committed_valid = 1;
                        mount_cal_step = 2;
                        changed = 1;
                        fprintf(stdout,
                            "[mount] CalDown bottom: y_hat=(%.3f, %.3f, %.3f)\n",
                            (double)y_hat_committed[0], (double)y_hat_committed[1],
                            (double)y_hat_committed[2]);
                    } else if (mount_cal_step == 2) {
                        // Bottom gave +Y = +sensor_up; top gives +Y = -sensor_up.
                        float dx = y_hat_committed[0] + ux;
                        float dy = y_hat_committed[1] + uy;
                        float dz = y_hat_committed[2] + uz;
                        y_hat_committed[0] = 0.5f * (y_hat_committed[0] - ux);
                        y_hat_committed[1] = 0.5f * (y_hat_committed[1] - uy);
                        y_hat_committed[2] = 0.5f * (y_hat_committed[2] - uz);
                        float n = sqrtf(y_hat_committed[0]*y_hat_committed[0] +
                                        y_hat_committed[1]*y_hat_committed[1] +
                                        y_hat_committed[2]*y_hat_committed[2]);
                        if (n > 1e-3f) {
                            y_hat_committed[0] /= n;
                            y_hat_committed[1] /= n;
                            y_hat_committed[2] /= n;
                            y_hat_committed_valid = 1;
                            mount_cal_step = 3;
                            changed = 1;
                            fprintf(stdout,
                                "[mount] CalDown top: y_hat=(%.3f, %.3f, %.3f) "
                                "bottom/top delta=%.4f%s\n",
                                (double)y_hat_committed[0],
                                (double)y_hat_committed[1],
                                (double)y_hat_committed[2],
                                (double)sqrtf(dx*dx + dy*dy + dz*dz),
                                q_mount_valid ? " (mount cal complete)" : "");
                        }
                    }
                }

                if (changed) {
                    recompute_q_mount();
                    sync_mount_cal_to_ui(&ctrl);
                    control_persist_settings(&ctrl);
                    sphere.history_count = 0;
                }
            }
        }

        // Handle window resize
        if (!headless)
        {
            int w, h;
            glfwGetFramebufferSize(win, &w, &h);
            if (w > 0 && h > 0)
            {
                glViewport(0, 0, w, h);
                camera_set_projection(&g_cam, (float)w / (float)h,
                                      45.0f, 0.1f, 100.0f);
            }
            camera_update(&g_cam);
        }

        // Grab new points from worker
        int got_new = 0;
        pthread_mutex_lock(&ctx.mtx);
        if (ctx.telem.frame_idx != last_frame_idx)
        {
            local_npts = ctx.npoints_front;
            if (local_npts > 0)
                memcpy(local_pts, ctx.points_front,
                       (size_t)local_npts * sizeof(point_data_t));
            last_frame_idx = ctx.telem.frame_idx;
            got_new = 1;
        }
        telemetry_t telem = ctx.telem;
        pthread_mutex_unlock(&ctx.mtx);

        if (!got_new && headless) {
            usleep(1000); // Sleep 1ms to avoid 100% CPU in headless mode
            continue;
        }

        // Fold points to primary Voronoi cell when mirroring is enabled
        if (got_new && ctrl.show_mirrors && local_npts > 0)
        {
            const int R = MIRROR_SEARCH_RANGE;

            for (int i = 0; i < local_npts; ++i)
            {
                float gx = local_pts[i].gx;
                float gy = local_pts[i].gy;

                // Find nearest reciprocal lattice point
                float best_dist = 1e20f;
                int best_n1 = 0, best_n2 = 0;
                for (int n1 = -R; n1 <= R; ++n1)
                {
                    for (int n2 = -R; n2 <= R; ++n2)
                    {
                        float lx = (float)n1 * r1x + (float)n2 * r2x;
                        float ly = (float)n1 * r1y + (float)n2 * r2y;
                        float dx = gx - lx;
                        float dy = gy - ly;
                        float dist = dx * dx + dy * dy;
                        if (dist < best_dist)
                        {
                            best_dist = dist;
                            best_n1 = n1;
                            best_n2 = n2;
                        }
                    }
                }

                // Fold into primary Voronoi cell
                float fold_x = (float)best_n1 * r1x + (float)best_n2 * r2x;
                float fold_y = (float)best_n1 * r1y + (float)best_n2 * r2y;
                local_pts[i].gx = gx - fold_x;
                local_pts[i].gy = gy - fold_y;
            }
        }

        // In camera mode, build an IMU world-frame copy for saving (and for
        // live display when viewfinder stability is on). local_pts stays in
        // antenna frame so stability-off viewfinder can show raw directions.
        int imu_pts_valid = 0;
        if (!headless && got_new && ctrl.camera_mode && local_npts > 0)
        {
            imu_world_rotate_points(local_pts, imu_pts, local_npts,
                                    &imu_ctx,
                                    ctrl.lo_start_mhz, ctrl.lo_end_mhz);
            imu_pts_valid = 1;
        }

        const point_data_t *display_pts = local_pts;
        point_data_t *save_pts = local_pts;
        if (imu_pts_valid) {
            save_pts = imu_pts;
            if (!ctrl.viewfinder_mode || ctrl.stability_mode)
                display_pts = imu_pts;
        }

        if (!headless)
        {
            // Update point history (decay + new points)
            // Mirror copies are rendered on the GPU via lattice-offset draw calls
            //
            // Viewfinder mode always uses the decay history buffer for the
            // live display, even while shutter is recording. The bin grid
            // is bypassed so the viewfinder shows a smooth fading trail
            // instead of accumulating splats; disk streaming continues
            // unchanged via sphere_shutter_stream_only().
            if (ctrl.viewfinder_mode)
            {
                if (ctrl.shutter_active && got_new && local_npts > 0) {
                    int npts = local_npts;
                    if (ctrl.shutter_threshold_filter)
                        npts = filter_pts_by_threshold(save_pts, npts,
                                                       ctrl.intensity_threshold);
                    if (npts > 0)
                        sphere_shutter_stream_only(&sphere, save_pts, npts);
                }
                sphere_update_points(&sphere,
                                     (got_new ? display_pts : NULL),
                                     (got_new ? local_npts : 0),
                                     ctrl.decay_factor, ctrl.intensity_threshold);
            }
            else if (ctrl.shutter_active)
            {
                // Long-exposure path: append raw points to the accumulator
                // and only rebuild the binned display occasionally.
                if (got_new && local_npts > 0) {
                    int npts = local_npts;
                    if (ctrl.shutter_threshold_filter)
                        npts = filter_pts_by_threshold(save_pts, npts,
                                                       ctrl.intensity_threshold);
                    if (npts > 0)
                        sphere_shutter_add(&sphere, save_pts, npts);
                }

                sphere.shutter_dirty_frames++;
                if (sphere.shutter_dirty_frames >= SHUTTER_DISPLAY_REBUILD_EVERY_N_FRAMES)
                {
                    sphere_shutter_rebuild_display(&sphere);
                    sphere.shutter_dirty_frames = 0;
                }
            }
            else if (got_new)
                sphere_update_points(&sphere, display_pts, local_npts,
                                     ctrl.decay_factor, ctrl.intensity_threshold);
            else
                sphere_update_points(&sphere, NULL, 0,
                                     ctrl.decay_factor, ctrl.intensity_threshold);

            // Build view options + MVP. Viewfinder mode bypasses the orbit
            // camera entirely with a fixed pinhole-from-origin transform.
            sphere_view_opts_t vopts = {0};
            float vmvp[16];
            const float *use_mvp = g_cam.mvp;
            vopts.show_hex_reticle = 1;

            // Live centroid overlay -- only meaningful in viewfinder mode
            // since the (gx, gy) coordinate is in the antenna frame and the
            // viewfinder is the only display that maps it directly to screen.
            if (ctrl.phase_cal_mode && ctrl.viewfinder_mode) {
                vopts.show_hex_reticle = 0;
                float cgx = 0.0f, cgy = 0.0f;
                int cvalid = 0;
                phase_cal_get_live_centroid(&phase_cal, &cgx, &cgy, &cvalid);
                if (cvalid) {
                    vopts.show_centroid = 1;
                    vopts.centroid_gx   = cgx;
                    vopts.centroid_gy   = cgy;
                }
            }

            if (ctrl.viewfinder_mode) {
                int fbw = WIN_WIDTH, fbh = WIN_HEIGHT;
                glfwGetFramebufferSize(win, &fbw, &fbh);
                float aspect = (fbh > 0) ? (float)fbw / (float)fbh : 1.0f;

                vopts.viewfinder = 1;
                vopts.flat       = ctrl.camera_mode ? 1 : 0;

                if (vopts.flat) {
                    // Flat ortho: fit the hex's vertex-to-vertex extent
                    // (R_hex = 2*pi/sqrt(3)) onto the smaller framebuffer
                    // dimension so the hex doesn't get squashed by aspect.
                    float Rhex = 2.0f * (float)M_PI / sqrtf(3.0f);
                    float s    = 1.0f / Rhex;
                    if (aspect >= 1.0f) {
                        vopts.ortho_sx = s / aspect;
                        vopts.ortho_sy = s;
                    } else {
                        vopts.ortho_sx = s;
                        vopts.ortho_sy = s * aspect;
                    }
                    // mvp is unused when uFlat=1 in the shaders, but pass
                    // identity so any downstream code that touches it sees
                    // something sane.
                    memset(vmvp, 0, sizeof(vmvp));
                    vmvp[0] = vmvp[5] = vmvp[10] = vmvp[15] = 1.0f;
                } else {
                    // Wide-FOV pinhole at origin, looking down +Z (the
                    // boresight = center of the hex). World "up" = +Y.
                    float proj[16], view_m[16];
                    float eye[3]    = {0.0f, 0.0f, 0.0f};
                    float center[3] = {0.0f, 0.0f, 1.0f};
                    float up[3]     = {0.0f, 1.0f, 0.0f};
                    mat4_perspective(proj, 110.0f * (float)M_PI / 180.0f,
                                     aspect, 0.01f, 10.0f);
                    mat4_lookat(view_m, eye, center, up);
                    mat4_mul(vmvp, proj, view_m);
                    vopts.ortho_sx = 1.0f;
                    vopts.ortho_sy = 1.0f;
                }
                use_mvp = vmvp;
            }

            // GL render
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            sphere_render(&sphere, use_mvp,
                          ctrl.point_size, ctrl.point_gain,
                          ctrl.show_bottom, ctrl.show_mirrors,
                          (float)ctrl.lo_start_mhz, (float)ctrl.lo_end_mhz,
                          &vopts);

            glfwSwapBuffers(win);
        }

        // Update GLFW title
        if (!headless)
        {
            char title[256];
            const char *mask_str = telem.mask_active ? "  mask=ON" : "";
            snprintf(title, sizeof(title),
                     "CSI Hemisphere | sweep fps=%.1f  pts=%u  hist=%d%s",
                     telem.fps, telem.points, sphere.history_count, mask_str);
            glfwSetWindowTitle(win, title);
        }

        // Push current IMU status + calibration into the control window.
        if (!headless) {
            ctrl.imu_status = imu_worker_get_status(
                &imu_ctx, ctrl.imu_device_info, sizeof(ctrl.imu_device_info));
            imu_cal_status_t cal;
            imu_worker_get_cal_status(&imu_ctx, &cal);
            ctrl.cal_accel = cal.cal_accel;
            ctrl.cal_gyro  = cal.cal_gyro;
            ctrl.cal_mag   = cal.cal_mag;
            ctrl.cal_rv    = cal.cal_rv;

            // Compute live heading/pitch/roll for the control window display
            if (ctrl.imu_status == IMU_STATUS_CONNECTED) {
                float dqw, dqx, dqy, dqz;
                get_antenna_world_orientation(&imu_ctx, &dqw, &dqx, &dqy, &dqz);

                float sinr = 2.0f * (dqw * dqx + dqy * dqz);
                float cosr = 1.0f - 2.0f * (dqx * dqx + dqy * dqy);
                ctrl.roll_deg = atan2f(sinr, cosr) * (180.0f / (float)M_PI);

                float sinp = 2.0f * (dqw * dqy - dqz * dqx);
                if (sinp >= 1.0f)       ctrl.pitch_deg = 90.0f;
                else if (sinp <= -1.0f) ctrl.pitch_deg = -90.0f;
                else                    ctrl.pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

                float siny = 2.0f * (dqw * dqz + dqx * dqy);
                float cosy = 1.0f - 2.0f * (dqy * dqy + dqz * dqz);
                float yaw = atan2f(siny, cosy) * (180.0f / (float)M_PI);
                ctrl.heading_deg = 90.0f - yaw;
                if (ctrl.heading_deg < 0.0f)   ctrl.heading_deg += 360.0f;
                if (ctrl.heading_deg >= 360.0f) ctrl.heading_deg -= 360.0f;
            }

            ctrl.mask_cal_frames = telem.mask_cal_frames;
            ctrl.mask_active     = telem.mask_active;

            ctrl.phase_cal_loaded = phase_cal.loaded;

            ctrl.shutter_accum_count_ui = shutter_stream_bytes_written();
            ctrl.shutter_full_ui        = sphere.shutter_full;

            // Forward DCD save request from UI to IMU worker
            if (ctrl.save_dcd_requested) {
                ctrl.save_dcd_requested = 0;
                imu_worker_request_save_dcd(&imu_ctx);
            }
            ctrl.save_dcd_result = imu_ctx.save_dcd_result;
        }

        // Render control window
        if (!headless) {
            control_render(&ctrl, &telem);
        }
    }

    // ---- Cleanup ----
    // If the user closes the app mid-shutter, finalize the streamed file
    // so we don't leave a .partial sitting on disk forever.
    if (shutter_stream_active()) {
        snapshot_orientation_t snap_orient = {0};
        fill_snap_orientation(&imu_ctx, &snap_orient);
        shutter_stream_end(&snap_orient, NULL);
    }

    imu_worker_stop(&imu_ctx);

    ctx.quit = 1;
    pthread_join(worker_th, NULL);

    if (!headless) {
        sphere_destroy(&sphere);
    }

    spur_mask_destroy(&ctx.mask);
    phase_cal_destroy(&phase_cal);

    free(local_pts);
    free(imu_pts);
    free(pts_a);
    free(pts_b);

    pthread_mutex_destroy(&ctx.mtx);
    pthread_mutex_destroy(&ctx.range_mtx);

    for (int u = 0; u < CHANNELS_USED; ++u)
    {
        fftwf_destroy_plan(plan[u]);
        fftwf_free(fft_out[u]);
    }
    fftwf_free(fft_in);

    if (!headless) {
        glfwDestroyWindow(win);
        glfwTerminate();
        control_destroy(&ctrl);
    }

    munmap(ring, map_len);
    close(fd);

    return 0;
}

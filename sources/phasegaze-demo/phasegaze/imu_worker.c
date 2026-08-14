// imu_worker.c
// Background thread that manages the BNO080 connection and polls orientation.

#include "imu_worker.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <glob.h>

// ---- device discovery -------------------------------------------------------

static const uint8_t scan_addrs[] = { BNO080_DEFAULT_ADDR, BNO080_ALT_ADDR, 0 };

// Scan /dev/i2c-* for a responding BNO080. Returns 1 and fills bus_out /
// addr_out on success, 0 if nothing found.
static int find_imu(char *bus_out, int bus_len, uint8_t *addr_out)
{
    glob_t g;
    if (glob("/dev/i2c-*", 0, NULL, &g) != 0 || g.gl_pathc == 0)
        return 0;

    int found = 0;
    for (size_t i = 0; i < g.gl_pathc && !found; i++) {
        const char *path = g.gl_pathv[i];
        if (access(path, F_OK) != 0) continue;

        for (int a = 0; scan_addrs[a] && !found; a++) {
            bno080_t probe;
            if (bno080_open(&probe, path, scan_addrs[a]) != 0)
                continue;

            bno080_product_id_t pid;
            bno080_reset(&probe);
            if (bno080_get_product_id(&probe, &pid) == 0) {
                snprintf(bus_out, bus_len, "%s", path);
                *addr_out = scan_addrs[a];
                found = 1;
            }
            bno080_close(&probe);
        }
    }
    globfree(&g);
    return found;
}

// ---- background thread ------------------------------------------------------

static void *imu_thread(void *arg)
{
    imu_ctx_t *ctx = (imu_ctx_t *)arg;

    while (!ctx->quit) {
        // Announce that we are scanning.
        pthread_mutex_lock(&ctx->lock);
        ctx->status = IMU_STATUS_SEARCHING;
        snprintf(ctx->device_info, sizeof(ctx->device_info), "searching...");
        pthread_mutex_unlock(&ctx->lock);

        char bus[128];
        uint8_t addr;
        if (!find_imu(bus, sizeof(bus), &addr)) {
            pthread_mutex_lock(&ctx->lock);
            ctx->status = IMU_STATUS_NOT_FOUND;
            snprintf(ctx->device_info, sizeof(ctx->device_info), "no device found");
            pthread_mutex_unlock(&ctx->lock);

            // Retry every 5 seconds.
            for (int i = 0; i < 50 && !ctx->quit; i++)
                usleep(100000);
            continue;
        }

        bno080_t imu;
        if (bno080_open(&imu, bus, addr) != 0) {
            pthread_mutex_lock(&ctx->lock);
            ctx->status = IMU_STATUS_NOT_FOUND;
            snprintf(ctx->device_info, sizeof(ctx->device_info), "open failed");
            pthread_mutex_unlock(&ctx->lock);

            for (int i = 0; i < 20 && !ctx->quit; i++)
                usleep(100000);
            continue;
        }

        bno080_reset(&imu);
        bno080_enable_report(&imu, REPORT_ROTATION_VECTOR, 1000);
        // Sensor reports at 10 Hz for per-sensor cal status visibility
        bno080_enable_report(&imu, REPORT_MAGNETIC_FIELD, 100000);
        bno080_enable_report(&imu, REPORT_ACCELEROMETER, 100000);
        bno080_enable_report(&imu, REPORT_GYROSCOPE, 100000);

        pthread_mutex_lock(&ctx->lock);
        ctx->status = IMU_STATUS_CONNECTED;
        snprintf(ctx->device_info, sizeof(ctx->device_info),
                 "%s @ 0x%02X", bus, addr);
        pthread_mutex_unlock(&ctx->lock);

        // Polling loop — runs until I/O error or quit.
        while (!ctx->quit) {
            // Handle DCD save request from main thread
            if (ctx->save_dcd_requested) {
                ctx->save_dcd_requested = 0;
                ctx->save_dcd_result = 1; // pending
                int rc = bno080_save_dcd(&imu);
                ctx->save_dcd_result = rc;
            }

            int n = bno080_receive(&imu);
            if (n < 0) {
                pthread_mutex_lock(&ctx->lock);
                ctx->status = IMU_STATUS_ERROR;
                snprintf(ctx->device_info, sizeof(ctx->device_info),
                         "error — reconnecting");
                pthread_mutex_unlock(&ctx->lock);
                break;
            }
            if (n == 0) {
                usleep(1000);
                continue;
            }

            int id = bno080_parse_input_report(&imu);
            if (id == REPORT_ROTATION_VECTOR) {
                float lqx = imu.rotation_vector.x;
                float lqy = imu.rotation_vector.y;
                float lqz = imu.rotation_vector.z;
                float lqw = imu.rotation_vector.w;

                // The physical IMU is mounted such that:
                // Sensor +Y = Forward, Sensor +X = Right, Sensor +Z = Up.
                // The software expects the Antenna frame to be:
                // Antenna +X = Left, Antenna +Y = Up, Antenna +Z = Forward.
                //
                // We apply a fixed baseline rotation q_baseline that maps the Antenna frame
                // into the Sensor frame so the rest of the software works out-of-the-box.
                // q_baseline = (w=0, x=0, y=0.70710678f, z=0.70710678f)
                // We compute q_aw = q_raw * q_baseline:
                float bw = 0.0f;
                float bx = 0.0f;
                float by = 0.70710678f;
                float bz = 0.70710678f;

                float mw = lqw*bw - lqx*bx - lqy*by - lqz*bz;
                float mx = lqw*bx + lqx*bw + lqy*bz - lqz*by;
                float my = lqw*by - lqx*bz + lqy*bw + lqz*bx;
                float mz = lqw*bz + lqx*by - lqy*bx + lqz*bw;

                pthread_mutex_lock(&ctx->lock);
                ctx->qw = mw;
                ctx->qx = mx;
                ctx->qy = my;
                ctx->qz = mz;
                ctx->cal_rv = imu.cal_rv;
                ctx->accuracy_rad = imu.rotation_vector.accuracy;
                pthread_mutex_unlock(&ctx->lock);
            }
            else if (id == REPORT_MAGNETIC_FIELD) {
                pthread_mutex_lock(&ctx->lock);
                ctx->cal_mag = imu.cal_mag;
                pthread_mutex_unlock(&ctx->lock);
            }
            else if (id == REPORT_ACCELEROMETER) {
                pthread_mutex_lock(&ctx->lock);
                ctx->cal_accel = imu.cal_accel;
                pthread_mutex_unlock(&ctx->lock);
            }
            else if (id == REPORT_GYROSCOPE) {
                pthread_mutex_lock(&ctx->lock);
                ctx->cal_gyro = imu.cal_gyro;
                pthread_mutex_unlock(&ctx->lock);
            }
        }

        bno080_disable_report(&imu, REPORT_ROTATION_VECTOR);
        bno080_disable_report(&imu, REPORT_MAGNETIC_FIELD);
        bno080_disable_report(&imu, REPORT_ACCELEROMETER);
        bno080_disable_report(&imu, REPORT_GYROSCOPE);
        bno080_close(&imu);

        // Brief pause before attempting reconnect.
        if (!ctx->quit) {
            for (int i = 0; i < 20 && !ctx->quit; i++)
                usleep(100000);
        }
    }

    return NULL;
}

// ---- public API -------------------------------------------------------------

void imu_worker_init(imu_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    pthread_mutex_init(&ctx->lock, NULL);
    ctx->qw = 1.0f; // identity quaternion
    ctx->status = IMU_STATUS_SEARCHING;
    snprintf(ctx->device_info, sizeof(ctx->device_info), "searching...");
}

int imu_worker_start(imu_ctx_t *ctx)
{
    ctx->quit = 0;
    if (pthread_create(&ctx->thread, NULL, imu_thread, ctx) != 0) {
        perror("imu_worker: pthread_create");
        return -1;
    }
    return 0;
}

void imu_worker_stop(imu_ctx_t *ctx)
{
    ctx->quit = 1;
    pthread_join(ctx->thread, NULL);
    pthread_mutex_destroy(&ctx->lock);
}

imu_status_t imu_worker_get_status(imu_ctx_t *ctx, char *info_out, int info_len)
{
    pthread_mutex_lock(&ctx->lock);
    imu_status_t s = ctx->status;
    if (info_out && info_len > 0)
        snprintf(info_out, info_len, "%s", ctx->device_info);
    pthread_mutex_unlock(&ctx->lock);
    return s;
}

void imu_worker_get_orientation(imu_ctx_t *ctx,
                                float *qw, float *qx, float *qy, float *qz)
{
    pthread_mutex_lock(&ctx->lock);
    *qw = ctx->qw;
    *qx = ctx->qx;
    *qy = ctx->qy;
    *qz = ctx->qz;
    pthread_mutex_unlock(&ctx->lock);
}

void imu_worker_get_cal_status(imu_ctx_t *ctx, imu_cal_status_t *out)
{
    pthread_mutex_lock(&ctx->lock);
    out->cal_accel    = ctx->cal_accel;
    out->cal_gyro     = ctx->cal_gyro;
    out->cal_mag      = ctx->cal_mag;
    out->cal_rv       = ctx->cal_rv;
    out->accuracy_rad = ctx->accuracy_rad;
    pthread_mutex_unlock(&ctx->lock);
}

void imu_worker_request_save_dcd(imu_ctx_t *ctx)
{
    ctx->save_dcd_result = 1;   // mark pending
    ctx->save_dcd_requested = 1;
}

// imu_worker.h
// BNO080 IMU background thread for csi_sweep.
//
// Tries to find and connect to the IMU on startup. If none is found it
// retries periodically. Once connected it polls orientation data. On I/O
// error it closes the device and tries to reconnect.

#ifndef IMU_WORKER_H
#define IMU_WORKER_H

#include <pthread.h>
#include <stdint.h>
#include "bno080.h"

typedef enum {
    IMU_STATUS_SEARCHING = 0,   // startup scan in progress
    IMU_STATUS_NOT_FOUND,       // scan complete, no device
    IMU_STATUS_CONNECTED,       // live, receiving data
    IMU_STATUS_ERROR,           // I/O error, attempting reconnect
} imu_status_t;

// Calibration + accuracy snapshot for the snapshot path.
typedef struct {
    uint8_t cal_accel;       // 0-3 (CAL_UNRELIABLE..CAL_HIGH)
    uint8_t cal_gyro;        // 0-3
    uint8_t cal_mag;         // 0-3
    uint8_t cal_rv;          // 0-3, rotation vector overall
    float   accuracy_rad;    // rotation vector accuracy estimate (radians)
} imu_cal_status_t;

typedef struct {
    pthread_mutex_t lock;
    volatile int    quit;

    imu_status_t status;
    char         device_info[64];   // e.g. "/dev/i2c-1 @ 0x4A"

    // Axis-mapped orientation quaternion (same default mapping as viz3d).
    // Identity (1,0,0,0) until the first report arrives.
    float qw, qx, qy, qz;

    // Per-sensor calibration accuracy (0-3) and rotation vector accuracy.
    uint8_t cal_accel;
    uint8_t cal_gyro;
    uint8_t cal_mag;
    uint8_t cal_rv;
    float   accuracy_rad;

    // Set by main thread to request a DCD save; cleared by IMU thread.
    volatile int save_dcd_requested;
    volatile int save_dcd_result;    // 0 = ok, -1 = fail, 1 = pending

    pthread_t thread;
} imu_ctx_t;

// Initialise the context (zero-fill + mutex init). Must be called before start.
void imu_worker_init(imu_ctx_t *ctx);

// Spawn the background thread. Returns 0 on success.
int  imu_worker_start(imu_ctx_t *ctx);

// Signal the thread to stop and join it. Destroys the mutex.
void imu_worker_stop(imu_ctx_t *ctx);

// Read current status under lock. Copies device_info into info_out if non-NULL.
imu_status_t imu_worker_get_status(imu_ctx_t *ctx, char *info_out, int info_len);

// Read the latest axis-mapped orientation quaternion under lock.
void imu_worker_get_orientation(imu_ctx_t *ctx,
                                float *qw, float *qx, float *qy, float *qz);

// Read calibration accuracy under lock.
void imu_worker_get_cal_status(imu_ctx_t *ctx, imu_cal_status_t *out);

// Request persistent save of dynamic calibration data (DCD) to BNO080 flash.
// Non-blocking; sets a flag that the IMU thread picks up.
void imu_worker_request_save_dcd(imu_ctx_t *ctx);

#endif // IMU_WORKER_H

// snapshot.h
// Save a RF picture (.rfpic) snapshot of the current sphere history.
//
// v1 layout: rfpic_header_t (40 bytes) + points + notes
// v2 layout: rfpic_header_v2_t (72 bytes) + points + notes
//   v2 adds IMU orientation metadata (quaternion, heading/pitch/roll, cal status)

#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <stdint.h>
#include "sphere_render.h"
#include "imu_worker.h"

// v1 header — 40 bytes (kept for reference / backward-compat readers).
typedef struct {
    char     magic[6];       // "RFPIC\0"
    uint8_t  version;        // = 1
    uint8_t  camera_mode;    // 1 = IMU world-stabilized, 0 = antenna-frame
    int64_t  timestamp;      // Unix epoch seconds
    double   lo_start_mhz;   // sweep start [MHz]
    double   lo_end_mhz;     // sweep stop  [MHz]
    uint32_t point_count;
    uint32_t _pad;
} rfpic_header_t;

// v2 header — 72 bytes. First 36 bytes identical to v1; old _pad replaced.
typedef struct {
    char     magic[6];        // "RFPIC\0"
    uint8_t  version;         // = 2
    uint8_t  camera_mode;     // 0 = antenna-frame, 1 = IMU world-stabilized
    int64_t  timestamp;       // Unix epoch seconds
    double   lo_start_mhz;
    double   lo_end_mhz;
    uint32_t point_count;

    // Orientation metadata (replaces v1 _pad at offset 36)
    uint8_t  imu_valid;       // 1 = orientation fields are populated
    uint8_t  cal_accel;       // calibration accuracy 0-3
    uint8_t  cal_gyro;        // calibration accuracy 0-3
    uint8_t  cal_mag;         // calibration accuracy 0-3

    // Absolute orientation quaternion (antenna → world, north-referenced)
    float    orient_qw;
    float    orient_qx;
    float    orient_qy;
    float    orient_qz;
    float    orient_accuracy; // rotation vector accuracy (radians)

    // GPano-style Euler angles derived from the quaternion
    float    heading_deg;     // degrees CW from magnetic north, [0, 360)
    float    pitch_deg;       // degrees above horizon, [-90, 90]
    float    roll_deg;        // degrees, 0 = level, [-180, 180]
} rfpic_header_v2_t;

// Per-point record — same field order and layout as render_point_t (24 bytes).
typedef struct {
    float gx, gy;       // phase-gradient coordinates
    float r, g, b;      // display color [0,1]
    float intensity;    // normalized signal magnitude [0,1]
} rfpic_point_t;

// Orientation data passed to snapshot_save. If imu_valid is 0, orientation
// fields are zeroed in the file header.
typedef struct {
    int     imu_valid;
    float   qw, qx, qy, qz;
    float   accuracy_rad;
    uint8_t cal_accel, cal_gyro, cal_mag;
} snapshot_orientation_t;

// Save pts[0..count-1] to ~/Desktop/rf_pics/rfpic_YYYYMMDD_HHMMSS.rfpic.
// orient may be NULL (equivalent to imu_valid=0).
// If path_out is non-NULL, the saved path is written into it (up to path_out_len bytes).
// Returns 0 on success, -1 on error, -2 on verification failure.
int snapshot_save(const render_point_t *pts, int count,
                  double lo_start_mhz, double lo_end_mhz,
                  int camera_mode,
                  const snapshot_orientation_t *orient,
                  char *path_out, int path_out_len);

// Rename an already-saved .rfpic file to append a human label before the extension:
//   rfpic_YYYYMMDD_HHMMSS.rfpic  →  rfpic_YYYYMMDD_HHMMSS_label.rfpic
// Label is sanitized: [a-zA-Z0-9_-] kept, spaces become '_', all else stripped,
// result truncated to 32 chars. Returns 0 on success, -1 on error.
int snapshot_apply_label(const char *path, const char *label);

// ------------------------------------------------------------
// Low-level helpers used by both snapshot_save and the streaming
// shutter writer in shutter_stream.c.
// ------------------------------------------------------------

#include <stdio.h>
#include <time.h>

// Writes the 72-byte rfpic v2 header at the current position of f.
// Pass orient=NULL or imu_valid=0 to leave IMU fields zeroed.
// Returns 0 on success, -1 on write error. The caller is responsible
// for any necessary fseek() before/after.
int snapshot_write_header(FILE *f,
                          double lo_start_mhz, double lo_end_mhz,
                          int camera_mode, time_t ts,
                          const snapshot_orientation_t *orient,
                          uint32_t point_count);

// Appends the trailing human-readable NOTES block, closes the file,
// then re-opens it briefly to verify the "RFPIC\0" magic at offset 0.
// Returns 0 on success, -1 on I/O error, -2 on verification failure.
int snapshot_append_notes_and_close(FILE *f, const char *path);

#endif // SNAPSHOT_H

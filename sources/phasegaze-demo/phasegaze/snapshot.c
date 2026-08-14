// snapshot.c
// Write .rfpic v2 snapshot files to ~/Desktop/rf_pics/

#include "snapshot.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

_Static_assert(sizeof(rfpic_header_v2_t) == 72, "rfpic_header_v2_t must be 72 bytes");
_Static_assert(sizeof(rfpic_point_t)     == 24, "rfpic_point_t must be 24 bytes");
_Static_assert(sizeof(render_point_t)    == 24, "render_point_t must be 24 bytes");

// Convert quaternion (w,x,y,z) to GPano-style heading/pitch/roll.
// The BNO080 rotation vector quaternion represents the rotation from the
// East-North-Up (ENU) reference frame to the sensor frame.
//
// heading = degrees CW from magnetic north, [0, 360)
// pitch   = degrees above horizon, [-90, 90]
// roll    = degrees, 0 = level, [-180, 180]
static void quat_to_euler_gpano(float qw, float qx, float qy, float qz,
                                float *heading, float *pitch, float *roll)
{
    // q_aw maps Antenna frame (X=Left, Y=Up, Z=Forward) to World frame (X=East, Y=North, Z=Up).
    // Extract standard Heading, Pitch, Roll from the rotation matrix.
    // Forward vector in World frame:
    float f_e = 2.0f * (qx * qz + qw * qy); // East
    float f_n = 2.0f * (qy * qz - qw * qx); // North
    float f_u = 1.0f - 2.0f * (qx * qx + qy * qy); // Up

    *heading = atan2f(f_e, f_n) * (180.0f / (float)M_PI);
    if (*heading < 0.0f)   *heading += 360.0f;
    if (*heading >= 360.0f) *heading -= 360.0f;

    // Pitch: positive is nose up.
    if (f_u >= 1.0f)
        *pitch = 90.0f;
    else if (f_u <= -1.0f)
        *pitch = -90.0f;
    else
        *pitch = asinf(f_u) * (180.0f / (float)M_PI);

    // Roll: Left vector Up component vs Up vector Up component
    float l_u = 2.0f * (qx * qz - qw * qy); // Up component of Left
    float u_u = 2.0f * (qy * qz + qw * qx); // Up component of Up

    *roll = atan2f(l_u, u_u) * (180.0f / (float)M_PI);
}

// Appended after binary data so every file is self-documenting.
static const char NOTES[] =
"\n"
"--- RFPIC FORMAT NOTES (v2) ---\n"
"Binary layout (little-endian host byte order, no internal padding):\n"
"\n"
"  Header (72 bytes):\n"
"    Offset  0  char[6]   magic            \"RFPIC\\0\"\n"
"    Offset  6  uint8     version          = 2\n"
"    Offset  7  uint8     camera_mode      0=antenna-frame  1=IMU world-stabilized\n"
"    Offset  8  int64     timestamp        Unix epoch seconds\n"
"    Offset 16  float64   lo_start_mhz     sweep start frequency\n"
"    Offset 24  float64   lo_end_mhz       sweep stop frequency\n"
"    Offset 32  uint32    point_count\n"
"    Offset 36  uint8     imu_valid        1 = orientation data populated\n"
"    Offset 37  uint8     cal_accel        calibration accuracy 0-3\n"
"    Offset 38  uint8     cal_gyro         calibration accuracy 0-3\n"
"    Offset 39  uint8     cal_mag          calibration accuracy 0-3\n"
"    Offset 40  float32   orient_qw        absolute orientation quaternion W\n"
"    Offset 44  float32   orient_qx        orientation quaternion X\n"
"    Offset 48  float32   orient_qy        orientation quaternion Y\n"
"    Offset 52  float32   orient_qz        orientation quaternion Z\n"
"    Offset 56  float32   orient_accuracy  rotation vector accuracy (radians)\n"
"    Offset 60  float32   heading_deg      degrees CW from magnetic north [0,360)\n"
"    Offset 64  float32   pitch_deg        degrees above horizon [-90,90]\n"
"    Offset 68  float32   roll_deg         degrees, 0 = level [-180,180]\n"
"\n"
"  Point records: point_count x 24 bytes each, starting at offset 72.\n"
"    +0   float32  gx         phase-gradient x\n"
"    +4   float32  gy         phase-gradient y\n"
"    +8   float32  r          display color red   [0,1]\n"
"    +12  float32  g          display color green [0,1]\n"
"    +16  float32  b          display color blue  [0,1]\n"
"    +20  float32  intensity  normalized signal magnitude [0,1]\n"
"\n"
"  v1 compatibility: version=1 files have a 40-byte header (no orientation)\n"
"  with points starting at offset 40. The first 36 bytes are identical.\n"
"\n"
"Reconstruction — 3D direction from (gx, gy):\n"
"\n"
"  Physical parameters:\n"
"    d = 0.0455 m (antenna nearest-neighbor spacing)\n"
"    c = 299.792458 m/us\n"
"    scale_factor(f_mhz) = 2 * pi * (d / c) * f_mhz\n"
"\n"
"  camera_mode = 0 (antenna-frame):\n"
"    gx/gy for each point use the scale factor for that point's own RF\n"
"    frequency.  Recover freq_mhz from the color hue (see below), then:\n"
"      sf = scale_factor(freq_mhz)\n"
"      u = gx / sf\n"
"      v = gy / sf\n"
"      w = sqrt(1 - u*u - v*v)     (w >= 0 means upper hemisphere)\n"
"\n"
"  camera_mode = 1 (IMU world-stabilized):\n"
"    All points were normalized to the sweep center scale factor and then\n"
"    rotated into absolute north-referenced world frame (q_cur · q_mount).\n"
"    Use:\n"
"      sf = scale_factor((lo_start_mhz + lo_end_mhz) / 2)\n"
"      u = gx / sf\n"
"      v = gy / sf\n"
"      w = sqrt(1 - u*u - v*v)\n"
"\n"
"  Spherical angles from (u, v, w):\n"
"    elevation = asin(w)          radians; 0=horizon, pi/2=zenith\n"
"    azimuth   = atan2(u, v)      radians; 0=forward, clockwise positive\n"
"\n"
"Orientation metadata (v2 only, when imu_valid = 1):\n"
"  The quaternion is antenna → world (q_cur · q_mount when mount cal is set,\n"
"  otherwise raw BNO080 rotation vector), north-referenced via 9-axis fusion.\n"
"  Euler angles follow GPano convention:\n"
"    heading = degrees CW from magnetic north\n"
"    pitch   = degrees above horizon\n"
"    roll    = 0 when level\n"
"\n"
"Recovering frequency from color:\n"
"  Colors are encoded as HSV with saturation=1 and value=intensity.\n"
"  Hue encodes frequency linearly:\n"
"    hue = (freq_mhz - FREQ_MIN_MHZ) / (FREQ_MAX_MHZ - FREQ_MIN_MHZ)\n"
"  To invert — recover hue from (r, g, b) [standard max-channel formula]:\n"
"    mx = max(r, g, b)\n"
"    mn = min(r, g, b)\n"
"    delta = mx - mn\n"
"    if mx == r:  hue = (g - b) / delta\n"
"    if mx == g:  hue = 2 + (b - r) / delta\n"
"    if mx == b:  hue = 4 + (r - g) / delta\n"
"    hue = (hue / 6) mod 1          (normalize to [0,1])\n"
"  Then:\n"
"    freq_mhz = hue * (FREQ_MAX_MHZ - FREQ_MIN_MHZ) + FREQ_MIN_MHZ\n"
"  Constants: FREQ_MIN_MHZ = 4900.0,  FREQ_MAX_MHZ = 6100.0\n"
"--- END RFPIC FORMAT NOTES ---\n";

int snapshot_apply_label(const char *path, const char *label)
{
    if (!path || !label || !label[0]) return -1;

    // Sanitize label into a clean filename segment.
    char clean[33];
    int ci = 0;
    for (int i = 0; label[i] && ci < 32; i++) {
        char c = label[i];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '_' || c == '-') {
            clean[ci++] = c;
        } else if (c == ' ') {
            clean[ci++] = '_';
        }
        // all other chars are dropped
    }
    clean[ci] = '\0';
    if (ci == 0) return -1;

    // Build new path: insert "_label" before the ".rfpic" extension.
    const char *ext = strrchr(path, '.');
    char newpath[720];
    if (ext) {
        int base_len = (int)(ext - path);
        snprintf(newpath, sizeof(newpath), "%.*s_%s%s", base_len, path, clean, ext);
    } else {
        snprintf(newpath, sizeof(newpath), "%s_%s", path, clean);
    }

    if (rename(path, newpath) != 0) {
        fprintf(stderr, "snapshot: rename %s -> %s: %s\n", path, newpath, strerror(errno));
        return -1;
    }
    fprintf(stdout, "snapshot: renamed -> %s\n", newpath);
    fflush(stdout);
    return 0;
}

int snapshot_write_header(FILE *f,
                          double lo_start_mhz, double lo_end_mhz,
                          int camera_mode, time_t ts,
                          const snapshot_orientation_t *orient,
                          uint32_t point_count)
{
    rfpic_header_v2_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, "RFPIC\0", 6);
    hdr.version      = 2;
    hdr.camera_mode  = (uint8_t)(camera_mode ? 1 : 0);
    hdr.timestamp    = (int64_t)ts;
    hdr.lo_start_mhz = lo_start_mhz;
    hdr.lo_end_mhz   = lo_end_mhz;
    hdr.point_count  = point_count;

    if (orient && orient->imu_valid) {
        hdr.imu_valid       = 1;
        hdr.cal_accel       = orient->cal_accel;
        hdr.cal_gyro        = orient->cal_gyro;
        hdr.cal_mag         = orient->cal_mag;
        hdr.orient_qw       = orient->qw;
        hdr.orient_qx       = orient->qx;
        hdr.orient_qy       = orient->qy;
        hdr.orient_qz       = orient->qz;
        hdr.orient_accuracy = orient->accuracy_rad;

        quat_to_euler_gpano(orient->qw, orient->qx, orient->qy, orient->qz,
                            &hdr.heading_deg, &hdr.pitch_deg, &hdr.roll_deg);
    }

    if (fwrite(&hdr, sizeof(hdr), 1, f) != 1) {
        fprintf(stderr, "snapshot: header write failed\n");
        return -1;
    }
    return 0;
}

int snapshot_append_notes_and_close(FILE *f, const char *path)
{
    if (!f) return -1;
    fputs(NOTES, f);
    if (fclose(f) != 0) {
        fprintf(stderr, "snapshot: fclose %s: %s\n", path ? path : "(null)", strerror(errno));
        return -1;
    }

    if (!path) return 0;

    // Re-open and verify the magic string actually made it to disk.
    FILE *vf = fopen(path, "rb");
    if (!vf) {
        fprintf(stderr, "snapshot: verification open failed\n");
        return -2;
    }
    char check[6];
    if (fread(check, 1, 6, vf) != 6 || memcmp(check, "RFPIC\0", 6) != 0) {
        fprintf(stderr, "snapshot: magic string verification failed!\n");
        fclose(vf);
        return -2;
    }
    fclose(vf);
    return 0;
}

int snapshot_save(const render_point_t *pts, int count,
                  double lo_start_mhz, double lo_end_mhz,
                  int camera_mode,
                  const snapshot_orientation_t *orient,
                  char *path_out, int path_out_len)
{
    const char *home = getenv("HOME");
    if (!home || home[0] == '\0') home = ".";

    char desktop[512];
    char dir[512];
    snprintf(desktop, sizeof(desktop), "%s/Desktop", home);
    snprintf(dir, sizeof(dir), "%s/Desktop/rf_pics", home);

    if (mkdir(desktop, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "snapshot: mkdir %s: %s\n", desktop, strerror(errno));
        return -1;
    }
    if (mkdir(dir, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "snapshot: mkdir %s: %s\n", dir, strerror(errno));
        return -1;
    }

    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char path[640];
    snprintf(path, sizeof(path), "%s/rfpic_%04d%02d%02d_%02d%02d%02d.rfpic",
             dir,
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);

    if (path_out && path_out_len > 0)
        snprintf(path_out, (size_t)path_out_len, "%s", path);

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "snapshot: fopen %s: %s\n", path, strerror(errno));
        return -1;
    }

    uint32_t pc = (uint32_t)(count > 0 ? count : 0);
    if (snapshot_write_header(f, lo_start_mhz, lo_end_mhz, camera_mode,
                               now, orient, pc) != 0) {
        fclose(f);
        return -1;
    }

    if (count > 0 && pts) {
        if (fwrite(pts, sizeof(render_point_t), (size_t)count, f) != (size_t)count) {
            fprintf(stderr, "snapshot: point write failed\n");
            fclose(f);
            return -1;
        }
    }

    int rc = snapshot_append_notes_and_close(f, path);
    if (rc != 0) return rc;

    if (orient && orient->imu_valid) {
        float h = 0, p = 0, r = 0;
        quat_to_euler_gpano(orient->qw, orient->qx, orient->qy, orient->qz, &h, &p, &r);
        fprintf(stdout, "snapshot: saved %d points -> %s  (heading=%.1f pitch=%.1f roll=%.1f)\n",
                count, path, h, p, r);
    } else {
        fprintf(stdout, "snapshot: saved %d points -> %s  (no IMU)\n", count, path);
    }
    fflush(stdout);
    return 0;
}

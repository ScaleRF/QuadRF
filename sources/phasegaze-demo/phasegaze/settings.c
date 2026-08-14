// settings.c
// Minimal JSON load/save for fixed schema (no external JSON library)

#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')
        p++;
    return p;
}

static int find_key(const char *buf, const char *key, const char **val_out)
{
    char quoted[80];
    snprintf(quoted, sizeof(quoted), "\"%s\"", key);
    const char *p = strstr(buf, quoted);
    if (!p)
        return -1;
    p = strchr(p + strlen(quoted), ':');
    if (!p)
        return -1;
    *val_out = skip_ws(p + 1);
    return 0;
}

static int parse_double_field(const char *buf, const char *key, double *out)
{
    const char *v;
    if (find_key(buf, key, &v) != 0)
        return -1;
    char *end = NULL;
    double x = strtod(v, &end);
    if (end == v)
        return -1;
    *out = x;
    return 0;
}

static int parse_int_field(const char *buf, const char *key, int *out)
{
    const char *v;
    if (find_key(buf, key, &v) != 0)
        return -1;
    char *end = NULL;
    long x = strtol(v, &end, 10);
    if (end == v)
        return -1;
    *out = (int)x;
    return 0;
}

int phasegaze_settings_get_path(char *buf, size_t buf_sz)
{
    if (buf_sz < 32)
        return -1;

    ssize_t n = readlink("/proc/self/exe", buf, buf_sz - 1);
    if (n < 0) {
        /* e.g. non-Linux: fall back to cwd */
        if ((size_t)snprintf(buf, buf_sz, "settings.json") >= buf_sz)
            return -1;
        return 0;
    }
    buf[n] = '\0';

    char *slash = strrchr(buf, '/');
    if (!slash)
        return -1;
    *slash = '\0';

    size_t plen = strlen(buf);
    if (plen + 1 + (sizeof("settings.json") - 1) + 1 > buf_sz)
        return -1;
    snprintf(buf + plen, buf_sz - plen, "/settings.json");
    return 0;
}

int phasegaze_settings_ensure_dir(void)
{
    return 0;
}

int phasegaze_settings_load(phasegaze_ui_settings *out)
{
    char path[512];
    if (phasegaze_settings_get_path(path, sizeof(path)) != 0)
        return -1;

    FILE *fp = fopen(path, "rb");
    if (!fp)
        return -1;

    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return -1; }
    long sz = ftell(fp);
    if (sz < 0 || sz > 65536) { fclose(fp); return -1; }
    rewind(fp);

    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(fp); return -1; }
    if (fread(buf, 1, (size_t)sz, fp) != (size_t)sz) { free(buf); fclose(fp); return -1; }
    fclose(fp);
    buf[sz] = '\0';

    phasegaze_ui_settings t;
    memset(&t, 0, sizeof(t));

    if (parse_double_field(buf, "lo_start_mhz", &t.lo_start_mhz) != 0) { free(buf); return -1; }
    if (parse_double_field(buf, "lo_end_mhz", &t.lo_end_mhz) != 0) { free(buf); return -1; }
    if (parse_double_field(buf, "point_size", &t.point_size) != 0) { free(buf); return -1; }
    if (parse_double_field(buf, "point_gain", &t.point_gain) != 0) { free(buf); return -1; }
    if (parse_double_field(buf, "decay_factor", &t.decay_factor) != 0) { free(buf); return -1; }
    if (parse_double_field(buf, "intensity_threshold", &t.intensity_threshold) != 0) { free(buf); return -1; }
    if (parse_int_field(buf, "show_bottom", &t.show_bottom) != 0) { free(buf); return -1; }
    if (parse_int_field(buf, "show_mirrors", &t.show_mirrors) != 0) { free(buf); return -1; }

    // RF gain: prefer the new integer key. Fall back to the legacy double "rf_gain"
    // (which was either dB or dBFS depending on the now-removed agc_mode).
    if (parse_int_field(buf, "rf_gain_db", &t.rf_gain_db) != 0) {
        double legacy;
        if (parse_double_field(buf, "rf_gain", &legacy) == 0)
            t.rf_gain_db = clampi((int)lround(legacy), 0, 63);
        else
            t.rf_gain_db = 30;
    } else {
        t.rf_gain_db = clampi(t.rf_gain_db, 0, 63);
    }

    // Digital BW: prefer the new integer "dig_bw_k". Fall back to the legacy
    // "dig_bw_mhz" by inverting bw = 240/k → k = round(240/bw).
    if (parse_int_field(buf, "dig_bw_k", &t.dig_bw_k) != 0) {
        double legacy_mhz;
        if (parse_double_field(buf, "dig_bw_mhz", &legacy_mhz) == 0 && legacy_mhz > 0.0)
            t.dig_bw_k = clampi((int)lround(240.0 / legacy_mhz), 5, 63);
        else
            t.dig_bw_k = 15;
    } else {
        t.dig_bw_k = clampi(t.dig_bw_k, 5, 63);
    }

    // Viewfinder mode: new field, default off if not present.
    if (parse_int_field(buf, "viewfinder_mode", &t.viewfinder_mode) != 0)
        t.viewfinder_mode = 0;
    else
        t.viewfinder_mode = t.viewfinder_mode ? 1 : 0;

    // Stability mode: default on (1) if not present in older settings files.
    if (parse_int_field(buf, "stability_mode", &t.stability_mode) != 0)
        t.stability_mode = 1;
    else
        t.stability_mode = t.stability_mode ? 1 : 0;

    // IMU mount calibration. All optional — missing fields just leave the
    // corresponding half marked invalid so the user can recalibrate.
    if (parse_int_field(buf, "mount_z_valid", &t.mount_z_valid) != 0)
        t.mount_z_valid = 0;
    if (parse_double_field(buf, "mount_z_sx", &t.mount_z_sx) != 0) t.mount_z_sx = 0.0;
    if (parse_double_field(buf, "mount_z_sy", &t.mount_z_sy) != 0) t.mount_z_sy = 0.0;
    if (parse_double_field(buf, "mount_z_sz", &t.mount_z_sz) != 0) t.mount_z_sz = 1.0;

    if (parse_int_field(buf, "mount_y_valid", &t.mount_y_valid) != 0)
        t.mount_y_valid = 0;
    if (parse_int_field(buf, "mount_y_samples", &t.mount_y_samples) != 0)
        t.mount_y_samples = 0;
    if (parse_double_field(buf, "mount_y_sx", &t.mount_y_sx) != 0) t.mount_y_sx = 0.0;
    if (parse_double_field(buf, "mount_y_sy", &t.mount_y_sy) != 0) t.mount_y_sy = 1.0;
    if (parse_double_field(buf, "mount_y_sz", &t.mount_y_sz) != 0) t.mount_y_sz = 0.0;

    free(buf);
    *out = t;
    return 0;
}

int phasegaze_settings_save(const phasegaze_ui_settings *in)
{
    if (phasegaze_settings_ensure_dir() != 0)
        return -1;

    char path[512];
    if (phasegaze_settings_get_path(path, sizeof(path)) != 0)
        return -1;

    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    FILE *fp = fopen(tmp, "wb");
    if (!fp)
        return -1;

    int n = fprintf(fp,
        "{\n"
        "  \"lo_start_mhz\": %.10g,\n"
        "  \"lo_end_mhz\": %.10g,\n"
        "  \"point_size\": %.10g,\n"
        "  \"point_gain\": %.10g,\n"
        "  \"decay_factor\": %.10g,\n"
        "  \"intensity_threshold\": %.10g,\n"
        "  \"show_bottom\": %d,\n"
        "  \"show_mirrors\": %d,\n"
        "  \"rf_gain_db\": %d,\n"
        "  \"dig_bw_k\": %d,\n"
        "  \"viewfinder_mode\": %d,\n"
        "  \"stability_mode\": %d,\n"
        "  \"mount_z_valid\": %d,\n"
        "  \"mount_z_sx\": %.10g,\n"
        "  \"mount_z_sy\": %.10g,\n"
        "  \"mount_z_sz\": %.10g,\n"
        "  \"mount_y_valid\": %d,\n"
        "  \"mount_y_samples\": %d,\n"
        "  \"mount_y_sx\": %.10g,\n"
        "  \"mount_y_sy\": %.10g,\n"
        "  \"mount_y_sz\": %.10g\n"
        "}\n",
        in->lo_start_mhz,
        in->lo_end_mhz,
        in->point_size,
        in->point_gain,
        in->decay_factor,
        in->intensity_threshold,
        in->show_bottom ? 1 : 0,
        in->show_mirrors ? 1 : 0,
        in->rf_gain_db,
        in->dig_bw_k,
        in->viewfinder_mode ? 1 : 0,
        in->stability_mode ? 1 : 0,
        in->mount_z_valid ? 1 : 0,
        in->mount_z_sx, in->mount_z_sy, in->mount_z_sz,
        in->mount_y_valid ? 1 : 0,
        in->mount_y_samples,
        in->mount_y_sx, in->mount_y_sy, in->mount_y_sz);

    if (n < 0 || fclose(fp) != 0) {
        unlink(tmp);
        return -1;
    }

    if (rename(tmp, path) != 0) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

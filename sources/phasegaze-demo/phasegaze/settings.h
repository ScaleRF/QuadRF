// settings.h
// Persist UI state to settings.json beside the csi_sweep executable (multi_file3)

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stddef.h>

typedef struct {
    double lo_start_mhz;
    double lo_end_mhz;
    double point_size;
    double point_gain;
    double decay_factor;
    double intensity_threshold;
    int show_bottom;
    int show_mirrors;
    int    rf_gain_db;    // manual RF gain in dB, integer 0..63
    int    dig_bw_k;      // digital filter divider k, integer 5..63
                          // (resulting bandwidth = 240/k MHz)
    int viewfinder_mode;  // 1 = viewfinder display (pinhole-from-origin)
    int stability_mode;   // 1 = apply IMU rotation in viewfinder live display (default); 0 = raw antenna-frame live display

    // IMU mount calibration (face / bottom / top gravity poses via CalDown).
    // mount_z_* = antenna +Z (boresight) in sensor coords (face-down pose).
    // mount_y_* = antenna +Y (array up) in sensor coords (bottom+top poses).
    // mount_y_samples is legacy (ignored on load; written as 0).
    int    mount_z_valid;
    int    mount_y_valid;
    int    mount_y_samples;
    double mount_z_sx, mount_z_sy, mount_z_sz;
    double mount_y_sx, mount_y_sy, mount_y_sz;
} phasegaze_ui_settings;

// Writes full path to settings.json into buf. Returns 0 on success.
int phasegaze_settings_get_path(char *buf, size_t buf_sz);

// Creates config directory if needed. Returns 0 on success.
int phasegaze_settings_ensure_dir(void);

// Loads from JSON file. Returns 0 on success; -1 if missing/unreadable/invalid.
int phasegaze_settings_load(phasegaze_ui_settings *out);

// Writes JSON atomically (temp file + rename). Returns 0 on success.
int phasegaze_settings_save(const phasegaze_ui_settings *in);

#endif

// control_window.h
// SDL2 control panel window for CSI sweep parameters

#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H

#include <stdbool.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "widgets.h"
#include "worker.h"
#include "imu_worker.h"

#define CTRL_WIN_WIDTH   800
#define CTRL_WIN_HEIGHT  600

typedef enum {
    BS_CLEARED,
    BS_CAL_ARMED,
    BS_CAPTURE,
    BS_COMPLETE
} boresight_state_t;

typedef struct {
    SDL_Window   *win;
    SDL_Renderer *ren;
    TTF_Font     *font;

    // Frequency range
    slider_t     start_slider;
    slider_t     stop_slider;
    textbox_t    start_textbox;
    textbox_t    stop_textbox;

    // SDR hardware sliders (left column)
    // RF gain is integer 0..63 dB (the chip's manual gain range).
    // Digital BW is exposed as the integer filter divider k in [5..63];
    // the actual bandwidth shown on screen is 240/k MHz.
    vertical_slider_t rf_gain_slider;
    vertical_slider_t dig_bw_slider;
    editable_number_t rf_gain_value;
    editable_number_t dig_bw_value;

    // Visualization parameter sliders
    vertical_slider_t point_size_slider;
    vertical_slider_t point_gain_slider;
    vertical_slider_t decay_factor_slider;
    vertical_slider_t intensity_threshold_slider;

    // Editable number fields for sliders
    editable_number_t point_size_value;
    editable_number_t point_gain_value;
    editable_number_t decay_factor_value;
    editable_number_t intensity_threshold_value;

    // Buttons
    button_t kill_button;
    button_t bottom_button;    // toggle show-bottom-hemisphere
    button_t mirror_button;    // toggle FOV mirror/alias display
    button_t camera_button;    // toggle camera (deterministic accumulation) mode
    button_t center_button;    // boresight cal (phase-cal mode only)
    button_t cal_down_button;  // cycle face/bottom/top/clear IMU mount cal (camera mode)
    button_t viewfinder_button; // toggle viewfinder (fixed pinhole-from-origin) display
    button_t stability_button;  // toggle IMU stabilization in viewfinder live display
    button_t calibrate_button; // toggle spur mask calibration
    button_t snapshot_button;              // save RF picture to ~/Desktop/rf_pics/
    button_t shutter_threshold_filter_button; // toggle: only save points >= threshold when shutter open
    button_t shutter_button;              // long-exposure recording
    button_t save_dcd_button;  // persist IMU calibration to flash

    button_t phase_cal_button;        // toggle: enter/exit phase cal (boresight) mode

    // Outputs (read by main loop)
    int    rf_gain_db;       // manual RF gain, integer 0..63
    int    dig_bw_k;         // digital filter divider k, integer 5..63
    float  point_size;
    float  point_gain;
    float  decay_factor;
    float  intensity_threshold;
    int    show_bottom;
    int    show_mirrors;
    int    camera_mode;
    int    viewfinder_mode;   // 1 = viewfinder (pinhole-from-origin) display
    int    stability_mode;    // 1 = apply IMU rotation in viewfinder (default on)
    int    center_requested;  // set to 1 on Boresight click, cleared by main
    int    cal_down_requested; // set to 1 on CalDown click, cleared by main

    // IMU mount calibration state (mirror of main.c's authoritative state).
    // Main writes these when calibration changes and asks the control window
    // to persist; control_window reads them at init from settings.json.
    // mount_cal_step: 0=next face, 1=next bottom, 2=next top, 3=complete/clear
    int    mount_cal_step;
    int    mount_z_valid;
    int    mount_y_valid;
    int    mount_y_samples;   // legacy field in settings.json (unused)
    float  mount_z_sx, mount_z_sy, mount_z_sz;  // antenna +Z in sensor coords
    float  mount_y_sx, mount_y_sy, mount_y_sz;  // antenna +Y in sensor coords
    int    calibrate_mode;
    int    quit_requested;
    int    shutter_threshold_filter; // 1 = only record points >= intensity_threshold; 0 = record all
    int    snapshot_requested;  // set to 1 on Snapshot click, cleared by main
    uint32_t snapshot_flash_time; // SDL tick when last snapshot was taken (for button flash)
    uint32_t snapshot_error_time; // SDL tick when a snapshot error occurred
    int    snapshot_count;      // total snapshots taken this session

    // Shutter mode (long-exposure recording)
    int      shutter_active;            // 1 while shutter mode is on
    int      shutter_toggle_requested;  // edge-triggered, cleared by main
    uint64_t shutter_accum_count_ui;    // bytes written to disk, mirrored by main
    int      shutter_full_ui;           // mirrored from sphere by main

    // Mask telemetry (written by main each frame from worker telemetry)
    int    mask_cal_frames;
    int    mask_active;

    int      phase_cal_mode;            // toggle (set by UI, read by main)
    int      phase_cal_loaded;          // mirrored from main: 1 if applied

    // Boresight delay-matching capture state (main is authoritative).
    boresight_state_t boresight_state;
    int      boresight_slot_idx;        // index 0..4 into capture order {2,3,4,0,1}
    double   phase_cal_saved_lo_start;  // freq range before phase cal entered
    double   phase_cal_saved_lo_end;

    // Frequency outputs (applied to worker ctx via range_mtx)
    double lo_start_mhz;
    double lo_end_mhz;
    int    freq_changed;

    // IMU status (written by main each frame, read by control_render)
    imu_status_t imu_status;
    char         imu_device_info[64];

    // IMU calibration status (written by main each frame)
    uint8_t cal_accel;       // 0-3
    uint8_t cal_gyro;        // 0-3
    uint8_t cal_mag;         // 0-3
    uint8_t cal_rv;          // 0-3

    // Live heading/pitch/roll (computed from quaternion by main loop)
    float  heading_deg;      // CW from north, [0, 360)
    float  pitch_deg;        // above horizon, [-90, 90]
    float  roll_deg;         // 0 = level, [-180, 180]

    // DCD save interface
    int    save_dcd_requested;    // set by UI, cleared by main
    int    save_dcd_result;       // 0 = ok, -1 = fail, 1 = pending
    uint32_t save_dcd_flash_time; // SDL tick for button flash feedback

    // Label dialog — shown after a deliberate snapshot or shutter-stop save.
    // main.c sets label_pending_path and label_dialog_active=1 after saving.
    // control_window.c sets label_dialog_confirmed when user confirms or skips.
    // main.c then calls snapshot_apply_label if confirmed with non-empty text.
    int    label_dialog_active;     // 1 = overlay is visible
    char   label_text[64];          // text typed so far (max 32 meaningful chars)
    int    label_dialog_confirmed;  // 0=waiting, 1=OK/Enter, -1=Skip/Escape
    char   label_pending_path[640]; // path of the file to potentially rename
} control_window_t;

// Returns 0 on success
int  control_init(control_window_t *cw);
void control_destroy(control_window_t *cw);

// Process SDL2 events, update internal state. Call once per frame.
void control_process_events(control_window_t *cw);

// Render the control window. telem may be NULL.
void control_render(control_window_t *cw, const telemetry_t *telem);

// Force a settings.json write using the current cw state. Use after main.c
// updates cw->mount_*_* in response to a CalDown click so the new
// calibration is durable across restarts.
void control_persist_settings(control_window_t *cw);

// Set LO start/stop programmatically (updates sliders and flags freq_changed).
void control_set_freq_range(control_window_t *cw, double start_mhz,
                            double stop_mhz);

#endif // CONTROL_WINDOW_H

/*
 * bno080.h - BNO080 IMU driver over I2C (SHTP protocol)
 *
 * Raspberry Pi 5 - I2C address 0x4A
 * Protocol: Sensor Hub Transport Protocol (SHTP)
 */

#ifndef BNO080_H
#define BNO080_H

#include <stdint.h>
#include <stdbool.h>

/* ---------- I2C / device config ---------- */

#define BNO080_DEFAULT_ADDR   0x4A
#define BNO080_ALT_ADDR       0x4B
#define BNO080_MAX_PACKET     512
#define BNO080_MAX_PAYLOAD    (BNO080_MAX_PACKET - 4)

/* ---------- SHTP channels ---------- */

#define CHAN_COMMAND           0   /* device command */
#define CHAN_EXECUTABLE        1   /* executable / DFU */
#define CHAN_CONTROL           2   /* sensor hub control */
#define CHAN_INPUT_REPORT      3   /* sensor input reports */
#define CHAN_WAKE_REPORT       4   /* wake input reports */
#define CHAN_GYRO_ROTATION     5   /* gyro rotation vector */

/* ---------- Report IDs ---------- */

/* Sensor reports (input) */
#define REPORT_ACCELEROMETER          0x01
#define REPORT_GYROSCOPE              0x02
#define REPORT_MAGNETIC_FIELD         0x03
#define REPORT_LINEAR_ACCELERATION    0x04
#define REPORT_ROTATION_VECTOR        0x05
#define REPORT_GRAVITY                0x06
#define REPORT_UNCAL_GYROSCOPE        0x07
#define REPORT_GAME_ROTATION_VECTOR   0x08
#define REPORT_GEOMAG_ROTATION_VECTOR 0x09
#define REPORT_TAP_DETECTOR           0x10
#define REPORT_STEP_COUNTER           0x11
#define REPORT_STABILITY_CLASSIFIER   0x13
#define REPORT_RAW_ACCELEROMETER      0x14
#define REPORT_RAW_GYROSCOPE          0x15
#define REPORT_RAW_MAGNETOMETER       0x16
#define REPORT_STEP_DETECTOR          0x18
#define REPORT_SHAKE_DETECTOR         0x19

/* Control reports */
#define CMD_SET_FEATURE           0xFD
#define CMD_GET_FEATURE_RESP      0xFC
#define CMD_PRODUCT_ID_REQ        0xF9
#define CMD_PRODUCT_ID_RESP       0xF8
#define CMD_TIMESTAMP_REBASE      0xFA
#define CMD_BASE_TIMESTAMP        0xFB
#define CMD_FRS_WRITE_REQ         0xF7
#define CMD_FRS_WRITE_DATA        0xF6
#define CMD_FRS_WRITE_RESP        0xF5
#define CMD_FRS_READ_REQ          0xF4
#define CMD_FRS_READ_RESP         0xF3
#define CMD_COMMAND_REQ           0xF2
#define CMD_COMMAND_RESP          0xF1

/* Command IDs (inside CMD_COMMAND_REQ) */
#define COMMAND_ME_CALIBRATE      7    /* configure ME calibration */
#define COMMAND_SAVE_DCD          6    /* save dynamic calibration data */

/* Stability classifications */
#define STABILITY_UNKNOWN         0
#define STABILITY_ON_TABLE        1
#define STABILITY_STATIONARY      2
#define STABILITY_STABLE          3
#define STABILITY_MOTION          4
#define STABILITY_RESERVED        5

/* Calibration accuracy levels (from status byte bits 0-1) */
#define CAL_UNRELIABLE            0
#define CAL_LOW                   1
#define CAL_MEDIUM                2
#define CAL_HIGH                  3

/* ---------- Data types ---------- */

typedef struct {
    float x, y, z, w;
    float accuracy;         /* radians, for rotation vectors */
} bno080_quat_t;

typedef struct {
    float x, y, z;
    float accuracy;
} bno080_vec3_t;

typedef struct {
    uint8_t  sw_major;
    uint8_t  sw_minor;
    uint32_t sw_patch;
    uint32_t sw_build;
    uint8_t  sw_part;
} bno080_product_id_t;

typedef struct {
    int      fd;            /* i2c file descriptor */
    uint8_t  addr;
    uint8_t  seq[6];        /* per-channel SHTP sequence numbers */
    uint8_t  cmd_seq;       /* command request sequence (for response matching) */

    /* receive buffer */
    uint8_t  rx_buf[BNO080_MAX_PACKET];
    uint16_t rx_len;
    uint8_t  rx_chan;

    /* latest sensor data */
    bno080_quat_t rotation_vector;
    bno080_quat_t game_rotation_vector;
    bno080_quat_t geomag_rotation_vector;
    bno080_vec3_t accelerometer;
    bno080_vec3_t linear_acceleration;
    bno080_vec3_t gyroscope;
    bno080_vec3_t magnetometer;
    bno080_vec3_t gravity;
    uint8_t       stability;
    uint16_t      step_count;
    bool          step_detected;
    bool          tap_detected;
    bool          shake_detected;

    /* calibration status (0-3) per sensor, from report status byte */
    uint8_t       cal_accel;
    uint8_t       cal_gyro;
    uint8_t       cal_mag;
    uint8_t       cal_rv;           /* rotation vector overall */

    /* sensor internal timestamp (microseconds) */
    uint32_t      last_timestamp_us;
    uint32_t      last_latency_us;
} bno080_t;

/* ---------- API ---------- */

/**
 * Open and initialise a BNO080 on the given I2C bus.
 * Returns 0 on success, -1 on error (check errno).
 */
int  bno080_open(bno080_t *dev, const char *i2c_bus, uint8_t addr);

/**
 * Close the I2C file descriptor.
 */
void bno080_close(bno080_t *dev);

/**
 * Soft-reset the sensor and wait for the advertisement packet.
 */
int  bno080_reset(bno080_t *dev);

/**
 * Request and print product ID info.
 */
int  bno080_get_product_id(bno080_t *dev, bno080_product_id_t *id);

/**
 * Enable a sensor report at the given interval (microseconds).
 * Common intervals:  10000 = 100 Hz,  20000 = 50 Hz,  100000 = 10 Hz
 */
int  bno080_enable_report(bno080_t *dev, uint8_t report_id,
                          uint32_t interval_us);

/**
 * Disable a sensor report.
 */
int  bno080_disable_report(bno080_t *dev, uint8_t report_id);

/**
 * Read one SHTP packet. Returns the number of data bytes (excluding
 * 4-byte header) or -1 on error. Populates dev->rx_buf, rx_len, rx_chan.
 * Timeout in milliseconds (0 = non-blocking).
 */
int  bno080_receive(bno080_t *dev);

/**
 * Parse a received input report and store the data in dev.
 * Returns the report ID or -1 if the packet wasn't a sensor report.
 */
int  bno080_parse_input_report(bno080_t *dev);

/**
 * Convenience: receive + parse in a loop until a sensor report
 * arrives or the timeout expires. Returns report ID or -1.
 */
int  bno080_poll(bno080_t *dev, int timeout_ms);

/**
 * Save dynamic calibration data to the BNO080's flash.
 * Call after calibration is complete so it persists across reboots.
 */
int  bno080_save_dcd(bno080_t *dev);

/**
 * Configure which sensors participate in ME calibration.
 * Pass true/false for each. Typical: accel=true, gyro=true, mag=true.
 */
int  bno080_calibrate(bno080_t *dev, bool accel, bool gyro, bool mag);

/* ---------- Helpers ---------- */

const char *bno080_report_name(uint8_t report_id);
const char *bno080_stability_name(uint8_t classification);

#endif /* BNO080_H */

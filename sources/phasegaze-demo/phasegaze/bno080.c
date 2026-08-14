/*
 * bno080.c - BNO080 IMU driver over I2C (SHTP protocol)
 */

#include "bno080.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* ------------------------------------------------------------------ */
/*  Low-level I2C / SHTP                                              */
/* ------------------------------------------------------------------ */

static int shtp_write(bno080_t *dev, uint8_t channel,
                      const uint8_t *payload, uint16_t len)
{
    uint8_t buf[BNO080_MAX_PACKET];
    uint16_t total = len + 4;           /* 4-byte SHTP header */

    if (total > BNO080_MAX_PACKET) return -1;

    buf[0] = total & 0xFF;              /* length LSB */
    buf[1] = (total >> 8) & 0x7F;       /* length MSB, continuation=0 */
    buf[2] = channel;
    buf[3] = dev->seq[channel]++;

    if (len > 0)
        memcpy(&buf[4], payload, len);

    if (write(dev->fd, buf, total) != (ssize_t)total) {
        perror("bno080: i2c write");
        return -1;
    }
    return 0;
}

static int shtp_read(bno080_t *dev)
{
    /*
     * BNO080 over I2C: a single read returns the entire SHTP packet
     * (header + payload).  Read the max size and parse the header
     * to determine actual length.
     */
    memset(dev->rx_buf, 0, sizeof(dev->rx_buf));
    ssize_t n = read(dev->fd, dev->rx_buf, BNO080_MAX_PACKET);
    if (n < 4) {
        return -1;  /* no data or error */
    }

    uint16_t pkt_len = (uint16_t)(dev->rx_buf[0]) |
                       ((uint16_t)(dev->rx_buf[1] & 0x7F) << 8);

    if (pkt_len == 0 || pkt_len == 0x7FFF) {
        return 0;   /* no data available */
    }

    /* clamp to what we actually received */
    if (pkt_len > (uint16_t)n)
        pkt_len = (uint16_t)n;

    dev->rx_len  = pkt_len > 4 ? pkt_len - 4 : 0;
    dev->rx_chan = dev->rx_buf[2];

    return dev->rx_len;
}

static void msleep(int ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------------ */
/*  Q-point conversions                                               */
/* ------------------------------------------------------------------ */

static float q_to_float(int16_t val, int q)
{
    return (float)val / (float)(1 << q);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

int bno080_open(bno080_t *dev, const char *i2c_bus, uint8_t addr)
{
    memset(dev, 0, sizeof(*dev));
    dev->addr = addr;

    dev->fd = open(i2c_bus, O_RDWR);
    if (dev->fd < 0) {
        perror("bno080: open i2c bus");
        return -1;
    }

    if (ioctl(dev->fd, I2C_SLAVE, addr) < 0) {
        perror("bno080: set i2c address");
        close(dev->fd);
        return -1;
    }

    printf("bno080: opened %s addr 0x%02X (fd=%d)\n", i2c_bus, addr, dev->fd);
    return 0;
}

void bno080_close(bno080_t *dev)
{
    if (dev->fd >= 0) {
        close(dev->fd);
        dev->fd = -1;
    }
}

int bno080_reset(bno080_t *dev)
{
    printf("bno080: waiting for device to come up...\n");

    /*
     * After power-on or reset, the BNO080 sends an advertisement
     * on channel 0 and an "unsolicited initialisation" on channel 1.
     * We drain those packets.
     */
    for (int attempt = 0; attempt < 50; attempt++) {
        int n = shtp_read(dev);
        if (n > 0 && dev->rx_chan == CHAN_EXECUTABLE) {
            /*
             * Channel 1 "executable" with 0x01 means "reset complete".
             */
            if (dev->rx_buf[4] == 0x01) {
                printf("bno080: device reset complete\n");
                return 0;
            }
        }
        if (n > 0 && dev->rx_chan == CHAN_COMMAND) {
            printf("bno080: received advertisement (%d bytes)\n", n);
        }
        msleep(50);
    }

    /*
     * Even if we didn't see a clean reset message, the device may
     * still be ready. Try to proceed.
     */
    printf("bno080: no explicit reset response, proceeding anyway\n");
    return 0;
}

int bno080_get_product_id(bno080_t *dev, bno080_product_id_t *id)
{
    /* Send Product ID Request on channel 2 */
    uint8_t cmd[2] = { CMD_PRODUCT_ID_REQ, 0x00 };
    if (shtp_write(dev, CHAN_CONTROL, cmd, 2) < 0)
        return -1;

    /* Wait for response */
    for (int attempt = 0; attempt < 50; attempt++) {
        msleep(20);
        int n = shtp_read(dev);
        if (n < 0) continue;

        if (dev->rx_chan == CHAN_CONTROL && dev->rx_buf[4] == CMD_PRODUCT_ID_RESP) {
            uint8_t *d = &dev->rx_buf[4]; /* skip SHTP header */
            id->sw_major  = d[2];
            id->sw_minor  = d[3];
            id->sw_patch  = (uint32_t)d[12] | ((uint32_t)d[13] << 8);
            id->sw_build  = (uint32_t)d[8] | ((uint32_t)d[9] << 8) |
                            ((uint32_t)d[10] << 16) | ((uint32_t)d[11] << 24);
            id->sw_part   = d[4];
            return 0;
        }
    }

    fprintf(stderr, "bno080: product ID response timeout\n");
    return -1;
}

int bno080_enable_report(bno080_t *dev, uint8_t report_id,
                         uint32_t interval_us)
{
    uint8_t cmd[17];
    memset(cmd, 0, sizeof(cmd));

    cmd[0]  = CMD_SET_FEATURE;          /* Set Feature Command */
    cmd[1]  = report_id;
    /* bytes 2-4: feature flags, change sensitivity — leave as 0 */
    cmd[5]  = (interval_us      ) & 0xFF;
    cmd[6]  = (interval_us >>  8) & 0xFF;
    cmd[7]  = (interval_us >> 16) & 0xFF;
    cmd[8]  = (interval_us >> 24) & 0xFF;
    /* bytes 9-12: batch interval — leave as 0 */
    /* bytes 13-16: sensor specific config — leave as 0 */

    printf("bno080: enabling %-28s (report 0x%02X) at %u us interval\n",
           bno080_report_name(report_id), report_id, interval_us);

    return shtp_write(dev, CHAN_CONTROL, cmd, sizeof(cmd));
}

int bno080_disable_report(bno080_t *dev, uint8_t report_id)
{
    printf("bno080: disabling %-28s (report 0x%02X)\n",
           bno080_report_name(report_id), report_id);

    uint8_t cmd[17];
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = CMD_SET_FEATURE;
    cmd[1] = report_id;
    /* interval = 0 means disable; all other bytes already 0 */

    return shtp_write(dev, CHAN_CONTROL, cmd, sizeof(cmd));
}

int bno080_receive(bno080_t *dev)
{
    return shtp_read(dev);
}

int bno080_parse_input_report(bno080_t *dev)
{
    if (dev->rx_chan != CHAN_INPUT_REPORT &&
        dev->rx_chan != CHAN_WAKE_REPORT  &&
        dev->rx_chan != CHAN_GYRO_ROTATION)
        return -1;

    if (dev->rx_len < 1) return -1;

    /*
     * Input report format (after SHTP header):
     *   byte 0: report ID (0xFB = timestamp base, then sensor reports follow)
     *
     * A typical packet has:
     *   [timestamp_rebase (5 bytes)] [report1] [report2] ...
     */
    uint8_t *data = &dev->rx_buf[4];    /* skip 4-byte SHTP header */
    uint16_t remaining = dev->rx_len;
    int last_report = -1;

    while (remaining > 0) {
        uint8_t id = data[0];

        /* skip timestamp rebase record */
        if (id == CMD_TIMESTAMP_REBASE) {
            if (remaining < 5) break;
            dev->last_timestamp_us = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                                     ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
            data += 5;
            remaining -= 5;
            continue;
        }
        if (id == CMD_BASE_TIMESTAMP) {
            if (remaining < 5) break;
            dev->last_timestamp_us = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                                     ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
            data += 5;
            remaining -= 5;
            continue;
        }

        last_report = id;

        switch (id) {
        case REPORT_ACCELEROMETER: {
            if (remaining < 10) goto done;
            dev->cal_accel = data[2] & 0x03;
            int16_t x = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            int16_t y = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            int16_t z = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
            dev->accelerometer.x = q_to_float(x, 8);
            dev->accelerometer.y = q_to_float(y, 8);
            dev->accelerometer.z = q_to_float(z, 8);
            data += 10; remaining -= 10;
            break;
        }
        case REPORT_GYROSCOPE: {
            if (remaining < 10) goto done;
            dev->cal_gyro = data[2] & 0x03;
            int16_t x = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            int16_t y = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            int16_t z = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
            dev->gyroscope.x = q_to_float(x, 9);
            dev->gyroscope.y = q_to_float(y, 9);
            dev->gyroscope.z = q_to_float(z, 9);
            data += 10; remaining -= 10;
            break;
        }
        case REPORT_MAGNETIC_FIELD: {
            if (remaining < 10) goto done;
            dev->cal_mag = data[2] & 0x03;
            int16_t x = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            int16_t y = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            int16_t z = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
            dev->magnetometer.x = q_to_float(x, 4);
            dev->magnetometer.y = q_to_float(y, 4);
            dev->magnetometer.z = q_to_float(z, 4);
            data += 10; remaining -= 10;
            break;
        }
        case REPORT_LINEAR_ACCELERATION: {
            if (remaining < 10) goto done;
            int16_t x = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            int16_t y = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            int16_t z = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
            dev->linear_acceleration.x = q_to_float(x, 8);
            dev->linear_acceleration.y = q_to_float(y, 8);
            dev->linear_acceleration.z = q_to_float(z, 8);
            data += 10; remaining -= 10;
            break;
        }
        case REPORT_ROTATION_VECTOR: {
            if (remaining < 14) goto done;
            dev->cal_rv = data[2] & 0x03;
            dev->last_latency_us = (uint32_t)data[3]; // often 0, but can be latency
            int16_t i = (int16_t)((uint16_t)data[4]  | ((uint16_t)data[5]  << 8));
            int16_t j = (int16_t)((uint16_t)data[6]  | ((uint16_t)data[7]  << 8));
            int16_t k = (int16_t)((uint16_t)data[8]  | ((uint16_t)data[9]  << 8));
            int16_t r = (int16_t)((uint16_t)data[10] | ((uint16_t)data[11] << 8));
            uint16_t a = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
            dev->rotation_vector.x = q_to_float(i, 14);
            dev->rotation_vector.y = q_to_float(j, 14);
            dev->rotation_vector.z = q_to_float(k, 14);
            dev->rotation_vector.w = q_to_float(r, 14);
            dev->rotation_vector.accuracy = (float)a / (float)(1 << 12);  /* unsigned Q12 */
            data += 14; remaining -= 14;
            break;
        }
        case REPORT_GRAVITY: {
            if (remaining < 10) goto done;
            int16_t x = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            int16_t y = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
            int16_t z = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
            dev->gravity.x = q_to_float(x, 8);
            dev->gravity.y = q_to_float(y, 8);
            dev->gravity.z = q_to_float(z, 8);
            data += 10; remaining -= 10;
            break;
        }
        case REPORT_GAME_ROTATION_VECTOR: {
            if (remaining < 12) goto done;
            int16_t i = (int16_t)((uint16_t)data[4]  | ((uint16_t)data[5]  << 8));
            int16_t j = (int16_t)((uint16_t)data[6]  | ((uint16_t)data[7]  << 8));
            int16_t k = (int16_t)((uint16_t)data[8]  | ((uint16_t)data[9]  << 8));
            int16_t r = (int16_t)((uint16_t)data[10] | ((uint16_t)data[11] << 8));
            dev->game_rotation_vector.x = q_to_float(i, 14);
            dev->game_rotation_vector.y = q_to_float(j, 14);
            dev->game_rotation_vector.z = q_to_float(k, 14);
            dev->game_rotation_vector.w = q_to_float(r, 14);
            data += 12; remaining -= 12;
            break;
        }
        case REPORT_GEOMAG_ROTATION_VECTOR: {
            if (remaining < 14) goto done;
            int16_t i = (int16_t)((uint16_t)data[4]  | ((uint16_t)data[5]  << 8));
            int16_t j = (int16_t)((uint16_t)data[6]  | ((uint16_t)data[7]  << 8));
            int16_t k = (int16_t)((uint16_t)data[8]  | ((uint16_t)data[9]  << 8));
            int16_t r = (int16_t)((uint16_t)data[10] | ((uint16_t)data[11] << 8));
            int16_t a = (int16_t)((uint16_t)data[12] | ((uint16_t)data[13] << 8));
            dev->geomag_rotation_vector.x = q_to_float(i, 14);
            dev->geomag_rotation_vector.y = q_to_float(j, 14);
            dev->geomag_rotation_vector.z = q_to_float(k, 14);
            dev->geomag_rotation_vector.w = q_to_float(r, 14);
            dev->geomag_rotation_vector.accuracy = q_to_float(a, 12);
            data += 14; remaining -= 14;
            break;
        }
        case REPORT_STABILITY_CLASSIFIER: {
            if (remaining < 6) goto done;
            dev->stability = data[4];
            data += 6; remaining -= 6;
            break;
        }
        case REPORT_STEP_COUNTER: {
            if (remaining < 12) goto done;
            dev->step_count = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
            data += 12; remaining -= 12;
            break;
        }
        case REPORT_STEP_DETECTOR: {
            if (remaining < 8) goto done;
            dev->step_detected = true;
            data += 8; remaining -= 8;
            break;
        }
        case REPORT_TAP_DETECTOR: {
            if (remaining < 6) goto done;
            dev->tap_detected = true;
            data += 6; remaining -= 6;
            break;
        }
        case REPORT_SHAKE_DETECTOR: {
            if (remaining < 6) goto done;
            dev->shake_detected = true;
            data += 6; remaining -= 6;
            break;
        }
        default:
            /* unknown report; skip to end */
            goto done;
        }
    }

done:
    return last_report;
}

int bno080_poll(bno080_t *dev, int timeout_ms)
{
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        int n = bno080_receive(dev);
        if (n > 0) {
            int id = bno080_parse_input_report(dev);
            if (id >= 0) return id;
        }
        msleep(2);
        elapsed += 2;
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/*  Calibration commands                                              */
/* ------------------------------------------------------------------ */

int bno080_save_dcd(bno080_t *dev)
{
    /*
     * Send CMD_COMMAND_REQ with command SAVE_DCD.
     * Format: 12-byte command (report ID + command + padding).
     */
    uint8_t cmd[12];
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = CMD_COMMAND_REQ;
    cmd[1] = dev->cmd_seq++;  /* command sequence */
    cmd[2] = COMMAND_SAVE_DCD;
    /* bytes 3-11: all zeros */

    printf("bno080: saving dynamic calibration data...\n");
    if (shtp_write(dev, CHAN_CONTROL, cmd, sizeof(cmd)) < 0)
        return -1;

    /* Wait for response */
    for (int attempt = 0; attempt < 50; attempt++) {
        msleep(20);
        int n = shtp_read(dev);
        if (n < 0) continue;
        if (dev->rx_chan == CHAN_CONTROL && dev->rx_buf[4] == CMD_COMMAND_RESP) {
            uint8_t resp_cmd = dev->rx_buf[6];
            if (resp_cmd == COMMAND_SAVE_DCD) {
                uint8_t status = dev->rx_buf[9];
                if (status == 0) {
                    printf("bno080: DCD saved successfully\n");
                    return 0;
                } else {
                    fprintf(stderr, "bno080: DCD save failed (status=%u)\n", status);
                    return -1;
                }
            }
        }
    }
    fprintf(stderr, "bno080: DCD save response timeout\n");
    return -1;
}

int bno080_calibrate(bno080_t *dev, bool accel, bool gyro, bool mag)
{
    uint8_t cmd[12];
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = CMD_COMMAND_REQ;
    cmd[1] = dev->cmd_seq++;
    cmd[2] = COMMAND_ME_CALIBRATE;
    cmd[3] = accel ? 1 : 0;   /* accel cal enable */
    cmd[4] = gyro  ? 1 : 0;   /* gyro cal enable */
    cmd[5] = mag   ? 1 : 0;   /* mag cal enable */
    /* bytes 6-11: reserved, set to 0 */

    printf("bno080: configuring calibration: accel=%d gyro=%d mag=%d\n",
           accel, gyro, mag);
    return shtp_write(dev, CHAN_CONTROL, cmd, sizeof(cmd));
}

/* ------------------------------------------------------------------ */
/*  Name helpers                                                      */
/* ------------------------------------------------------------------ */

const char *bno080_report_name(uint8_t report_id)
{
    switch (report_id) {
    case REPORT_ACCELEROMETER:          return "Accelerometer";
    case REPORT_GYROSCOPE:              return "Gyroscope";
    case REPORT_MAGNETIC_FIELD:         return "Magnetic Field";
    case REPORT_LINEAR_ACCELERATION:    return "Linear Acceleration";
    case REPORT_ROTATION_VECTOR:        return "Rotation Vector";
    case REPORT_GRAVITY:                return "Gravity";
    case REPORT_UNCAL_GYROSCOPE:        return "Uncalibrated Gyroscope";
    case REPORT_GAME_ROTATION_VECTOR:   return "Game Rotation Vector";
    case REPORT_GEOMAG_ROTATION_VECTOR: return "Geomagnetic Rotation Vec";
    case REPORT_TAP_DETECTOR:           return "Tap Detector";
    case REPORT_STEP_COUNTER:           return "Step Counter";
    case REPORT_STABILITY_CLASSIFIER:   return "Stability Classifier";
    case REPORT_RAW_ACCELEROMETER:      return "Raw Accelerometer";
    case REPORT_RAW_GYROSCOPE:          return "Raw Gyroscope";
    case REPORT_RAW_MAGNETOMETER:       return "Raw Magnetometer";
    case REPORT_STEP_DETECTOR:          return "Step Detector";
    case REPORT_SHAKE_DETECTOR:         return "Shake Detector";
    default:                            return "Unknown";
    }
}

const char *bno080_stability_name(uint8_t classification)
{
    switch (classification) {
    case STABILITY_UNKNOWN:    return "Unknown";
    case STABILITY_ON_TABLE:   return "On Table";
    case STABILITY_STATIONARY: return "Stationary";
    case STABILITY_STABLE:     return "Stable";
    case STABILITY_MOTION:     return "In Motion";
    default:                   return "Reserved";
    }
}

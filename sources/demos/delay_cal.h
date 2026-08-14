// delay_cal.h
// Constant per-antenna phase offsets for AR vision (antennas 1-3 vs 0).
// Source: mean of 10 boresight delay-cal samples at 5180-5885 MHz.
// eps10=0.497804, eps20=-0.910468, eps30=-1.408272 rad
// Applied in time domain (pre-FFT) via precomputed exp(-j*eps) rotators.

#ifndef DELAY_CAL_H
#define DELAY_CAL_H

typedef struct {
    float c;
    float s;
} delay_cal_rot_t;

static const delay_cal_rot_t DELAY_CAL_ROT[3] = {
    { 0.8786332874f,  0.4774971689f },  // ch1
    { 0.6133759260f, -0.7897910948f },  // ch2
    { 0.1618094945f, -0.9868220141f },  // ch3
};

static inline void delay_cal_rotate_buf(float (*buf)[2], int n, int ch_idx)
{
    const float c = DELAY_CAL_ROT[ch_idx].c;
    const float s = DELAY_CAL_ROT[ch_idx].s;
    for (int i = 0; i < n; ++i) {
        float r = buf[i][0];
        float m = buf[i][1];
        buf[i][0] = r * c + m * s;
        buf[i][1] = m * c - r * s;
    }
}

#endif // DELAY_CAL_H

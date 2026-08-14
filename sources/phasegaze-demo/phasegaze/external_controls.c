// external_controls.c
// Implementation of the External Controls Manager.

#include "external_controls.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>

#define RF_GAIN_MIN  0
#define RF_GAIN_MAX  63
#define BW_K_MIN     5
#define BW_K_MAX     63

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Shells out to the jtag CLI. Blocks briefly; only called on user interaction
// or once at startup, so the cost is fine.
static void jtag_call(const char *args)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), SDR_JTAG_CLI " %s", args);
    int r = system(cmd);
    if (r != 0)
        fprintf(stderr, "Warning: jtag call returned %d: %s\n", r, cmd);
}

void external_controls_lockdown(int initial_gain_db, int initial_bw_k)
{
    int gain = clampi(initial_gain_db, RF_GAIN_MIN, RF_GAIN_MAX);
    int k    = clampi(initial_bw_k,    BW_K_MIN,    BW_K_MAX);
    float bw_mhz = 240.0f / (float)k;

    // One combined --rx command so the chip is reprogrammed in a single JTAG
    // transaction: 4-antenna interleaved, no internal test tone, manual gain
    // (which disables AGC), and the desired digital BW.
    char args[160];
    snprintf(args, sizeof(args),
             "--rx antennas=15,interleave=1,tone_en=0,gain=%d,bw=%.3f",
             gain, (double)bw_mhz);
    jtag_call(args);

    // Force TX into standby.
    jtag_call("--tx off");
}

void external_controls_set_rf_gain(int gain_db)
{
    int g = clampi(gain_db, RF_GAIN_MIN, RF_GAIN_MAX);
    char args[64];
    snprintf(args, sizeof(args), "--rx gain=%d", g);
    jtag_call(args);
}

void external_controls_set_filter_divider(int k)
{
    int kc = clampi(k, BW_K_MIN, BW_K_MAX);
    float bw_mhz = 240.0f / (float)kc;
    char args[64];
    snprintf(args, sizeof(args), "--rx bw=%.3f", (double)bw_mhz);
    jtag_call(args);
}

float external_controls_bw_mhz_for_k(int k)
{
    int kc = clampi(k, BW_K_MIN, BW_K_MAX);
    return 240.0f / (float)kc;
}

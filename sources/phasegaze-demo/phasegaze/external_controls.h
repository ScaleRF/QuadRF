// external_controls.h
// External Controls Manager — owns every JTAG CLI invocation that originates
// from csi_sweep. Talks to the same quadrf-jtag binary the Flask
// web UI uses, so this is just a second front-end to the same radio plumbing.
//
// At sweep startup, lockdown() pins the radio into a known-good state for
// CSI capture (AGC off, TX off, 4-antenna interleaved). Mid-run, the GUI
// sliders only touch RX RF gain and the digital filter divider k.

#ifndef EXTERNAL_CONTROLS_H
#define EXTERNAL_CONTROLS_H

// Pin the radio into a known-good state for CSI sweep:
//   --rx antennas=15,interleave=1,tone_en=0,gain=<initial_gain_db>,bw=<240/initial_bw_k>
//   --tx off
// Setting gain=N implicitly disables AGC (manual gain and AGC are mutually
// exclusive on the chip; there's no separate "agc=off" verb in the CLI).
//
// One-shot: if the web UI later flips AGC/TX/antennas, csi_sweep does not
// re-assert. initial_gain_db is clamped to [0,63]; initial_bw_k to [5,63].
void external_controls_lockdown(int initial_gain_db, int initial_bw_k);

// Manual RF gain in dB. Clamped to [0, 63].
void external_controls_set_rf_gain(int gain_db);

// Filter divider k. Clamped to [5, 63]. The chip's actual digital bandwidth
// will be 240/k MHz (rounded by jtag).
void external_controls_set_filter_divider(int k);

// Convenience: 240.0 / k, with k clamped to [5, 63] to avoid div-by-zero
// nonsense. Used by the GUI to render the bw readout.
float external_controls_bw_mhz_for_k(int k);

#endif // EXTERNAL_CONTROLS_H

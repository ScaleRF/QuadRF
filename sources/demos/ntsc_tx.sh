#!/bin/bash
# Camera Encoder: FM-modulate NTSC and transmit it on analog 5.8 GHz FPV.
#
# Default is a built-in color-bar still on Raceband 5. Override with
# QUADRF_NTSC_TX_INPUT (image or mp4), QUADRF_NTSC_TX_PATTERN, QUADRF_NTSC_TX_CH,
# QUADRF_NTSC_TX_GAIN (0..63, keep <= 25 for close-range tests).

CH="${QUADRF_NTSC_TX_CH:-R5}"
GAIN="${QUADRF_NTSC_TX_GAIN:-20}"
PATTERN="${QUADRF_NTSC_TX_PATTERN:-bars}"
INPUT="${QUADRF_NTSC_TX_INPUT:-}"
FREQ_MHZ=5806
case "$CH" in
  R1) FREQ_MHZ=5658 ;;
  R2) FREQ_MHZ=5695 ;;
  R3) FREQ_MHZ=5732 ;;
  R4) FREQ_MHZ=5769 ;;
  R5) FREQ_MHZ=5806 ;;
  R6) FREQ_MHZ=5843 ;;
  R7) FREQ_MHZ=5880 ;;
  R8) FREQ_MHZ=5917 ;;
esac

cleanup() {
  trap - EXIT INT TERM
  quadrf-jtag --tx off >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

quadrf-jtag --tx antennas=15,bw=20,freq=$FREQ_MHZ,gain=$GAIN

mod_args=(--ch "$CH" --gain "$GAIN")
if [ -n "$INPUT" ]; then
  mod_args+=(--input "$INPUT")
else
  mod_args+=(--pattern "$PATTERN")
fi

quadrf-ntsc-mod "${mod_args[@]}"

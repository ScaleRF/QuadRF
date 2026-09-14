#!/bin/bash
# Camera Decoder: receive an NTSC signal off the front-end and play it.
#
# Pick mpv's video output. In a graphical session (DISPLAY set, e.g. launched
# from the KasmVNC desktop) draw into that session even when an HDMI monitor is
# plugged in: vc4-kms and KasmVNC already hold DRM, so mpv cannot become DRM
# master and --vo=drm fails. Only fall back to direct KMS/DRM from a bare
# console with a monitor attached.
MPV_VO_FLAG="--vo=x11"
if [ -z "${DISPLAY:-}" ]; then
  for status_file in /sys/class/drm/*HDMI*/status; do
    if [ -f "$status_file" ] && grep -q "^connected$" "$status_file" 2>/dev/null; then
      MPV_VO_FLAG="--vo=drm"
      break
    fi
  done
fi

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

# Prefer the sibling script for safe testing from an extracted release; an
# installed launcher still falls back to the system copy.
CH_LUA="$SCRIPT_DIR/ntsc_ch.lua"
if [ ! -f "$CH_LUA" ]; then
  CH_LUA=/usr/share/quadrf/ntsc_ch.lua
fi

# Prefer a freshly built sibling binary for safe A/B testing. Set
# QUADRF_NTSC_DEMOD to override it, or fall back to the installed executable.
DEMOD_BIN="${QUADRF_NTSC_DEMOD:-quadrf-ntsc-demod}"
if [ -z "${QUADRF_NTSC_DEMOD:-}" ] && [ -x "$SCRIPT_DIR/quadrf-ntsc-demod" ]; then
  DEMOD_BIN="$SCRIPT_DIR/quadrf-ntsc-demod"
fi

demod_pid=""
cleanup() {
  trap - EXIT INT TERM
  rm -f /dev/shm/quadrf-ntsc-status* /tmp/quadrf-ntsc-status* \
    /dev/shm/quadrf-ntsc-ctrl* /tmp/quadrf-ntsc-ctrl* \
    /dev/shm/quadrf-ntsc-tune* /tmp/quadrf-ntsc-tune* 2>/dev/null || true
  if [ -n "$demod_pid" ]; then
    kill "$demod_pid" 2>/dev/null || true
    wait "$demod_pid" 2>/dev/null || true
    demod_pid=""
  fi
}
trap cleanup EXIT INT TERM

# Remove a stale request before the decoder starts. The decoder itself owns
# initial and live frequency changes through its one SoapySDR device handle.
rm -f /dev/shm/quadrf-ntsc-tune* /tmp/quadrf-ntsc-tune* 2>/dev/null || true
quadrf-jtag --rx autosteer=1,antennas=15,interleave=0,tone_en=0,bw=12.0,agc=-14.0

# Wait for mpv only. q closes the window; SIGPIPE does not reach
# quadrf-ntsc-demod if it is blocked in SoapySDR::readStream, so a
# shell pipeline would leave the decoder holding CSI.
exec {video_fd}< <(exec "$DEMOD_BIN" --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --sat 1.0 "$@")
demod_pid=$!

# Raw-video timestamps are synthetic. During RF acquisition, frames arrive
# sparsely; mpv's normal late-frame policy can then keep dropping good live
# frames after lock because the source cannot run faster than real time to
# catch up. With no audio clock to follow, display every frame that arrives.
mpv --profile=low-latency --no-cache --audio=no --framedrop=no \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=59.94 \
  --script="$CH_LUA" --osd-font-size=40 --osd-duration=1500 \
  --input-ipc-server=/tmp/quadrf-ntsc-mpv \
  $MPV_VO_FLAG - <&$video_fd
exec {video_fd}<&-

# quadrf-ntsc-demod writes raw yuyv422 640x480 on stdout, so it can just as
# easily be redirected to a file or piped into ffmpeg for streaming.

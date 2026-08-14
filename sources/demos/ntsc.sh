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

quadrf-jtag --rx autosteer=1,antennas=1,interleave=0,tone_en=0,bw=12.0,agc=-14.0

quadrf-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --hsync_min 25 --hsync_max 160 --sat 3.0 --hue -3.0 \
| mpv --profile=low-latency --no-cache \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=60 $MPV_VO_FLAG -

# quadrf-ntsc-demod writes raw yuyv422 640x480 on stdout, so it can just as
# easily be redirected to a file or piped into ffmpeg for streaming.

sleep 3

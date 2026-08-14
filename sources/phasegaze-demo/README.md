# phasegaze

A spatial RF viewer: sweep the LO, extract phase gradients across the four
channels and paint the result onto a hemisphere.

`quadrf-demos` ships this tree as source under `/usr/share/quadrf/phasegaze`
rather than as a binary, since it needs a display with working OpenGL and is
usually edited before it is useful. Copy it into your home directory and build
it there.

Note that it builds a binary called `csi_sweep`, which is a different program
from `quadrf-rf-vision` in `sources/demos/`.

## Requirements

**Hardware**

- QuadRF: PGA CSI device exposed as `/dev/csi_stream0`
- (optional) BNO080 IMU (I2C)
- Display with OpenGL support

**Software**

```bash
sudo apt install build-essential libfftw3-dev libglfw3-dev libsdl2-dev libsdl2-ttf-dev
```

Device paths and sweep defaults live in `phasegaze/config.h`. Runtime settings are persisted in `phasegaze/settings.json`.

## Build & run

```bash
cd phasegaze
make          # release
make DEBUG=1  # debug
./csi_sweep
```

Two windows open: an OpenGL hemisphere view and an SDL2 control panel. Use the panel to set frequency sweep range, RF gain, point size/gain, decay, and intensity threshold. Snapshots and shutter recordings are saved as `.rfpic` files (with optional IMU metadata).

## Viewer

Open `viewer/merged_viewer.html` in a browser to inspect saved `.rfpic` point clouds. Load a file via the picker; optionally overlay an equirectangular panorama. To align: Ctrl+drag moves the image, Shift+drag rolls it.

## Layout

```
phasegaze/          Main application (builds csi_sweep)
  main.c            GLFW view + event loop
  worker.c          CSI read, FFT, phase-gradient extraction
  sphere_render.c   Hemisphere rendering
  control_window.c  SDL2 parameter UI
  imu_worker.c      BNO080 orientation
  snapshot.c        .rfpic export
  shutter_stream.c  Long-exposure disk streaming
viewer/
  merged_viewer.html  Offline .rfpic viewer
```

## Calibration utilities

Python helpers for phase calibration and artifact analysis:

```bash
pip install numpy pandas matplotlib scipy
python phasegaze/find_artifacts.py calibration_points.csv
python phasegaze/plot_delay_cal.py
```

Calibration data is stored in `phasegaze/phase_calibration.json`.

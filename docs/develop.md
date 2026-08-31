# Developing and Building Applications on QuadRF

Develop custom SDR applications on the appliance against installed libraries, or modify and rebuild the packaged demos and SoapySDR modules from source.

## Prerequisites

The standard `quadrf` install provides the build toolchain and header files:

- **Compiler toolchain**: `build-essential`, `cmake`, `pkg-config`
- **Development headers**: `quadrf-dev` provides `fpga_csi.h`, `Farrow.hpp`, `NEON.hpp`, and CMake package configuration (`find_package(QuadRF)`)
- **Example sources**: `quadrf-demos` provides reference sources under `/usr/share/quadrf/examples`

For applications requiring FFTW or SDL2 (such as `quadrf-rf-vision`, `quadrf-psd`, and `quadrf-nearfield`), install the development libraries:

```bash
sudo apt install -y libfftw3-dev libsdl2-dev
```

---

## 1. Quick Start: Build and Run from Source

To quickly compile and run a single C++ source file using `pkg-config` and `g++`:

```bash
mkdir -p ~/my-app && cd ~/my-app
cp /usr/share/quadrf/examples/hello.cpp ./main.cpp
g++ -O3 -std=c++17 main.cpp -o my-app $(pkg-config --cflags --libs SoapySDR)
./my-app
```

For applications reading composite video (such as NTSC demodulation), compile with ARM NEON optimizations and pipe the raw YUYV stream directly into `mpv`:

```bash
g++ -O3 -march=native -ffast-math -std=c++17 \
  /usr/share/quadrf/examples/ntsc_demod.cpp -o my-ntsc-demod \
  $(pkg-config --cflags --libs SoapySDR)

quadrf-jtag --rx autosteer=1,antennas=15,interleave=0,tone_en=0,bw=12.0,agc=-14.0,freq=5806

./my-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --hsync_min 25 --hsync_max 160 --sat 3.0 --hue -3.0 \
| mpv --profile=low-latency --no-cache \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=60 \
  --script=/usr/share/quadrf/ntsc_ch.lua --osd-font-size=40 --osd-duration=1500 \
  --vo=x11 -
```

---

## 2. Modifying and Building Packaged Examples

To create a project based on the packaged demo sources:

1. Copy the examples tree to your workspace:

   ```bash
   mkdir -p ~/my-sdr-project
   cp -r /usr/share/quadrf/examples/* ~/my-sdr-project/
   cd ~/my-sdr-project
   ```

2. Inspect or edit `CMakeLists.txt`:

   ```cmake
   cmake_minimum_required(VERSION 3.12)
   project(my_sdr_project C CXX)

   if(NOT CMAKE_BUILD_TYPE)
       set(CMAKE_BUILD_TYPE Release)
   endif()

   set(CMAKE_CXX_STANDARD 17)
   set(CMAKE_CXX_STANDARD_REQUIRED ON)

   add_compile_options(-O3)
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
       add_compile_options(-mcpu=cortex-a76 -mtune=cortex-a76)
   endif()

   find_package(QuadRF REQUIRED)
   find_package(PkgConfig REQUIRED)

   pkg_check_modules(FFTW3F REQUIRED fftw3f)
   pkg_check_modules(SDL2 REQUIRED sdl2)

   add_executable(my-ntsc-demod ntsc_demod.cpp)
   target_compile_options(my-ntsc-demod PRIVATE -ffast-math)
   target_link_libraries(my-ntsc-demod PRIVATE QuadRF::quadrf)
   ```

3. Build the executables:

   ```bash
   cmake -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j4
   ```

The compiled binaries will be placed in `build/`.

---

## 3. Creating an Application with Desktop and Web UI Launchers

To integrate a custom binary into the QuadRF environment (appearing on the remote desktop and the web control panel at `https://quadrf.local/`):

### Step 1: Install Binary and Launcher Script

Install your binary and create an executable startup script:

```bash
sudo cp build/my-ntsc-demod /usr/local/bin/my-ntsc-demod

sudo tee /usr/local/bin/my-camera << 'EOF' > /dev/null
#!/bin/bash
quadrf-jtag --rx autosteer=1,antennas=15,interleave=0,tone_en=0,bw=12.0,agc=-14.0,freq=5806
/usr/local/bin/my-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --hsync_min 25 --hsync_max 160 --sat 3.0 --hue -3.0 \
| mpv --profile=low-latency --no-cache \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=60 \
  --script=/usr/share/quadrf/ntsc_ch.lua --osd-font-size=40 --osd-duration=1500 \
  --input-ipc-server=/tmp/my-camera-mpv \
  --vo=x11 -
EOF

sudo chmod +x /usr/local/bin/my-camera
```

### Step 2: Register Desktop Entry

Create `/usr/share/applications/com.example.MyCamera.desktop`:

```ini
[Desktop Entry]
Type=Application
Version=1.0
Name=My Camera
Comment=Custom FPV Camera Receiver
Exec=/usr/local/bin/my-camera
TryExec=/usr/local/bin/my-camera
Icon=quadrf-video-decoder
Terminal=false
Categories=HamRadio;Science;
X-QuadRF-Desktop=true
```

Sync the desktop to publish the icon on the operator desktop:

```bash
sudo /usr/lib/quadrf/sync-desktop-apps
```

### Step 3: Register Systemd Service

Create `/etc/systemd/system/my-camera.service`:

```ini
[Unit]
Description=My Camera Receiver
Wants=load-quadrf.service quadrf-desktop.service
After=load-quadrf.service quadrf-desktop.service

[Service]
Type=simple
User=dietpi
Environment=DISPLAY=:1
Environment=SDL_VIDEODRIVER=x11
Environment=QT_QPA_PLATFORM=xcb
Environment=GDK_BACKEND=x11
ExecStart=/usr/local/bin/my-camera
TimeoutStopSec=8
Restart=no

[Install]
WantedBy=multi-user.target
```

Reload systemd daemon:

```bash
sudo systemctl daemon-reload
```

### Step 4: Register Web Control Launcher

Create `/usr/share/quadrf/apps.d/my-camera.json`:

```json
{
  "apps": [
    {
      "id": "mycamera",
      "desktop_entry": "com.example.MyCamera.desktop",
      "service": "my-camera.service",
      "binaries": ["my-camera", "my-ntsc-demod"],
      "exclusive": true,
      "open": "desktop"
    }
  ]
}
```

Verify the app registration from the command line:

```bash
sudo quadrf-app status
sudo quadrf-app start mycamera
sudo quadrf-app stop mycamera
```

To package this application as a `.deb` for distribution, see [Applications](applications.md).

---

## 4. Rebuilding SoapySDR Modules and Core Tree from Git

To modify the Farrow resampler, CSI interface, or SoapySDR modules (`mipi`, `quadrf`), build the repository directly on the appliance.

### Step 1: Clone Repository and Install Build Dependencies

```bash
git clone https://github.com/ScaleRF/QuadRF.git
cd QuadRF
sudo apt install -y device-tree-compiler libfftw3-dev libsdl2-dev libsoapysdr-dev libzmq3-dev
```

### Step 2: Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

Rebuilt SoapySDR modules are generated at:
- `build/sources/soapy/libmipi.so`
- `build/sources/quadrfd/soapy/quadrf.so`

### Step 3: Test Rebuilt Modules

Copy the rebuilt modules over the installed module directory:

```bash
sudo cp build/sources/soapy/libmipi.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/
sudo cp build/sources/quadrfd/soapy/quadrf.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/
SoapySDRUtil --probe="driver=mipi"
```

### Step 4: Revert to Packaged Modules

To revert all modules to the official packaged release:

```bash
sudo apt-get install --reinstall -y quadrf-soapy
SoapySDRUtil --probe="driver=mipi"
```

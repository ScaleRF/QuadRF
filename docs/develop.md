# Developing on the QuadRF

Modify demos, write custom apps, and rebuild modules from source.

## Prerequisites

The standard `quadrf` install provides the build toolchain and header files:

- **Compiler toolchain**: `build-essential`, `cmake`, `pkg-config`
- **Development headers**: `quadrf-dev` provides `fpga_csi.h`, `Farrow.hpp`, `NEON.hpp`, and CMake package configuration (`find_package(QuadRF)`)
- **Example sources**: `quadrf-demos` provides reference sources under `/usr/share/quadrf/examples`

For applications requiring FFTW or SDL2 (such as `quadrf-rf-vision`, `quadrf-psd`, and `quadrf-nearfield`), install the libraries:

```bash
sudo apt install -y libfftw3-dev libsdl2-dev
```

---

## 1. Build and Run from Source

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

## 2. Modify Demos

The `quadrf-demos` package installs example sources and a CMake project under
`/usr/share/quadrf/examples`. Copy that tree into a workspace you can edit:

```bash
mkdir -p ~/my-sdr-project
cp -r /usr/share/quadrf/examples/* ~/my-sdr-project/
cd ~/my-sdr-project
```

The copy includes a `CMakeLists.txt` that builds each example
(`quadrf-hello`, `quadrf-ntsc-demod`, `quadrf-rf-vision`, `quadrf-psd`,
`quadrf-nearfield`) against the installed `quadrf-dev` package. Edit the
`.c` / `.cpp` files, then build:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

Binaries land in `build/`. Edit `CMakeLists.txt` if you add a source
file, or change compiler flags. For a new program:

```cmake
add_executable(my-app my_app.cpp)
target_link_libraries(my-app PRIVATE QuadRF::quadrf)
```

---

## 3. Create an App with Desktop and Web UI Launchers

Four files put a binary on the remote desktop and on the Applications list at
`https://quadrf.local/`: the binary, a `.desktop` entry, a systemd unit, and a
JSON descriptor. This example registers a copy of the packaged PSD plot as
`my-psd`. If you built your own binary in the previous section, copy that
instead of `/usr/bin/quadrf-psd`.

### 1. Install the binary

```bash
sudo cp /usr/bin/quadrf-psd /usr/local/bin/my-psd
```

### 2. Desktop entry

```bash
sudo tee /usr/share/applications/com.example.MyPsd.desktop > /dev/null << 'EOF'
[Desktop Entry]
Type=Application
Version=1.0
Name=My PSD
Comment=Live spectrum from the CSI ring buffer
Exec=/usr/local/bin/my-psd
TryExec=/usr/local/bin/my-psd
Icon=quadrf-psd
Terminal=false
Categories=HamRadio;Science;
X-QuadRF-Desktop=true
EOF

sudo /usr/lib/quadrf/sync-desktop-apps
```

`X-QuadRF-Desktop=true` opts the icon onto the operator desktop. `Icon=` is an
icon *name* (here the packaged PSD icon), not a file path.

### 3. systemd unit

The unit runs as `dietpi` on the remote desktop display. Do not enable it at
boot; the control page starts and stops it.

```bash
sudo tee /etc/systemd/system/my-psd.service > /dev/null << 'EOF'
[Unit]
Description=My PSD Plot
Wants=load-quadrf.service quadrf-desktop.service
After=load-quadrf.service quadrf-desktop.service

[Service]
Type=simple
User=dietpi
Group=dietpi
Environment=HOME=/home/dietpi
Environment=DISPLAY=:1
Environment=XAUTHORITY=/home/dietpi/.Xauthority
Environment=SDL_VIDEODRIVER=x11
ExecStart=/usr/local/bin/my-psd
TimeoutStopSec=8
Restart=no
EOF

sudo systemctl daemon-reload
```

### 4. Control-page descriptor

```bash
sudo tee /usr/share/quadrf/apps.d/my-psd.json > /dev/null << 'EOF'
{
  "apps": [
    {
      "id": "my-psd",
      "desktop_entry": "com.example.MyPsd.desktop",
      "service": "my-psd.service",
      "binaries": ["my-psd"],
      "exclusive": true,
      "open": "desktop"
    }
  ]
}
EOF

sudo quadrf-app status
sudo quadrf-app start my-psd
sudo quadrf-app stop my-psd
```

`status` should list `"id": "my-psd"` with the name and icon from the desktop
entry. Reload `https://quadrf.local/` and My PSD appears in Applications;
start and stop work from there as well. `exclusive` is `true` for anything
that uses the radio, so starting this app stops the other radio apps.

To package as a `.deb`, and for details, see [Applications](applications.md).

---

## 4. Rebuilding SoapySDR Modules and Core Tree from Git

To modify the Farrow resampler, CSI interface, or SoapySDR modules (`mipi`, `quadrf`), build the repo directly on the quad.

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

### Revert to Packaged Modules

To revert all modules to the official packaged release:

```bash
sudo apt-get install --reinstall -y quadrf-soapy
SoapySDRUtil --probe="driver=mipi"
```

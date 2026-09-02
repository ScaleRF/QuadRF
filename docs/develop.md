# Developing on the QuadRF

Modify demos, write custom apps, and rebuild the radio stack from source.

## Prerequisites

The standard `quadrf` install provides the build toolchain and header files:

- **Compiler toolchain**: `build-essential`, `cmake`, `pkg-config`
- **Development headers**: `quadrf-dev` provides headers (`fpga_csi.h`, `Farrow.hpp`), CMake configuration (`find_package(QuadRF)`), and source trees under `/usr/src/`
- **Kernel headers and DKMS**: `quadrf-fpga-dkms` and `linux-headers-rpi-2712` (for kernel driver builds)
- **Example sources**: `quadrf-demos` provides reference sources under `/usr/share/quadrf/examples`

If you are compiling additional demos or rebuilding Soapy drivers, install the required development libraries:

```bash
sudo apt install -y libfftw3-dev libsdl2-dev libzmq3-dev device-tree-compiler
```

---

## 1. Working with Example Demos

The `quadrf-demos` package provides complete reference applications under `/usr/share/quadrf/examples`. Copy this tree into a working directory to experiment with and modify the code:

```bash
mkdir -p ~/my-demos
cp -r /usr/share/quadrf/examples/* ~/my-demos/
cd ~/my-demos
```

### Build with Make

Compile all example binaries directly using the bundled Makefile:

```bash
make -j4
```

You can also build individual applications using convenience targets:

```bash
make hello       # Sanity test: verifies CSI header and SoapySDR driver detection
make psd         # Live spectrum display using FFTW and SDL2
make rf-vision   # 30 fps swept-LO phase scatter display
make ntsc-demod  # Low-latency NTSC analog video demodulator
make nearfield   # 4x4 MIMO near-field phasor display
```

Install custom-built binaries to `/usr/local/bin`:

```bash
sudo make install
```

Clean build artifacts:

```bash
make clean
```

### Running Example Applications

Run the sanity check directly:

```bash
./quadrf-hello
```

To run video demodulation, tune the front-end with `quadrf-jtag` and pipe raw YUYV frames from `quadrf-ntsc-demod` directly into `mpv`:

```bash
quadrf-jtag --rx autosteer=1,antennas=15,interleave=0,tone_en=0,bw=12.0,agc=-14.0,freq=5806

./quadrf-ntsc-demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --hsync_min 25 --hsync_max 160 --sat 3.0 --hue -3.0 \
| mpv --profile=low-latency --no-cache \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=60 \
  --script=/usr/share/quadrf/ntsc_ch.lua --osd-font-size=40 --osd-duration=1500 \
  --vo=x11 -
```

### Alternative: Building with CMake

A `CMakeLists.txt` is also included if you prefer CMake or are integrating into a larger CMake project:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

To link an external application against QuadRF in CMake:

```cmake
find_package(QuadRF REQUIRED)
add_executable(my-app main.cpp)
target_link_libraries(my-app PRIVATE QuadRF::quadrf)
```

---

## 2. Registering Apps on the Desktop and Web UI

QuadRF allows custom applications to appear on both the operator desktop (`:1`) and the web control panel at `https://quadrf.local/`.

### Register an Application

Once your binary is installed (e.g. at `/usr/local/bin/my-psd`), register it with `quadrf-app`:

```bash
sudo quadrf-app register \
  --id my-psd \
  --name "My PSD" \
  --exec /usr/local/bin/my-psd \
  --icon quadrf-psd
```

`quadrf-app register` handles the configuration in one step:
1. Writes `/usr/share/applications/my-psd.desktop` and syncs the launcher icon to `/home/dietpi/Desktop/`.
2. Creates `/etc/systemd/system/my-psd.service` configured with `DISPLAY=:1`, `User=dietpi`, and X11 authentication.
3. Adds the application catalog entry in `/usr/share/quadrf/apps.d/my-psd.json`. By default, `--exclusive` is active so starting your app stops conflicting radio tasks.
4. Reloads systemd and syncs the desktop session.

### Test and Control the Application

Verify that the application is registered and query its status:

```bash
sudo quadrf-app status
```

Start and stop the app from the command line:

```bash
sudo quadrf-app start my-psd
sudo quadrf-app stop my-psd
```

Open `https://quadrf.local/` in your browser. "My PSD" appears in the Applications drawer where you can launch or stop it directly from the web interface.

### Unregister an Application

To remove the desktop icon, systemd service, and web UI entry:

```bash
sudo quadrf-app unregister my-psd
```

> **Packaging Note:** To distribute your application as a standalone `.deb` package with post-install triggers, see [Applications](applications.md).

---

## 3. Rebuilding Components from Source

Every component in the QuadRF stack can be modified and rebuilt directly on the Raspberry Pi using standard `make` workflows.

Source trees and Makefiles are installed on the board under `/usr/src/`:

| Component | Source Path | Target Output | Install Location |
| --- | --- | --- | --- |
| **CSI Driver & Overlay** | `/usr/src/quadrf-fpga-<version>/csi/` | `fpga-csi.ko`, `fpga-csi.dtbo` | `/lib/modules/.../updates/dkms/`, `/boot/firmware/overlays/` |
| **DSI Driver & Overlay** | `/usr/src/quadrf-fpga-<version>/dsi/` | `fpga-dsi.ko`, `fpga-dsi.dtbo` | `/lib/modules/.../updates/dkms/`, `/boot/firmware/overlays/` |
| **Transceiver Utility** | `/usr/src/quadrf-jtag/` | `quadrf-jtag` | `/usr/bin/quadrf-jtag` |
| **SoapySDR Module** | `/usr/src/quadrf-soapy/` | `libmipi.so` | `/usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/` |

If working from a git clone instead of `/usr/src/`, the identical Makefiles are located under `sources/fpga/drivers/csi/`, `sources/fpga/drivers/dsi/`, `sources/fpga/jtag_src/`, and `sources/soapy/`.

---

### Kernel Drivers and Overlays (`fpga-csi`, `fpga-dsi`)

The CSI driver handles DMA image capture from the RP1 MIPI CSI-2 receiver into circular userspace buffers. The DSI driver handles high-speed transmit data over the MIPI DSI interface. Both drivers use device tree overlays (`.dtbo`) to configure RP1 pinmuxing, clocks, and interrupts.

To rebuild and install the driver and overlay:

```bash
cd /usr/src/quadrf-fpga-*/csi
sudo make install
sudo quadrf-load
```

*(For the transmit driver, run the same commands inside `/usr/src/quadrf-fpga-*/dsi`)*.

> **Note:** If you edit the device tree source (`.dts`), reboot the board (`sudo systemctl reboot --force`) so the bootloader applies the new overlay. If only modifying C driver code (`fpga-csi.c`), running `sudo quadrf-load` reloads the module without rebooting.

To revert back to the packaged kernel drivers:

```bash
sudo apt install --reinstall -y quadrf-fpga-dkms quadrf-boot
sudo quadrf-load
```

---

### Transceiver Control CLI (`quadrf-jtag`)

`quadrf-jtag` controls the MAX2850 (TX) and MAX2851 (RX) front-end transceiver chips via bit-banged JTAG through ioctls on `/dev/csi_stream0`.

To rebuild and install `quadrf-jtag`:

```bash
cd /usr/src/quadrf-jtag
sudo make install
```

Test the binary:

```bash
quadrf-jtag --help
```

To revert back to the packaged binary:

```bash
sudo apt install --reinstall -y quadrf-fpga
```

---

### SoapySDR Module (`mipi`)

`libmipi.so` is the primary hardware driver. It reads CSI circular DMA buffers, applies polynomial Farrow sample rate conversion and ARM NEON SIMD packing/unpacking, and interfaces with `quadrf-jtag`.

To rebuild and install the SoapySDR driver:

```bash
cd /usr/src/quadrf-soapy
sudo make install
```

Verify that SoapySDR detects the rebuilt module:

```bash
SoapySDRUtil --probe="driver=mipi"
```

To revert back to the packaged SoapySDR module:

```bash
sudo apt install --reinstall -y quadrf-soapy
```

---

### Web Control Panel (Flask GUI)

The web interface is written in Python using Flask and Socket.IO. The application source runs directly from `/usr/share/quadrf/gui/`.

To modify the web UI:
1. Edit files under `/usr/share/quadrf/gui/` (or copy modified files from your git clone).
2. Restart the GUI systemd service:

```bash
sudo systemctl restart quadrf-gui
```

Check service status and logs:

```bash
systemctl status quadrf-gui
journalctl -u quadrf-gui -n 50 -f
```

To revert back to the packaged GUI:

```bash
sudo apt install --reinstall -y quadrf-gui
sudo systemctl restart quadrf-gui
```

---

### Restoring All Components to Stock

To discard all local rebuilds and return the entire software stack to official apt repository packages:

```bash
sudo apt install --reinstall -y \
  quadrf-boot \
  quadrf-fpga \
  quadrf-fpga-dkms \
  quadrf-soapy \
  quadrf-gui \
  quadrf-demos

sudo quadrf-load
```

# Building software on the QuadRF

Compile on the board against the installed packages, or rebuild the drivers from this tree.

## Custom apps and demos

To create a custom SDR app or modify a demo (for example, Spatial RF Vision), copy the example sources into your own workspace and build against the installed QuadRF packages.

The standard install (`quadrf`) includes:

- Compilation tools (`build-essential`, `cmake`, `pkg-config`)
- **`quadrf-dev`**: C++ API headers and CMake configuration
- **`quadrf-demos`**: Pre-built demo applications and their source code

**On the QuadRF:**

1. Copy the demo source into your own workspace:

   ```bash
   cp -r /usr/share/quadrf/examples ~/my-sdr-app
   cd ~/my-sdr-app
   ```

2. Build with CMake:

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

3. Run the binary. It links against the QuadRF drivers (`libquadrf.so`).

To ship the result as a `.deb` with a desktop icon and control-page launcher, see [Applications](applications.md).

## Core drivers from source

To modify the CSI FPGA interface, Farrow resampler, or SoapySDR modules, build the drivers from source.

**On the QuadRF:**

1. Uninstall the apt packages first so the build does not link against the packaged libraries:

   ```bash
   sudo apt remove quadrf quadrf-dev quadrf-demos quadrf-soapy quadrf-fpga
   ```

2. Clone the repository onto the QuadRF:

   ```bash
   git clone https://github.com/ScaleRF/QuadRF.git
   cd QuadRF
   ```

3. Install the remaining build dependencies (`cmake`, `build-essential`, and `pkg-config` are on the image and survive step 1):

   ```bash
   sudo apt install device-tree-compiler libfftw3-dev libsdl2-dev libsoapysdr-dev libzmq3-dev
   ```

4. Build from source:

   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

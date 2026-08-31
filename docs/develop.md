# Building software on the QuadRF

Compile on the board against the installed packages, or rebuild the SoapySDR
modules and demos from this tree.

## Custom apps

The standard install (`quadrf`) includes:

- Compilation tools (`build-essential`, `cmake`, `pkg-config`)
- **`quadrf-dev`**: headers (`fpga_csi.h`, `Farrow.hpp`, ...) and `find_package(QuadRF)`
- **`quadrf-demos`**: pre-built demos and their sources under `/usr/share/quadrf/examples`

Apps link **SoapySDR**. The QuadRF modules (`mipi`, `quadrf`) load at runtime;
there is no `libquadrf.so`.

**On the QuadRF**, a minimal app:

```bash
mkdir -p ~/my-sdr-app && cd ~/my-sdr-app
```

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.12)
project(my_sdr CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(QuadRF REQUIRED)
add_executable(my-sdr main.cpp)
target_link_libraries(my-sdr PRIVATE QuadRF::quadrf)
```

`main.cpp` can start from the packaged hello example, or:

```cpp
#include <fpga_csi.h>
#include <SoapySDR/Device.hpp>
#include <iostream>
int main() {
    for (const auto& d : SoapySDR::Device::enumerate()) {
        auto it = d.find("driver");
        if (it != d.end()) std::cout << it->second << "\n";
    }
}
```

```bash
mkdir build && cd build
cmake ..
make
./my-sdr
```

To rebuild the shipped demos (Spatial RF Vision, PSD, NTSC, Near-Field Phasors) from the example
sources, install the FFTW and SDL2 headers if they are not already present
(`quadrf-dev` Recommends them):

```bash
sudo apt install libfftw3-dev libsdl2-dev
cp -r /usr/share/quadrf/examples ~/my-sdr-app
cd ~/my-sdr-app
mkdir build && cd build
cmake ..
make
```

`./quadrf-hello` is the small SoapySDR probe. The other binaries need the radio
and, for the SDL apps, a display.

To ship a `.deb` with a desktop icon and control-page launcher, see [Applications](applications.md).

## SoapySDR modules and demos from source

To modify the Farrow resampler, SoapySDR modules, or demo programs, build this
tree on the board. Leave the apt packages installed: cmake does not link against
a packaged `libquadrf`, and removing `quadrf-fpga` / `quadrf-soapy` takes the
radio down.

The CSI and DSI kernel drivers are **not** built by this cmake. They come from
`quadrf-fpga-dkms` (sources under `/usr/src/quadrf-fpga-*` and
`sources/fpga/drivers/` in this tree).

**On the QuadRF:**

1. Clone the repository:

   ```bash
   git clone https://github.com/ScaleRF/QuadRF.git
   cd QuadRF
   ```

2. Install build dependencies (`cmake`, `build-essential`, and `pkg-config` are
   already on a full `quadrf` image):

   ```bash
   sudo apt install device-tree-compiler libfftw3-dev libsdl2-dev libsoapysdr-dev libzmq3-dev
   ```

3. Build:

   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

Soapy modules land in `build/sources/soapy/libmipi.so` and
`build/sources/quadrfd/soapy/quadrf.so`. SoapySDR searches the packaged
module directory first, so `SOAPY_SDR_PLUGIN_PATH` will not replace `mipi`
or `quadrf`. Copy the rebuilt files over the packaged copies to test:

```bash
sudo cp sources/soapy/libmipi.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/
sudo cp sources/quadrfd/soapy/quadrf.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/quadrf.so
SoapySDRUtil --probe="driver=mipi"
```

`sudo apt-get install --reinstall quadrf-soapy` restores the packaged modules.

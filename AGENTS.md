# QuadRF System Reference & Agent Guidelines

QuadRF is a 4-channel full-duplex (4x4 MIMO) software-defined radio phased array operating in the 4.9 – 6.0 GHz C-band. This reference provides AI agents and developers with the hardware topology, signal pipeline, supported features, and build workflows.

---

## 1. System Architecture & Hardware

QuadRF pairs a Raspberry Pi 5 with an RF frontend and FPGA digital baseband:

```
[ Rhombus Antenna Array (4x Dual-Pol RHCP/LHCP) ]
                   │
                   ▼  [ RF Polarization Switches ]
       ┌───────────┴───────────┐
       │ RX                    │ TX
       ▼                       ▲
[ 4x LNAs ]             [ 4x 1W PAs (SE5003L) ]
       │                       │
       ▼                       ▲
[ MAX2851 Receiver ]  [ MAX2850 Transmitter ]
  (Shared 40 MHz MEMS LO)     (Shared 40 MHz MEMS LO)
       │ (4x Baseband I/Q)     ▲ (4x Baseband I/Q)
       ▼                       │
[ Discrete ΣΔ Comparators ] [ 2nd-order ΣΔ Modulators ]
  (LVDS 1-bit @ 704 MHz)       │
       │                       │
       ▼                       ▲
┌──────────────────────────────────────────────┐
│          Lattice ECP5 FPGA                   │
│  - RX: Sinc³ CIC Decimator                   │
│  - TX/RX Digital Phase Shifters              │
│  - Beamformer / 4-Channel Interleaver        │
│  - MIPI CSI-2 Generator / MIPI DSI Receiver  │
└──────────────┬───────────────────────────────┘
  MIPI CSI-2   │ 4-lane FFC       ▲ MIPI DSI (2-lane FFC)
  (RX Stream)  ▼                  │ (TX Stream)
┌──────────────────────────────────────────────┐
│           Raspberry Pi 5 (RP1 Chip)          │
│  - CSI-2 Rx -> PCIe 2.0 x4 DMA -> System RAM │
│  - DSI Tx <- System RAM Staging Buffer       │
│  - Control: JTAG / SPI / GPIO Harness        │
└──────────────┬───────────────────────────────┘
               │ /dev/csi_stream0, /dev/dsi_stream0, SoapySDR (driver=mipi)
               ▼
[ Userland Applications & Web GUI ] (RF Vision, Data Link, GNU Radio)
```

### Core Hardware Components
- **Host Platform**: Raspberry Pi 5 (Broadcom BCM2712 Quad-Core Cortex-A76 @ 2.4 GHz, RP1 I/O southbridge, DietPi / Debian 12 ARM64).
- **RF Transceiver Pair**:
  - **MAX2851**: 4-channel direct-conversion receiver IC.
  - **MAX2850**: 4-channel direct-conversion transmitter IC.
  - **Clock Reference**: High-stability 40.000 MHz MEMS oscillator feeding both transceiver LO synthesizers and the FPGA PLL.
- **Transmit Power**: Four discrete SiGe power amplifiers (**SE5003L**) delivering **1 Watt (+30 dBm)** RF power per antenna (4 W total transmit power across the array).
- **Tx Gain Control**:
  - Programmable TX gain (0 to 63).
  - Software regulatory lock: TX gain is capped at 40 while `/var/lib/quadrf/tx_full_power` is absent. The file's presence lifts the cap. `/usr/sbin/quadrf-tx-unlock` creates `/var/lib/quadrf` and writes `yes` into that file, mode `644`. The Controls page calls the script through `POST /api/tx_unlock`. Removing the file restores the cap.
- **Rx Gain control & Staging** (FPGA splits the 0–63 dB command on 0x6A):
  - LNA (D[7:5]): 0 / 8 / 16 / 24 / 32 / 40 dB (40 dB RF range, 8 dB steps).
  - VGA (D[3:0]): 0 to 30 dB typical in 2 dB steps.
  - Analog control range is 70 dB (40 + 30). Typical conversion gain at max is 68 dB.
- **Antenna Array Geometry**: Four circular dual-feed patch antennas arranged in a horizontal 'laying on its side' planar **rhombus (diamond)** configuration with **45.5 mm** (0.85 lambda @ 5.6 GHz) element spacing (radius between nearest neighbors, ie: 1 and 3); (When viewed from the back of the array, looking in the direction it points):
  - Antenna 1: Top Center
  - Antenna 2: Left
  - Antenna 3: Bottom Center
  - Antenna 4: Right
  - Enables spatial beamforming and direction-of-arrival (DOA) estimation.
- **Polarization & Duplexing**:
  - Dual circular polarization: TX is always **RHCP**. RX is switch-selectable **RHCP** or **LHCP**.
  - Full-duplex operation is supported via cross-polarization and matching network isolation.
- **Discrete Sigma-Delta ADC & Baseband**:
  - Differential BJT stages (**UMX5NTR**); FPGA LVDS inputs are 1-bit comparators at 704 MHz.
  - Digital decimation filters (set in the GUI) produce 8+8-bit I/Q (~7-bit ENOB with beamform) at up to ~40 MHz per antenna.
- **Exclusive Hardware Access (Single-Client)**:
  - The RF frontend and `/dev/csi_stream0` stream are strictly single-client. If a background or catalog service (`quadrf-rf-vision`, `quadrf-phasegaze`, `quadrf-data`, `quadrf-ntsc`) is active, concurrent attempts to open `/dev/csi_stream0` or stream via SoapySDR will fail with device busy.
  - Query running applications with `quadrf-app status` and release the device by stopping the competing unit via `quadrf-app stop <app>`.

---

## 2. Operating Frequencies & Tuning Limits

- **Official Supported Frequency Range**: **4.900 GHz to 6.000 GHz** (C-Band).
- **Synthesizer Tuning Range**:
  - Synthesizer uses an integer-N / 20-bit fractional-N PLL with an 80 MHz comparison frequency derived from the 40.000 MHz MEMS oscillator.
  - The internal Voltage-Controlled Oscillator (VCO) sub-band selector (VAS) can experimentally lock from approximately **4.5 to 6.7 GHz**, bounded by the hardware tuning limits of the VCO sub-bands. Note that the included patch antennas are best matched at 5.6 GHz. Damage may occur when transmitting at unsupported frequencies with power.
- **Bandwidth**: ~40 MHz per element (switchable 20 MHz / 40 MHz analog filters, plus adjustable FPGA digital CIC decimation filter).

---

## 3. Supported Software & Userland Ecosystem

- **SoapySDR Plugin (`driver=mipi`)**: Native `CS8` (8-bit I/Q) and converted `CF32` streams across 4 channels.
- **Web Interface** (`quadrf.local` by default; extra units on the LAN are `quadrf-2.local`, `quadrf-3.local`, or a custom name from Network Setup):
  - Control panel: `http://quadrf.local/` (also `/GUI/`, `/split`)
  - Remote desktop (KasmVNC): `http://quadrfd.local/`, `http://quadrf-desktop.local/`, or `http://quadrf.local:6080/` (extra units: `quadrf-2d.local` / `quadrf-2-desktop.local`)
- **Application Suite**:
  - **RF Vision (Advanced)** (`quadrf-phasegaze.service`): Real-time spatial RF camera and swept CSI heatmap visualization at 30+ FPS.
  - **RF Vision (Basic)** (`quadrf-rf-vision.service`): Swept-LO phase scatter and browser AR overlay.
  - **802.11 OFDM Link** (`quadrf-data`): IEEE 802.11 OFDM IP tunnel bridging Linux `tun0` across QuadRF units.
  - **NTSC Demodulator** (`quadrf-ntsc-demod`): Real-time analog video demodulation.
  - **4x4 MIMO**: Simple 4ch MIMO demo.
  - **Nearfield** (`quadrf-nearfield`): Near-field RF sensing with TDM TX cycling.
  - **GNU Radio**: Soapy QuadRF Source and Sink Blocks.

---

## 4. Diagnostics & Common Workflows

```bash
# Verify SoapySDR Device Detection
SoapySDRUtil --find="driver=mipi"

# Inspect Receiver / Transmitter Status & Registers
quadrf-jtag --status rx
quadrf-jtag --status tx

# Configure Receiver Parameters
quadrf-jtag --rx antennas=0xF,freq=5500,bw=40,pol=rhcp,interleave=1

# Configure Transmitter Parameters
quadrf-jtag --tx antennas=0xF,freq=5500,bw=40,gain=40

# Inspect CSI / DSI stream nodes
ls -l /dev/csi_stream0 /dev/dsi_stream0

# Reload FPGA Bitstream
quadrf-load

# Check Active Applications (Frontend is single-client)
quadrf-app status
# Stop competing app if /dev/csi_stream0 is busy:
# quadrf-app stop <ar|phasegaze|data|ntsc>

# Check Systemd Services
systemctl status quadrf-*

# Reboot
sudo reboot
# (Use 'sudo systemctl reboot --force' only if a stuck process or service blocks shutdown)
```

---

## 5. Build, Test, and Compilation Workflows

- **Release Build Invariants**:
  Build in `Release` mode with Cortex-A76 NEON SIMD optimizations enabled:
  ```bash
  mkdir -p build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j$(nproc)
  ```
- **NEON SIMD Vectorization**:
  DSP algorithms (FFT, Farrow resamplers, Viterbi decoders, correlators) target ARMv8.2-A NEON SIMD to sustain real-time multi-channel processing throughput. It is recommended to write user programs in C++ with NEON SIMD optimization.

---

## 6. Detailed Topic Guides

- [MIPI Streaming Pipeline](file:///usr/share/quadrf/agent/docs/system_pipeline.md) (`agent/docs/system_pipeline.md`): CSI/DSI char devices, RP1 CSI2-DMA, RIO/PIO JTAG pacing, and Soapy `driver=mipi`.
- [Filing issues and pull requests](file:///usr/share/quadrf/agent/docs/contribution_protocol.md) (`agent/docs/contribution_protocol.md`): Search before filing, issue and pull request sections, labels, and commit conventions.

---

## 7. Local Overrides (`AGENTS.local.md`)

This file (`AGENTS.md`) is maintained upstream and updated on package releases via `apt upgrade`.

If you have board-specific IP addresses, custom pin mappings, or local test bench notes:
- Place them in `AGENTS.local.md` in this directory.
- `AGENTS.local.md` is preserved across package updates and ignored by source control.

---

## 8. Licensing

- **Software Stack & Drivers**: Licensed under the **GNU General Public License (GPL-3 / GPL-2)**. See `licenses/` and [GitHub repository](https://github.com/ScaleRF/QuadRF).
- **Antenna Architecture & Hardware Designs**: Released design files licensed under **Creative Commons Attribution-ShareAlike 4.0 International (`CC BY-SA 4.0`)** with a defensive patent covenant (`licenses/PATENT_COVENANT.md`).

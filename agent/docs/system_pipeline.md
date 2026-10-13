# QuadRF System Streaming Pipeline Guide

This guide describes the end-to-end data and control streaming architecture on QuadRF: from the Lattice ECP5 FPGA across MIPI CSI-2 and MIPI DSI into the Linux kernel, the SoapySDR abstraction layer, and application services.

## 1. FPGA Framing over MIPI CSI-2 (RX)

1. **Virtual Frame Structure**:
   - The FPGA packs digitized 4-channel I/Q into CSI-2 frames (default geometry 1024×128 RAW8).
   - The host driver exposes those spans on `/dev/csi_stream0` (not V4L2 / `/dev/video0`).
   - **Interleaved mode** (`quadrf-jtag --rx interleave=1`, FPGA 0x25=1): each line multiplexes time-aligned I/Q pairs across all 4 receive channels.
   - **Beamformed mode** (`quadrf-jtag --rx interleave=0`, FPGA 0x25=0): the FPGA applies phase offsets $\theta_1..\theta_4$ (0x2A / 0x2B) and sums the channels into a single stream.
   - CLI and Soapy default is beamformed (`interleave=0`). Soapy turns interleave on only when `setupStream(RX)` opens more than one channel. The web UI 4-antenna mode also writes `interleave=1`.
2. **Low Host Overhead**:
   - RP1 CSI2-DMA copies spans into a kernel ring. Userspace `mmap`s `/dev/csi_stream0`.
   - Path is on-chip RP1 (PCIe to the BCM2712), not USB.

---

## 2. FPGA Baseband over MIPI DSI (TX)

1. **High-Speed Transmit Path**:
   - Baseband goes out the Raspberry Pi 5 MIPI **DSI0** port (`dsi@110000` in `fpga-dsi.dts`), **2 data lanes**, over FFC to the FPGA.
   - The FPGA DSI receiver aligns lanes and unpacks DSI frames.
2. **Resampling & $\Sigma\Delta$ Modulation**:
   - Native DSI payload rate is ~86.08 Msps. A host Farrow resampler converts the user TX rate to that rate.
   - The FPGA applies per-channel digital phase offsets ($\theta_1..\theta_4$) for transmit beamforming (`quadrf-jtag --tx p1..p4`, FPGA 0x2C / 0x2D).
   - An 8x interpolator upsamples to 704 MHz. Second-order $\Sigma\Delta$ modulators then emit 1-bit LVDS I/Q; the MAX2850's analog LPFs reconstruct baseband for the upconverters.

---

## 3. SoapySDR Translation Layer (`driver=mipi`)

The interface between `/dev/csi_stream0` (RX), `/dev/dsi_stream0` (TX), and SDR applications is implemented in `sources/soapy/MipiDevice.cpp`:

- **Device Query**:
  ```bash
  SoapySDRUtil --find="driver=mipi"
  SoapySDRUtil --probe="driver=mipi"
  ```
- **Buffer Management**:
  - `MipiDevice` `mmap`s the CSI and DSI rings.
  - `readStream` / `writeStream` convert native CS8 in-process. There is no V4L2 queue and no worker thread.
- **Stream Formats**:
  - Native: `SOAPY_SDR_CS8` (8-bit I/Q, full-scale 127).
  - Converted: `SOAPY_SDR_CF32` (`std::complex<float>`).

---

## 4. Hardware Control & RP1 Register Pacing

The host controls MAX2850/MAX2851 transceiver registers and FPGA phase/filter settings via an 8-wire harness:

1. **RP1 RIO Direct GPIO**:
   - Raspberry Pi 5 southbridge RP1 RIO (Register I/O) hardware allows direct register bit-banging JTAG, taking ~1.7 µs per register write.
2. **RP1 PIO Deterministic State Machine**:
   - An RP1 PIO (Programmable I/O) state machine provides fixed register write slots (e.g. 2005 ns slots) tied to `CLOCK_MONOTONIC`.
   - Kernel driver (`fpga-csi`) implements DMA ring streaming via `CSI_IOC_JTAG_STREAM_*` ioctls with a dedicated feeder kthread, wrapped in C++ via `FpgaJtag`.
   - Used for time-critical register updates (such as rapid phase dithering or fast frequency hops).
3. **Control CLI Tools**:
   - `quadrf-jtag`: Primary CLI utility for configuring RF frequency, gain, bandwidth, polarization, and reading chip status.
   - `quadrf-load`: Helper that flashes `/usr/share/quadrf/fpga/quadrf.svf`.

---

## 5. Systemd Service Hierarchy & Application Ecosystem

| Unit / tool | Description |
| :--- | :--- |
| `quadrf-journal.service` | Bind-mounts `/var/lib/quadrf/journal` over `/var/log/journal` so journald persists past DietPi ramlog. |
| `quadrf-reboot-clear.service` | Removes `/var/lib/quadrf/reboot-required` on a clean boot. |
| `quadrf-rf-vision.service` | In-tree spatial RF camera; nginx serves it at `/ws`. |
| `quadrf-phasegaze.service` | Separately packaged PhaseGaze hemisphere view at `/phasegaze` (not built from this repo). |
| `quadrf-data.service` | IEEE 802.11 OFDM TUN IP data link (`quadrf-data --gui`, `tun0`). |
| `quadrf-app` | CLI, not a systemd unit. Starts/stops catalog apps (`quadrf-app status`). Enforces single-client mutual exclusion on the RF frontend. |

The radio frontend and `/dev/csi_stream0` / `/dev/dsi_stream0` character devices support only one active userland consumer at a time. `quadrf-app start <id>` automatically stops competing exclusive applications and waits for device node release before launching the requested service.

---

## 6. Diagnostic Commands

```bash
# Application exclusivity check (inspect running consumers)
quadrf-app status

# CSI / DSI stream nodes
ls -l /dev/csi_stream0 /dev/dsi_stream0

# Device tree (Pi 5 / RP1)
find /proc/device-tree -iname '*csi*' -o -iname '*dsi*'

# Driver logs
dmesg | grep -iE 'fpga-csi|fpga-dsi|csi_stream|dsi_stream'
```

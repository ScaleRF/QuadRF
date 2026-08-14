MIPI, SoapySDR, and GNU Radio drivers, web VNC, SDR controls GUI, and demos for the [QuadRF](https://scalerf.com/) on Raspberry Pi 5.

Packages are built and published by GitHub Actions. Tagged releases attach `.deb` files and update the signed apt repository on GitHub Pages.

## Install

On a Raspberry Pi 5 running [DietPi](https://dietpi.com/):

```bash
# trust key + add repo
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://scalerf.github.io/QuadRF/quadrf.gpg \
  | sudo tee /etc/apt/keyrings/quadrf.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/quadrf.gpg] https://scalerf.github.io/QuadRF bookworm main" \
  | sudo tee /etc/apt/sources.list.d/quadrf.list >/dev/null

# install
sudo apt update
sudo apt install quadrf

# required once after install
# `quadrf-boot` updates `config.txt` and the device-tree overlays, taking effect after a reboot
sudo reboot
```

After reboot:

```bash
quadrf status
```

Lists services, CSI/DSI drivers, SoapySDR devices and interface addresses. The
web UI is at http://quadrf.local/.

Details: [docs/install.md](../docs/install.md).

The factory FPGA bitstream is not published in this repository.
`sources/fpga/quadrf.svf` is an empty placeholder. Hardware units ship with the
production bitstream separately; this package tree will not program the FPGA
from the placeholder file.

## Packages

Install metapackages `quadrf` or `quadrf-headless`, or pick components. [docs/install.md](../docs/install.md#packages).

| Package            | Contents                                                                                                                                                  |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `quadrf`           | **Metapackage**: boot, common, fpga, soapy, gui, network, demos, desktop, gnuradio, ups; recommends mesh (`quadrf-phy`, `quadrf-meshtasticd`) |
| `quadrf-headless`  | **Metapackage**: boot, common, fpga, soapy, gui, network; recommends mesh (`quadrf-phy`, `quadrf-meshtasticd`)                                            |
| `quadrf-common`    | `/etc/quadrf/quadrf.conf`, shared helpers, the `quadrf` command                                                                                           |
| `quadrf-boot`      | Device-tree overlays, firmware configuration                                                                                                              |
| `quadrf-fpga`      | Bitstream, `quadrf-jtag`, `load-quadrf.service`                                                                                                           |
| `quadrf-fpga-dkms` | CSI and DSI kernel drivers, built by DKMS                                                                                                                 |
| `quadrf-soapy`     | `mipi` and `quadrf` SoapySDR modules, SoapyRemote service                                                                                                 |
| `quadrf-gui`       | Flask control panel on port 8080                                                                                                                          |
| `quadrf-network`   | nginx, dnsmasq, mDNS, access point                                                                                                                        |
| `quadrf-demos`     | Spatial RF Vision, PSD plot, NTSC decoder (`mpv`), AR                                                                                                     |
| `quadrf-desktop`   | KasmVNC session with QuadRF launchers                                                                                                                 |
| `quadrf-gnuradio`  | Example flowgraphs                                                                                                                                        |
| `quadrf-ups`       | UPS HAT battery monitor                                                                                                                                   |

## Dependencies

SoapySDR, GNU Radio, gr-osmosdr, nginx, dnsmasq, hostapd, mpv and others from
Debian. Four artefacts do not, and are mirrored or rebuilt into the QuadRF
repository with versions pinned in `packaging/pins.env`:

- `quadrf-openocd`, the Raspberry Pi OpenOCD fork with RP1 GPIO support
- `kasmvncserver`, the upstream KasmVNC release
- `qradiolink`, the SDR transceiver used by the KasmVNC desktop launcher
- `quadrf-phy` / `quadrf-meshtasticd`, built from the separate
[quadrf-mesh](https://github.com/radioroy/quadrf-mesh) repo — the LoRa
PHY and a patched `meshtasticd` that runs the Meshtastic protocol over
it. `Recommends:` on `quadrf` / `quadrf-headless`, so a plain
`apt install quadrf` carries mesh support when those packages are in the
apt repository; drop them with `--no-install-recommends`.

## Documentation

| Doc                                              | Contents                            |
| ------------------------------------------------ | ----------------------------------- |
| [docs/overview.md](../docs/overview.md)          | Overview                            |
| [docs/install.md](../docs/install.md)            | Install, configure, upgrade, remove |
| [docs/certbot.md](../docs/certbot.md)            | TLS certificates                    |
| [packaging/README.md](../packaging/README.md)    | Building and publishing packages    |
| [GitHub Releases](https://github.com/ScaleRF/QuadRF/releases) | Package release notes |
| [TODO.md](../TODO.md)                            | TODO                                |

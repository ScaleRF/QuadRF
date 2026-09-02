<img align="left" src="microsd.png" width="150px">

The [QuadRF Kit](https://www.crowdsupply.com/scale-rf/quadrf#) ships with a microSD card containing all software pre-installed. If you want to start fresh or build your own image containing the latest QuadRF software, this is your guide!

<br clear="all">

### What you need

- A QuadRF Kit with its built-in Raspberry Pi 5
- A 16 GB or larger microSD card, or the 32 GB card that came with your QuadRF Kit
- A microSD card writer

### Build the DietPi base image

1. Download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Download the base [DietPi Trixie image for Raspberry Pi 5](https://dietpi.com/downloads/images/DietPi_RPi5-ARMv8-Trixie.img.xz).
3. Open Raspberry Pi Imager and select **Raspberry Pi 5**.
4. Under **Operating System**, scroll to the bottom and select **Use custom**.
5. Select the DietPi Trixie image you downloaded.
6. Select your microSD card and write the image.

### Connect the Pi to the internet

To install the QuadRF-specific software, the Pi 5 must be connected to the internet. You can configure Wi-Fi and an unattended first boot by editing the DietPi configuration files on the microSD card before moving it to the Pi.

In **`dietpi.txt`**, find and change the corresponding settings to:

```ini
AUTO_SETUP_NET_WIFI_ENABLED=1

# Install Avahi so DietPi and dietpi.local resolve on the local network 
AUTO_SETUP_INSTALL_SOFTWARE_ID=152  # (remember to uncomment the line!)

# Complete DietPi's first-run setup without a monitor or keyboard
AUTO_SETUP_AUTOMATED=1
SURVEY_OPTED_IN=0
```

DietPi recommends replacing its public default password when enabling unattended setup. In the same file, change:

```ini
AUTO_SETUP_GLOBAL_PASSWORD=choose-a-temporary-password
```

Remember this password; you will use it for your first SSH login.

In **`dietpi-wifi.txt`**, enter your Wi-Fi credentials:

```ini
aWIFI_SSID[0]='your Wi-Fi name'
aWIFI_KEY[0]='your Wi-Fi password'
```

Save both files and safely eject the microSD card.

### Connect to the Pi 5 via SSH

Insert the microSD card into the QuadRF Raspberry Pi 5 using the rear slot. Connect USB-C power or turn on the Mobile Battery Pack.

The first boot takes longer than a normal boot because DietPi expands the filesystem, updates the base system, and installs Avahi. Allow approximately **5–10 minutes** before trying to connect. No monitor or keyboard is required.

From Linux, macOS, or a current Windows terminal, connect with:

```bash
ssh dietpi@dietpi.local
```

For Windows, you may alternatively use [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html) and enter `dietpi.local` in the **Host Name** field.

Use the following credentials:

```text
Username: dietpi
Password: the value you set in AUTO_SETUP_GLOBAL_PASSWORD
```

Hostnames are not case-sensitive, but lowercase `dietpi.local` is conventional.

If `dietpi.local` does not resolve after the first-run setup has finished, find the Pi's IP address in your router's connected-device or DHCP list and connect directly (`ssh dietpi@192.168.x.x`). Some guest  Wi-Fi networks block the multicast traffic used by `.local` discovery. Worst case, you can connect a Micro-HDMI display and USB keyboard directly to the QuadRF.

## Install the QuadRF software

Once you are logged into the QuadRF Pi 5, run the following commands:

```bash
# Trust the repository key and add the QuadRF repository
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://scalerf.github.io/QuadRF/quadrf.gpg \
  | sudo tee /etc/apt/keyrings/quadrf.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/quadrf.gpg] https://scalerf.github.io/QuadRF trixie main" \
  | sudo tee /etc/apt/sources.list.d/quadrf.list >/dev/null

# Install QuadRF
sudo apt update
sudo apt install quadrf

# Required once after installation
# quadrf-boot updates config.txt and the device-tree overlays.
sudo reboot
```

After the reboot, run the following command to check the services, CSI/DSI drivers, SoapySDR devices, and interface addresses:

```bash
quadrf status
```

### That's it!

You now have an up-to-date QuadRF microSD image and can get started with the many QuadRF applications. See [scalerf.com/docs](https://scalerf.com/docs/) for more information.

After the reboot, the hostname is `quadrf.local`. Open:

```text
http://quadrf.local/setup/security/
```

Install the locally generated certificate, then use:

```text
https://quadrf.local/
```

To put the QuadRF into **AP Mode** so you can connect with your phone, open the control GUI and enable the Hotspot toggle.

## Packages

Install the `quadrf` or `quadrf-headless` metapackage, or select individual components. See [docs/overview.md](../docs/overview.md#2-package-architecture).

| Package | Contents |
| --- | --- |
| `quadrf` | **Metapackage:** boot, common, FPGA, SoapySDR, GUI, network, development files, demos, desktop, GNU Radio, and UPS support |
| `quadrf-headless` | **Metapackage:** boot, common, FPGA, SoapySDR, GUI, and network support |
| `quadrf-common` | `/etc/quadrf/quadrf.conf`, shared helpers, and the `quadrf` command |
| `quadrf-boot` | Device-tree overlays and firmware configuration |
| `quadrf-fpga` | Bitstream, `quadrf-jtag`, and `load-quadrf.service` |
| `quadrf-fpga-dkms` | CSI and DSI kernel drivers built by DKMS |
| `quadrf-soapy` | `mipi` SoapySDR module and the SoapyRemote service |
| `quadrf-gui` | Flask control panel on port 8080 |
| `quadrf-network` | nginx, dnsmasq, mDNS, access point support, and OpenSSH/SFTP |
| `quadrf-dev` | C++ headers and `find_package(QuadRF)` support for building against the installed SoapySDR modules |
| `quadrf-demos` | Spatial RF Vision, PSD plot, NTSC decoder (`mpv`), near-field phasors, AR, and example sources under `/usr/share/quadrf/examples` |
| `quadrf-desktop` | KasmVNC session with QuadRF launchers |
| `quadrf-gnuradio` | Example GNU Radio flowgraphs |
| `quadrf-ups` | UPS HAT battery monitor |

## Dependencies

SoapySDR, GNU Radio, gr-osmosdr, nginx, dnsmasq, hostapd, mpv, and other dependencies come from Debian. The following packages are mirrored or rebuilt in the QuadRF repository, with versions pinned in `packaging/pins.env`:

- `quadrf-openocd`: the Raspberry Pi OpenOCD fork with RP1 GPIO support
- `kasmvncserver`: the upstream KasmVNC release
- `qradiolink`: the SDR transceiver used by the KasmVNC desktop launcher

## Additional setup documentation

| Document | Contents |
| --- | --- |
| [docs/overview.md](../docs/overview.md) | Overview |
| [docs/troubleshooting.md](../docs/troubleshooting.md) | Diagnostic matrix, service logs, and recovery procedures |
| [docs/applications.md](../docs/applications.md) | Desktop icons, control pages, and packaging an application |
| [docs/develop.md](../docs/develop.md) | Compile custom applications and rebuild drivers on the board |
| [docs/tls.md](../docs/tls.md) | HTTPS setup |
| [packaging/README.md](../packaging/README.md) | Build and publish packages |
| [GitHub Releases](https://github.com/ScaleRF/QuadRF/releases) | Package release notes |
| [TODO.md](../TODO.md) | Project TODO list |

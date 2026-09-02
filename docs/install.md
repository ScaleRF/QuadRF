## Installing

On a Raspberry Pi 5 running [DietPi](https://dietpi.com/):

The Pi needs network access and `sudo`. Installer assumes default account: `dietpi`.

Packages are published by GitHub Actions to the signed apt repository on GitHub Pages.

```bash
# trust key + add repo
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://scalerf.github.io/QuadRF/quadrf.gpg \
  | sudo tee /etc/apt/keyrings/quadrf.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/quadrf.gpg] https://scalerf.github.io/QuadRF trixie main" \
  | sudo tee /etc/apt/sources.list.d/quadrf.list >/dev/null

# install
sudo apt update
sudo apt install quadrf          # complete appliance
# sudo apt install quadrf-headless # radio, network and web GUI, no desktop

# required once after install
# `quadrf-boot` updates `config.txt` and the device-tree overlays, taking effect after a reboot
sudo reboot
```

After reboot:

```bash
quadrf status
```

Lists services, CSI/DSI drivers, SoapySDR devices and interface addresses. Open
[the local HTTPS setup](http://quadrf.local/setup/security/) once, install
the QuadRF root certificate, and then use
[https://quadrf.local/](https://quadrf.local/).

See [install/README.md](../install/README.md) for the package list and
[packaging/README.md](../packaging/README.md) for how releases are built.



## Packages

```mermaid
flowchart TB
  subgraph metas["Metapackages"]
    FULL["quadrf"]
    HEAD["quadrf-headless"]
  end

  subgraph core["Always pulled by both"]
    COMMON["quadrf-common"]
    BOOT["quadrf-boot"]
    FPGA["quadrf-fpga"]
    DKMS["quadrf-fpga-dkms"]
    SOAPY["quadrf-soapy"]
    GUI["quadrf-gui"]
    NET["quadrf-network"]
  end

  subgraph full_only["Only via quadrf"]
    DEV["quadrf-dev"]
    DEMOS["quadrf-demos"]
    DESK["quadrf-desktop"]
    UPS["quadrf-ups"]
    GRC["quadrf-gnuradio"]
    QRL["qradiolink"]
  end

  FULL --> COMMON & BOOT & FPGA & SOAPY & GUI & NET & DEV & DEMOS & DESK & UPS & GRC
  HEAD --> COMMON & BOOT & FPGA & SOAPY & GUI & NET

  BOOT --> COMMON
  FPGA --> BOOT & DKMS
  SOAPY --> FPGA
  GUI --> FPGA
  DEV --> FPGA & SOAPY
  DEMOS --> FPGA
  DESK --> QRL
  NET --> COMMON
  UPS --> COMMON
  GRC --> COMMON
```

The `quadrf` package includes a remote desktop, applications described in [Applications](applications.md), and the headers and example sources used to compile on the quad ([Building software](develop.md)). Extra `.deb` packages with a desktop entry and/or a control-page descriptor load automatically. `quadrf-headless` omits the desktop, the included radio apps, and `quadrf-dev`.

## Configure

Site settings live in `/etc/quadrf/quadrf.conf`:


| Setting             | Default          | Effect                                         |
| ------------------- | ---------------- | ---------------------------------------------- |
| `QUADRF_USER`       | `dietpi`         | Account for the desktop, web GUI and applications |
| `QUADRF_BOOT_DIR`   | `/boot/firmware` | Firmware partition (`config.txt`, `overlays/`) |
| `QUADRF_HOSTNAME`   | `quadrf`         | mDNS / nginx name (`HOSTNAME.local`; desktop is `HOSTNAMEd.local` and `HOSTNAME-desktop.local`) |
| `QUADRF_AP_SSID`    | `QuadRF`         | Fallback access point SSID                     |
| `QUADRF_AP_PASS`    | empty            | Fallback AP WPA2 passphrase; empty = open      |
| `QUADRF_AP_ADDRESS` | `192.168.44.1`   | Pi address in access point mode                |
| `QUADRF_WIFI_MODE`  | `sta`            | Last Wi-Fi choice: `sta`, `ap`, or `off` (falls back to hotspot if no saved network) |
| `QUADRF_WIFI_FALLBACK` | `yes`         | If client join fails, start the hotspot        |
| `QUADRF_OPENOCD`    | empty            | Override OpenOCD; used to program the FPGA with BYO bitstream     |


After editing:

```bash
sudo quadrf apply
```

This re-runs the package config hooks under `/usr/lib/quadrf/apply.d/`
(firmware block, service account drop-ins, generated nginx/dnsmasq/hostapd/
sudoers, desktop layout).

### Multiple QuadRFs on one network

The QuadRF starts with the mDNS hostname **quadrf**. When a second QuadRF
joins the LAN, it picks the next free name **quadrf-2**, then **quadrf-3**,
and so on. Its control panel, remote desktop, and HTTPS setup all use that
new name, ex: **https://quadrf-2.local/** ...

The direct-connect address does not change when several units share a LAN.

In **Network Setup** you can edit the name (the `quadrf` part of
**quadrf.local**) and **Save**. On a custom name (or **quadrf-2**), **Don't
yield this name** stops the unit from becoming **name-2** if that name is
already taken. **Reset** tries **quadrf.local** again. Save reloads nginx
then runs `quadrf apply`. The page at the old name will drop; open the new
`HOSTNAME.local` URL.

The initial web entry point is HTTP. A fresh installation also creates a local
root CA and a separate HTTPS server certificate. Open `http://HOSTNAME.local/setup/security/` to install that root on
iOS, Android, macOS, Windows, Linux or ChromeOS, then use
`https://HOSTNAME.local/`. See
[HTTPS setup](tls.md) for the exact steps.

## Upgrade

```bash
sudo apt update && sudo apt upgrade
```

Changed packages are download; services restart as needed. A kernel upgrade
rebuilds the CSI/DSI drivers via DKMS if matching `linux-headers` are installed.

## Remove

```bash
sudo apt remove 'quadrf-*'               # all packages
sudo apt purge 'quadrf-*'                # and configuration
```

Removing `quadrf-boot` strips the QuadRF block from `config.txt` and the
overlays; reboot afterward.

## Troubleshooting


| Symptom                                         | Check                                                                                                                              |
| ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `load-quadrf.service` fails                     | Overlays need a reboot. Confirm the QuadRF block in `/boot/firmware/config.txt` and that `quadrf status` shows the drivers loaded. |
| Drivers missing after a kernel upgrade          | `dkms status`. `quadrf-fpga-dkms` depends on `linux-headers-rpi-2712` (Pi 5) or `linux-headers-rpi-v8`. Then `sudo dkms autoinstall`. |
| `quadrf.local` does not resolve                 | Check `systemctl status avahi-daemon quadrf-mdns`. Name is published per interface; from the LAN it is the ethernet/client address, not `192.168.44.1`. Client must support mDNS. |
| `quadrf-soapy-server` bind fails / restart loop | Stock `soapyremote-server` must stay masked; `quadrf-soapy` owns port 55132.                                                       |
| No fallback access point                        | Default SSID `QuadRF` is open unless `QUADRF_AP_PASS` is set. `QUADRF_WIFI_MODE` must not be `off`, and `QUADRF_WIFI_FALLBACK` defaults to yes. `hostapd` must be active (`journalctl -u hostapd`). Trixie skips the unit unless `/etc/hostapd/quadrf.conf` satisfies the condition drop-in. |
| `networking.service` / `misplaced option`       | Comment leftover `wireless-power` / `wpa-conf` lines in `/etc/network/interfaces`. |
| apt/HTTPS fails while the AP is up              | Remove `address=/#/` from `/etc/dnsmasq.d/quadrf-wlan0-ap.conf` and `systemctl restart dnsmasq`. |
| USB `10.55.0.1` listed but unreachable          | `quadrf-usb` only assigns the island while a host is enumerated. Plug the Pi USB-C data port (not the UPS charge jack). Most laptops cannot provide 5V at 5A, causing possible brownouts. Please isolate the Pi's power from the usb connection to a device. |
| Laptop on ethernet has only IPv6                | `quadrf-ethernet` follows carrier: ~12s with no DHCP then `10.55.1.1`. A leftover router lease is dropped if that gateway is gone. |
| nginx will not start                            | `nginx -t`. Site is generated at `/etc/nginx/sites-available/quadrf`; regenerate with `sudo quadrf apply`.                         |
| QuadRF services show `disabled`                 | Packages enable them on install. On a unit that predates this tree: `sudo systemctl enable --now load-quadrf quadrf-gui quadrf-hotspot quadrf-ethernet quadrf-usb quadrf-desktop quadrf-ups quadrf-soapy-server`. |


Service logs: `journalctl -u load-quadrf -u quadrf-gui -b`. After a hang/reboot, `journalctl -b -1` is the previous boot. The journal is on disk at `/var/lib/quadrf/journal`.

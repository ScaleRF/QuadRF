# Troubleshooting

Diagnostic procedures and recovery steps for common hardware, driver, and networking failure modes.

## Quick Diagnostic Commands

Query the overall health of the RF front-end, kernel drivers, and network interfaces:

```bash
quadrf status
```

Inspect logs for the hardware bring-up and web services:

```bash
journalctl -u load-quadrf -u quadrf-gui -b
```

Inspect logs from the previous boot (persisted at `/var/lib/quadrf/journal`):

```bash
journalctl -b -1
```

---

## Diagnostic Matrix

| Symptom | Probable Cause | Corrective Action |
| --- | --- | --- |
| `load-quadrf.service` fails during boot | Device tree overlays not applied by bootloader, or hardware uninitialized | Ensure the QuadRF block is present in `/boot/firmware/config.txt`. Run `sudo quadrf apply` and force a reboot with `sudo systemctl reboot --force`. |
| Kernel drivers missing after kernel upgrade | Kernel headers updated or DKMS module not compiled for current kernel | Verify status with `dkms status`. Ensure `linux-headers-rpi-2712` (or `linux-headers-rpi-v8`) is installed, then run `sudo dkms autoinstall`. |
| CSI or DSI driver hangs after a source rebuild | Running application build instead of kernel module rebuild | Compiling custom apps with `cmake` or `make` does not update the kernel modules. Rebuild the drivers directly as described in [Developing on the QuadRF](develop.md#kernel-drivers-and-overlays-fpga-csi-fpga-dsi):<br>`cd /usr/src/quadrf-fpga-*/csi && sudo make install && sudo quadrf-load` |
| `quadrf.local` does not resolve on LAN | Router blocks mDNS multicast, or Avahi daemon stopped | Check daemon status: `systemctl status avahi-daemon quadrf-mdns`. If your Wi-Fi network blocks multicast traffic between clients, connect directly via IP (`ssh dietpi@<IP>`) or use the fallback AP (`192.168.44.1`). |
| `quadrf-soapy-server` crashes or loops on restart | Port 55132 conflict with upstream SoapyRemote | The stock Debian `soapyremote-server` package conflicts with `quadrf-soapy`. Verify the stock service is masked: `sudo systemctl mask soapyremote-server && sudo systemctl restart quadrf-soapy-server`. |
| Fallback Wi-Fi hotspot does not appear | Hotspot disabled in configuration or hostapd failure | Verify `QUADRF_WIFI_MODE` is not set to `off` in `/etc/quadrf/quadrf.conf`. Check the service log: `journalctl -u hostapd -u quadrf-hotspot`. |
| APT or HTTPS fails while connected to fallback AP | Captive portal DNS redirect active in dnsmasq | When acting as an AP, dnsmasq redirects queries. Comment out `address=/#/` in `/etc/dnsmasq.d/quadrf-wlan0-ap.conf` and restart dnsmasq: `sudo systemctl restart dnsmasq`. |
| USB gadget IP (`10.55.0.1`) unreachable or brownouts occur | Host USB port cannot supply 5V @ 5A, or connected to charge-only port | Connect to the Raspberry Pi 5 USB-C data port, not the UPS charging jack. Most host laptop USB ports cannot supply sufficient current under RF load, causing Pi brownouts. Power the Pi separately from the USB gadget data connection. |
| Direct Ethernet (`10.55.1.1`) connection slow to respond | Carrier debounce timing | `quadrf-ethernet` waits approximately 12 seconds for an upstream DHCP server before falling back to static IP `10.55.1.1`. Wait for this fallback window to expire. |
| Nginx fails to start | Syntax error in generated site configuration | Run `nginx -t` to identify syntax errors. Regenerate the default site configurations by running `sudo quadrf apply`. |
| QuadRF services show `disabled` | Services not enabled after manual package installation | Enable all standard appliance services:<br>`sudo systemctl enable --now load-quadrf quadrf-gui quadrf-hotspot quadrf-ethernet quadrf-usb quadrf-desktop quadrf-ups quadrf-soapy-server` |

---

## Recovery Procedures

### Reloading Drivers Without Rebooting

If modifying C driver code (`fpga-csi.c` or `fpga-dsi.c`), reload the kernel modules directly:

```bash
sudo quadrf-load
```

> **Note:** If you modify the device tree source (`.dts`), you must reboot the board (`sudo systemctl reboot --force`) so the bootloader can reload the compiled `.dtbo` overlay into memory.

### Resetting Appliance Configuration

If configuration files under `/etc/nginx/`, `/etc/dnsmasq.d/`, or `/etc/hostapd/` become corrupted, regenerate all configuration files from the master settings in `/etc/quadrf/quadrf.conf`:

```bash
sudo quadrf apply
```

### Restoring Stock Packages

To discard all local source builds under `/usr/local/` or `/usr/src/` and restore packaged binaries:

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

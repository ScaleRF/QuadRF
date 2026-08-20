# Overview

A QuadRF kit uses the QuadRF SDR tile, a Raspberry Pi 5, an optional UPS battery, and the `quadrf-*`
packages.

## Layers

```mermaid
flowchart TB
  subgraph surfaces["Surfaces"]
    WEB["Browser: nginx"]
    KASM["KasmVNC desktop"]
    TERM["SSH / terminal"]
  end

  subgraph apps["Applications"]
    FLASK["Control panel (Flask)"]
    DEMOS["Demo applications"]
    GRC["GNU Radio Companion"]
    SOAPY_CLI["SoapySDRUtil"]
  end

  subgraph sdr["SDR stack"]
    SOAPY["SoapySDR + quadrf modules"]
    GR["GNU Radio"]
    REMOTE["SoapyRemote server"]
  end

  subgraph fpga["FPGA and drivers"]
    JTAG["quadrf-jtag"]
    KO["fpga-csi.ko / fpga-dsi.ko"]
    BIT["quadrf.svf bitstream"]
  end

  WEB --> FLASK
  WEB --> KASM
  WEB --> DEMOS
  KASM --> DEMOS
  KASM --> GRC
  TERM --> JTAG
  FLASK --> JTAG
  DEMOS --> KO
  GRC --> SOAPY
  SOAPY_CLI --> SOAPY
  REMOTE --> SOAPY
  SOAPY --> KO
  JTAG --> BIT
  KO --> BIT
```

| Layer | Role | Package |
|-------|------|---------|
| FPGA and drivers | Program the bitstream, load the CSI and DSI modules, initialise the front-end | `quadrf-fpga`, `quadrf-fpga-dkms`, `quadrf-boot` |
| SDR stack | Present the hardware to SoapySDR clients, locally and over the network | `quadrf-soapy` |
| Surfaces | Web control panel, remote desktop, shell | `quadrf-gui`, `quadrf-desktop`, `quadrf-network` |

## Reaching the QuadRF

| Context | Address | Notes |
|---------|---------|-------|
| Home LAN | `quadrf.local` | mDNS through avahi. Ethernet or Wi-Fi with a router DHCP lease. Initial setup uses HTTP; controls use HTTPS. |
| Ethernet direct-connect | `10.55.1.1` | No router on the cable: Pi is `10.55.1.1`, the PC gets a lease from dnsmasq. No gateway. |
| USB gadget | `10.55.0.1` | `g_ether` on `usb0` when a host enumerates the gadget. Same names as Ethernet. Most laptops cannot provide 5V at 5A, causing possible brownouts. Please isolate the Pi's power from the usb connection to a device. |
| Fallback access point | `192.168.44.1` | SSID `QuadRF`, open unless `QUADRF_AP_PASS` is set, started when no saved Wi-Fi answers |

Use `http://quadrf.local/setup/security/` once to install the per-install local
root certificate, then use `https://quadrf.local/` as the canonical browser
address in every mode. The direct IP addresses are fallbacks when the client
does not support mDNS.

## Web entry points

nginx answers on ports 80 and 443. HTTP remains available for first-use
certificate installation; controls, AR camera access and WebSockets use HTTPS.

| Path / name | Serves | Backend |
|------|--------|---------|
| `/` | Control panel | Flask on 8080 |
| `quadrfd.local`, `quadrf-desktop.local`, or `:6080` | Remote desktop | KasmVNC on 8444 |
| `quadrfd.local/split` | Desktop + control panel | Kasm iframe + Flask |
| `/AR/` | Browser AR overlay | Static, `/usr/share/quadrf/ar/` |
| `/ws` | WebSocket for Spatial RF Vision | `quadrf-rf-vision` on 8000 |
| `/setup/security/` | Platform-detected HTTPS setup | Instructions for iOS, Android, macOS, Windows, Linux, ChromeOS and common browsers, plus locally generated root downloads |

`/GUI/` redirects to `/`. `/GUI/split` on the desktop host redirects to `/split`.
`/VNC/` redirects to port 6080. KasmVNC is not served
under a path prefix; it needs the root of a host (or port). The desktop
hostname is `HOSTNAMEd.local` (and `HOSTNAME-desktop.local`) rather than `desktop.HOSTNAME.local`
because browsers only multicast-resolve a single label under `.local`.

The control panel also exposes `/api/status`, `/api/control` and `/api/apps`.

## Service chain

```text
load-quadrf.service          OpenOCD -> bitstream -> drivers -> quadrf-jtag --init
        |
quadrf-gui.service           control panel on 8080 (starts even if the radio is missing)
quadrf-soapy-server.service  SoapyRemote on 55132
        |
nginx.service                front end for /, HOSTNAMEd.local, HOSTNAME-desktop.local, :6080, /AR/, /ws
quadrf-desktop.service       KasmVNC on 8444
quadrf-ups.service           UPS HAT state for the desktop panel
quadrf-hotspot.service       access point when no known network answers
quadrf-ethernet.service      eth0 DHCP client, or 10.55.1.1 if no lease; follows carrier
quadrf-usb.service           usb0 island at 10.55.0.1 while a gadget host is attached
```

`quadrf-soapy-server` wants `load-quadrf` but still starts if the FPGA bring-up
fails. The GUI, desktop, nginx and the network units do not require the radio,
so a unit with no board attached still offers the web surfaces and the fallback
access point.

## Paths

| Path | Contents |
|------|----------|
| `/etc/quadrf/quadrf.conf` | Site settings |
| `/usr/bin/quadrf` | Status and configuration command |
| `/usr/bin/quadrf-jtag` | Front-end control |
| `/usr/bin/quadrf-rf-vision`, `-psd`, `-ntsc` | Demo applications |
| `/usr/share/quadrf/` | Bitstream, OpenOCD configuration, GUI, AR page, flowgraphs, desktop assets |
| `/usr/lib/quadrf/apply.d/` | Configuration hooks run by `quadrf apply` |
| `/var/lib/quadrf/` | Runtime state, including the pending-reboot record |

## Desktop

The KasmVNC session runs openbox with a tint2 panel and pcmanfm drawing the
desktop. Launchers are installed as normal desktop entries in
`/usr/share/applications` and copied into the `Desktop` directory,
each with `TryExec` so entries for software that is not installed stay hidden.

| Launcher | Runs | Package |
|----------|------|---------|
| Spatial RF Vision | `quadrf-rf-vision` | `quadrf-demos` |
| PSD Plot | `quadrf-psd` | `quadrf-demos` |
| Camera Decoder | `quadrf-ntsc` | `quadrf-demos` |
| GNU Radio | `gnuradio-companion` | `quadrf-gnuradio` |
| QRadioLink | `qradiolink` | `quadrf-desktop` |
| Terminal, Software Install | `xfce4-terminal`, `/boot/dietpi/dietpi-software` | `quadrf-desktop` |

## Demos

| Binary | Name | What it does |
|--------|------|--------------|
| `quadrf-rf-vision` | Spatial RF Vision | Swept-LO phase scatter in an SDL window, WebSocket feed on port 8000 for `/AR/` |
| `quadrf-psd` | PSD Plot | FFT spectrum from the CSI ring buffer, one or four channels |
| `quadrf-ntsc` | Camera Decoder | SoapySDR receive into an NTSC decoder, played by mpv; Decode drone 5GHz analog FPV video live |

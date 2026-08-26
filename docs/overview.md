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
    DEMOS["Included applications"]
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
| Home LAN | `quadrf.local` | mDNS, one A record per path (ethernet, USB, client, hotspot). A client on the LAN sees the LAN address, not the hotspot. Initial setup uses HTTP; controls use HTTPS. |
| Ethernet direct-connect | `10.55.1.1` | No router on the cable: Pi is `10.55.1.1`, the PC gets a lease from dnsmasq. No gateway. |
| USB gadget | `10.55.0.1` | `g_ether` on `usb0` when a host enumerates the gadget. Same names as Ethernet. Most laptops cannot provide 5V at 5A, causing possible brownouts. Please isolate the Pi's power from the usb connection to a device. |
| Fallback access point | `192.168.44.1` | SSID `QuadRF`, open unless `QUADRF_AP_PASS` is set. Default boot tries a saved client network, then this AP. The control panel can switch client, hotspot, or off without a reboot. |

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

The control panel also exposes `/api/status`, `/api/control`, `/api/apps` and `/api/network/*` for live Wi-Fi mode.

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
quadrf-hotspot.service       wlan0 client, hotspot, or off (honours last GUI mode)
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
| `/usr/bin/quadrf-rf-vision`, `-psd`, `-ntsc` | Included radio applications |
| `/usr/share/quadrf/` | Bitstream, OpenOCD configuration, GUI, AR page, flowgraphs, desktop assets |
| `/usr/lib/quadrf/apply.d/` | Configuration hooks run by `quadrf apply` |
| `/var/lib/quadrf/` | Runtime state, including the pending-reboot record |

## Applications

The remote desktop is a Linux session in the browser (KasmVNC). A full install
includes the applications below. Start Spatial RF Vision, Camera Decoder, and
PSD Plot from the Applications list on the control page or from the desktop
icons. Only one of those three can use the radio at a time.

Install a `.deb` with a desktop entry and/or a control-page descriptor, and
it shows up on the desktop and Applications list automatically. See [Applications](applications.md).

| Application | What it does | Opens |
|-------------|--------------|-------|
| Spatial RF Vision | Swept-LO phase scatter of the RF scene; WebSocket feed on port 8000 for `/AR/` | Browser AR page |
| Camera Decoder | NTSC demodulation of analog 5 GHz FPV video, played with mpv | Remote desktop |
| PSD Plot | Live FFT spectrum from the CSI ring buffer, one or four channels | Remote desktop |
| GNU Radio Companion | Flowgraph editor with QuadRF receive and transmit examples | Remote desktop |
| QRadioLink | Digital-voice and analog transceiver | Remote desktop |
| Terminal, Text Editor, Software Install | Shell, Mousepad, and DietPi's software installer | Remote desktop |

`quadrf-headless` omits the desktop and the three radio applications.

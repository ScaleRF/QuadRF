# Applications

Applications on the QuadRF integrate with two user surfaces: the operator remote desktop (`:1`) and the web control panel at `https://quadrf.local/`.

Applications packaged as Debian packages (`.deb`) register automatically upon installation:

| Target Surface | Required File in Package | Purpose |
| --- | --- | --- |
| **Remote Desktop** | `/usr/share/applications/*.desktop`<br>`/usr/share/icons/hicolor/...` | Desktop launcher with `X-QuadRF-Desktop=true` and application icon. |
| **Web Control Panel** | `/lib/systemd/system/*.service`<br>`/usr/share/quadrf/apps.d/*.json` | Systemd unit for execution lifecycle and catalog JSON descriptor. |

To test and register custom binaries directly on the board without creating a Debian package, see [Developing on the QuadRF](develop.md#2-registering-apps-on-the-desktop-and-web-ui).

---

## 1. Included Applications

The complete `quadrf` package ships with the following built-in applications:

| Application | Description | Surface |
| --- | --- | --- |
| **Spatial RF Vision** | Swept-LO phase scatter of the RF scene (30 fps) | Browser AR view at `/AR/` |
| **Camera Decoder** | Real-time NTSC demodulation of analog FPV video | Remote desktop |
| **PSD Plot** | Live FFT power spectral density (1 to 4 channels) | Remote desktop |
| **Near-Field Phasors** | Coherent 4x4 MIMO near-field phasors with TDM TX cycling | Remote desktop |
| **GNU Radio Companion** | Flowgraph development with QuadRF source/sink blocks | Remote desktop |
| **QRadioLink** | Multi-mode digital voice and analog transceiver | Remote desktop |
| **Terminal** | Appliance shell session | Remote desktop |
| **File Manager** | PCManFM graphical file manager | Remote desktop |
| **Text Editor** | Mousepad editor | Remote desktop |
| **Software Install** | DietPi package management interface | Remote desktop |

The `quadrf-headless` package excludes the desktop environment and bundled graphical tools. To remove only the four hardware demonstration programs from a standard install:

```bash
sudo apt remove quadrf-demos
```

---

## 2. Installing Third-Party Applications

Install standalone Debian packages directly using `apt`:

```bash
sudo apt install ./my-radio-app_1.0.0_arm64.deb
```

For applications hosted in an APT repository:

```bash
sudo apt update
sudo apt install my-radio-app
```

Installation places the desktop launcher on `/home/dietpi/Desktop/` and adds the application to the web control panel. Removing the package unregisters both interfaces:

```bash
sudo apt remove my-radio-app
```

Applications can be started or stopped from the web UI drawer, or via terminal with `sudo quadrf-app start <id>` and `sudo quadrf-app stop <id>` (see [Developing on the QuadRF](develop.md#test-and-control-the-application) or `man quadrf-app`).

---

## 3. Packaging an Application (`.deb`)

To distribute an application, package the binary, desktop entry, systemd unit, and catalog descriptor into a Debian package (`arm64` for compiled binaries, `all` for scripts).

> **Tip:** Before authoring package metadata, verify your binary on the board using `sudo quadrf-app register` as described in [Developing on the QuadRF](develop.md#2-registering-apps-on-the-desktop-and-web-ui).

### Package File Layout

```text
/usr/bin/my-app
/usr/share/applications/com.example.MyApp.desktop
/usr/share/icons/hicolor/scalable/apps/com.example.MyApp.svg
/lib/systemd/system/my-app.service
/usr/share/quadrf/apps.d/my-app.json
/usr/share/metainfo/com.example.MyApp.metainfo.xml (optional)
```

---

### Desktop Entry

Desktop files must use reverse-DNS naming. The `X-QuadRF-Desktop=true` flag instructs the desktop session to pin the launcher to `/home/dietpi/Desktop/`:

```ini
[Desktop Entry]
Type=Application
Version=1.0
Name=My Radio App
Comment=Real-time spectrum monitor
Exec=/usr/bin/my-app
TryExec=/usr/bin/my-app
Icon=com.example.MyApp
Terminal=false
Categories=HamRadio;Science;
X-QuadRF-Desktop=true
```

- `TryExec`: Hides the icon if the binary is absent or unexecutable.
- `Icon`: Name of the installed SVG/PNG icon (without file extension or directory path).
- Validate the entry using: `desktop-file-validate com.example.MyApp.desktop`.

---

### Systemd Service Unit

The systemd service manages execution lifecycle for the web control panel. Because GUI applications run within the KasmVNC session, the unit must target `DISPLAY=:1` under user `dietpi`:

```ini
[Unit]
Description=My Radio App
Wants=load-quadrf.service quadrf-desktop.service
After=load-quadrf.service quadrf-desktop.service

[Service]
Type=simple
User=dietpi
Group=dietpi
Environment=HOME=/home/dietpi
Environment=DISPLAY=:1
Environment=XAUTHORITY=/home/dietpi/.Xauthority
Environment=SDL_VIDEODRIVER=x11
ExecStart=/usr/bin/my-app
TimeoutStopSec=8
Restart=no

[Install]
# Do not enable at boot; quadrf-app starts the unit on demand.
```

---

### Catalog Descriptor (`apps.d/*.json`)

The web UI reads application metadata from JSON files in `/usr/share/quadrf/apps.d/`:

```json
{
  "apps": [
    {
      "id": "my-app",
      "desktop_entry": "com.example.MyApp.desktop",
      "service": "my-app.service",
      "binaries": ["my-app"],
      "exclusive": true,
      "open": "desktop"
    }
  ]
}
```

| Field | Required | Description |
| --- | --- | --- |
| `id` | Yes | Unique string matching `^[a-z0-9][a-z0-9._-]{0,63}$`. |
| `desktop_entry` | Recommended | Basename of the `.desktop` file used to extract the display name, comment, and icon. |
| `service` | Yes | Systemd unit name (`<name>.service`). |
| `binaries` | Recommended | Array of process names stopped via `pkill` if the user also launched a copy outside systemd. |
| `exclusive` | No | Set to `true` if the app opens CSI/DSI device nodes. Starting an exclusive app stops other running radio apps. |
| `open` | No | Destination after launch: `"desktop"` runs on the KasmVNC session without opening a tab; URL paths like `"/AR/"` open that endpoint in a new tab. |
| `ready_port` | No | Local TCP port that must accept connections before the application is marked running. |

---

### Package Maintainer Scripts and Triggers

- **Desktop sync trigger**: The `quadrf-desktop` package registers a dpkg trigger on `/usr/share/applications`. When a package installs or removes a `.desktop` file with `X-QuadRF-Desktop=true`, desktop icons update automatically.
- **Service reloading**: Run `systemctl daemon-reload` in `postinst` and `postrm` to register the new service unit with systemd:

```bash
#!/bin/sh
set -e
if [ "$1" = "configure" ]; then
    systemctl daemon-reload
fi
```

---

### Software Catalog Metadata (AppStream)

To publish metadata to software centers, include an AppStream metainfo file:

```text
/usr/share/metainfo/com.example.MyApp.metainfo.xml
```

Validate with:

```bash
appstreamcli validate com.example.MyApp.metainfo.xml
```

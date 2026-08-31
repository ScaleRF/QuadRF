# Applications

If a `.deb` installed on the QuadRF includes a desktop entry and control-page descriptor, an icon appears on the remote desktop and a launcher appears on the Applications list at
[https://quadrf.local/](https://quadrf.local/). QuadRF picks them up on `apt install` and drops them on `apt remove`. To register the same files by hand without packaging, see [Developing](develop.md#3-create-an-app-with-desktop-and-web-ui-launchers).

| File in the `.deb` | Function |
|--------------------|----------|
| A `.desktop` file with `X-QuadRF-Desktop=true`, plus an icon | Remote desktop |
| `/usr/share/quadrf/apps.d/*.json` and a systemd unit | Control page |

## Included applications

These come with the complete `quadrf` package.

| Application | What it does | Opens |
|-------------|--------------|-------|
| Spatial RF Vision | Swept-LO phase scatter of the RF scene, 30 fps | Browser AR page at `/AR/` |
| Camera Decoder | NTSC demodulation of analog FPV video | Remote desktop |
| PSD Plot | Live FFT spectrum, one or four channels | Remote desktop |
| Near-Field Phasors | Coherent 4x4 MIMO near-field phasors with TDM TX | Remote desktop |
| GNU Radio Companion | Flowgraph editor with QuadRF examples | Remote desktop |
| QRadioLink | Digital-voice and analog transceiver | Remote desktop |
| Terminal | Shell on the appliance | Remote desktop |
| File Manager | PCManFM file browser | Remote desktop |
| Text Editor | Mousepad, for config and source files | Remote desktop |
| Software Install | DietPi software installer | Remote desktop |

The `quadrf-headless` metapackage omits the remote desktop and every
application in the table. To drop only the four radio apps from a full
install:

```sh
sudo apt remove quadrf-demos
```

## Installing another application

If you have a `.deb` file:

```sh
sudo apt install ./example_1.0_arm64.deb
```

If there's an APT repository, add it, then:

```sh
sudo apt update
sudo apt install example
```

The desktop icon and control-page row appear as soon as the package is
installed, for whichever of those files the `.deb` included. Removing the
package removes both:

```sh
sudo apt remove example
```

A Debian package and its maintainer scripts run as root. Install packages and
repository signing keys only from publishers you trust.

## Make and share an application

Build a Debian package for the QuadRF (`arm64` if you compiled anything, or
`all` if the contents are architecture-independent). Put the `.deb` on a
release page, or publish a signed APT repository so people can update with
`apt`.

### Desktop icon

Install the program plus a standard desktop entry and icon:

```text
/usr/bin/example-spectrum
/usr/share/applications/org.example.ExampleSpectrum.desktop
/usr/share/icons/hicolor/scalable/apps/org.example.ExampleSpectrum.svg
```

Name the desktop file with a reverse-DNS id based on a domain you control.
`X-QuadRF-Desktop=true` is the one QuadRF-specific line: it opts the icon onto
the remote desktop. Use an icon *name*, not a file path.

```ini
[Desktop Entry]
Type=Application
Version=1.0
Name=Example Spectrum
Comment=View the example receiver spectrum
Exec=example-spectrum
TryExec=example-spectrum
Icon=org.example.ExampleSpectrum
Terminal=false
Categories=HamRadio;Science;
X-QuadRF-Desktop=true
```

`TryExec` hides the icon if the program is missing. Check the file with
`desktop-file-validate org.example.ExampleSpectrum.desktop`. The entry follows
the [Desktop Entry Specification](https://specifications.freedesktop.org/desktop-entry/latest-single/);
named icons follow the
[Icon Theme Specification](https://specifications.freedesktop.org/icon-theme/latest/).

That is enough for a desktop-only application. GNU Radio Companion on QuadRF
is this kind of app: an icon, no control-page row.

### Control page

To start and stop the app from the browser, the same package also installs a
systemd unit (not enabled at boot) and a short JSON file:

```text
/lib/systemd/system/quadrf-example-spectrum.service
/usr/share/quadrf/apps.d/example-spectrum.json
```

The unit must run as `dietpi` on display `:1`. QuadRF does not inject that
for third-party services.

```ini
[Unit]
Description=Example Spectrum
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
ExecStart=/usr/bin/example-spectrum
TimeoutStopSec=8
Restart=no
```

```json
{
  "apps": [
    {
      "id": "org.example.example-spectrum",
      "desktop_entry": "org.example.ExampleSpectrum.desktop",
      "service": "quadrf-example-spectrum.service",
      "binaries": ["example-spectrum"],
      "exclusive": true,
      "open": "desktop"
    }
  ]
}
```

Keep the visible name, description, and icon in the `.desktop` file. The JSON
only tells QuadRF which service to start:

| Field | Required | Meaning |
|-------|----------|---------|
| `id` | Yes | Identifier the control page uses. Letters, digits, dots, underscores, hyphens. |
| `desktop_entry` | Recommended | Basename of the `.desktop` file (name, comment, and icon). |
| `service` | Yes | systemd unit to start and stop. |
| `binaries` | Recommended | Process names to stop if a copy was launched from the desktop instead. |
| `exclusive` | No | `true` if the app uses the radio, so starting it stops other radio apps. |
| `open` | No | `desktop` opens the remote desktop; a path such as `/AR/` opens that page. |
| `ready_port` | No | Local TCP port that must accept connections before launch is reported as ready. |

The control page sends only a registered `id`. It never runs a command line
from the browser.

### Software catalog metadata

AppStream metadata is optional. Include it if you want the app described in
standard software catalogs:

```text
/usr/share/metainfo/org.example.ExampleSpectrum.metainfo.xml
```

Use the same reverse-DNS id as the desktop file, keep the metainfo in the same
package as the app, and check it with:

```sh
appstreamcli validate org.example.ExampleSpectrum.metainfo.xml
```

See the [AppStream metadata documentation](https://www.freedesktop.org/software/appstream/docs/chap-Metadata.html).

## Command line

The same start/stop path is available over SSH:

```sh
quadrf-app status
sudo quadrf-app start psd
sudo quadrf-app stop psd
```

The included radio apps use the ids `ar` (Spatial RF Vision), `ntsc` (Camera
Decoder), `psd` (PSD Plot), and `nearfield` (Near-Field Phasors). See
`man quadrf-app`.

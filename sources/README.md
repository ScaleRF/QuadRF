# Source trees

| Directory | Contents |
|-----------|----------|
| `common/` | `quadrf.conf` and the shell helpers shared by the maintainer scripts |
| `cli/` | The `quadrf` command |
| `boot/` | Apply hook for the firmware configuration |
| `fpga/` | `quadrf.svf` placeholder (factory bitstream is not published), `jtag` CLI, kernel driver sources, device-tree overlays, OpenOCD board configuration |
| `soapy/` | SoapySDR MIPI module |
| `quadrfd/` | QuadRF Soapy module and thin client |
| `flask/` | Web control panel |
| `network/` | nginx, dnsmasq, hostapd, DHCP, interface drop-ins, hotspot and Wi-Fi scripts |
| `demos/` | Spatial RF Vision, PSD plot, NTSC decoder, near-field phasors and the AR page |
| `demos/apps/` | Control-page runtime descriptors owned by the demo package |
| `phasegaze-demo/` | Larger spatial application, shipped as source |
| `desktop/` | KasmVNC session, openbox and tint2 configuration, launchers |
| `icons/` | Icons for the launchers |
| `kasmvnc/` | KasmVNC defaults and the branded `www` overlay |
| `ups_hat/` | UPS HAT daemon and battery icons |
| `grc_projects/` | GNU Radio example flowgraphs |
| `gptme/` | Agentic Radio launcher and workspace |
| `systemd/` | Unit files |
| `man/` | Manual pages |

Files named `NN-name` at the top of a component directory are apply hooks. They
are installed into `/usr/lib/quadrf/apply.d/` and run in order by
`quadrf apply`, so each package configures itself and nothing else.

Upstream: SoapySDR, GNU Radio and the rest come
from Debian; OpenOCD and KasmVNC are handled in `packaging/thirdparty/`.

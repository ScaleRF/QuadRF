# TODO

## Open

| Item | Notes |
|------|-------|
| SoapySDR tuning API | `setFrequency` and `getFrequency` are stubs in both modules: `mipi` is a no-op, `quadrf` only logs the RPC. Wire them to the MAX285x through the JTAG ioctl the `quadrf-jtag` CLI and the Flask GUI already use, so GRC and other Soapy clients can set the LO. |
| Certificate expiry warning | Only relevant after an operator imports a real cert. Renewal is manual DNS-01. Nothing warns before `notAfter`; a check in `quadrf status` would be enough. |
| Protect `quadrf.com` DNS | Packages no longer ship a shared SSL cert. If the zone is owned, lock DNS/CAA and drop any public A record that only serves the hotspot IP. |
| nginx `/` landing page | `/` proxies KasmVNC. A split view with the control panel alongside the desktop was planned and never built. |
| gr-ieee802-11 Wi-Fi demo | Present on the production image but never installed. No launcher ships for it. |
| Agentic Radio workspace | The gptme workspace on the production SD (`lessons/quadrf-api.md`, sweep scripts, `soapy_module/`) is not in this repository. Copy it in rather than rewriting it. The launcher already hides itself when gptme is absent. |
| NTSC demo with HDMI attached | The camera decoder misbehaves when a display is connected to the Pi. |
| mDNS after an idle Kasm session | On Ubuntu clients `quadrf.local` can stop resolving after a few minutes of inactivity and take about 90 seconds to return. |
| Tile connection guide | `docs/tile-connection.md` is a stub: mounting, CSI and DSI cabling, power, pre-install checks. |
| Screenshots | The desktop and the control panel, for the README. |

## Before public release

- [ ] Confirm the licence for `quadrf.svf` and every vendored source
- [x] Remove internal paths and machine-specific notes from the docs
- [ ] Install from the published repository onto a clean SD and verify the result
- [ ] Decide what stays pinned in `packaging/pins.env` and what may float
- [ ] Tag `v1.0.0` against a tested DietPi image and kernel; GitHub Actions writes the first public release notes
- [ ] Enable GitHub Pages (Settings → Pages → GitHub Actions) and set `QUADRF_GPG_PRIVATE_KEY`

# Filing issues and pull requests

A bug or a feature request is a GitHub issue. A code change is a pull request linked to that issue. Use the sections below so reports can be sorted by component, by whether a board is required, and by whether the failure was reproduced.

Repository: https://github.com/ScaleRF/QuadRF

A problem with no patch is an issue. Show the exact title and body, and file only after the user says yes. Commit, push, or merge only when the user explicitly asks.

`gh auth status` has to succeed before `gh issue create`, `gh issue comment`, or `gh pr create`. When `gh` is missing or unauthenticated, leave it that way and give the user the finished text to submit.

Read `CONTRIBUTING.md` before a change to antenna geometry, RF board design, FPGA logic, calibration, or anything that may be export-controlled or covered by a patent claim. Leave out restricted technical data, third-party material of unknown license, and RF-board manufacturing files.

If the repository provides a GitHub issue form, use that form and put the same facts in it.

## Search before filing

A duplicate costs more time than no report.

```bash
gh search issues --repo ScaleRF/QuadRF "<symptom>"
gh issue list --repo ScaleRF/QuadRF --state all --search "<program, register, or frequency>"
```

Search on the failing program, register, frequency, or error text: `quadrf-wifi`, PhaseGaze, `0x6A`, `5500`.

`gh search issues` accepts only `open` or `closed` for `--state`. Leave `--state` off so the search covers both. `gh issue list --state all` already does.

Read a plausible match before deciding:

```bash
gh issue view <number> --repo ScaleRF/QuadRF --comments
```

The same program failing is a different bug when the trigger or the log differs. When it is the same bug, comment on that issue only if the comment adds a reproduction, a log, a frequency, or a version the thread does not have. A closed issue that still reproduces on the current packages is a regression comment on that issue. When the only addition is that the bug happened again, tell the user and file nothing.

```bash
gh issue comment <number> --repo ScaleRF/QuadRF --body "..."
```

## Facts to collect

On a board, run these and paste the output. Leave a radio line blank when the command does not show it.

```bash
hostname
dpkg -l 'quadrf*'
quadrf-app status
ls -l /dev/csi_stream0 /dev/dsi_stream0
quadrf-jtag --status rx
quadrf-jtag --status tx
```

For streaming, DMA, or driver issues, check recent kernel logs: `dmesg | grep -iE 'fpga-csi|fpga-dsi|csi_stream|dsi_stream' | tail -n 40`.

For one service, `journalctl -u <unit> --no-pager -n 80`. Pick `<unit>` from the service that failed (`quadrf-data`, `quadrf-gui`, `quadrf-ntsc`, `quadrf-nearfield`, `quadrf-rf-vision`, and the other `quadrf-*.service` units).

Choose the component label from the program or the path that failed, using the table below.

## Installed tree

`/usr/share/quadrf` is the packaged tree. Read it. A package update overwrites edits there, so a patch is made in a clone of https://github.com/ScaleRF/QuadRF.

## Labels

Apply one component label and one type label (`bug` or `feature`).

| Label | Covers |
| :--- | :--- |
| `hw` | MAX2850/MAX2851, RF switches, PAs, antennas, board |
| `dsp` | Baseband, OFDM, demod, filters, NEON |
| `driver` | CSI/DSI kernel driver, `/dev/csi_stream0`, `/dev/dsi_stream0` |
| `fpga` | ECP5 bitstream and FPGA logic |
| `soapy` | SoapySDR `driver=mipi` |
| `demo` | PhaseGaze, data link, NTSC, nearfield, GNU Radio apps |
| `gui` | Web control panel |
| `network` | Hostname, mDNS, nginx, tunnels |
| `pkg` | Debian packages, CMake, systemd |
| `needs-hardware` | Reproduction or verification needs a QuadRF board |
| `has-repro` | The report includes steps that produced the failure |

Those component names are also the commit prefixes (`dsp:`, `pkg:`, and the rest). Write the component in the body as well, so the report still sorts if a label is missing. Apply a label only when it already exists on the repository.

## Bug

```markdown
## What happened

## What you expected

## How often

always | sometimes | once

## Steps

## Component

hw | dsp | driver | fpga | soapy | demo | gui | network | pkg

## Unit

- Hostname:
- Packages: output of `dpkg -l 'quadrf*'`

## Radio setup

Leave a line out when the value is unknown.

- Frequency (MHz):
- Bandwidth (MHz):
- RX polarization:
- TX gain:
- Antennas:

## Logs

## Notes
```

`Notes` is optional. A cause goes there only with the measurement that supports it. Logs are the command output from "Facts to collect," kept to the failing service.

## Feature

```markdown
## What you are trying to do

## What you do today

## Component

hw | dsp | driver | fpga | soapy | demo | gui | network | pkg

## Unit

- Hostname:
- Packages:
```

Include `Unit` when the request comes from a board.

## Pull request

Link the issue. Use structured YAML frontmatter so maintainer-side AI agents and CI workflows can automatically parse the test commands, target hardware requirements, and reproduction steps:

```markdown
---
component: hw | dsp | driver | fpga | soapy | demo | gui | network | pkg
type: bugfix | enhancement | test | docs
breaking_change: false
hardware_required: true | false
target_hardware: "QuadRF RPi5 + MAX285x"
automated_test_command: "ctest --output-on-failure -R <test-name>"
reproduce_steps:
  - "quadrf-app status"
  - "quadrf-jtag --status rx"
files_changed:
  - "<path>"
---

## Problem

## Root cause

## Change

## Testing

Where it ran: QuadRF board | host without the radio

Command:

Result:

## API or config

changed | unchanged
```

`Change` names the functions or pipeline stages touched and why that approach was taken. `Testing` uses a command from the next section and includes the command output. Paths come from the diff.

## Build and test

From a `build` directory:

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
ctest --output-on-failure
```

One existing test:

```bash
ctest --output-on-failure -R <test-name>
```

`<test-name>` is a test already registered with CTest. On a board, also include the `quadrf-jtag --status` lines for the setup under test.

## Branches and commits

Branch: `<component>/<short-description>`, for example `dsp/fix-viterbi-buffer` or `pkg/add-wifi-demo-install`.

Commit subject, imperative, with the component prefix: `dsp: Fix RX buffer overrun`.

## Signing

End an issue, a comment, or a pull request with the model and harness that wrote it:

> Filed by <model name> via <agent harness>.

Use the real names. When either name is uncertain, say so in that line.

## Filing from this tree

After the user has agreed to the text:

```bash
gh issue create --repo ScaleRF/QuadRF --title "..." --label bug,dsp,needs-hardware --body "..."
gh pr create --repo ScaleRF/QuadRF --title "..." --body "..."
```

Pass the body with a heredoc. Apply a label only when it already exists on the repository.

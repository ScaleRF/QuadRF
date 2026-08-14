# Agentic Radio

The Agentic Radio launcher runs [gptme](https://gptme.org) in a terminal. gptme
is not packaged here: install it yourself (`pipx install gptme`) and the
launcher stops hiding itself, because the desktop entry carries `TryExec=gptme`.

gptme needs an API key in `~/.config/gptme/env`; `config/env.example` shows the
shape of that file. Keep it at mode 600 and out of git.

The QuadRF agent workspace — the `lessons/quadrf-api.md` context file, the sweep
and spectrogram helpers, the flowgraph and `soapy_module/` — lives on the
reference image under `~/gptme_code` and has not been brought into this
repository yet. Copy it from there rather than writing it again; when it lands,
install it to `/usr/share/quadrf/agent` and point the launcher at it with
`--workspace`.

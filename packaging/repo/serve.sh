#!/bin/bash
# Serve the signed apt tree over HTTP for LAN installs.
# Default bind is all interfaces on port 8080 so a unit can reach this host.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GNUPGHOME="${GNUPGHOME:-${here}/.gnupg}"
port="${QUADRF_REPO_PORT:-8080}"
bind="${QUADRF_REPO_BIND:-0.0.0.0}"

command -v python3 >/dev/null || { echo "python3 is not installed" >&2; exit 1; }

if [ ! -d "${here}/dists" ] || [ ! -d "${here}/pool" ]; then
    echo "repository is empty; run: make -C packaging repo" >&2
    exit 1
fi

"${here}/export-key.sh"

# Pick a LAN address for the install snippet when one is obvious.
hint=""
if command -v hostname >/dev/null; then
    for addr in $(hostname -I 2>/dev/null); do
        case "${addr}" in
            127.*|172.1[6-9].*|172.2[0-9].*|172.3[0-1].*|10.*|::*) continue ;;
            *) hint="http://${addr}:${port}"; break ;;
        esac
    done
fi
if [ -z "${hint}" ]; then
    hint="http://<this-host>:${port}"
fi

cat <<EOF
Serving ${here} on ${bind}:${port}
Install on a unit with:

  curl -fsSL ${hint}/quadrf.gpg | sudo tee /etc/apt/keyrings/quadrf.gpg >/dev/null
  echo 'deb [signed-by=/etc/apt/keyrings/quadrf.gpg] ${hint} trixie main' \\
    | sudo tee /etc/apt/sources.list.d/quadrf.list
  sudo apt update
  sudo apt install quadrf

EOF

cd "${here}"
exec python3 -m http.server "${port}" --bind "${bind}"

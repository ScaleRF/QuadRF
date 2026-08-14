#!/bin/bash
# Mirror the pinned KasmVNC package into out/ so the QuadRF repository can
# serve it: quadrf-desktop depends on kasmvncserver, which Debian does not ship.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../pins.env"

OUT="${OUT:-${here}/../out}"
name="kasmvncserver_${KASMVNC_SUITE}_${KASMVNC_VERSION}_${KASMVNC_ARCH}.deb"
url="https://github.com/kasmtech/KasmVNC/releases/download/v${KASMVNC_VERSION}/${name}"

mkdir -p "${OUT}"
target="${OUT}/${name}"

if [ -f "${target}" ] && echo "${KASMVNC_SHA256}  ${target}" | sha256sum -c - >/dev/null 2>&1; then
    echo "kasmvnc: ${name} already present"
    exit 0
fi

echo "kasmvnc: fetching ${url}"
curl -fL --retry 3 -o "${target}.part" "${url}"
echo "${KASMVNC_SHA256}  ${target}.part" | sha256sum -c -
mv "${target}.part" "${target}"
echo "kasmvnc: ${target}"

#!/bin/bash
# Mirror the pinned quadrf-mesh packages into out/ so the QuadRF repository
# can serve them. The full metapackage Depends on quadrf-mesh, which Debian
# does not ship.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../pins.env"

OUT="${OUT:-${here}/../out}"
mkdir -p "${OUT}"

fetch_deb() {
    local name="$1"
    local sha="$2"
    local target="${OUT}/${name}"
    local url="${QUADRF_MESH_REPO}/releases/download/${QUADRF_MESH_TAG}/${name}"

    if [ -f "${target}" ] && echo "${sha}  ${target}" | sha256sum -c - >/dev/null 2>&1; then
        echo "quadrf-mesh: ${name} already present"
        return 0
    fi

    echo "quadrf-mesh: fetching ${url}"
    curl -fL --retry 3 -o "${target}.part" "${url}"
    echo "${sha}  ${target}.part" | sha256sum -c -
    mv "${target}.part" "${target}"
    echo "quadrf-mesh: ${target}"
}

fetch_deb "quadrf-mesh_${QUADRF_MESH_VERSION}_all.deb" "${QUADRF_MESH_SHA256}"
fetch_deb "quadrf-lora-phy_${QUADRF_MESH_VERSION}_arm64.deb" "${QUADRF_MESH_LORA_PHY_SHA256}"
fetch_deb "quadrf-meshtasticd_${QUADRF_MESH_VERSION}_arm64.deb" "${QUADRF_MESH_MESHTASTICD_SHA256}"

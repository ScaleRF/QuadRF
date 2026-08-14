#!/bin/bash
# Copy the quadrf-phy / quadrf-meshtasticd .debs, already built in the
# quadrf-mesh repo (./packaging/build-all.sh there), into out/ here.
#
# Mesh packages are optional Recommends. This is a local copy from a sibling
# checkout when QUADRF_MESH_DIR is set — not an automated fetch. GitHub
# Actions in this repository skip this target.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../../pins.env"

OUT="${OUT:-${here}/../../out}"
QUADRF_MESH_DIR="${QUADRF_MESH_DIR:-${here}/../../../../quadrf-mesh}"
SRC="${QUADRF_MESH_DIR}/packaging/out"

if [ ! -d "${QUADRF_MESH_DIR}/.git" ]; then
    echo "mirror-quadrf-mesh: no repo at ${QUADRF_MESH_DIR} (set QUADRF_MESH_DIR)" >&2
    exit 1
fi

actual_commit="$(git -C "${QUADRF_MESH_DIR}" rev-parse --short HEAD)"
if [ "${actual_commit}" != "${QUADRF_MESH_COMMIT}" ]; then
    echo "mirror-quadrf-mesh: warning: ${QUADRF_MESH_DIR} is at ${actual_commit}," \
         "pins.env expects ${QUADRF_MESH_COMMIT}" >&2
fi

mkdir -p "${OUT}"
phy_deb="${SRC}/quadrf-phy_${QUADRF_PHY_VERSION}_arm64.deb"
mesh_deb="${SRC}/quadrf-meshtasticd_${QUADRF_MESHTASTICD_VERSION}_arm64.deb"

for deb in "${phy_deb}" "${mesh_deb}"; do
    [ -f "${deb}" ] || {
        echo "mirror-quadrf-mesh: missing ${deb}" >&2
        echo "  run './packaging/build-all.sh' in ${QUADRF_MESH_DIR} first" >&2
        exit 1
    }
    cp -f "${deb}" "${OUT}/"
    echo "mirror-quadrf-mesh: $(basename "${deb}")"
done

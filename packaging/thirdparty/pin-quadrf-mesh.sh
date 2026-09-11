#!/bin/bash
# Point packaging/pins.env at a quadrf-mesh GitHub release (tag or latest).
# The quadrf metapackage Depends on QUADRF_MESH_VERSION at build time.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
pins="${here}/../pins.env"
. "${pins}"

repo="${QUADRF_MESH_REPO#https://github.com/}"
tag="${1:-}"

if [ -z "${tag}" ]; then
    tag="$(curl -fsSL -H 'Accept: application/vnd.github+json' \
        "https://api.github.com/repos/${repo}/releases/latest" \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
fi
case "${tag}" in
    v*) ;;
    *) tag="v${tag}" ;;
esac

sums="$(mktemp)"
trap 'rm -f "${sums}"' EXIT
echo "quadrf-mesh: pinning ${tag} from ${QUADRF_MESH_REPO}"
curl -fsSL -o "${sums}" "${QUADRF_MESH_REPO}/releases/download/${tag}/SHA256SUMS"

sha_for() {
    local name="$1" line sha
    line="$(awk -v n="${name}" '$2 ~ "/" n "$" || $2 == n {print; exit}' "${sums}")"
    if [ -z "${line}" ]; then
        echo "quadrf-mesh: ${name} not in ${tag} SHA256SUMS" >&2
        exit 1
    fi
    sha="${line%% *}"
    printf '%s\n' "${sha}"
}

mesh_name="$(awk '/quadrf-mesh_.*_all\.deb$/ {n=$2; sub(".*/","",n); print n; exit}' "${sums}")"
if [ -z "${mesh_name}" ]; then
    echo "quadrf-mesh: no quadrf-mesh_*_all.deb in ${tag} SHA256SUMS" >&2
    exit 1
fi
version="${mesh_name#quadrf-mesh_}"
version="${version%_all.deb}"

mesh_sha="$(sha_for "quadrf-mesh_${version}_all.deb")"
phy_sha="$(sha_for "quadrf-lora-phy_${version}_arm64.deb")"
mtd_sha="$(sha_for "quadrf-meshtasticd_${version}_arm64.deb")"

set_pin() {
    local key="$1" value="$2"
    python3 -c '
import pathlib, sys
path, key, value = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
text = path.read_text()
prefix = key + "="
found = False
out = []
for line in text.splitlines(True):
    if line.startswith(prefix):
        out.append(prefix + value + ("\n" if line.endswith("\n") else ""))
        found = True
    else:
        out.append(line)
if not found:
    sys.exit(f"{key} not found in {path}")
path.write_text("".join(out))
' "${pins}" "${key}" "${value}"
}

set_pin QUADRF_MESH_VERSION "${version}"
set_pin QUADRF_MESH_TAG "${tag}"
set_pin QUADRF_MESH_SHA256 "${mesh_sha}"
set_pin QUADRF_MESH_LORA_PHY_SHA256 "${phy_sha}"
set_pin QUADRF_MESH_MESHTASTICD_SHA256 "${mtd_sha}"

echo "quadrf-mesh: pins.env -> ${tag} (${version})"
echo "  quadrf-mesh          ${mesh_sha}"
echo "  quadrf-lora-phy      ${phy_sha}"
echo "  quadrf-meshtasticd   ${mtd_sha}"

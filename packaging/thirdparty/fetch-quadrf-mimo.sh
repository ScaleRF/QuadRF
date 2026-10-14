#!/bin/bash
# Mirror the pinned quadrf-mimo package into out/ so the QuadRF repository
# can serve it. The full metapackage Depends on quadrf-mimo.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../pins.env"

OUT="${OUT:-${here}/../out}"
mkdir -p "${OUT}"

fetch_deb() {
    local name="$1"
    local sha="$2"
    local target="${OUT}/${name}"
    local repo="${QUADRF_MIMO_REPO#https://github.com/}"
    local url="${QUADRF_MIMO_REPO}/releases/download/${QUADRF_MIMO_TAG}/${name}"

    if [ -f "${target}" ] && echo "${sha}  ${target}" | sha256sum -c - >/dev/null 2>&1; then
        echo "quadrf-mimo: ${name} already present"
        return 0
    fi

    echo "quadrf-mimo: fetching ${name} from ${QUADRF_MIMO_TAG}"
    auth_hdr=()
    if [ -n "${GH_TOKEN:-}" ]; then
        auth_hdr=(-H "Authorization: Bearer ${GH_TOKEN}")
    elif [ -n "${GITHUB_TOKEN:-}" ]; then
        auth_hdr=(-H "Authorization: Bearer ${GITHUB_TOKEN}")
    elif command -v gh >/dev/null 2>&1 && gh auth token >/dev/null 2>&1; then
        auth_hdr=(-H "Authorization: Bearer $(gh auth token)")
    fi

    if ! curl -fL "${auth_hdr[@]}" --retry 3 -o "${target}.part" "${url}" 2>/dev/null; then
        gh release download "${QUADRF_MIMO_TAG}" --repo "${repo}" -p "${name}" -O "${target}.part"
    fi

    echo "${sha}  ${target}.part" | sha256sum -c -
    mv "${target}.part" "${target}"
    echo "quadrf-mimo: ${target}"
}

fetch_deb "quadrf-mimo_${QUADRF_MIMO_VERSION}_arm64.deb" "${QUADRF_MIMO_SHA256}"

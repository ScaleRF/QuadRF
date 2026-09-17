#!/bin/bash
# Point packaging/pins.env at a quadrf-phasegaze GitHub release (tag or latest).
# The quadrf metapackage Depends on QUADRF_PHASEGAZE_VERSION at build time.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
pins="${here}/../pins.env"
. "${pins}"

repo="${QUADRF_PHASEGAZE_REPO#https://github.com/}"
tag="${1:-}"

auth_hdr=()
if [ -n "${GH_TOKEN:-}" ]; then
    auth_hdr=(-H "Authorization: Bearer ${GH_TOKEN}")
elif [ -n "${GITHUB_TOKEN:-}" ]; then
    auth_hdr=(-H "Authorization: Bearer ${GITHUB_TOKEN}")
elif command -v gh >/dev/null 2>&1 && gh auth token >/dev/null 2>&1; then
    auth_hdr=(-H "Authorization: Bearer $(gh auth token)")
fi

if [ -z "${tag}" ]; then
    tag="$(curl -fsSL "${auth_hdr[@]}" -H 'Accept: application/vnd.github+json' \
        "https://api.github.com/repos/${repo}/releases/latest" 2>/dev/null \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])' 2>/dev/null || true)"
    if [ -z "${tag}" ] && command -v gh >/dev/null 2>&1; then
        tag="$(gh release view --repo "${repo}" --json tagName --jq .tagName 2>/dev/null || true)"
    fi
fi

if [ -z "${tag}" ]; then
    echo "quadrf-phasegaze: could not determine latest release tag" >&2
    exit 1
fi

case "${tag}" in
    v*) ;;
    *) tag="v${tag}" ;;
esac

sums="$(mktemp)"
trap 'rm -f "${sums}"' EXIT
echo "quadrf-phasegaze: pinning ${tag} from ${QUADRF_PHASEGAZE_REPO}"
if ! curl -fsSL "${auth_hdr[@]}" -o "${sums}" "${QUADRF_PHASEGAZE_REPO}/releases/download/${tag}/SHA256SUMS" 2>/dev/null; then
    gh release download "${tag}" --repo "${repo}" -p "SHA256SUMS" -O "${sums}" --clobber
fi

sha_for() {
    local name="$1" line sha
    line="$(awk -v n="${name}" '$2 ~ "/" n "$" || $2 == n {print; exit}' "${sums}")"
    if [ -z "${line}" ]; then
        echo "quadrf-phasegaze: ${name} not in ${tag} SHA256SUMS" >&2
        exit 1
    fi
    sha="${line%% *}"
    printf '%s\n' "${sha}"
}

deb_name="$(awk '/quadrf-phasegaze_.*_arm64\.deb$/ {n=$2; sub(".*/","",n); print n; exit}' "${sums}")"
if [ -z "${deb_name}" ]; then
    echo "quadrf-phasegaze: no quadrf-phasegaze_*_arm64.deb in ${tag} SHA256SUMS" >&2
    exit 1
fi
version="${deb_name#quadrf-phasegaze_}"
version="${version%_arm64.deb}"

deb_sha="$(sha_for "quadrf-phasegaze_${version}_arm64.deb")"

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

set_pin QUADRF_PHASEGAZE_VERSION "${version}"
set_pin QUADRF_PHASEGAZE_TAG "${tag}"
set_pin QUADRF_PHASEGAZE_SHA256 "${deb_sha}"

echo "quadrf-phasegaze: pins.env -> ${tag} (${version})"
echo "  quadrf-phasegaze     ${deb_sha}"

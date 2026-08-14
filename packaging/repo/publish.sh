#!/bin/bash
# Add everything in out/ to the apt repository under repo/.
#
# reprepro signs the release with the key selected by SignWith in
# conf/distributions. GitHub Actions imports QUADRF_GPG_PRIVATE_KEY;
# a local maintainer build falls back to gen-key.sh in packaging/repo/.gnupg.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
top="${here}/../.."
OUT="${OUT:-${here}/../out}"
SUITE="${SUITE:-bookworm}"
export GNUPGHOME="${GNUPGHOME:-${here}/.gnupg}"

command -v reprepro >/dev/null || { echo "reprepro is not installed" >&2; exit 1; }

if [ -n "${QUADRF_GPG_PRIVATE_KEY:-}" ]; then
    "${here}/import-key.sh"
else
    "${here}/gen-key.sh"
fi
"${here}/export-key.sh"

shopt -s nullglob
debs=()
for deb in "${OUT}"/*.deb; do
    case "${deb}" in
        *-dbgsym_*.deb) continue ;;
        *) debs+=("${deb}") ;;
    esac
done
if [ ${#debs[@]} -eq 0 ]; then
    echo "no packages in ${OUT}; run make first" >&2
    exit 1
fi

# quadrf and its siblings (quadrf-common, quadrf-soapy, ...) build from one
# debian/changelog and the metapackages pin exact versions of each other
# (quadrf-soapy (= ...), etc). Publishing only some of them at a newer
# version strands the metapackage on a version this run is about to drop
# from the pool, which is exactly what apt cannot resolve. Refuse a partial
# set instead of publishing one.
quadrf_version="$(dpkg-parsechangelog -l"${top}/debian/changelog" -SVersion)"
mapfile -t quadrf_pkgs < <(awk '/^Package: /{print $2}' "${top}/debian/control")
present=()
missing=()
for pkg in "${quadrf_pkgs[@]}"; do
    if compgen -G "${OUT}/${pkg}_${quadrf_version}_*.deb" >/dev/null; then
        present+=("${pkg}")
    else
        missing+=("${pkg}")
    fi
done
if [ ${#present[@]} -gt 0 ] && [ ${#missing[@]} -gt 0 ]; then
    echo "refusing to publish a partial quadrf ${quadrf_version} build:" >&2
    printf '  present: %s\n' "${present[*]}" >&2
    printf '  missing (not at %s): %s\n' "${quadrf_version}" "${missing[*]}" >&2
    echo "run 'make -C packaging quadrf' for a full rebuild before publishing" >&2
    exit 1
fi

for deb in "${debs[@]}"; do
    name="$(dpkg-deb -f "${deb}" Package)"
    # Local rebuilds keep the same version; drop the old copy first.
    reprepro -b "${here}" remove "${SUITE}" "${name}" >/dev/null 2>&1 || true
    reprepro -b "${here}" includedeb "${SUITE}" "${deb}"
done

reprepro -b "${here}" list "${SUITE}"

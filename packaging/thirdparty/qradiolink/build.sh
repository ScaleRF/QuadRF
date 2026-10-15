#!/bin/bash
# Build qradiolink from the pinned upstream tag. Not in Debian; the QuadRF
# desktop launcher needs the binary, so the package is rebuilt into our repo
# the same way as quadrf-openocd. Upstream ships its own debian/.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../../pins.env"

OUT="${OUT:-${here}/../../out}"
work="${here}/work"
src="${work}/qradiolink"

mkdir -p "${work}" "${OUT}"

# Codeberg drops the TCP handshake from GitHub-hosted runners. One miss fails
# the whole release, so retry the clone and the later fetch.
git_try() {
    local attempt
    for attempt in 1 2 3 4 5; do
        if git -c http.connectTimeout=60 "$@"; then
            return 0
        fi
        echo "qradiolink: git failed (attempt ${attempt}); retrying" >&2
        sleep $((attempt * 5))
    done
    return 1
}

if [ ! -d "${src}/.git" ]; then
    cloned=0
    for attempt in 1 2 3 4 5; do
        rm -rf "${src}"
        if git -c http.connectTimeout=60 clone "${QRADIOLINK_REPO}" "${src}"; then
            cloned=1
            break
        fi
        echo "qradiolink: clone failed (attempt ${attempt}); retrying" >&2
        sleep $((attempt * 5))
    done
    if [ "${cloned}" -ne 1 ]; then
        rm -rf "${src}"
        echo "qradiolink: clone failed" >&2
        exit 1
    fi
fi
git_try -C "${src}" fetch origin --tags
git -C "${src}" checkout -f "${QRADIOLINK_COMMIT}"

# Trixie renamed these; keep a bookworm fallback so either suite builds.
sed -i \
    -e 's/libvolk2-dev/libvolk-dev | libvolk2-dev/' \
    -e 's/qtgstreamer-plugins-qt5/gstreamer1.0-qt5 | qtgstreamer-plugins-qt5/' \
    "${src}/debian/control"

if command -v mk-build-deps >/dev/null; then
    apt-get update
    ( cd "${src}" && mk-build-deps -i -r -t "apt-get -y --no-install-recommends" )
fi

( cd "${src}" && dpkg-buildpackage -b -uc -us )
mv "${work}"/qradiolink_*.deb "${OUT}/"
echo "qradiolink: $(ls "${OUT}"/qradiolink_*.deb)"

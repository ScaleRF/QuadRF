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

if [ ! -d "${src}/.git" ]; then
    git clone "${QRADIOLINK_REPO}" "${src}"
fi
git -C "${src}" fetch origin --tags
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

#!/bin/bash
# Build quadrf-openocd from the Raspberry Pi OpenOCD fork at the pinned commit.
#
# The fork carries the RP1 GPIO support the Pi 5 needs to bit-bang JTAG. It
# installs under /usr/lib/quadrf/openocd so it can sit next to the distribution
# openocd package.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../../pins.env"

OUT="${OUT:-${here}/../../out}"
work="${here}/work"
src="${work}/openocd"

mkdir -p "${work}" "${OUT}"

if [ ! -d "${src}/.git" ]; then
    git clone --recurse-submodules "${OPENOCD_REPO}" "${src}"
fi
git -C "${src}" fetch origin
git -C "${src}" checkout -f "${OPENOCD_COMMIT}"
git -C "${src}" submodule update --init --recursive

rm -rf "${src}/debian"
cp -a "${here}/debian" "${src}/debian"
sed -e "s|@VERSION@|${OPENOCD_VERSION}|" \
    -e "s|@DATE@|$(date -R)|" \
    "${here}/changelog.in" > "${src}/debian/changelog"

( cd "${src}" && dpkg-buildpackage -b -uc -us )
mv "${work}"/quadrf-openocd_*.deb "${OUT}/"
echo "openocd: $(ls "${OUT}"/quadrf-openocd_*.deb)"

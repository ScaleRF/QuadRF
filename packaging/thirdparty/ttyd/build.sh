#!/bin/bash
# Package the pinned upstream ttyd static binary into a .deb for the QuadRF
# apt repository: quadrf-gui depends on ttyd, which Debian does not ship.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
. "${here}/../../pins.env"

OUT="${OUT:-${here}/../../out}"
work="${here}/work"
deb_version="${TTYD_VERSION}-1"
deb_name="ttyd_${deb_version}_arm64.deb"
target="${OUT}/${deb_name}"
bin_cache="${work}/ttyd.aarch64"
url="https://github.com/tsl0922/ttyd/releases/download/${TTYD_VERSION}/ttyd.aarch64"

mkdir -p "${work}" "${OUT}"

if [ ! -f "${bin_cache}" ] || ! echo "${TTYD_SHA256}  ${bin_cache}" | sha256sum -c - >/dev/null 2>&1; then
    echo "ttyd: fetching ${url}"
    curl -fL --retry 3 -o "${bin_cache}.part" "${url}"
    echo "${TTYD_SHA256}  ${bin_cache}.part" | sha256sum -c -
    mv "${bin_cache}.part" "${bin_cache}"
fi

stage="${work}/pkg"
rm -rf "${stage}"
mkdir -p "${stage}/DEBIAN" "${stage}/usr/bin" "${stage}/usr/share/doc/ttyd"

install -m 0755 "${bin_cache}" "${stage}/usr/bin/ttyd"

cat > "${stage}/usr/share/doc/ttyd/copyright" <<'EOF'
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/
Upstream-Name: ttyd
Source: https://github.com/tsl0922/ttyd

Files: *
Copyright: 2016-2024 Shuanglei Tao <tsl0922@gmail.com>
License: MIT
EOF

installed_kb="$(du -sk "${stage}/usr" | awk '{print $1}')"

cat > "${stage}/DEBIAN/control" <<EOF
Package: ttyd
Version: ${deb_version}
Section: web
Priority: optional
Architecture: arm64
Installed-Size: ${installed_kb}
Maintainer: QuadRF Engineering <contact@ScaleRF.com>
Homepage: https://github.com/tsl0922/ttyd
Description: command-line tool for sharing terminal over the web
 ttyd is a simple command-line tool for sharing terminal over the web,
 built on libwebsockets and xterm.js. Packaged from the upstream static
 arm64 release binary for the QuadRF control panel web terminal.
EOF

dpkg-deb --root-owner-group --build "${stage}" "${target}" >/dev/null
echo "ttyd: ${target}"

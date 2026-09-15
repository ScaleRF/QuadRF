#!/usr/bin/env bash
# deploy_and_test.sh - Build on quad (/boq) and deploy QuadRF Data Link
set -euo pipefail

NODE1="${NODE1:-quadrf.local}"
NODE2="${NODE2:-quadrf-2.local}"
PASS="${QUADRF_PASS:-dietpi2}"

run_ssh() {
  local host="$1"
  shift
  sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 "dietpi@$host" "$@"
}

run_scp() {
  local src="$1"
  local dest="$2"
  sshpass -p "$PASS" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 "$src" "$dest"
}

ROOT_DIR="$(cd "$(dirname "$0")/../../../../" && pwd)"
cd "$ROOT_DIR"

export SSHPASS="$PASS"

echo "=== [1/5] Syncing source tree to build host ($NODE1) ==="
rsync -a -e "sshpass -e ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" \
  --delete --exclude build-arm/ \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'cmake-build-*/' \
  --exclude 'obj-*/' \
  --exclude 'debian/quadrf/' \
  --exclude 'debian/quadrf-*/' \
  --exclude 'packaging/out/' \
  --exclude 'images/' \
  --exclude 'hardware/' \
  --exclude '.cursor/' \
  ./ "dietpi@$NODE1:/home/dietpi/quadrf-build/"

echo "=== [2/5] Compiling quadrf-data on $NODE1 ==="
run_ssh "$NODE1" "cd /home/dietpi/quadrf-build && cmake -S . -B build-arm -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build-arm -j4 --target quadrf-data"

echo "=== [3/5] Installing quadrf-data, desktop icon, and system integration on $NODE1 ==="
run_ssh "$NODE1" "
  echo '$PASS' | sudo -S install -m755 /home/dietpi/quadrf-build/build-arm/sources/demos/wifi/quadrf-data /usr/bin/quadrf-data && \
  echo '$PASS' | sudo -S /usr/sbin/setcap cap_net_admin+eip /usr/bin/quadrf-data && \
  echo '$PASS' | sudo -S cp /home/dietpi/quadrf-build/build-arm/sources/soapy/libmipi.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/libmipi.so && \
  echo '$PASS' | sudo -S install -m644 /home/dietpi/quadrf-build/sources/desktop/applications/com.scalerf.QuadRF.DataLink.desktop /usr/share/applications/com.scalerf.QuadRF.DataLink.desktop && \
  echo '$PASS' | sudo -S install -m644 /home/dietpi/quadrf-build/sources/icons/quadrf-data-link.svg /usr/share/icons/hicolor/scalable/apps/quadrf-data-link.svg && \
  echo '$PASS' | sudo -S mkdir -p /usr/share/quadrf/apps.d && \
  echo '$PASS' | sudo -S install -m644 /home/dietpi/quadrf-build/sources/desktop/apps.d/quadrf-data.json /usr/share/quadrf/apps.d/quadrf-data.json && \
  echo '$PASS' | sudo -S install -m644 /home/dietpi/quadrf-build/sources/systemd/quadrf-data.service /lib/systemd/system/quadrf-data.service && \
  echo '$PASS' | sudo -S systemctl daemon-reload && \
  echo '$PASS' | sudo -S /usr/lib/quadrf/sync-desktop-apps 2>/dev/null || true
"

echo "=== [4/5] Deploying quadrf-data and system integration to $NODE2 ==="
# Deploy to Node 2 directly from build host
run_ssh "$NODE1" "
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/build-arm/sources/demos/wifi/quadrf-data dietpi@$NODE2:/tmp/quadrf-data && \
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/build-arm/sources/soapy/libmipi.so dietpi@$NODE2:/tmp/libmipi.so && \
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/sources/desktop/applications/com.scalerf.QuadRF.DataLink.desktop dietpi@$NODE2:/tmp/com.scalerf.QuadRF.DataLink.desktop && \
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/sources/icons/quadrf-data-link.svg dietpi@$NODE2:/tmp/quadrf-data-link.svg && \
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/sources/desktop/apps.d/quadrf-data.json dietpi@$NODE2:/tmp/quadrf-data.json && \
  scp -o StrictHostKeyChecking=no /home/dietpi/quadrf-build/sources/systemd/quadrf-data.service dietpi@$NODE2:/tmp/quadrf-data.service && \
  ssh -o StrictHostKeyChecking=no dietpi@$NODE2 '
    echo \"$PASS\" | sudo -S install -m755 /tmp/quadrf-data /usr/bin/quadrf-data && \
    echo \"$PASS\" | sudo -S /usr/sbin/setcap cap_net_admin+eip /usr/bin/quadrf-data && \
    echo \"$PASS\" | sudo -S cp /tmp/libmipi.so /usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8/libmipi.so && \
    echo \"$PASS\" | sudo -S install -m644 /tmp/com.scalerf.QuadRF.DataLink.desktop /usr/share/applications/com.scalerf.QuadRF.DataLink.desktop && \
    echo \"$PASS\" | sudo -S install -m644 /tmp/quadrf-data-link.svg /usr/share/icons/hicolor/scalable/apps/quadrf-data-link.svg && \
    echo \"$PASS\" | sudo -S mkdir -p /usr/share/quadrf/apps.d && \
    echo \"$PASS\" | sudo -S install -m644 /tmp/quadrf-data.json /usr/share/quadrf/apps.d/quadrf-data.json && \
    echo \"$PASS\" | sudo -S install -m644 /tmp/quadrf-data.service /lib/systemd/system/quadrf-data.service && \
    echo \"$PASS\" | sudo -S systemctl daemon-reload && \
    echo \"$PASS\" | sudo -S /usr/lib/quadrf/sync-desktop-apps 2>/dev/null || true && \
    rm -f /tmp/quadrf-data /tmp/libmipi.so /tmp/com.scalerf.QuadRF.DataLink.desktop /tmp/quadrf-data-link.svg /tmp/quadrf-data.json /tmp/quadrf-data.service
  '
"

echo "=== [5/5] Verifying installed binaries and CLI help ==="
run_ssh "$NODE1" "/usr/bin/quadrf-data --help" | head -n 10
run_ssh "$NODE2" "/usr/bin/quadrf-data --help" | head -n 10

echo "=== Build and deployment complete! ==="

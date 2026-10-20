#!/usr/bin/env bash
# QuadRF first-boot factory entrypoint (run as root through systemd).
# Uses a temporary wpa_supplicant + dhclient session; never changes DietPi's
# persistent Wi-Fi/network configuration. Deletes itself for 0 or 1 failed TX
# after verified production completion; retains itself for other failures.
set -Eeuo pipefail
umask 077
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH:-}"

# Delete the script actually invoked (including a local test copy on PASS).
readonly SELF="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SHARE=/dev/shm/host_share
readonly LOCAL=/run/quadrf-production
readonly SERVER='10.42.0.1:/srv/hotspot_share'
readonly JTAG="$(command -v quadrf-jtag || true)"

# Factory-only credentials: edit on the master image. startup.sh must remain
# root-readable only (mode 0700); the credentials disappear with the script.
readonly WIFI_SSID='QuadRF_Prod'
readonly WIFI_PASSWORD='QuadRF_Password'
readonly WPA_CONF="$LOCAL/wpa_temp.conf"
readonly WPA_PID="$LOCAL/wpa_supplicant.pid"
readonly DHCP_PID="$LOCAL/dhclient.pid"
readonly DHCP_LEASE="$LOCAL/dhclient.leases"
SUCCESS=0

note() { echo "startup: $*"; logger -t quadrf-startup -- "$*" 2>/dev/null || :; }

# LED register = (blink_setting << 5) | brightness, with 5 bits for each.
led() {
    [[ -n "$JTAG" ]] || return 1
    timeout 8 "$JTAG" write 0x04 "$1" >/dev/null 2>&1 &&
    timeout 8 "$JTAG" write 0x05 "$2" >/dev/null 2>&1 &&
    timeout 8 "$JTAG" write 0x06 "$3" >/dev/null 2>&1
}

# Stop only the factory-owned processes at the END of a successful run.
# The initial killall (below) is intentional for the dedicated factory boot,
# matching the original DietPi workflow. It affects other WPA connections.
stop_factory_wifi() {
    local pid
    if [[ -f "$DHCP_PID" ]]; then
        # Release the factory DHCP address, not the machine's saved network config.
        timeout 10 dhclient -r -pf "$DHCP_PID" -lf "$DHCP_LEASE" wlan0 >/dev/null 2>&1 || :
    fi
    if [[ -f "$WPA_PID" ]]; then
        pid=$(<"$WPA_PID")
        if [[ "$pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$pid/cmdline" ]] &&
           tr '\0' ' ' <"/proc/$pid/cmdline" | grep -q 'wpa_supplicant'; then
            kill "$pid" 2>/dev/null || :
        fi
    fi
    rm -f -- "$WPA_CONF" "$WPA_PID" "$DHCP_PID" "$DHCP_LEASE"
}

# Each failed direct TX path adds 1 Hz to the yellow blink frequency.
# FPGA register: bits [9:5] = Hz, bits [4:0] = brightness (31 = full).
# Both red and green channels receive the same word to display yellow.
yellow_for_failed_tx() {
    local count=$1 word
    [[ "$count" =~ ^[1-4]$ ]] || return 1
    printf -v word '0x%04x' "$(( (count << 5) | 31 ))"
    led "$word" "$word" 0
}

on_exit() {
    local rc=$? failed
    trap - EXIT
    if (( SUCCESS == 0 )); then
        case "$rc" in
            3|4|5)
                failed=$((rc-1))
                note "RF SOFT FAIL: $failed failed direct TX path(s); startup.sh retained; yellow ${failed}-Hz blink"
                yellow_for_failed_tx "$failed" || note 'WARNING: cannot display yellow LED'
                ;;
            *)
                note "HARD FAIL: production incomplete (exit $rc); startup.sh retained"
                led 0x003f 0 0 || note 'WARNING: cannot display red LED (FPGA/JTAG may be unavailable)'
                (( rc != 0 )) || rc=1
                ;;
        esac
    fi
    # Leave factory Wi-Fi up on reject for technicians, but discard plaintext
    # WPA config on both success and failure.
    rm -f -- "$WPA_CONF"
    exit "$rc"
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

(( EUID == 0 )) || { note 'Run as root'; exit 1; }
for cmd in ip killall wpa_passphrase wpa_supplicant dhclient mount.nfs; do
    command -v "$cmd" >/dev/null || { note "Missing required tool: $cmd"; exit 1; }
done
install -d -m 0700 "$LOCAL" "$SHARE"

# Keep the connection sequence close to the original manual factory script.
# Do NOT use NetworkManager, modify /etc/wpa_supplicant, or restart DietPi's
# persistent network services. Factory boot owns wlan0 temporarily.
note "Connecting wlan0 to factory SSID $WIFI_SSID"
killall wpa_supplicant 2>/dev/null || :
ip link set wlan0 down
ip link set wlan0 up

# /run is tmpfs; 0600 prevents other users reading the WPA material. Remove
# wpa_passphrase's commented cleartext '#psk=...' line too.
wpa_passphrase "$WIFI_SSID" "$WIFI_PASSWORD" | sed '/^[[:space:]]*#psk=/d' > "$WPA_CONF"
chmod 0600 "$WPA_CONF"
wpa_supplicant -B -i wlan0 -c "$WPA_CONF" -P "$WPA_PID"

# dhclient -1 makes a single DHCP attempt. Keep retrying for a bounded period
# so a broken AP cannot strand a unit indefinitely; restart on next boot.
connected=0
for attempt in $(seq 1 12); do
    if timeout --kill-after=3 35 dhclient -1 -pf "$DHCP_PID" -lf "$DHCP_LEASE" wlan0 >/dev/null 2>&1; then
        if ip -4 addr show dev wlan0 | grep -q 'inet '; then
            connected=1
            break
        fi
    fi
    note "Waiting for AP / DHCP (${attempt}/12)"
    sleep 5
done
(( connected == 1 )) || { note 'Failed to obtain factory Wi-Fi address'; exit 1; }
note "Connected! wlan0=$(ip -4 -o addr show wlan0 | awk '{print $4; exit}')"
# The live wpa_supplicant keeps its PSK in memory; the on-disk config is
# no longer needed. The secret remains in SELF until it is deleted on PASS.
rm -f -- "$WPA_CONF"

mounted=0
for attempt in 1 2 3 4 5; do
    if mountpoint -q "$SHARE"; then
        [[ $(findmnt -n -o SOURCE --target "$SHARE") == "$SERVER" ]] || {
            note 'Unexpected NFS mount already present'; exit 1;
        }
        mounted=1
        break
    fi
    if timeout 25 mount -t nfs -o rw,nosuid,nodev,noexec,soft,timeo=20,retrans=2 \
        "$SERVER" "$SHARE"; then
        mounted=1
        break
    fi
    note "Waiting for factory NFS share (${attempt}/5)"
    sleep 4
done
(( mounted == 1 )) || { note 'Cannot mount factory NFS share'; exit 1; }

# Use the latest trusted server script every run; no per-image SHA file to update.
# Stage it in RAM because the NFS export is intentionally mounted noexec.
[[ -r "$SHARE/production.sh" ]] || { note 'production.sh not found on NFS'; exit 1; }
[[ -r "$SHARE/production.env" ]] || { note 'production.env not found on NFS'; exit 1; }
timeout 30 cp -- "$SHARE/production.sh" "$LOCAL/production.sh"
chmod 0700 "$LOCAL/production.sh"
led 0 0 0x003f || note 'WARNING: blue progress LED unavailable'
note 'Running current production.sh from factory NFS share (staged in /run)'
if bash "$LOCAL/production.sh"; then
    prod_rc=0
else
    prod_rc=$?
fi
case "$prod_rc" in
    0) note 'production.sh PASS (all 4 direct TX paths)' ;;
    2) note 'production.sh ACCEPTED EXCEPTION: exactly 1 failed TX; shipping permitted with 1-Hz yellow' ;;
    3|4|5)
        note "production.sh RF SOFT FAIL: $((prod_rc-1)) failed TX; startup.sh retained"
        exit "$prod_rc"
        ;;
    *)
        note "production.sh HARD FAIL (exit $prod_rc); see per-MAC production log"
        exit "$prod_rc"
        ;;
esac

# Both ordinary PASS (0 failed) and the explicitly accepted 1-TX exception
# (exit 2) have a verified NFS log and may proceed to final cleanup.
# A power failure before self-deletion causes a full retest on next boot.
[[ -f "$SELF" ]] || { note 'Cannot locate startup.sh for self-removal'; exit 1; }
timeout 20 umount "$SHARE" || { note 'Cannot unmount factory NFS share'; exit 1; }
rm -f -- "$LOCAL/production.sh"
if (( prod_rc == 0 )); then
    led 0 0x001f 0 || { note 'Cannot display green PASS LED'; exit 1; }
else
    yellow_for_failed_tx 1 || { note 'Cannot display yellow accepted-exception LED'; exit 1; }
fi
stop_factory_wifi
rm -- "$SELF"
SUCCESS=1
if (( prod_rc == 0 )); then
    note 'PASS: startup.sh deleted; leave power on to inspect steady green LED'
else
    note 'ACCEPTED EXCEPTION: startup.sh deleted; leave power on to inspect 1-Hz yellow LED'
fi
exit 0

# Shared helpers for the quadrf-* packages.
#
#   . /usr/share/quadrf/functions.sh
#
# Sourced by maintainer scripts (which run under "set -e") and by the runtime
# helpers in /usr/lib/quadrf, so every function here has to be safe to call
# more than once and must not depend on bash.

QUADRF_CONF="${QUADRF_CONF:-/etc/quadrf/quadrf.conf}"

quadrf_load_conf() {
    QUADRF_USER=dietpi
    QUADRF_BOOT_DIR=/boot/firmware
    QUADRF_HOSTNAME=quadrf
    QUADRF_AP_SSID=QuadRF
    QUADRF_AP_PASS=
    QUADRF_AP_ADDRESS=192.168.44.1
    QUADRF_OPENOCD=

    if [ -r "${QUADRF_CONF}" ]; then
        . "${QUADRF_CONF}"
    fi
}

# Single-label mDNS names for the KasmVNC vhost.
# desktop.quadrf.local is two labels; browsers send that to unicast DNS and
# get NXDOMAIN. DNS is case-insensitive
quadrf_desktop_mdns() {
    quadrf_load_conf
    printf '%s-desktop.local\n' "${QUADRF_HOSTNAME}"
}

quadrf_desktop_mdns_short() {
    quadrf_load_conf
    printf '%sd.local\n' "${QUADRF_HOSTNAME}"
}

quadrf_home() {
    quadrf_load_conf
    getent passwd "${QUADRF_USER}" | cut -d: -f6
}

quadrf_openocd() {
    quadrf_load_conf
    if [ -n "${QUADRF_OPENOCD}" ]; then
        printf '%s\n' "${QUADRF_OPENOCD}"
        return 0
    fi
    for candidate in /usr/lib/quadrf/openocd/bin/openocd /usr/bin/openocd; do
        if [ -x "${candidate}" ]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done
    return 1
}

# Replace, or append, a block of lines delimited by QuadRF markers. Content is
# read from stdin. Files we do not own (config.txt, avahi-daemon.conf) are
# edited this way so the block can be lifted back out on purge.
quadrf_block_apply() {
    file="$1"
    tag="$2"
    begin="# >>> quadrf ${tag} >>>"
    end="# <<< quadrf ${tag} <<<"
    tmp="$(mktemp)"

    if [ -f "${file}" ]; then
        sed "/^${begin}\$/,/^${end}\$/d" "${file}" > "${tmp}"
    fi

    {
        printf '%s\n' "${begin}"
        cat
        printf '%s\n' "${end}"
    } >> "${tmp}"

    if [ -f "${file}" ]; then
        cat "${tmp}" > "${file}"
    else
        install -D -m 644 "${tmp}" "${file}"
    fi
    rm -f "${tmp}"
}

quadrf_block_remove() {
    file="$1"
    tag="$2"
    begin="# >>> quadrf ${tag} >>>"
    end="# <<< quadrf ${tag} <<<"

    [ -f "${file}" ] || return 0
    tmp="$(mktemp)"
    sed "/^${begin}\$/,/^${end}\$/d" "${file}" > "${tmp}"
    cat "${tmp}" > "${file}"
    rm -f "${tmp}"
}

# Set key=value in a flat configuration file, replacing an existing (possibly
# commented out) assignment.
quadrf_setting_set() {
    file="$1"
    key="$2"
    value="$3"

    [ -f "${file}" ] || return 1
    if grep -qE "^[[:space:]]*#?[[:space:]]*${key}=" "${file}"; then
        sed -i -E "s|^[[:space:]]*#?[[:space:]]*${key}=.*|${key}=${value}|" "${file}"
    else
        printf '%s=%s\n' "${key}" "${value}" >> "${file}"
    fi
}

quadrf_sh_quote() {
    printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\\\''/g")"
}

# Write a quoted assignment into /etc/quadrf/quadrf.conf.
quadrf_conf_set() {
    key="$1"
    value="$2"
    file="${QUADRF_CONF:-/etc/quadrf/quadrf.conf}"
    quoted="$(quadrf_sh_quote "${value}")"
    [ -f "${file}" ] || return 1

    if grep -qE "^[[:space:]]*#?[[:space:]]*${key}=" "${file}"; then
        tmp="$(mktemp)"
        awk -v key="${key}" -v val="${quoted}" '
            $0 ~ "^[[:space:]]*#?[[:space:]]*" key "=" { print key "=" val; next }
            { print }
        ' "${file}" > "${tmp}"
        cat "${tmp}" > "${file}"
        rm -f "${tmp}"
    else
        printf '%s=%s\n' "${key}" "${quoted}" >> "${file}"
    fi
}

# hostapd.conf for the fallback AP. Empty QUADRF_AP_PASS means open.
quadrf_write_hostapd() {
    dest="${1:-/etc/hostapd/quadrf.conf}"
    tmp="$(mktemp)"

    cat > "${tmp}" <<EOF
interface=wlan0
driver=nl80211
ssid=${QUADRF_AP_SSID}
hw_mode=g
channel=6
country_code=US
ieee80211n=1
wmm_enabled=1
auth_algs=1
EOF
    if [ -n "${QUADRF_AP_PASS}" ]; then
        cat >> "${tmp}" <<EOF
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=${QUADRF_AP_PASS}
EOF
    else
        echo "wpa=0" >> "${tmp}"
    fi
    install -m 644 "${tmp}" "${dest}"
    rm -f "${tmp}"
}

# Units that act on behalf of the operator account get their User=/Group= from
# a drop-in instead of the shipped unit, so QUADRF_USER stays configurable.
quadrf_user_dropin() {
    unit="$1"
    quadrf_load_conf

    if ! getent passwd "${QUADRF_USER}" >/dev/null; then
        echo "quadrf: user ${QUADRF_USER} does not exist; ${unit} left as shipped" >&2
        return 0
    fi

    dir="/etc/systemd/system/${unit}.d"
    home="$(getent passwd "${QUADRF_USER}" | cut -d: -f6)"
    group="$(id -gn "${QUADRF_USER}")"

    mkdir -p "${dir}"
    cat > "${dir}/10-quadrf-user.conf" <<EOF
[Service]
User=${QUADRF_USER}
Group=${group}
Environment=HOME=${home}
EOF
}

quadrf_user_dropin_remove() {
    rm -f "/etc/systemd/system/$1.d/10-quadrf-user.conf"
    rmdir "/etc/systemd/system/$1.d" 2>/dev/null || true
}

quadrf_reboot_required() {
    reason="$1"
    mkdir -p /var/lib/quadrf
    printf '%s\n' "${reason}" >> /var/lib/quadrf/reboot-required
    sort -u -o /var/lib/quadrf/reboot-required /var/lib/quadrf/reboot-required
    touch /run/reboot-required 2>/dev/null || true
}

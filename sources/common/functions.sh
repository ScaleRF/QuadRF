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
    QUADRF_TLS_DOMAIN=my.quadrf.com
    QUADRF_AP_SSID=QuadRF
    QUADRF_AP_ADDRESS=192.168.44.1
    QUADRF_OPENOCD=

    if [ -r "${QUADRF_CONF}" ]; then
        . "${QUADRF_CONF}"
    fi
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

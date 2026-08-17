#!/bin/bash
# KasmVNC session: openbox + pcmanfm + tint2 (panel via openbox autostart).

unset SESSION_MANAGER

export XDG_CURRENT_DESKTOP=LXDE
export XDG_MENU_PREFIX=lxde-
export GTK_THEME=Adwaita
export DISPLAY="${DISPLAY:-:1}"
export XDG_SESSION_TYPE=x11
# HDMI brings up vc4 KMS on card*. Without these, Qt eglfs / SDL kmsdrm /
# GTK may take the physical monitor and the VNC session sees no window.
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
export SDL_VIDEODRIVER=x11
unset WAYLAND_DISPLAY

# vncserver is a systemd User= service, not a PAM login, so logind never
# creates /run/user/<uid>. Qt then warns and dumps runtime state in /tmp.
if [ -z "${XDG_RUNTIME_DIR:-}" ]; then
  if [ -d "/run/user/$(id -u)" ]; then
    export XDG_RUNTIME_DIR="/run/user/$(id -u)"
  else
    export XDG_RUNTIME_DIR="/tmp/runtime-${USER}"
    mkdir -p "${XDG_RUNTIME_DIR}"
    chmod 700 "${XDG_RUNTIME_DIR}"
  fi
fi

[ -r "${HOME}/.Xresources" ] && xrdb "${HOME}/.Xresources"

# Resize: restart openbox and wrangle window geometry on RandR events.
(
  LAST_TIME=0
  stdbuf -oL xev -root -event randr | while read -r line; do
    if [[ "${line}" == *"RRScreenChangeNotify"* ]]; then
      CURRENT_TIME=$(date +%s)
      if (( CURRENT_TIME - LAST_TIME >= 1 )); then
        LAST_TIME=$CURRENT_TIME
        ( openbox --restart; /usr/lib/quadrf/kasm-wrangle ) &
      fi
    fi
  done
) &

# eval "$(dbus-launch --exit-with-session)" dies with the command-substitution
# subshell, so GRC/GTK never get a session bus (dconf "dbus-launch" dialogs,
# Qt flowgraph windows that never raise). exec keeps the bus for the session.
if [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ] && command -v dbus-launch >/dev/null; then
  exec dbus-launch --exit-with-session openbox-session
fi
exec openbox-session

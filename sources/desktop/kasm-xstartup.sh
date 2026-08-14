#!/bin/bash
# KasmVNC session: openbox + pcmanfm + tint2 (panel via openbox autostart).

unset SESSION_MANAGER

export XDG_CURRENT_DESKTOP=LXDE
export XDG_MENU_PREFIX=lxde-
export GTK_THEME=Adwaita

if [ -z "${DBUS_SESSION_BUS_ADDRESS}" ]; then
  eval "$(dbus-launch --sh-syntax --exit-with-session)"
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

exec openbox-session

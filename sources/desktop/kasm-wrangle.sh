#!/bin/bash

# Give Openbox a split second to finish its own internal restart/resizing
sleep 0.5

# Get the new screen resolution (e.g., 1098x758)
RES=$(xrandr | grep '\*' | awk '{print $1}')
MAX_W=$(echo $RES | cut -dx -f1)
MAX_H=$(echo $RES | cut -dx -f2)

# Loop through all windows managed by Openbox
wmctrl -lG | while read id desk x y w h host title; do
    # Skip hidden/system windows (desktop is -1)
    if [ "$desk" -lt 0 ]; then continue; fi

    NEW_X=$x
    NEW_Y=$y
    NEW_W=$w
    NEW_H=$h

    # 1. If window is wider than the new screen, shrink it
    if [ "$w" -gt "$MAX_W" ]; then NEW_W=$MAX_W; fi
    # 2. If window is taller than the new screen, shrink it
    if [ "$h" -gt "$MAX_H" ]; then NEW_H=$MAX_H; fi
    
    # 3. If window is shoved off the right side, pull it back
    if [ $((NEW_X + NEW_W)) -gt "$MAX_W" ]; then NEW_X=$((MAX_W - NEW_W)); fi
    # 4. If window is shoved off the bottom, pull it up
    if [ $((NEW_Y + NEW_H)) -gt "$MAX_H" ]; then NEW_Y=$((MAX_H - NEW_H)); fi
    
    # 5. Failsafe: Ensure it didn't get pushed past the top/left margins
    if [ "$NEW_X" -lt 0 ]; then NEW_X=0; fi
    if [ "$NEW_Y" -lt 0 ]; then NEW_Y=0; fi

    # Apply the new coordinates only if something changed
    if [ "$x" != "$NEW_X" ] || [ "$y" != "$NEW_Y" ] || [ "$w" != "$NEW_W" ] || [ "$h" != "$NEW_H" ]; then
        wmctrl -i -r "$id" -e 0,$NEW_X,$NEW_Y,$NEW_W,$NEW_H
    fi
done

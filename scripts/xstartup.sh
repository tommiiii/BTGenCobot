#!/bin/bash
# Unset SESSION_MANAGER to avoid conflicts
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Start D-Bus
service dbus start

# Set some basic X resources (ignore errors)
xrdb $HOME/.Xresources 2>/dev/null || true

# Disable XFCE compositor to fix Gazebo Qt transparent windows under llvmpipe
(sleep 5 && xfconf-query -c xfwm4 -p /general/use_compositing -s false) &

# Start XFCE desktop environment
exec startxfce4

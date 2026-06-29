#!/bin/bash
# Kill any existing VNC servers
vncserver -kill :1 2>/dev/null || true

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no

# Wait a moment for VNC to start
sleep 3

# Start websockify for VNC access
echo "Starting VNC web server on port 6080..."
websockify --web=/usr/share/novnc 6080 localhost:5901 2>/dev/null || websockify 6080 localhost:5901 &

# Keep the container running
wait

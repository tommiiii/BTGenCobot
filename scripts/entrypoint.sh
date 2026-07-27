#!/bin/bash
# Fix hostname resolution for VNC (needed with host networking)
if ! grep -q "$(hostname)" /etc/hosts; then
  echo "" >> /etc/hosts
  echo "127.0.0.1 $(hostname)" >> /etc/hosts
fi

source /opt/ros/jazzy/setup.bash

# Add system cmake paths for vendor packages (console_bridge, fmt, etc.)
export MULTIARCH=$(dpkg-architecture -qDEB_HOST_MULTIARCH)
export CMAKE_PREFIX_PATH="/usr/lib/${MULTIARCH}/console_bridge:/usr/lib/${MULTIARCH}/cmake:/opt/ros/jazzy/lib/${MULTIARCH}/cmake:$CMAKE_PREFIX_PATH"

# Add ROS Python packages to PYTHONPATH
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:$PYTHONPATH"

# Backward compatibility: source a root workspace if present
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
# Source any project workspaces under /workspace/* if present
for d in /workspace/*; do
  if [ -f "$d/install/setup.bash" ]; then
    source "$d/install/setup.bash"
  fi
done

# Start VNC server if not already running
if ! pgrep -x "Xvnc" > /dev/null; then
  /root/start_vnc.sh &
  sleep 5
fi

# Set OpenGL environment variables for better rendering
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export GALLIUM_DRIVER=llvmpipe
export GZ_SIM_RENDER_ENGINE=ogre

# Start a shell or execute the provided command
if [ "$#" -eq 0 ]; then
  /bin/bash
else
  exec "$@"
fi

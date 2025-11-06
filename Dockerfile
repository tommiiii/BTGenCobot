FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash", "-c"]

# Install essential dependencies
# Use BuildKit cache mount to persist apt cache between builds
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-slam-toolbox \
  ros-jazzy-foxglove-bridge \
  tigervnc-standalone-server \
  tigervnc-xorg-extension \
  novnc \
  websockify \
  xfce4 \
  xfce4-terminal

# Install Python ML dependencies (CPU-only PyTorch to avoid CUDA bloat and system package conflicts)
# Use --ignore-installed to avoid conflicts with system packages
# Use BuildKit cache mount to persist pip cache between builds (MUCH faster rebuilds)
RUN --mount=type=cache,target=/root/.cache/pip \
  pip3 install --break-system-packages --ignore-installed typing-extensions \
  torch --index-url https://download.pytorch.org/whl/cpu && \
  pip3 install --break-system-packages --ignore-installed typing-extensions \
  transformers \
  outlines \
  accelerate

# Create workspace
WORKDIR /workspace

# Copy project files
COPY src/ ./src/
COPY models/ ./models/

# Setup VNC
RUN mkdir -p /root/.vnc && \
  echo "vncpassword" | vncpasswd -f > /root/.vnc/passwd && \
  chmod 600 /root/.vnc/passwd && \
  echo '#!/bin/bash\nstartxfce4 &' > /root/.vnc/xstartup && \
  chmod +x /root/.vnc/xstartup

# Setup ROS environment
RUN source /opt/ros/jazzy/setup.bash && \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
  echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
  set -e\n\
  \n\
  # Start VNC server if DISPLAY is set to :1\n\
  if [ "$DISPLAY" = ":1" ]; then\n\
  vncserver :1 -geometry 1920x1080 -depth 24 -localhost no &\n\
  sleep 2\n\
  websockify --web=/usr/share/novnc 6080 localhost:5901 &\n\
  fi\n\
  \n\
  # Source ROS setup\n\
  source /opt/ros/jazzy/setup.bash\n\
  [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash\n\
  \n\
  exec "$@"\n\
  ' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

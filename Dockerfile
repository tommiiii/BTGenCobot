# Use AMD64 platform for better Python wheel availability (pre-built PyTorch CPU, transformers, etc.)
# Note: Runs via emulation on Apple Silicon, but gives access to full PyPI ecosystem
FROM --platform=linux/amd64 osrf/ros:jazzy-desktop
SHELL ["/bin/bash", "-c"]

# Install all ROS2 dependencies (navigation, simulation, control, etc.)
# Note: Nav2 is the navigation framework, SLAM Toolbox provides mapping/localization
# They work together but are separate packages (Nav2 can use pre-made maps OR SLAM)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-route \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-slam-toolbox \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-turtlebot4-description \
  ros-jazzy-turtlebot4-msgs \
  ros-jazzy-turtlebot4-simulator \
  ros-jazzy-irobot-create-description \
  ros-jazzy-irobot-create-nodes \
  ros-jazzy-irobot-create-gz-plugins \
  ros-jazzy-irobot-create-gz-sim \
  ros-jazzy-irobot-create-gz-bringup \
  ros-jazzy-teleop-twist-keyboard \
  git

# Install VNC packages and neovim dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y \
  tigervnc-standalone-server \
  tigervnc-xorg-extension \
  novnc \
  websockify \
  xfce4 \
  xfce4-goodies \
  dbus-x11 \
  software-properties-common \
  curl \
  wget \
  git \
  unzip \
  ripgrep \
  fd-find \
  build-essential \
  ninja-build \
  gettext \
  cmake \
  nodejs \
  npm \
  sqlite3 \
  libsqlite3-dev \
  luarocks \
  python3-venv \
  jupyter-core

# Install ImageMagick separately to handle conflicts
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && \
  apt-get remove -y graphicsmagick-libmagick-dev-compat 2>/dev/null || true && \
  apt-get install -y --no-install-recommends \
  imagemagick \
  libmagickwand-dev

# Install Python packages for nvim plugins (molten, dap-python, etc.)
# Use --ignore-installed to avoid conflicts with system packages
RUN --mount=type=cache,target=/root/.cache/pip \
  pip3 install --break-system-packages --ignore-installed \
  pynvim \
  debugpy \
  jupyter \
  jupyter-client \
  ipykernel \
  nbformat \
  pillow \
  cairosvg \
  pnglatex \
  plotly \
  pyperclip

# Install tree-sitter CLI for neovim treesitter parsers
RUN npm install -g tree-sitter-cli

# Install latest neovim from source for better plugin compatibility
RUN cd /tmp && \
  git clone --depth 1 --branch stable https://github.com/neovim/neovim && \
  cd neovim && \
  make CMAKE_BUILD_TYPE=Release && \
  make install && \
  cd / && \
  rm -rf /tmp/neovim

# Install lua packages for image.nvim
RUN luarocks install magick

# Install Python ML dependencies (CPU-only PyTorch to avoid CUDA bloat)
# Use --ignore-installed to avoid conflicts with system packages
# Use BuildKit cache mount to persist pip cache between builds (faster rebuilds)
RUN --mount=type=cache,target=/root/.cache/pip \
  echo "Installing PyTorch CPU-only (no CUDA)..." && \
  pip3 install --break-system-packages --ignore-installed typing-extensions \
  torch --index-url https://download.pytorch.org/whl/cpu && \
  echo "Installing transformers, outlines, accelerate..." && \
  pip3 install --break-system-packages \
  transformers outlines accelerate

# Create workspace
WORKDIR /workspace

# Clone OpenManipulator-X description (not available in Jazzy apt repos)
RUN mkdir -p /workspace/src && cd /workspace/src && \
  git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git && \
  git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git && \
  git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git


# Copy project files
COPY src/ ./src/
COPY models/ ./models/
COPY robot_description/ ./robot_description/
COPY worlds/ ./worlds/

# Create VNC directory and setup scripts
RUN mkdir -p /root/.vnc

# Create basic X resources file
RUN echo '! Basic X resources' > /root/.Xresources && \
  echo '! This file can be empty - it prevents xrdb errors' >> /root/.Xresources

# Create VNC startup script
RUN echo '#!/bin/bash' > /root/.vnc/xstartup && \
  echo '# Unset SESSION_MANAGER to avoid conflicts' >> /root/.vnc/xstartup && \
  echo 'unset SESSION_MANAGER' >> /root/.vnc/xstartup && \
  echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /root/.vnc/xstartup && \
  echo '' >> /root/.vnc/xstartup && \
  echo '# Start D-Bus' >> /root/.vnc/xstartup && \
  echo 'service dbus start' >> /root/.vnc/xstartup && \
  echo '' >> /root/.vnc/xstartup && \
  echo '# Set some basic X resources (ignore errors)' >> /root/.vnc/xstartup && \
  echo 'xrdb $HOME/.Xresources 2>/dev/null || true' >> /root/.vnc/xstartup && \
  echo '' >> /root/.vnc/xstartup && \
  echo '# Start XFCE desktop environment' >> /root/.vnc/xstartup && \
  echo 'exec startxfce4' >> /root/.vnc/xstartup && \
  chmod +x /root/.vnc/xstartup

# Create VNC password file (password: "vncpassword")
RUN mkdir -p /root/.config && \
  echo "vncpassword" | vncpasswd -f > /root/.vnc/passwd && \
  chmod 600 /root/.vnc/passwd

# Create noVNC startup script
RUN echo '#!/bin/bash' > /root/start_vnc.sh && \
  echo '# Kill any existing VNC servers' >> /root/start_vnc.sh && \
  echo 'vncserver -kill :1 2>/dev/null || true' >> /root/start_vnc.sh && \
  echo '' >> /root/start_vnc.sh && \
  echo '# Start VNC server' >> /root/start_vnc.sh && \
  echo 'vncserver :1 -geometry 1920x1080 -depth 24 -localhost no' >> /root/start_vnc.sh && \
  echo '' >> /root/start_vnc.sh && \
  echo '# Wait a moment for VNC to start' >> /root/start_vnc.sh && \
  echo 'sleep 3' >> /root/start_vnc.sh && \
  echo '' >> /root/start_vnc.sh && \
  echo '# Start websockify for VNC access' >> /root/start_vnc.sh && \
  echo 'echo "Starting VNC web server on port 6080..."' >> /root/start_vnc.sh && \
  echo 'websockify --web=/usr/share/novnc 6080 localhost:5901 2>/dev/null || \' >> /root/start_vnc.sh && \
  echo 'websockify 6080 localhost:5901 &' >> /root/start_vnc.sh && \
  echo '' >> /root/start_vnc.sh && \
  echo '# Keep the container running' >> /root/start_vnc.sh && \
  echo 'wait' >> /root/start_vnc.sh && \
  chmod +x /root/start_vnc.sh

# Setup ROS environment and create entrypoint script
RUN source /opt/ros/jazzy/setup.bash && \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
  echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# Setup entrypoint with VNC support and OpenGL
RUN echo '#!/bin/bash' > /entrypoint.sh && \
  echo '# Fix hostname resolution for VNC (needed with host networking)' >> /entrypoint.sh && \
  echo 'if ! grep -q "$(hostname)" /etc/hosts; then' >> /entrypoint.sh && \
  echo '  echo "" >> /etc/hosts' >> /entrypoint.sh && \
  echo '  echo "127.0.0.1 $(hostname)" >> /etc/hosts' >> /entrypoint.sh && \
  echo 'fi' >> /entrypoint.sh && \
  echo '' >> /entrypoint.sh && \
  echo 'source /opt/ros/jazzy/setup.bash' >> /entrypoint.sh && \
  echo '# Backward compatibility: source a root workspace if present' >> /entrypoint.sh && \
  echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /entrypoint.sh && \
  echo '# Source any project workspaces under /workspace/* if present' >> /entrypoint.sh && \
  echo 'for d in /workspace/*; do' >> /entrypoint.sh && \
  echo '  if [ -f "$d/install/setup.bash" ]; then' >> /entrypoint.sh && \
  echo '    source "$d/install/setup.bash"' >> /entrypoint.sh && \
  echo '  fi' >> /entrypoint.sh && \
  echo 'done' >> /entrypoint.sh && \
  echo '' >> /entrypoint.sh && \
  echo '# Start VNC server if not already running' >> /entrypoint.sh && \
  echo 'if ! pgrep -x "Xvnc" > /dev/null; then' >> /entrypoint.sh && \
  echo '  /root/start_vnc.sh &' >> /entrypoint.sh && \
  echo '  sleep 5' >> /entrypoint.sh && \
  echo 'fi' >> /entrypoint.sh && \
  echo '' >> /entrypoint.sh && \
  echo '# Set OpenGL environment variables for better rendering' >> /entrypoint.sh && \
  echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> /entrypoint.sh && \
  echo 'export LIBGL_ALWAYS_INDIRECT=1' >> /entrypoint.sh && \
  echo 'export QT_QUICK_BACKEND=software' >> /entrypoint.sh && \
  echo 'export QT_X11_NO_MITSHM=1' >> /entrypoint.sh && \
  echo 'export GALLIUM_DRIVER=llvmpipe' >> /entrypoint.sh && \
  echo '' >> /entrypoint.sh && \
  echo '# Start a shell or execute the provided command' >> /entrypoint.sh && \
  echo 'if [ "$#" -eq 0 ]; then' >> /entrypoint.sh && \
  echo '  /bin/bash' >> /entrypoint.sh && \
  echo 'else' >> /entrypoint.sh && \
  echo '  exec "$@"' >> /entrypoint.sh && \
  echo 'fi' >> /entrypoint.sh && \
  chmod +x /entrypoint.sh

# Set display for VNC
ENV DISPLAY=:1

# Expose ports
EXPOSE 6080 5901

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

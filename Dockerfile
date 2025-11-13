# Native ARM64 build for Apple Silicon
# osrf/ros:jazzy-desktop only has amd64 images, so we use ros:jazzy and install desktop manually
FROM ros:jazzy
SHELL ["/bin/bash", "-c"]

# Install all dependencies in one big layer for better caching
RUN apt-get update && apt-get install -y \
  ros-jazzy-desktop \
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
  ros-jazzy-turtlebot3-description \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-teleop-twist-keyboard \
  tigervnc-standalone-server \
  tigervnc-xorg-extension \
  novnc \
  websockify \
  xfce4 \
  xfce4-goodies \
  dbus-x11 \
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
  jupyter-core \
  imagemagick \
  libfmt-dev \
  libconsole-bridge-dev \
  ros-jazzy-iceoryx-binding-c && \
  rm -rf /var/lib/apt/lists/*

# Install Python packages for nvim plugins (molten, dap-python, etc.) and tree-sitter CLI
# Use --ignore-installed to avoid conflicts with system packages
RUN pip3 install --break-system-packages --ignore-installed \
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
  pyperclip && \
  npm install -g tree-sitter-cli

# Install latest neovim from prebuilt ARM64 release (much faster than building from source)
RUN cd /tmp && \
  wget -q https://github.com/neovim/neovim/releases/download/v0.11.5/nvim-linux-arm64.tar.gz && \
  tar xzf nvim-linux-arm64.tar.gz && \
  cp -r nvim-linux-arm64/bin/* /usr/local/bin/ && \
  cp -r nvim-linux-arm64/lib/* /usr/local/lib/ && \
  cp -r nvim-linux-arm64/share/* /usr/local/share/ && \
  rm -rf /tmp/nvim-linux-arm64*

# Install Python ML dependencies (CPU-only PyTorch to avoid CUDA bloat)
# Use --ignore-installed to avoid conflicts with system packages
RUN echo "Installing PyTorch CPU-only (no CUDA)..." && \
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


# Copy project files (models/ excluded - should be volume mounted)
COPY src/ ./src/
COPY robot_description/ ./robot_description/

# Create VNC directory, scripts, and configuration in a single layer
RUN mkdir -p /root/.vnc /root/.config && \
  printf '! Basic X resources\n! This file can be empty - it prevents xrdb errors\n' > /root/.Xresources && \
  printf '#!/bin/bash\n\
# Unset SESSION_MANAGER to avoid conflicts\n\
unset SESSION_MANAGER\n\
unset DBUS_SESSION_BUS_ADDRESS\n\
\n\
# Start D-Bus\n\
service dbus start\n\
\n\
# Set some basic X resources (ignore errors)\n\
xrdb $HOME/.Xresources 2>/dev/null || true\n\
\n\
# Start XFCE desktop environment\n\
exec startxfce4\n' > /root/.vnc/xstartup && \
  chmod +x /root/.vnc/xstartup && \
  echo "vncpassword" | vncpasswd -f > /root/.vnc/passwd && \
  chmod 600 /root/.vnc/passwd && \
  printf '#!/bin/bash\n\
# Kill any existing VNC servers\n\
vncserver -kill :1 2>/dev/null || true\n\
\n\
# Start VNC server\n\
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no\n\
\n\
# Wait a moment for VNC to start\n\
sleep 3\n\
\n\
# Start websockify for VNC access\n\
echo "Starting VNC web server on port 6080..."\n\
websockify --web=/usr/share/novnc 6080 localhost:5901 2>/dev/null || websockify 6080 localhost:5901 &\n\
\n\
# Keep the container running\n\
wait\n' > /root/start_vnc.sh && \
  chmod +x /root/start_vnc.sh

# Create symlinks for unversioned gz-sim plugin names and setup ROS environment
# TurtleBot4 expects libgz-sim-*-system.so but Harmonic provides libgz-sim8-*-system.so
RUN cd /opt/ros/jazzy/opt/gz_sim_vendor/lib/ && \
  for file in libgz-sim8-*-system.so; do \
    ln -sf "$file" "${file/libgz-sim8-/libgz-sim-}"; \
  done && \
  source /opt/ros/jazzy/setup.bash && \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
  echo 'export CMAKE_PREFIX_PATH="/usr/lib/aarch64-linux-gnu/console_bridge:/usr/lib/aarch64-linux-gnu/cmake:/opt/ros/jazzy/lib/aarch64-linux-gnu/cmake:$CMAKE_PREFIX_PATH"' >> ~/.bashrc && \
  echo 'export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:$PYTHONPATH"' >> ~/.bashrc && \
  echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# Setup entrypoint with VNC support and OpenGL
RUN printf '#!/bin/bash\n\
# Fix hostname resolution for VNC (needed with host networking)\n\
if ! grep -q "$(hostname)" /etc/hosts; then\n\
  echo "" >> /etc/hosts\n\
  echo "127.0.0.1 $(hostname)" >> /etc/hosts\n\
fi\n\
\n\
source /opt/ros/jazzy/setup.bash\n\
\n\
# Add system cmake paths for vendor packages (console_bridge, fmt, etc.)\n\
export CMAKE_PREFIX_PATH="/usr/lib/aarch64-linux-gnu/console_bridge:/usr/lib/aarch64-linux-gnu/cmake:/opt/ros/jazzy/lib/aarch64-linux-gnu/cmake:$CMAKE_PREFIX_PATH"\n\
\n\
# Add ROS Python packages to PYTHONPATH\n\
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:$PYTHONPATH"\n\
\n\
# Backward compatibility: source a root workspace if present\n\
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash\n\
# Source any project workspaces under /workspace/* if present\n\
for d in /workspace/*; do\n\
  if [ -f "$d/install/setup.bash" ]; then\n\
    source "$d/install/setup.bash"\n\
  fi\n\
done\n\
\n\
# Start VNC server if not already running\n\
if ! pgrep -x "Xvnc" > /dev/null; then\n\
  /root/start_vnc.sh &\n\
  sleep 5\n\
fi\n\
\n\
# Set OpenGL environment variables for better rendering\n\
export LIBGL_ALWAYS_SOFTWARE=1\n\
export LIBGL_ALWAYS_INDIRECT=1\n\
export QT_QUICK_BACKEND=software\n\
export QT_X11_NO_MITSHM=1\n\
export GALLIUM_DRIVER=llvmpipe\n\
\n\
# Start a shell or execute the provided command\n\
if [ "$#" -eq 0 ]; then\n\
  /bin/bash\n\
else\n\
  exec "$@"\n\
fi\n' > /entrypoint.sh && \
  chmod +x /entrypoint.sh

# Set display for VNC
ENV DISPLAY=:1

# Expose ports
EXPOSE 6080 5901 8765

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

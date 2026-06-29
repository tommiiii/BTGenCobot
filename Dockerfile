# x86_64/AMD64 build
# osrf/ros:jazzy-desktop only has amd64 images, so we use ros:jazzy and install desktop manually
FROM ros:jazzy
SHELL ["/bin/bash", "-c"]

# Install minimal dependencies for robot simulation
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-desktop \
  python3-pip \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-route \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-slam-toolbox \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-imu-sensor-broadcaster \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-position-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-pal-statistics \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-twist-mux \
  ros-jazzy-twist-stamper \
  ros-jazzy-joy-linux \
  ros-jazzy-teleop-twist-keyboard \
  tigervnc-standalone-server \
  tigervnc-tools \
  tigervnc-xorg-extension \
  novnc \
  websockify \
  xfce4 \
  xfce4-terminal \
  dbus-x11 \
  wget \
  git \
  build-essential \
  cmake \
  libfmt-dev \
  libconsole-bridge-dev \
  ros-jazzy-iceoryx-binding-c

# Install Python packages for vision (GroundingDINO-Tiny)
# GroundingDINO is natively integrated in transformers (no trust_remote_code needed)
RUN --mount=type=cache,target=/root/.cache/pip \
  pip3 install --break-system-packages \
  torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
  pip3 install --break-system-packages \
  'numpy<2.0' \
  pillow \
  opencv-python>=4.8.0 \
  'transformers>=4.42,<5.0' \
  scipy \
  sympy \
  ikpy>=3.3

# Create workspace
WORKDIR /workspace

# Note: src/ and robot_description/ are volume-mounted from host (see docker-compose.yml)
# No need to copy or clone - everything is mounted at runtime

# Copy VNC scripts and configuration
COPY scripts/xstartup.sh /root/.vnc/xstartup
COPY scripts/start_vnc.sh /root/start_vnc.sh
RUN mkdir -p /root/.vnc /root/.config && \
  printf '! Basic X resources\n! This file can be empty - it prevents xrdb errors\n' > /root/.Xresources && \
  chmod +x /root/.vnc/xstartup && \
  echo "vncpassword" | vncpasswd -f > /root/.vnc/passwd && \
  chmod 600 /root/.vnc/passwd && \
  chmod +x /root/start_vnc.sh

# Create symlinks for unversioned gz-sim plugin names and setup ROS environment
# TurtleBot4 expects libgz-sim-*-system.so but Harmonic provides libgz-sim8-*-system.so
RUN cd /opt/ros/jazzy/opt/gz_sim_vendor/lib/ && \
  for file in libgz-sim8-*-system.so; do \
    ln -sf "$file" "${file/libgz-sim8-/libgz-sim-}"; \
  done && \
  source /opt/ros/jazzy/setup.bash && \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
  echo 'export CMAKE_PREFIX_PATH="/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/console_bridge:/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/cmake:/opt/ros/jazzy/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/cmake:$CMAKE_PREFIX_PATH"' >> ~/.bashrc && \
  echo 'export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages:$PYTHONPATH"' >> ~/.bashrc && \
  echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# Setup entrypoint with VNC support and OpenGL
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set display for VNC
ENV DISPLAY=:1

# Expose ports
EXPOSE 6080 5901 8765 8080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

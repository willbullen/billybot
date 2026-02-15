# =====================================================================
# Stage 1: Build librealsense2 from source (no pip/apt for aarch64)
# First build takes ~20-40 min on Jetson; cached after that.
# Only rebuilds when LIBRS_VERSION changes.
# Releases: github.com/IntelRealSense/librealsense/tags
# =====================================================================
FROM ros:humble-ros-base AS librealsense-builder

ARG DEBIAN_FRONTEND=noninteractive
ARG LIBRS_VERSION=2.55.1

# Builder dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    python3 \
    python3-dev \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download librealsense source
WORKDIR /usr/src
RUN curl -L https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v${LIBRS_VERSION} -o librealsense.tar.gz \
    && tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz \
    && ln -s /usr/src/librealsense-${LIBRS_VERSION} /usr/src/librealsense

# Build with Python bindings + RSUSB backend (no kernel module needed in Docker)
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
        -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
        -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
        -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS:bool=true \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DFORCE_RSUSB_BACKEND=ON \
        -DCMAKE_BUILD_TYPE=Release \
        ../ \
    && make -j$(nproc) all \
    && make install

# Stage pyrealsense2 Python bindings to a known path for multi-stage COPY
# (cmake may place them under /opt/librealsense/lib/pythonX.Y/... or /usr/lib/...)
RUN PYRS_DIR=$(find /opt/librealsense /usr/lib /usr/local -name "pyrealsense2" -type d 2>/dev/null | head -1) && \
    if [ -n "$PYRS_DIR" ]; then cp -r "$PYRS_DIR" /opt/pyrealsense2; \
    else echo "ERROR: pyrealsense2 Python bindings not found after build" && exit 1; fi

# =====================================================================
# Stage 2: ByYourCommand ROS 2 Container
# Base: ROS 2 Humble on Ubuntu 22.04
# =====================================================================
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Copy librealsense2 from builder (shared libs, headers, tools)
COPY --from=librealsense-builder /opt/librealsense /usr/local/
# Copy pyrealsense2 Python bindings to system dist-packages
COPY --from=librealsense-builder /opt/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
# Copy udev rules for RealSense device permissions
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
# Register shared libraries with the dynamic linker
RUN ldconfig

# Install system dependencies (audio_common* not in Humble apt; built from source below)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-launch-xml \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-joint-state-publisher \
    ros-humble-diagnostic-updater \
    ros-humble-nav2-bringup \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-navfn-planner \
    ros-humble-dwb-core \
    ros-humble-dwb-plugins \
    ros-humble-dwb-critics \
    ros-humble-slam-toolbox \
    ros-humble-depthimage-to-laserscan \
    libboost-all-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-alsa \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    portaudio19-dev \
    libportaudio2 \
    ffmpeg \
    libasound2-dev \
    libasound2-plugins \
    alsa-utils \
    pulseaudio-utils \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Route ALSA through PulseAudio so all audio apps (PyAudio, GStreamer)
# transparently use PulseAudio for device access and sample-rate conversion
RUN printf 'pcm.!default {\n    type pulse\n}\nctl.!default {\n    type pulse\n}\n' \
    > /etc/asound.conf

# Create workspace and add audio_common from source (no Humble release)
WORKDIR /ros2_ws
RUN git clone https://github.com/ros-drivers/audio_common.git --branch ros2 --depth 1 src/audio_common

WORKDIR /ros2_ws/src/by_your_command

# Copy package files first for dependency caching (no setup.py/setup.cfg; ament_cmake only)
COPY package.xml CMakeLists.txt ./
COPY setup/requirements.txt setup/requirements.txt

# Install Python dependencies
RUN pip3 install --no-cache-dir -r setup/requirements.txt

# Copy full source
COPY . .

# Build needed packages (skip sound_play/audio_play - broken on Humble, not used)
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-up-to audio_capture by_your_command --symlink-install

# Source workspace on entry
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

# Default environment
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "by_your_command", "oai_realtime.launch.py"]

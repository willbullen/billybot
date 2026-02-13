# ByYourCommand ROS 2 Container
# Base: ROS 2 Humble on Ubuntu 22.04
# For Jetson deployment, replace with: dustynv/ros:humble-desktop-l4t-r36.4.0
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

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

# ByYourCommand ROS 2 Container
# Base: ROS 2 Humble on Ubuntu 22.04
# For Jetson deployment, replace with: dustynv/ros:humble-desktop-l4t-r36.4.0
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-audio-common \
    ros-humble-audio-common-msgs \
    ros-humble-launch \
    ros-humble-launch-ros \
    portaudio19-dev \
    libportaudio2 \
    ffmpeg \
    libasound2-dev \
    alsa-utils \
    pulseaudio-utils \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws/src/by_your_command

# Copy package files first for dependency caching
COPY package.xml CMakeLists.txt setup.py setup.cfg ./
COPY setup/requirements.txt setup/requirements.txt

# Install Python dependencies
RUN pip3 install --no-cache-dir -r setup/requirements.txt

# Copy full source
COPY . .

# Build the ROS 2 package
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select by_your_command --symlink-install

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

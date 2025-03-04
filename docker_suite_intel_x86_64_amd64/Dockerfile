# Use NVIDIA's base image for Jetson Orin Nano with CUDA support
#FROM nvcr.io/nvidia/l4t-base:r36.2.0
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
ENV VERSION_ROS="ROS2"
ENV ROS_DISTRO=humble
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Update and install essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    wget \
    python3-pip \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 sources
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
#     && echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 Humble and other dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ros2cli \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    libqt5x11extras5 \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Livox SDK dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcap-dev \
    && rm -rf /var/lib/apt/lists/*
# Install unzip
RUN apt-get update && apt-get install -y --no-install-recommends unzip \
    && rm -rf /var/lib/apt/lists/*
# Clone and build Livox SDK
WORKDIR /workspace
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && make install

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib" >> /etc/bash.bashrc

# Source ROS 2 setup in all shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/profile
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/profile
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip \
    && unzip gtsam.zip \
    && rm -rf gtsam.zip \
    && cd gtsam-4.1.1 \
    && mkdir build  \
    && cd build \
    && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. \
    && make install -j$(nproc)

RUN cd /workspace
RUN git clone https://github.com/jbeder/yaml-cpp.git \
    && cd yaml-cpp \
    && mkdir build \
    && cd build \
    && cmake .. && make -j$(nproc)\
    && make install

# Setup workspace
WORKDIR /root/ros2_ws

# Clone and build Livox ROS Driver
RUN mkdir -p src \
    && cd src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && cd livox_ros_driver2 \
    && cp -f package_ROS2.xml package.xml \
    && rm -rf launch_ROS1
RUN cd /root/ros2_ws/src \
    && mkdir -p FAST-LIO-SAM \
    && mkdir -p FAST-LIO-SAM/fast_lio_sam \
    && mkdir -p FAST-LIO-SAM/imgs \
    && mkdir -p FAST-LIO-SAM/third_party 
COPY docker_suite_intel_x86_64_amd64/livox_config/HAP_config.json src/livox_ros_driver2/config/HAP_config.json
COPY docker_suite_intel_x86_64_amd64/livox_launch/hap.launch.py src/livox_ros_driver2/launch/hap.launch.py
COPY docker_suite_intel_x86_64_amd64/livox_launch_ROS2/hap.launch.py src/livox_ros_driver2/launch_ROS2/hap.launch.py

COPY ./fast_lio_sam src/FAST-LIO-SAM/fast_lio_sam
COPY ./imgs src/FAST-LIO-SAM/imgs
COPY ./third_party src/FAST-LIO-SAM/third_party
COPY ./LICENSE src/FAST-LIO-SAM/
COPY ./*.md src/FAST-LIO-SAM/
# Copy launch files and prepare the environment
RUN cp -rf src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch

# Build ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DROS_EDITION=$VERSION_ROS -DHUMBLE_ROS=${ROS_DISTRO}

# Install python3 dependencies for colcon and ROS 2 build tools
RUN pip3 install -U colcon-common-extensions

# Make sure to source ROS setup and workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

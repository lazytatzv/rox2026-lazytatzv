# Flexible ROS 2 Distro selection
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Use bash for all RUN commands to support source and arrays
SHELL ["/bin/bash", "-c"]

# Environment Configuration
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root/lazytatzv_ws
WORKDIR $WORKSPACE

# Install System Utilities & Developer Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build & Tools
    build-essential curl git python3-colcon-common-extensions \
    python3-pip python3-rosdep python3-vcstool \
    # Utilities
    tmux vim nano tree lsof iputils-ping iproute2 usbutils \
    # Hardware/Library dependencies
    evtest libboost-all-dev \
    # Extra ROS tools not necessarily in package.xml
    ros-${ROS_DISTRO}-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Architecture-specific dependencies
ARG TARGETARCH
RUN apt-get update && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        apt-get install -y --no-install-recommends python3-gpiozero libgpiod-dev; \
    else \
        apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-foxglove-bridge \
            ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-graph ccache; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update --include-eol-distros || true

# Copy workspace source code
# Path is relative to build context (expected to be lazytatzv_ws/)
COPY ./src ./src

# Install ROS dependencies via rosdep
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Enhanced shell environment for developers
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "source $WORKSPACE/install/setup.bash" \
    "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" \
    >> /root/.bashrc

CMD ["bash"]
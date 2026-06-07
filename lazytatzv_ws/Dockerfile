# Flexible ROS 2 Distro selection
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

# Environment Configuration
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root/lazytatzv_ws
ENV CCACHE_DIR=/root/.ccache
WORKDIR $WORKSPACE

# 1. Install System Utilities, Linter Tools, and ccache
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential curl git python3-colcon-common-extensions \
    python3-pip python3-rosdep python3-vcstool \
    evtest libboost-all-dev ccache \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-ament-uncrustify \
    black \
    && rm -rf /var/lib/apt/lists/*

# 2. Layer Optimization: Install dependencies first
COPY ./src /tmp/src
RUN apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths /tmp/src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/src

# 3. Setup ccache for colcon
RUN ln -s /usr/bin/ccache /usr/local/bin/gcc && \
    ln -s /usr/bin/ccache /usr/local/bin/g++ && \
    ln -s /usr/bin/ccache /usr/local/bin/cc && \
    ln -s /usr/bin/ccache /usr/local/bin/c++

# 4. Copy actual source code
COPY ./src ./src

# Enhanced shell environment
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "if [ -f $WORKSPACE/install/setup.bash ]; then source $WORKSPACE/install/setup.bash; fi" \
    "export PATH=/usr/lib/ccache:\$PATH" \
    >> /root/.bashrc

CMD ["bash"]

# Flexible ROS 2 Distro selection
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Use bash
SHELL ["/bin/bash", "-c"]

# --- 1. Infrastructure & Mirror Optimization ---
# Switch to JP mirror for lightning fast downloads and install core tools
RUN sed -i 's@http://archive.ubuntu.com@http://jp.archive.ubuntu.com@g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential curl git python3-colcon-common-extensions \
    python3-pip python3-rosdep python3-vcstool \
    evtest libboost-all-dev ccache \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-ament-uncrustify \
    black \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Environment Configuration (Stable Layer) ---
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root/lazytatzv_ws
ENV CCACHE_DIR=/root/.ccache
WORKDIR $WORKSPACE

# --- 3. Dependency Layer (rosdep) ---
# Copy ONLY package.xml files first to cache dependencies separately from code
COPY ./src /tmp/src
RUN apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths /tmp/src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/src

# --- 4. Development Tools Setup ---
RUN ln -sf /usr/bin/ccache /usr/local/bin/gcc && \
    ln -sf /usr/bin/ccache /usr/local/bin/g++ && \
    ln -sf /usr/bin/ccache /usr/local/bin/cc && \
    ln -sf /usr/bin/ccache /usr/local/bin/c++

# --- 5. Source Code Layer ---
COPY ./src ./src

# Final env setup
RUN printf "%s\n" \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "if [ -f $WORKSPACE/install/setup.bash ]; then source $WORKSPACE/install/setup.bash; fi" \
    "export PATH=/usr/lib/ccache:\$PATH" \
    >> /root/.bashrc

CMD ["bash"]

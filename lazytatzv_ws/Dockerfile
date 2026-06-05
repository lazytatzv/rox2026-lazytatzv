ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

ARG TARGETARCH=amd64
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /root/lazytatzv_ws

RUN set -eux; \
        base_packages=( \
            git \
            nano \
            vim \
            curl \
            tree \
            tmux \
            lsof \
            build-essential \
            python3-pip \
            python3-colcon-common-extensions \
            python3-vcstool \
            python3-argcomplete \
            python3-rosdep \
            iputils-ping \
            iproute2 \
            usbutils \
            ros-${ROS_DISTRO}-joy \
            ros-${ROS_DISTRO}-teleop-twist-joy \
            ros-${ROS_DISTRO}-twist-mux \
            evtest \
            libboost-all-dev \
        ); \
        arch_packages=(); \
        case "${TARGETARCH}" in \
            arm64) \
                arch_packages+=(python3-gpiozero libgpiod-dev); \
                ;; \
            amd64) \
                arch_packages+=( \
                    ros-${ROS_DISTRO}-rqt \
                    ros-${ROS_DISTRO}-rqt-graph \
                    ros-${ROS_DISTRO}-rviz2 \
                    ros-${ROS_DISTRO}-foxglove-bridge \
                    ccache \
                ); \
                ;; \
        esac; \
        apt-get update; \
        apt-get install -y --no-install-recommends "${base_packages[@]}" "${arch_packages[@]}"; \
        rm -rf /var/lib/apt/lists/*

RUN set -eux; \
        rosdep init 2>/dev/null || true; \
        rosdep update

COPY ./lazytatzv_ws/src ./src

RUN set -eux; \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        rosdep install \
            --from-paths src \
            --ignore-src \
            -r \
            -y && \
        colcon build --symlink-install

RUN printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        'source /root/lazytatzv_ws/install/setup.bash' \
        >> /root/.bashrc

CMD ["bash"]
# syntax=docker/dockerfile:1.7
FROM docker.io/osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=unoarm
ARG USER_UID=1000
ARG USER_GID=1000

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=25 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        iproute2 \
        iputils-ping \
        jq \
        less \
        lsb-release \
        nano \
        net-tools \
        nodejs \
        npm \
        openssh-client \
        pkg-config \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-pandas \
        python3-pip \
        python3-rosdep \
        python3-seaborn \
        python3-venv \
        python3-yaml \
        ripgrep \
        ros-dev-tools \
        ros-humble-control-msgs \
        ros-humble-joint-state-publisher-gui \
        ros-humble-moveit \
        ros-humble-moveit-configs-utils \
        ros-humble-moveit-ros-planning-interface \
        ros-humble-random-numbers \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-warehouse-ros-sqlite \
        ros-humble-xacro \
        sudo \
        vim \
        xauth \
    && (rosdep init || true) \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir mujoco

# Ubuntu 22.04 provides Node.js 12, while the frontend's Vite toolchain
# requires Node.js 18 or newer. Keep npm from apt only as a bootstrap and pin
# the development image to a reproducible Node.js 20 LTS release.
RUN npm install --global n \
    && n 20.19.5 \
    && hash -r \
    && /usr/local/bin/npm cache clean --force \
    && node --version \
    && /usr/local/bin/npm --version

RUN set -eux \
    && existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" \
    && if [ -n "${existing_user}" ]; then userdel -r "${existing_user}" 2>/dev/null || userdel "${existing_user}"; fi \
    && existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)" \
    && if [ -n "${existing_group}" ]; then groupdel "${existing_group}" 2>/dev/null || true; fi \
    && groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && mkdir -p "/home/${USERNAME}/.colcon" \
    && chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY docker/entrypoint.sh /usr/local/bin/unoarm-docker-entrypoint
RUN chmod 0755 /usr/local/bin/unoarm-docker-entrypoint

USER ${USERNAME}
WORKDIR /workspace/unoarm

RUN echo 'source /opt/ros/humble/setup.bash' >> "/home/${USERNAME}/.bashrc" \
    && echo '[ -f /workspace/unoarm/install/setup.bash ] && source /workspace/unoarm/install/setup.bash' >> "/home/${USERNAME}/.bashrc" \
    && echo '[ -f /workspace/unoarm/CSP_direct_drive/install/setup.bash ] && source /workspace/unoarm/CSP_direct_drive/install/setup.bash' >> "/home/${USERNAME}/.bashrc"

ENTRYPOINT ["/usr/local/bin/unoarm-docker-entrypoint"]
CMD ["bash"]

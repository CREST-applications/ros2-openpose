ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update && apt-get install -y \
    avahi-utils \
    python3-pip \
    libopencv-dev \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins

RUN pip install \
    opencv-python \
    cv_bridge \
    janus

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
COPY ./pypleiades /usr/local/lib/pleiades/pypleiades
RUN uv pip install --system -e /usr/local/lib/pleiades/pypleiades

ARG UID
ARG GID

RUN groupadd -g ${GID} user && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} user

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/user/.bashrc
RUN usermod -a -G video user

USER user
WORKDIR /home/user/

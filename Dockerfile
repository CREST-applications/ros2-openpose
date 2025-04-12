FROM ros:humble

RUN mkdir -p /root/ros2_ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    python3-pip \
    libopencv-dev \
    avahi-utils \
    ros-humble-v4l2-camera \
    ros-humble-rqt-image-view \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins

RUN pip3 install \
    uv \
    opencv-python \
    cv_bridge \
    janus

COPY ./pypleiades /usr/local/lib/pleiades/pypleiades
RUN uv pip install --system -e /usr/local/lib/pleiades/pypleiades

ARG UID
ARG GID

RUN groupadd -g ${GID} user && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} user

RUN echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc
RUN usermod -a -G video user

USER user
WORKDIR /home/user/

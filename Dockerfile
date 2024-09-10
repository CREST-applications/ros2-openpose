FROM ros:humble

RUN mkdir -p /root/ros2_ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    python3-pip \
    libopencv-dev \
    avahi-utils \
    ros-humble-v4l2-camera \
    ros-humble-rqt-image-view \
    ros-humble-image-transport

RUN pip3 install \
    uv \
    opencv-python \
    cv_bridge \
    pydantic \
    janus

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

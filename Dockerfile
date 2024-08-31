FROM ros:humble

RUN mkdir -p /root/ros2_ws/src
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    python3-pip \
    libopencv-dev

RUN pip3 install \
    opencv-python \
    cv_bridge \
    pydantic

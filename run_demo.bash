#! /bin/bash

TB3_HOST=tb3-sasaki.local
TB3_USER=user
WORK_DIR=/home/user/workspace/ros2-openpose

ssh ${TB3_USER}@${TB3_HOST} << EOF
    source /opt/ros/humble/setup.bash
    tmux new-session -d -s ros2-camera "ros2 run v4l2_camera v4l2_camera_node"
EOF

docker compose -f ${WORK_DIR}/compose.yml up -d

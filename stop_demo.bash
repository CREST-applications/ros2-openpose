#! /bin/bash

TB3_HOST=tb3-sasaki.local
TB3_USER=user
WORK_DIR=/home/user/workspace/ros2-openpose

ssh ${TB3_USER}@${TB3_HOST} "tmux kill-session -t ros2-camera"

docker compose -f ${WORK_DIR}/compose.yml down

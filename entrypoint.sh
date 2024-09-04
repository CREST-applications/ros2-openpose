#!/bin/bash

set -e

uv pip install --system -e /root/ros2_ws/src/pymec

exec "$@"

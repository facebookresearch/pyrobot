#!/bin/bash
set -e

# setup ros environment
source /opt/ros/noetic/setup.bash
source /root/create_ws/devel/setup.bash
source /root/locobot_ws/devel/setup.bash
source /root/.bashrc
cd workspace/
pip install -e /workspace/pyrobot

eval "bash"

exec "$@"

#!/bin/bash
source /opt/ros/noetic/setup.bash
mkdir -p /root/create_ws/src
cd /root/create_ws
catkin init
cd src
git clone https://github.com/AutonomyLab/libcreate.git
catkin build


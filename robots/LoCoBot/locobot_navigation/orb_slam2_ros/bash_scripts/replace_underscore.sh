#!/usr/bin/env bash
source /opt/ros/kinetic/setup.bash
source ~/low_cost_ws/devel/setup.bash
CDIR=$(pwd)
roscd orb_slam2_ros/cfg/
sed -i 's/\_/./g' default.yaml
cd $CDIR
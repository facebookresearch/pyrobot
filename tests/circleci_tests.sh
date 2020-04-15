#!/usr/bin/env bash

source ~/.bashrc
source /opt/ros/melodic/setup.bash
source ~/low_cost_ws/devel/setup.bash
source ~/pyenv_pyrobot_python3/bin/activate
source ~/pyrobot_catkin_ws/devel/setup.bash

pytest test_make_robots.py -v
pytest test_pyrobot_classes.py -v
pytest test_camera.py -v
pytest test_base_velocity_control.py -v
pytest test_base_position_control_inits.py -v
pytest test_base_controllers.py -v
pytest test_arm_controls.py -v
pytest test_arm_utils.py -v

exit 0

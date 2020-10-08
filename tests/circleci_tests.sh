#!/usr/bin/env bash

pytest test_make_world.py -v
sleep 30

pytest test_pyrobot_classes.py -v
sleep 30

pytest test_camera.py -v
sleep 30

pytest test_base_velocity_control.py -v
sleep 30

pytest test_base_position_control_inits.py -v
sleep 30

pytest test_base_controllers.py -v
sleep 30

pytest test_arm_controls.py -v
sleep 30

pytest test_arm_utils.py -v

exit 0

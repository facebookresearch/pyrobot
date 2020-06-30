#!/usr/bin/env bash

pytest test_make_robots.py -v
pytest test_pyrobot_classes.py -v
pytest test_camera.py -v
pytest test_base_velocity_control.py -v
pytest test_base_position_control_inits.py -v
pytest test_base_controllers.py -v
pytest test_arm_controls.py -v

exit 0

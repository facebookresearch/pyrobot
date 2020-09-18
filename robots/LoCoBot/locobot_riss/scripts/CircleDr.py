#!/usr/bin/python3.6
from pyrobot import Robot

robot = Robot('locobot')

current_state = robot.base.get_state('odom')
print(f'Starting Pose: {current_state}')
robot.base.set_vel(fwd_speed=5, turn_speed=5, exe_time=10)
robot_base.stop()
current_state = robot.base.get_state('odom')
print(f'Ending Pose: {current_state}')

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

BOT_NAME = 'locobot'
CONFIG = {'base_controller': 'ilqr', 'base_planner': 'movebase'} # not sure why but the tutorials say so


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def main():

    bot = Robot(BOT_NAME, CONFIG)
    print('Locobot Initialized')
    current_state = bot.base.get_state('odom') # can also be 'vslam' for visual slam
    print(f'Starting at {current_state}')


    while not rospy.is_shutdown():
        """
            This loop is where you can implement your algorithm/control
            robot.base.set_vel(fwd_speed=s,turn_speed=s,exe_time=t)
            robot.base.go_to_absolute(target_position, close_loop=False, smooth=True)
        """
        bot.base.go_to_absolute((4,-4,0), close_loop=True, smooth=True)

    bot.base.stop()


if __name__ == "__main__":
    app.run(main)

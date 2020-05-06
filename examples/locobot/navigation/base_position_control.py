# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

from pyrobot.locobot.base_control_utils import (
    get_trajectory_circle,
    get_trajectory_negcircle,
    _get_absolute_pose,
)
import sys

from pyrobot.locobot.bicycle_model import wrap_theta

def _test_relative_position_control(
    posn,
    create_robot,
    base_controller,
    base_planner,
    close_loop,
    smooth,
    trans_thresh,
    angular_thresh,
):
    bot = create_robot
    bot.base.load_controller(base_controller)
    bot.base.load_planner(base_planner)

    start_state = np.array(bot.base.get_state("odom"))
    desired_target = _get_absolute_pose(posn, start_state)
    bot.base.go_to_relative(posn, use_map=False, close_loop=close_loop, smooth=smooth)
    end_state = np.array(bot.base.get_state("odom"))

    print("*************", posn,"***********************")
    dist = np.linalg.norm(end_state[:2] - desired_target[:2])
    angle = end_state[2] - desired_target[2]
    angle = np.abs(wrap_theta(angle))
    try:
        if dist > trans_thresh:
            rospy.logerr(base_controller)
            rospy.logerr("distance error ")
        if not (angle * 180.0 / np.pi < angular_thresh):
            rospy.logerr(base_controller)
            rospy.logerr("ANGLE error ")
    except:
         rospy.logerr("Unexpected error:")


def main():


    base_planner = "movebase"

    bot = Robot("locobot", use_arm=False, use_camera=False)


    posns = [
        #[1.0, 0.0, 0.0],
        #[0.0, 0.0, np.pi],
        #[-1.0,  0.0, np.pi],
        #[1.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]

    for posn in posns:
        #posn = [1.0, 0.0, 0.0]
        print("##############################################", posn, "############################################")
        # _test_relative_position_control(
        # posn,
        # bot,
        # "proportional",
        # "movebase",
        # close_loop=True,
        # smooth=False,
        # trans_thresh=0.25,
        # angular_thresh=20)


        _test_relative_position_control(
        posn,
        bot,
        "gpmp",
        "movebase",
        close_loop=True,
        smooth=True,
        trans_thresh=0.25,
        angular_thresh=20)



        # _test_relative_position_control(
        # posn,
        # bot,
        # "movebase",
        # "movebase",
        # close_loop=True,
        # smooth=False,
        # trans_thresh=0.25,
        # angular_thresh=20)


        # _test_relative_position_control(
        # posn,
        # bot,
        # "ilqr",
        # "movebase",
        # close_loop=True,
        # smooth=False,
        # trans_thresh=0.25,
        # angular_thresh=20)





    # bot.base.load_controller("proportional")
    # bot.base.load_planner(base_planner)
    # bot.base.go_to_relative(posn, use_map=False, close_loop=True, smooth=False)
    # end_state = np.array(bot.base.get_state("odom"))
    # print("*************", end_state)




    # bot.base.load_controller("gpmp")
    # bot.base.load_planner(base_planner)
    # bot.base.go_to_relative(posn, use_map=False, close_loop=True, smooth=True)
    # end_state = np.array(bot.base.get_state("odom"))
    # print("*************", end_state,"***********************")

    # bot.base.load_controller("ilqr")
    # bot.base.load_planner(base_planner)
    # bot.base.go_to_relative(posn, use_map=False, close_loop=True, smooth=True)
    # end_state = np.array(bot.base.get_state("odom"))
    # print("*************", end_state)
    # #bot.base.stop()

    # bot.base.load_controller("movebase")
    # bot.base.load_planner(base_planner)
    # bot.base.go_to_relative(posn, use_map=False, close_loop=True, smooth=False)
    # end_state = np.array(bot.base.get_state("odom"))
    # print("*************", end_state)

if __name__ == "__main__":
    main()

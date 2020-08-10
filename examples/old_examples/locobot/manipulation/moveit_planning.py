# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with the src using planner
"""

from pyrobot import Robot


def main():
    target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]

    config = dict(moveit_planner="ESTkConfigDefault")

    bot = Robot("locobot", arm_config=config)
    bot.arm.go_home()

    for joint in target_joints:
        bot.arm.set_joint_positions(joint, plan=True)

    bot.arm.go_home()


if __name__ == "__main__":
    main()

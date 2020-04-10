# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding first four joint of arm with specific torque
Make sure you launch the main.launch using torque control
This script only works on the real robot
"""
from pyrobot import Robot


def main():
    # Example torque [it should be a list of 4 elements
    # with values in the range of (-4.1:4.1)]
    target_torques = 4 * [0.1]

    arm_config = dict(control_mode="torque")

    bot = Robot("locobot", arm_config=arm_config)
    bot.arm.set_joint_torques(target_torques)


if __name__ == "__main__":
    main()

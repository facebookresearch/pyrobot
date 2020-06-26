# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with torque control (based on intera examples)
"""

import signal
import sys
import time

import numpy as np
from pyrobot import Robot


def signal_handler(sig, frame):
    print("Exit")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def spring_damping(position_err, velocity_err, spring_coef, damping_coef):
    torques = -np.multiply(spring_coef, position_err)
    torques -= np.multiply(damping_coef, velocity_err)
    return torques


def main():
    bot = Robot(
        "sawyer", use_arm=True, use_base=False, use_camera=False, use_gripper=True
    )
    bot.arm.move_to_neutral()
    ini_joint_angles = np.array(bot.arm.get_joint_angles())

    spring_coef = np.array([30.0, 45.0, 15.0, 15.0, 9.0, 6.0, 4.5])
    damping_coef = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    while True:
        joint_angles = np.array(bot.arm.get_joint_angles())
        joint_velocities = np.array(bot.arm.get_joint_velocities())
        pos_err = joint_angles - ini_joint_angles
        vel_err = joint_velocities
        joint_torques = spring_damping(pos_err, vel_err, spring_coef, damping_coef)
        bot.arm.set_joint_torques(joint_torques)
        time.sleep(0.001)


if __name__ == "__main__":
    main()

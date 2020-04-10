# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for moving the end effector along x,y,z axis
"""
import time

import numpy as np
from pyrobot import Robot


def main():
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot(
        "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False
    )
    plan = True
    bot.arm.move_to_neutral()
    time.sleep(1)
    displacement = np.array([0.0, 0.0, 0.15])
    bot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    print(bot.arm.get_joint_angles())
    displacement = np.array([0.1, 0, 0.0])
    bot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    displacement = np.array([0.0, 0.25, 0.0])
    bot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    bot.arm.go_home()


if __name__ == "__main__":
    main()

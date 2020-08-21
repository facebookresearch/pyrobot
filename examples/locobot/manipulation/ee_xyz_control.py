# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for moving the end effector along x,y,z axis
"""
import numpy as np
from pyrobot import Robot


def main():
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot("locobot")
    bot.arm.go_home()
    displacement = np.array([0, 0, -0.15])
    bot.arm.move_ee_xyz(displacement, plan=True)
    bot.arm.go_home()


if __name__ == "__main__":
    main()

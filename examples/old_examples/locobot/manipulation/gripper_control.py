# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

""" Example for controlling arm gripper """

from __future__ import print_function

from pyrobot import Robot

if __name__ == "__main__":
    bot = Robot("locobot")

    print("Opening gripper")
    bot.gripper.open()

    print("Closing gripper")
    bot.gripper.close()

    print("Opening gripper")
    bot.gripper.open()

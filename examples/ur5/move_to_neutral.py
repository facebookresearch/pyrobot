# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot to a neutral pose
"""

import time

from pyrobot import Robot


def main():
    bot = Robot(
        "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False
    )
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.move_to_neutral()
    time.sleep(1)
    bot.arm.go_home()


if __name__ == "__main__":
    main()

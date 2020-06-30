# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot to specific gripper pose
"""
import time

import numpy as np
from pyrobot import Robot


def main():
    # Example poses
    # orientation can be either rotation matrix or quaternion
    target_poses = [
        {
            "position": np.array([0.279, 0.176, 0.217]),
            "orientation": np.array(
                [
                    [0.5380200, -0.6650449, 0.5179283],
                    [0.4758410, 0.7467951, 0.4646209],
                    [-0.6957800, -0.0035238, 0.7182463],
                ]
            ),
        },
        {
            "position": np.array([0.339, 0.0116, 0.255]),
            "orientation": np.array([0.245, 0.613, -0.202, 0.723]),
        },
    ]

    bot = Robot("locobot")
    bot.arm.go_home()

    for pose in target_poses:
        bot.arm.set_ee_pose(**pose)
        time.sleep(1)

    bot.arm.go_home()


if __name__ == "__main__":
    main()

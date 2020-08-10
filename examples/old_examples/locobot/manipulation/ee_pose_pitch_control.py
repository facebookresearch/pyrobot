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
    target_poses = [
        {"position": np.array([0.28, 0.17, 0.22]), "pitch": 0.5, "numerical": False},
        {
            "position": np.array([0.28, -0.17, 0.22]),
            "pitch": 0.5,
            "roll": 0.5,
            "numerical": False,
        },
    ]

    bot = Robot("locobot")
    bot.arm.go_home()

    for pose in target_poses:
        bot.arm.set_ee_pose_pitch_roll(**pose)
        time.sleep(1)

    bot.arm.go_home()


if __name__ == "__main__":
    main()

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
            "position": np.array([-0.38293327, 0.5077687, 0.72630546]),
            "orientation": np.array(
                [
                    [-0.88077548, -0.45764594, -0.12163366],
                    [-0.14853923, 0.02311451, 0.98863634],
                    [-0.44963391, 0.88883402, -0.08833707],
                ]
            ),
        },
    ]
    bot = Robot(
        "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False
    )

    bot.arm.move_to_neutral()
    time.sleep(1)
    for pose in target_poses:
        bot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)

    bot.arm.go_home()

    # Forward Kinematics Example
    angles = np.array([1.9278, -0.9005, -0.002, -1.0271, -0.5009, 0.5031])
    print(bot.arm.compute_fk_position(angles, "ee_link"))


if __name__ == "__main__":
    main()

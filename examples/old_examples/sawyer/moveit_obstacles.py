# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with position control using moveit planner
"""

from pyrobot import Robot
from pyrobot.utils.util import MoveitObjectHandler
import time
import numpy as np


def main():
    config = dict(moveit_planner="ESTkConfigDefault")
    bot = Robot(
        "sawyer",
        use_arm=True,
        use_base=False,
        use_camera=False,
        use_gripper=True,
        arm_config=config,
    )
    obstacle_handler = MoveitObjectHandler()
    # Add a table
    # position and orientation (quaternion: x, y, z, w) of the table
    pose = [0.8, 0.0, -0.23, 0.0, 0.0, 0.0, 1.0]
    # size of the table (x, y, z)
    size = (1.35, 2.0, 0.1)
    obstacle_handler.add_table(pose, size)

    target_poses = [
        {
            "position": np.array([0.8219, 0.0239, -0.1]),
            "orientation": np.array(
                [
                    [-0.3656171, 0.6683861, 0.6477531],
                    [0.9298826, 0.2319989, 0.2854731],
                    [0.0405283, 0.7067082, -0.7063434],
                ]
            ),
        },
        {
            "position": np.array([0.7320, 0.1548, -0.15]),
            "orientation": np.array([0.1817, 0.9046, -0.1997, 0.3298]),
        },
    ]
    bot.arm.go_home()
    time.sleep(1)
    for pose in target_poses:
        bot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)
    bot.arm.go_home()


if __name__ == "__main__":
    main()

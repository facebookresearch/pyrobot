# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with position control using moveit planner
"""

from pyrobot import Robot
from pyrobot.utils.util import MoveitObjectHandler
from pyrobot.utils.move_group_interface import MoveGroupInterface
import time
import numpy as np

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


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
    pose = [0.8, 0.0, -0.1, 0.0, 0.0, 0.0, 1.0]
    # size of the table (x, y, z)
    size = (1.35, 2.0, 0.1)
    obstacle_handler.add_table(pose, size)

    target_pos = np.array([0.45251711, 0.16039618, 0.08021886])
    target_ori = np.array(
        [
            [-0.00437824, 0.99994626, 0.00939675],
            [0.99998735, 0.0044013, -0.00243524],
            [-0.00247647, 0.00938597, -0.99995288],
        ]
    )

    bot.arm.go_home()
    time.sleep(1)
    traj = bot.arm.make_plan_pose(target_pos, target_ori)
    time.sleep(1)
    print(traj)
    for joints in traj:
        bot.arm.set_joint_positions(joints, plan=False)
        time.sleep(1)
    # bot.arm.set_ee_pose(target_pos, target_ori, plan=True)
    # time.sleep(1)
    # bot.arm.go_home()


if __name__ == "__main__":
    main()

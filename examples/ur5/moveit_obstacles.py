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
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot(
        "ur5",
        use_arm=True,
        use_base=False,
        use_camera=False,
        use_gripper=True,
        arm_config=config,
    )
    obstacle_handler = MoveitObjectHandler()
    # Add a table
    # position and orientation (quaternion: x, y, z, w) of the table
    pose = [0.0, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0]  # base_link frame
    # size of the table (x, y, z)
    size = (2.0, 2.0, 0.1)
    obstacle_handler.planning_scene_interface.clear()
    obstacle_handler.add_table(pose, size)

    plan = True
    bot.arm.move_to_neutral()
    time.sleep(1)
    displacement = np.array([-0.35, 0.0, 0.0])
    bot.arm.move_ee_xyz(displacement, plan=plan, eef_step=0.009)
    # time.sleep(1)
    print(bot.arm.get_joint_angles())
    displacement = np.array([0.0, 0, 0.25])
    bot.arm.move_ee_xyz(displacement, plan=plan, eef_step=0.009)
    time.sleep(1)
    displacement = np.array([0.0, 0.25, 0.0])
    bot.arm.move_ee_xyz(displacement, plan=plan, eef_step=0.009)
    time.sleep(1)

    bot.arm.go_home()

    # clear all the obstacles from the scene
    obstacle_handler.planning_scene_interface.clear()


if __name__ == "__main__":
    main()


# import time

# from pyrobot import Robot

# import numpy as np

# def main():
#     bot = Robot('ur5',
#                 use_arm=True,
#                 use_base=False,
#                 use_camera=False,
#                 use_gripper=False)
#     bot.arm.go_home()
#     time.sleep(1)
#     bot.arm.move_to_neutral()
#     time.sleep(1)
#     bot.arm.go_home()

#     angles = [0.1, 0.0, 0.0, 0.0, 0.0, 0.5]
#     bot.arm.set_joint_positions(angles, plan=True)
#     time.sleep(1)

#     angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     print(bot.arm.compute_fk_position(angles, 'ee_link'))

#     dis = np.array([ -0.5, 0.0, 0.2])
#     bot.arm.move_ee_xyz(displacement=dis, plan=True)

# if __name__ == "__main__":
#     main()

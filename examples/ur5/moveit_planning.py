# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with position control using moveit planner
"""

import time

from pyrobot import Robot


def main():
    target_joints = [
        [0.704, -0.455, -0.159, 1.395, -1.240, 1.069, 2.477],
        [-0.341, -0.384, -0.018, 1.533, -0.977, -1.492, -1.084],
    ]

    config = dict(moveit_planner="ESTkConfigDefault")
    bot = Robot(
        "ur5",
        use_arm=True,
        use_base=False,
        use_camera=False,
        use_gripper=False,
        arm_config=config,
    )
    bot.arm.go_home()

    time.sleep(1)
    for joint in target_joints:
        bot.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

    bot.arm.go_home()


if __name__ == "__main__":
    main()

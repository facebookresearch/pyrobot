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
    target_poses = [{'position': np.array([0.8219, 0.0239, 0.0996]),
                     'orientation': np.array([[-0.3656171, 0.6683861, 0.6477531],
                                              [0.9298826, 0.2319989, 0.2854731],
                                              [0.0405283, 0.7067082, -0.7063434]])},
                    {'position': np.array([0.7320, 0.1548, 0.0768]),
                     'orientation': np.array([0.1817, 0.9046, -0.1997, 0.3298])},
                    ]
    bot = Robot('ur5',
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
    bot.arm.go_home()
    time.sleep(1)
    for pose in target_poses:
        bot.arm.set_ee_pose(plan=True, **pose)
        time.sleep(1)

    bot.arm.go_home()


if __name__ == "__main__":
    main()



import time

from pyrobot import Robot

import numpy as np

def main():
    bot = Robot('ur5',
                use_arm=True,
                use_base=False,
                use_camera=False,
                use_gripper=False)
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.move_to_neutral()
    time.sleep(1)
    bot.arm.go_home()
    
    angles = [0.1, 0.0, 0.0, 0.0, 0.0, 0.5]
    bot.arm.set_joint_positions(angles, plan=True)
    time.sleep(1)

    angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(bot.arm.compute_fk_position(angles, 'ee_link'))

    dis = np.array([ -0.5, 0.0, 0.2])
    bot.arm.move_ee_xyz(displacement=dis, plan=True)

if __name__ == "__main__":
    main()

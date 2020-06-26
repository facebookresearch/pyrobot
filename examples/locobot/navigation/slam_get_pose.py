# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import numpy as np
from pyrobot import Robot


def main():
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot("locobot")
    print("\nInitial base pose:")
    ini_base_pose = bot.base.base_state.vslam.base_pose_xyyaw
    if None in ini_base_pose:
        raise RuntimeError(
            "No camera pose data received!!!\n"
            "Please check if ORB-SLAM2 is running properly\n"
            "and make sure the tracking is not lost!"
        )
    print(
        "X: {0:f}   Y: {1:f}   Yaw: {2:f}".format(
            ini_base_pose[0], ini_base_pose[1], ini_base_pose[2]
        )
    )
    print("\nInitial camera pose:")
    ini_c_trans, ini_c_rot, ini_c_T = bot.base.base_state.vslam.camera_pose
    print("Translation:", ini_c_trans)
    print("Rotation:")
    print(ini_c_rot)
    bot.base.set_vel(fwd_speed=0.1, turn_speed=0, exe_time=1)
    bot.base.stop()
    print("\nBase pose after moving:")
    af_base_pose = bot.base.base_state.vslam.base_pose_xyyaw
    print(
        "X: {0:f}   Y: {1:f}   Yaw: {2:f}".format(
            af_base_pose[0], af_base_pose[1], af_base_pose[2]
        )
    )
    print("\nCamera pose after moving:")
    af_c_trans, af_c_rot, aft_c_T = bot.base.base_state.vslam.camera_pose
    print("Translation:", af_c_trans)
    print("Rotation:")
    print(af_c_rot)


if __name__ == "__main__":
    main()

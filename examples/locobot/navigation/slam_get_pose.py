# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function

import logging
import numpy as np
from pyrobot import Robot


def main():
    np.set_printoptions(precision=4, suppress=True)
    bot = Robot('locobot')
    logging.info('\nInitial base pose:')
    ini_base_pose = bot.base.base_state.vslam.base_pose_xyyaw
    if None in ini_base_pose:
        raise RuntimeError('No camera pose data received!!!\n'
                           'Please check if ORB-SLAM2 is running properly\n'
                           'and make sure the tracking is not lost!')
    logging.info('X: {0:f}   Y: {1:f}   Yaw: {2:f}'.format(ini_base_pose[0],
                                                    ini_base_pose[1],
                                                    ini_base_pose[2]))
    logging.info('\nInitial camera pose:')
    ini_c_trans, ini_c_rot, ini_c_T = bot.base.base_state.vslam.camera_pose
    logging.info('Translation:', ini_c_trans)
    logging.info('Rotation:')
    logging.info(ini_c_rot)
    bot.base.set_vel(fwd_speed=0.1, turn_speed=0, exe_time=1)
    bot.base.stop()
    logging.info('\nBase pose after moving:')
    af_base_pose = bot.base.base_state.vslam.base_pose_xyyaw
    logging.info('X: {0:f}   Y: {1:f}   Yaw: {2:f}'.format(af_base_pose[0],
                                                    af_base_pose[1],
                                                    af_base_pose[2]))
    logging.info('\nCamera pose after moving:')
    af_c_trans, af_c_rot, aft_c_T = bot.base.base_state.vslam.camera_pose
    logging.info('Translation:', af_c_trans)
    logging.info('Rotation:')
    logging.info(af_c_rot)


if __name__ == "__main__":
    main()

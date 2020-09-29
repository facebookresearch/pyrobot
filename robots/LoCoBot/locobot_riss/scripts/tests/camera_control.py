#!/home/m_dyse/pyenvs/pyro3_env/bin/python
# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for controlling pan-tilt active camera system and get image data
"""
from __future__ import print_function

from pyrobot.utils.util import try_cv2_import

cv = try_cv2_import()

from pyrobot import Robot

import numpy as np

if __name__ == "__main__":
    bot = Robot("locobot")
    bot.camera.reset()

    camera_poses = [[0, 0.7], [0.4, 0.4], [0.4, -0.4], [-0.4, 0.4], [-0.4, -0.4]]
    for pose in camera_poses:
        print("Setting pan: {}, tilt: {}".format(pose[0], pose[1]))
        bot.camera.set_pan_tilt(pose[0], pose[1], wait=True)
        rgb = bot.camera.get_rgb()
        depth = bot.camera.get_depth()
        # actual_depth_values = depth.astype(np.float64) / 1000
        cv.imshow("Color", rgb[:, :, ::-1])
        cv.imshow("Depth", depth.astype(np.float64) * 1000)
        cv.waitKey(2000)
    bot.camera.reset()
    rgb = bot.camera.get_rgb()
    depth = bot.camera.get_depth()
    cv.imshow("Color", rgb[:, :, ::-1])
    cv.imshow("Depth", depth.astype(np.float64) * 1000)
    print(f'Depth shape: {depth.shape}, RGB shape: {rgb.shape}')
    print(depth[:50])
    print(depth[430:])
    cv.waitKey(5000)

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example to show how to read images from the camera. 
"""
from __future__ import print_function

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np
from pyrobot import Robot

if __name__ == "__main__":
    bot = Robot("locobot")
    rgb, depth = bot.camera.get_rgb_depth()
    actual_depth_values = depth.astype(np.float64) / 1000
    cv2.imshow("Color", rgb[:, :, ::-1])
    cv2.imshow("Depth", depth)
    cv2.waitKey(5000)

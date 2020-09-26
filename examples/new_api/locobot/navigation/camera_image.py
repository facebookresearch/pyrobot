"""
Example to show how to read images from the camera. 
"""
from __future__ import print_function

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm

import copy
import time

if __name__ == "__main__":
    world =  World(config_name='env/base_env.yaml')

    bot = world.robots['locobot']

    rgb, depth = bot['camera'].get_rgb_depth()
    cv2.imshow("Color", rgb[:, :, ::-1])
    # Multiply by 1000 just to increase contrast in depth image
    cv2.imshow("Depth", 1000* depth) 
    cv2.waitKey(5000)

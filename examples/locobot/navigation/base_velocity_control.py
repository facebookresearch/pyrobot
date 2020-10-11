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

def main():
    world =  World(config_name='env/base_env.yaml')

    bot = world.robots['locobot']

    bot['base'].set_vel(
        0.2,
        0.5,
        1.0,
    )
    bot['base'].stop()


if __name__ == "__main__":
    main()
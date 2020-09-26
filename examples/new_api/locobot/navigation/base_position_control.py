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

    displacement = np.array([5, 1, 0])

    world.algorithms['proportional_control'].go_to_relative(displacement)
    time.sleep(1)
    print("proportional_control completed!")

    world.algorithms['ilqr_control'].go_to_relative(displacement)
    time.sleep(1)
    print("ilqr_control completed!")

    world.algorithms['movebase_control'].go_to_relative(displacement)
    time.sleep(1)
    print("movebase_control completed!")

    bot['base'].stop()


if __name__ == "__main__":
    main()
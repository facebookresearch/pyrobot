from __future__ import print_function

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm

from pyrobot.algorithms.base_controller_impl.base_control_utils import (
    get_trajectory_circle,
    get_trajectory_negcircle,
)

import copy
import time

def get_trajectory(world, bot, trajectory_type):
    dt = 0.1
    v = 0.1
    w = 0.25

    if trajectory_type == "circle":
        r = v / w
        start_state = np.array([0, 0, 0])
        states, _ = get_trajectory_circle(start_state, dt, r, v, 2 * np.pi)

    elif trajectory_type == "twocircles":
        r = v / w
        start_state = np.array([0, 0, 0])
        states1, _ = get_trajectory_circle(start_state, dt, r, v, np.pi)
        states2, _ = get_trajectory_negcircle(states1[-1, :].copy(), dt, r, v, np.pi)
        states = np.concatenate([states1, states2], 0)
    else:
        raise ValueError("Trajectory type [%s] not implemented" % trajectory_type)

    return states

def main():
    world =  World(config_name='env/base_env.yaml')

    bot = world.robots['locobot']

    states = get_trajectory(world, bot, "circle")

    world.algorithms['ilqr_control'].track_trajectory(states)
    time.sleep(1)
    print("ilqr_control completed!")

    bot['base'].stop()


if __name__ == "__main__":
    main()
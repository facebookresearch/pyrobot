import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm
from pyrobot.algorithms.kinematics import Kinematics 

import copy
import time


def main():
    # Example poses
    # orientation can be either rotation matrix or quaternion
    target_poses = [
        {
            "position": np.array([0.279, 0.176, 0.217]),
            "orientation": np.array(
                [
                    [0.5380200, -0.6650449, 0.5179283],
                    [0.4758410, 0.7467951, 0.4646209],
                    [-0.6957800, -0.0035238, 0.7182463],
                ]
            ),
        },
        {
            "position": np.array([0.339, 0.0116, 0.255]),
            "orientation": np.array([0.245, 0.613, -0.202, 0.723]),
        },
    ]

    world =  World(config_name='env/arm_env.yaml')

    bot = world.robots['locobot']
    bot["arm"].go_home()

    for pose in target_poses:
        world.algorithms['moveit_kin_planner'].plan_end_effector_pose(pose["position"], pose["orientation"])
        time.sleep(1)

    bot["arm"].go_home()

if __name__ == "__main__":
    main()

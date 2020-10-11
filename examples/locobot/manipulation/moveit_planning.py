import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm
from pyrobot.algorithms.kinematics import Kinematics 

import copy
import time


def main():
    
    world =  World(config_name='env/arm_env.yaml')

    bot = world.robots['locobot']

    target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]

    bot['arm'].go_home()

    for joint in target_joints:
        world.algorithms['moveit_kin_planner'].plan_joint_angles(joint)
        time.sleep(1)

    bot['arm'].go_home()

if __name__ == "__main__":
    main()
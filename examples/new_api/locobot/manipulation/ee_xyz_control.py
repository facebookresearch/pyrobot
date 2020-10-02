import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World, make_algorithm
from pyrobot.algorithms.kinematics import Kinematics 

import copy


def main():
    
    world =  World(config_name='env/arm_env.yaml')

    bot = world.robots['locobot']

    bot["arm"].go_home()

    displacement = np.array([0, 0, -0.15])

    ee_pose = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    cur_pos, cur_quat = ee_pose
    tar_pos = cur_pos + displacement

    world.algorithms['moveit_kin_planner'].plan_end_effector_pose(tar_pos, cur_quat)

    bot["arm"].go_home()


if __name__ == "__main__":
    main()

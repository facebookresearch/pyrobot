#  Please use this file for debugging while re-factoring PyRobot

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

    displacement = np.array([0, 0, -0.15])
    displacement = displacement.reshape(-1, 1)
    ee_pose = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    cur_pos, cur_quat = ee_pose
    tar_pos = cur_pos + displacement

    world.algorithms['moveit_kin_planner'].plan_end_effector_pose(tar_pos, cur_quat)
    # we see kinematics dependency also gets initiated 
    print("Kinematics module initiated T/F:", "Kinematics" in new_planner.algorithms.keys() and isinstance(new_planner.algorithms["Kinematics"], Kinematics))

    # now we initiate a new kinematic module and swap out the kinematics module in new_planner
    new_kinematic_solver = make_algorithm("kdl_kinematics", world)
    new_planner.algorithms["Kinematics"] = new_kinematic_solver
    # we see exactly the same functional behavior
    new_planner.plan_end_effector_pose(cur_pos, cur_quat)

if __name__ == "__main__":
    main()

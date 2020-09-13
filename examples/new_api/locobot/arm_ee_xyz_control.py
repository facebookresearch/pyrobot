#  Please use this file for debugging while re-factoring PyRobot

import hydra

from hydra.experimental import initialize, compose
from omegaconf import DictConfig

import numpy as np

from pyrobot import World

import copy


def main():
    
    world =  World(config_name='env/simple_env.yaml')

    bot = world.robots['locobot']

    displacement = np.array([0, 0, -0.15])

    eef_step=0.005

    displacement = displacement.reshape(-1, 1)

    path_len = np.linalg.norm(displacement)
    num_pts = int(np.ceil(path_len / float(eef_step)))
    if num_pts <= 1:
        num_pts = 2

    ee_pose = bot['arm'].get_ee_pose(bot['arm'].configs.ARM_BASE_FRAME)

    cur_pos, cur_ori, cur_quat = ee_pose

    tar_pos = cur_pos + displacement
    # world.algorithms['moveit_planner'].plan_end_effector_pose(tar_pos, cur_quat)

    qinit = bot['arm'].get_joint_angles().tolist()
    joint_positions = world.algorithms['kdl_kinematics'].inverse_kinematics(
        tar_pos,
        cur_quat,
        init_joint_pos=qinit        
    )
    if joint_positions is None:
        rospy.logerr(
            "No IK solution found; " "check if target_pose is valid"67890-=
        )
        return False

    world.algorithms['moveit_planner'].plan_joint_angles(joint_positions)

    # for joint_positions in way_joint_positions:
    #     success = bot['arm'].set_joint_positions(
    #         joint_positions, plan=False, wait=True
    #     )





if __name__ == "__main__":
    main()

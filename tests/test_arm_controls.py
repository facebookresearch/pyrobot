# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import time

import numpy as np
import pyrobot.utils.util as prutil
import tf.transformations as tft
from pyrobot import World


@pytest.fixture(scope="module")
def create_world():
    return World(config_name='env/locobot_arm_env.yaml')


@pytest.mark.parametrize(
    "position", [[0.40, 0.72, -0.47, -1.4, 0.92], [-0.67, 0, 0.23, 1, -0.70]]
)
def test_position_control(create_world, position):
    world = create_world
    bot = world.robots['locobot']
    bot["arm"].go_home()
    time.sleep(1)
    bot['arm'].set_joint_positions(position)
    time.sleep(1)
    joint_angles = bot['arm'].get_joint_angles()
    ag_error = np.fabs(joint_angles.flatten() - np.array(position).flatten())
    assert np.max(ag_error) < 0.06


positions = [np.array([0.279, 0.176, 0.217]), np.array([0.339, 0.0116, 0.255])]
orientations = [
    np.array(
        [
            [0.5380200, -0.6650449, 0.5179283],
            [0.4758410, 0.7467951, 0.4646209],
            [-0.6957800, -0.0035238, 0.7182463],
        ]
    ),
    np.array([0.245, 0.613, -0.202, 0.723]),
]


@pytest.mark.parametrize(
    "position, orientation",
    [(positions[0], orientations[0]), (positions[1], orientations[1])],
)
def test_ee_pose_control(create_world, position, orientation):
    world = create_world
    bot = world.robots['locobot']
    bot["arm"].go_home()
    time.sleep(1)
    world.algorithms['moveit_kin_planner'].plan_end_effector_pose(position, orientation)
    time.sleep(1)
    trans, quat = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    pos_error = np.linalg.norm(np.array(position).flatten() - np.array(trans).flatten())
    if orientation.size == 4:
        tgt_quat = orientation.flatten()
    elif orientation.size == 3:
        tgt_quat = prutil.euler_to_quat(orientation)
    elif orientation.size == 9:
        tgt_quat = prutil.rot_mat_to_quat(orientation)
    else:
        raise TypeError(
            "Orientation must be in one of the following forms:"
            "rotation matrix, euler angles, or quaternion"
        )
    quat_diff = tft.quaternion_multiply(tft.quaternion_inverse(tgt_quat), quat)
    rot_similarity = quat_diff[3]
    assert rot_similarity > 0.98 and pos_error < 0.02

def test_ee_xyz_control(create_world):
    world = create_world
    bot = world.robots['locobot']
    bot["arm"].go_home()
    time.sleep(1)
    displacement = np.array([0, 0, -0.15])
    
    ee_pose = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    cur_pos, cur_quat = ee_pose
    tar_pos = cur_pos + displacement

    world.algorithms['moveit_kin_planner'].plan_end_effector_pose(tar_pos, cur_quat)

    time.sleep(1)
    new_pos, new_quat = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    pos_error = np.linalg.norm(np.array(new_pos).flatten() - np.array(tar_pos).flatten())
    quat_diff = tft.quaternion_multiply(tft.quaternion_inverse(cur_quat), new_quat)
    rot_similarity = quat_diff[3]

    rot_thresh = 0.98
    pos_thresh = 0.02
    assert rot_similarity > rot_thresh and pos_error < pos_thresh


def test_gripper_control(create_world):
    world = create_world
    bot = world.robots['locobot']
    bot["arm"].go_home()
    time.sleep(1)
    bot['gripper'].open()
    time.sleep(1)
    g_state = bot['gripper'].get_gripper_state()
    assert g_state == 0

    bot['gripper'].close()
    time.sleep(1)
    g_state = bot['gripper'].get_gripper_state()
    assert g_state == 3

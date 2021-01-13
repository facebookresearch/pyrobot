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
    return World(config_name="env/locobot_arm_env.yaml")


@pytest.mark.parametrize(
    "position", [[-0.017792060227770554, -0.7601235411041661, 0.019782607023391807, -2.342050140544315, 0.029840531355804868, 1.5411935298621688, 0.7534486589746342], 
                 [0.7744824607372283, -0.43700320980841656, 0.45066898826548923, -2.3097477633873527, 0.19184858164522384, 1.8999855434099828, 1.8803117832119263]]
)
def test_position_control(create_world, position):
    time.sleep(5)
    world = create_world
    bot = world.robots["franka"]
    bot["arm"].go_home()
    joint_angles = bot["arm"].get_joint_angles()
    print(joint_angles)
    time.sleep(1)
    bot["arm"].set_joint_positions(position)
    time.sleep(5)
    joint_angles = bot["arm"].get_joint_angles()
    print(joint_angles)
    ag_error = np.fabs(joint_angles.flatten() - np.array(position).flatten())
    assert np.max(ag_error) < 0.05


positions = [np.array([-0.03299337923408681, 0.41477200652206825, 0.5232554329947007]), np.array([0.339, 0.0116, 0.255])]
orientations = [
    np.array([-0.9990944419734439, -0.01655181947800144, 0.03517021631879523, -0.017302866058243438]),
    np.array([0.245, 0.613, -0.202, 0.723]),
]

@pytest.mark.parametrize(
    "position, orientation",
    [(positions[0], orientations[0]), (positions[1], orientations[1])],
)
def test_ee_pose_control(create_world, position, orientation):
    time.sleep(5)
    world = create_world
    bot = world.robots["franka"]
    bot["arm"].go_home()
    time.sleep(1)
    world.algorithms["moveit_planner"].plan_end_effector_pose(position, orientation)
    time.sleep(5)
    trans, quat = world.algorithms["tf_transform"].get_transform(
        bot["arm"].configs.ARM_BASE_FRAME, bot["arm"].configs.EE_FRAME
    )
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


# def test_gripper_control(create_world):
#     world = create_world
#     bot = world.robots["franka"]
#     bot["arm"].go_home()
#     time.sleep(1)
#     bot["gripper"].open()
#     time.sleep(1)
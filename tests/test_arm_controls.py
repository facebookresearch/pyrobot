# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import time

import numpy as np
import pyrobot.utils.util as prutil
import tf.transformations as tft
from pyrobot import Robot


@pytest.fixture(scope="module")
def create_robot():
    return Robot("locobot", use_base=False)


@pytest.mark.parametrize(
    "position", [[0.40, 0.72, -0.47, -1.4, 0.92], [-0.67, 0, 0.23, 1, -0.70]]
)
@pytest.mark.parametrize("plan", ["True", "False"])
def test_position_control(create_robot, position, plan):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.set_joint_positions(position, plan=plan)
    time.sleep(1)
    joint_angles = bot.arm.get_joint_angles()
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
@pytest.mark.parametrize("numerical", ["True", "False"])
def test_ee_pose_control(create_robot, position, orientation, numerical):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.set_ee_pose(position=position, orientation=orientation, numerical=numerical)
    time.sleep(1)
    trans, rot, quat = bot.arm.pose_ee
    pos_error = np.linalg.norm(position.flatten() - trans.flatten())
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


position = [
    np.array([0.28, 0.17, 0.22]),
    np.array([0.28, -0.17, 0.22]),
    np.array([0.09, 0.25, 0.34]),
]
pitch = [0.5, 0.5, np.pi / 2]
roll = [None, 0.5, np.pi / 4]
tgt_pos = [
    np.array([[0.2792254], [0.1694128], [0.2174248]]),
    np.array([[0.2793616], [-0.1692725], [0.2174295]]),
    np.array([[0.0973994], [0.2505398], [0.3373943]]),
]
tgt_quat = [
    np.array([-0.0930975, 0.2367324, 0.3539513, 0.900005]),
    np.array([0.3134469, 0.1415047, -0.4016185, 0.8487815]),
    np.array([-0.1477051, 0.6930908, 0.1401051, 0.6915048]),
]


@pytest.mark.parametrize(
    "position, " "pitch, " "roll, " "tgt_pos, " "tgt_quat",
    [
        (position[i], pitch[i], roll[i], tgt_pos[i], tgt_quat[i])
        for i in range(len(position))
    ],
)
def test_ee_pitch_control(create_robot, position, pitch, roll, tgt_pos, tgt_quat):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.set_ee_pose_pitch_roll(
        position=position, pitch=pitch, roll=roll, numerical=False, plan=False
    )
    time.sleep(1)
    trans, rot, quat = bot.arm.pose_ee
    pos_error = np.linalg.norm(position.flatten() - tgt_pos.flatten())
    quat_diff = tft.quaternion_multiply(tft.quaternion_inverse(tgt_quat), quat)
    rot_similarity = quat_diff[3]

    assert rot_similarity > 0.98 and pos_error < 0.01


@pytest.mark.parametrize("plan", ["True", "False"])
def test_ee_xyz_control(create_robot, plan):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    tgt_trans, tgt_rot, tgt_quat = bot.arm.pose_ee
    displacement = np.array([0, 0, -0.15])
    tgt_pos = tgt_trans.flatten() + displacement.flatten()
    bot.arm.move_ee_xyz(displacement, plan=plan)
    time.sleep(1)
    trans, rot, quat = bot.arm.pose_ee
    pos_error = np.linalg.norm(tgt_pos.flatten() - trans.flatten())
    quat_diff = tft.quaternion_multiply(tft.quaternion_inverse(tgt_quat), quat)
    rot_similarity = quat_diff[3]

    rot_thresh = 0.98
    pos_thresh = 0.01
    assert rot_similarity > rot_thresh and pos_error < pos_thresh


def test_gripper_control(create_robot):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    bot.gripper.open()
    time.sleep(1)
    g_state = bot.gripper.get_gripper_state()
    assert g_state == 0

    bot.gripper.close()
    time.sleep(1)
    g_state = bot.gripper.get_gripper_state()
    assert g_state == 3

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import time

import numpy as np
from pyrobot import World
import pyrobot.utils.util as prutil


@pytest.fixture(scope="module")
def create_world():
    return World(config_name="env/franka_arm_env.yaml")


def test_get_ee_pose(create_world):
    world = create_world
    bot = world.robots["franka"]
    bot["arm"].go_home()
    time.sleep(1)
    trans, quat = world.algorithms["tf_transform"].get_transform(
        bot["arm"].configs.ARM_BASE_FRAME, bot["arm"].configs.EE_FRAME
    )

    trans_des = np.array([0.31063742475629785, 0.0056197233217173004, 0.5865124226229956]).reshape(3, 1)
    quat_des = np.array([-0.9996609438774468, -0.014138720762457395, 0.020205652048640874, 0.008356164583208677])
    trans_err = np.linalg.norm(
        np.array(trans_des).flatten() - np.array(trans).flatten()
    )
    quat_err = np.linalg.norm(np.array(quat).flatten() - np.array(quat_des).flatten())
    assert trans_err < 0.005
    assert quat_err < 0.005


def test_compute_fk_position(create_world):
    world = create_world
    bot = world.robots["franka"]
    pos, rot = world.algorithms["kdl_kinematics"].forward_kinematics(
        np.array([-0.01769495, -0.76000349,  0.01969661, -2.34208835,  0.02958345,
        1.54145608,  0.75368403]), bot["arm"].configs.EE_FRAME
    )

    pos_des = np.array([0.31063742475629785, 0.0056197233217173004, 0.5865124226229956])
    rot_des = prutil.quat_to_rot_mat(np.array([-0.9996609438774468, -0.014138720762457395, 0.020205652048640874, 0.008356164583208677]))
    pos_err = np.linalg.norm(pos.flatten() - pos_des.flatten())
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.005
    assert rot_err < 0.005


def test_compute_ik(create_world):
    world = create_world
    bot = world.robots["franka"]

    pos_des = np.array([0.31063742475629785, 0.0056197233217173004, 0.5865124226229956])
    rot_des = prutil.quat_to_rot_mat(np.array([-0.9996609438774468, -0.014138720762457395, 0.020205652048640874, 0.008356164583208677]))

    sol = world.algorithms["kdl_kinematics"].inverse_kinematics(pos_des, rot_des)
    pos, rot = world.algorithms["kdl_kinematics"].forward_kinematics(
        sol, bot["arm"].configs.EE_FRAME
    )
    pos_err = np.linalg.norm(pos.flatten() - pos_des.flatten())
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.01
    assert rot_err < 0.1

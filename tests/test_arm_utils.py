# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import time

import numpy as np
from pyrobot import World

@pytest.fixture(scope="module")
def create_world():
    return World(config_name='env/arm_env.yaml')

def test_get_ee_pose(create_world):
    world = create_world
    bot = world.robots['locobot']
    bot["arm"].go_home()
    time.sleep(1)
    trans, quat = world.algorithms['tf_transform'].get_transform(bot['arm'].configs.ARM_BASE_FRAME, bot['arm'].configs.EE_FRAME)
    
    trans_des = np.array([0.4118, 0.0001, 0.3987]).reshape(3, 1)
    quat_des = np.array([0.0, 0.009, -0.0, 1.0])
    trans_err = np.linalg.norm(np.array(trans_des).flatten() - np.array(trans).flatten())
    quat_err = np.linalg.norm(np.array(quat).flatten() - np.array(quat_des).flatten())
    assert trans_err < 0.005
    assert quat_err < 0.005

def test_compute_fk_position(create_world):
    world = create_world
    bot = world.robots['locobot']
    pos, rot = world.algorithms['kdl_kinematics'].forward_kinematics(
        np.array([0.1, 0.2, 0.3, 0.4, 0.5]), bot['arm'].configs.EE_FRAME
    )
    
    pos_des = np.array([[0.3994], [0.0304], [0.2438]])
    rot_des = np.array(
        [[0.6185, -0.4613, 0.6361], [0.0621, 0.8357, 0.5457], [-0.7833, -0.298, 0.5455]]
    )
    pos_err = np.linalg.norm(pos - pos_des)
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.005
    assert rot_err < 0.005


def test_compute_ik(create_world):
    world = create_world
    bot = world.robots['locobot']

    pos_des = np.array([0.3821, 0.0304, 0.22])
    rot_des = np.array(
        [[0.6185, -0.4613, 0.6361], [0.0621, 0.8357, 0.5457], [-0.7833, -0.298, 0.5455]]
    )
    sol = world.algorithms['kdl_kinematics'].inverse_kinematics(pos_des, rot_des)
    pos, rot = world.algorithms['kdl_kinematics'].forward_kinematics(sol, bot['arm'].configs.EE_FRAME)
    pos_err = np.linalg.norm(pos.flatten() - pos_des.flatten())
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.01
    assert rot_err < 0.1

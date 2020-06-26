# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import time

import numpy as np
from pyrobot import Robot


@pytest.fixture(scope="module")
def create_robot():
    return Robot("locobot", use_base=False)


@pytest.fixture
def botname(request):
    return request.config.getoption("botname")


def test_get_ee_pose(create_robot, botname):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(0.4)
    trans, rot, quat = bot.arm.pose_ee
    if "lite" in botname:
        trans_des = np.array([0.3946, 0.0001, 0.3749]).reshape(3, 1)

    else:
        trans_des = np.array([0.4118, 0.0001, 0.3987]).reshape(3, 1)
    rot_des = np.array(
        [[0.9998, 0.0, 0.018], [-0.0, 1.0, -0.0001], [-0.018, 0.0001, 0.9998]]
    )
    quat_des = np.array([0.0, 0.009, -0.0, 1.0])
    trans_err = np.linalg.norm(trans_des - trans)
    rot_err = np.linalg.norm(rot - rot_des)
    quat_err = np.linalg.norm(quat - quat_des)
    assert trans_err < 0.005
    assert rot_err < 0.005
    assert quat_err < 0.005


def test_get_jacobian(create_robot):
    bot = create_robot
    jac = bot.arm.get_jacobian([0, 0, 0, 0, 0])
    jac_des = np.array(
        [
            [-0.0001, 0.2, 0.0, 0.0, 0.0],
            [0.3132, 0.0, 0.0, 0.0, 0.0],
            [0.0, -0.3132, -0.2632, -0.063, 0.0],
            [0.0, 0.0, 0.0, 0.0, -1.0],
            [0.0, 1.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0],
        ]
    )
    jac_err = np.linalg.norm(jac - jac_des)
    assert jac_err < 0.001


def test_compute_fk_position(create_robot, botname):
    bot = create_robot
    pos, rot = bot.arm.compute_fk_position(
        np.array([0.1, 0.2, 0.3, 0.4, 0.5]), bot.arm.configs.ARM.EE_FRAME
    )
    if "lite" in botname:
        pos_des = np.array([[0.3821], [0.0304], [0.22]])

    else:
        pos_des = np.array([[0.3994], [0.0304], [0.2438]])
    rot_des = np.array(
        [[0.6185, -0.4613, 0.6361], [0.0621, 0.8357, 0.5457], [-0.7833, -0.298, 0.5455]]
    )
    pos_err = np.linalg.norm(pos - pos_des)
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.001
    assert rot_err < 0.001


def test_compute_fk_velocity(create_robot):
    bot = create_robot
    vel = bot.arm.compute_fk_velocity(
        np.zeros(bot.arm.arm_dof),
        np.array([0.1, 0.2, 0.3, 0.4, 0.5]),
        bot.arm.configs.ARM.EE_FRAME,
    )
    vel_des = np.array([0.04, 0.0313, -0.1668, -0.5, 0.9, 0.1])
    vel_err = np.linalg.norm(vel - vel_des)
    assert vel_err < 0.001


@pytest.mark.parametrize("numerical", ["True", "False"])
def test_compute_ik(create_robot, numerical):
    bot = create_robot

    pos_des = np.array([0.3821, 0.0304, 0.22])
    rot_des = np.array(
        [[0.6185, -0.4613, 0.6361], [0.0621, 0.8357, 0.5457], [-0.7833, -0.298, 0.5455]]
    )
    sol = bot.arm.compute_ik(pos_des, rot_des, numerical=numerical)
    pos, rot = bot.arm.compute_fk_position(sol, bot.arm.configs.ARM.EE_FRAME)
    pos_err = np.linalg.norm(pos.flatten() - pos_des.flatten())
    rot_err = np.linalg.norm(rot - rot_des)
    assert pos_err < 0.01
    assert rot_err < 0.1

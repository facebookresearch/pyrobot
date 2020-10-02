# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import pytest
import time

import numpy as np
from pyrobot import World
from pyrobot.algorithms.base_controller_impl.base_control_utils import (
    get_trajectory_circle,
    get_trajectory_negcircle,
    _get_absolute_pose,
)
from pyrobot.algorithms.base_controller_impl.ilqr_utils import wrap_theta


# Launch for running the following tests:
# roslaunch locobot_control main.launch use_arm:=false use_base:=true base:=create use_rviz:=false use_sim:=true


@pytest.fixture(scope="module")
def create_world():
    return World(config_name='env/base_env.yaml')


@pytest.mark.parametrize("base_controller", ["ilqr_control"])
@pytest.mark.parametrize("close_loop", [True, False])
def test_trajectory_tracking_circle(
    create_world, base_controller, close_loop
):
    world = create_world
    bot = world.robots['locobot']

    dt = 1.0 / world.algorithms[base_controller].configs.BASE_CONTROL_RATE
    v = world.algorithms[base_controller].configs.MAX_ABS_FWD_SPEED / 2.0
    w = world.algorithms[base_controller].configs.MAX_ABS_TURN_SPEED / 2.0
    r = v / w
    start_state = np.array(world.algorithms["odom_localizer"].get_odom_state())
    states, _ = get_trajectory_circle(start_state, dt, r, v, np.pi)
    world.algorithms[base_controller].track_trajectory(states, close_loop=close_loop)
    end_state = np.array(world.algorithms["odom_localizer"].get_odom_state())
    dist = np.linalg.norm(states[-1, :2] - end_state[:2])
    assert dist < 0.1


def _test_relative_position_control(
    posn,
    create_world,
    base_controller,
    close_loop,
    smooth,
    trans_thresh,
    angular_thresh,
):
    world = create_world
    bot = world.robots['locobot']

    start_state = np.array(world.algorithms["odom_localizer"].get_odom_state())
    desired_target = _get_absolute_pose(posn, start_state)
    world.algorithms[base_controller].go_to_relative(posn, close_loop=close_loop, smooth=smooth)
    end_state = np.array(world.algorithms["odom_localizer"].get_odom_state())

    dist = np.linalg.norm(end_state[:2] - desired_target[:2])
    angle = end_state[2] - desired_target[2]
    angle = np.abs(wrap_theta(angle))
    assert dist < trans_thresh
    assert angle * 180.0 / np.pi < angular_thresh


posns = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, np.pi],
        [1.0, 1.0, np.pi / 2.0],
        # [1.0, 1.0, 0.0],
        # [-1.0, 0.0, 0.0],
    ],
    dtype=np.float32,
)


posns_movebase = np.array(
    [[2.0, -2.0, np.pi/2]],
    dtype=np.float32,
)


@pytest.mark.parametrize("base_controller", ["proportional_control"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control1(
    create_world, posn, base_controller, close_loop, smooth
):
    _test_relative_position_control(
        posn, create_world, base_controller, close_loop, smooth, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["movebase_control"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns_movebase)
def test_relative_position_control1_movebase(
    create_world, posn, base_controller, close_loop, smooth
):
    _test_relative_position_control(
        posn, create_world, base_controller, close_loop, smooth, 0.25, 20
    )


@pytest.mark.parametrize("base_controller", ["ilqr_control"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_close_sharp(
    create_world, posn, base_controller
):
    _test_relative_position_control(
        posn, create_world, base_controller, True, False, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["ilqr_control"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_open_sharp(
    create_world, posn, base_controller
):
    _test_relative_position_control(
        posn, create_world, base_controller, False, False, 0.10, 20
    )

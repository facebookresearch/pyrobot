# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import pytest
import time

import numpy as np
from pyrobot import Robot
from pyrobot.locobot.base_control_utils import (
    get_trajectory_circle,
    get_trajectory_negcircle,
    _get_absolute_pose,
)
from pyrobot.locobot.bicycle_model import wrap_theta


# Launch for running the following tests:
# roslaunch locobot_control main.launch use_arm:=false use_base:=true base:=create use_rviz:=false use_sim:=true


@pytest.fixture
def botname(request):
    return request.config.getoption("botname")


def _mkdir_if_missing(dir_name):
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)


def _get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("close_loop", [True, False])
def test_trajectory_tracking_circle(botname, base_controller, base_planner, close_loop):
    bot = Robot(
        botname,
        base_config={"base_controller": base_controller, "base_planner": base_planner},
        use_arm=False,
        use_camera=False,
    )
    dt = 1.0 / bot.configs.BASE.BASE_CONTROL_RATE
    v = bot.configs.BASE.MAX_ABS_FWD_SPEED / 2.0
    w = bot.configs.BASE.MAX_ABS_TURN_SPEED / 2.0
    r = v / w
    start_state = np.array(bot.base.get_state("odom"))
    states, _ = get_trajectory_circle(start_state, dt, r, v, np.pi)
    bot.base.track_trajectory(states, close_loop=close_loop)
    end_state = np.array(bot.base.get_state("odom"))
    dist = np.linalg.norm(states[-1, :2] - end_state[:2])
    assert dist < 0.1


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("close_loop", [True, False])
def test_trajectory_tracking_two_circles(
    botname, base_controller, base_planner, close_loop
):
    bot = Robot(
        botname,
        base_config={"base_controller": base_controller, "base_planner": base_planner},
        use_arm=False,
        use_camera=False,
    )
    dt = 1.0 / bot.configs.BASE.BASE_CONTROL_RATE
    v = bot.configs.BASE.MAX_ABS_FWD_SPEED / 2.0
    w = bot.configs.BASE.MAX_ABS_TURN_SPEED / 2.0
    r = v / w
    start_state = np.array(bot.base.get_state("odom"))
    states1, _ = get_trajectory_circle(start_state, dt, r, v, np.pi)
    states2, _ = get_trajectory_negcircle(states1[-1, :] * 1, dt, r, v, np.pi)
    states = np.concatenate([states1, states2], 0)

    bot.base.track_trajectory(states, close_loop=close_loop)
    end_state = np.array(bot.base.get_state("odom"))
    dist = np.linalg.norm(states[-1, :2] - end_state[:2])
    assert dist < 0.1


def _test_relative_position_control(
    posn,
    botname,
    base_controller,
    base_planner,
    close_loop,
    smooth,
    trans_thresh,
    angular_thresh,
):
    bot = Robot(
        botname,
        base_config={"base_controller": base_controller, "base_planner": base_planner},
        use_arm=False,
        use_camera=False,
    )
    start_state = np.array(bot.base.get_state("odom"))
    desired_target = _get_absolute_pose(posn, start_state)
    bot.base.go_to_relative(posn, use_map=False, close_loop=close_loop, smooth=smooth)
    end_state = np.array(bot.base.get_state("odom"))

    if False:
        # Plot results to file
        file_name = "{:s}_position_{:s}_controller_{:s}_planner_{:s}_close{:d}_smooth{:d}-{:s}.pdf"
        posn_str = ["{:0.1f}".format(x) for x in posn]
        file_name = file_name.format(
            botname,
            ",".join(posn_str),
            base_controller,
            base_planner,
            int(close_loop),
            int(smooth),
            _get_time_str(),
        )
        tmp_dir = os.path.join(os.path.dirname(__file__), "tmp")
        _mkdir_if_missing(tmp_dir)
        file_name = os.path.join(tmp_dir, file_name)
        bot.base.controller.plot_plan_execution(file_name)
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
        [1.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=np.float32,
)


@pytest.mark.parametrize("base_controller", ["proportional"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control1(
    botname, posn, base_controller, base_planner, close_loop, smooth
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, close_loop, smooth, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["movebase"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control1_movebase(
    botname, posn, base_controller, base_planner, close_loop, smooth
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, close_loop, smooth, 0.25, 20
    )


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_close_smooth(
    botname, posn, base_controller, base_planner
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, True, True, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_open_smooth(
    botname, posn, base_controller, base_planner
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, False, True, 0.10, 20
    )


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_close_sharp(
    botname, posn, base_controller, base_planner
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, True, False, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["ilqr"])
@pytest.mark.parametrize("base_planner", ["none"])
@pytest.mark.parametrize("posn", posns)
def test_relative_position_control2_open_sharp(
    botname, posn, base_controller, base_planner
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, False, False, 0.10, 20
    )


@pytest.mark.parametrize("base_controller", ["proportional", "ilqr"])
@pytest.mark.parametrize("base_planner", ["movebase"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns[:1, :])
def test_relative_position_control3(
    botname, posn, base_controller, base_planner, close_loop, smooth
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, close_loop, smooth, 0.05, 10
    )


@pytest.mark.parametrize("base_controller", ["movebase"])
@pytest.mark.parametrize("base_planner", ["movebase"])
@pytest.mark.parametrize("close_loop", [True])
@pytest.mark.parametrize("smooth", [False])
@pytest.mark.parametrize("posn", posns[:1, :])
def test_relative_position_control3_movebase(
    botname, posn, base_controller, base_planner, close_loop, smooth
):
    _test_relative_position_control(
        posn, botname, base_controller, base_planner, close_loop, smooth, 0.25, 20
    )

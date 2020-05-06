# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import numpy as np
from pyrobot import Robot




@pytest.fixture(scope="module")
def create_robot():
    return Robot("locobot", use_arm=False, use_camera=False)

@pytest.mark.parametrize("base_controller", ["ilqr", "proportional", "movebase"])
@pytest.mark.parametrize("base_planner", ["movebase", "none"])
def test_vel_cmd_v(create_robot, base_controller, base_planner):
    bot = create_robot
    bot.base.load_controller(base_controller)
    bot.base.load_planner(base_planner)

    state_before = np.array(bot.base.get_state("odom"))
    bot.base.set_vel(fwd_speed=0.2, turn_speed=0.0, exe_time=2)
    bot.base.stop()
    state_after = np.array(bot.base.get_state("odom"))
    dist = np.linalg.norm(state_after[:2] - state_before[:2])
    assert 0.3 < dist < 0.5


@pytest.mark.parametrize("base_controller", ["ilqr", "proportional", "movebase"])
@pytest.mark.parametrize("base_planner", ["movebase", "none"])
def test_vel_cmd_w(create_robot, base_controller, base_planner):
    bot = create_robot
    bot.base.load_controller(base_controller)
    bot.base.load_planner(base_planner)

    state_before = np.array(bot.base.get_state("odom"))
    bot.base.set_vel(fwd_speed=0.0, turn_speed=0.5, exe_time=2)
    bot.base.stop()
    state_after = np.array(bot.base.get_state("odom"))
    dist = np.linalg.norm(state_after[:2] - state_before[:2])
    dt = state_after[2] - state_before[2]
    dt = np.mod(dt + np.pi, 2 * np.pi) - np.pi
    assert dist < 0.1
    assert 0.75 < dt < 1.25

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import numpy as np
from pyrobot import World


@pytest.fixture(scope="module")
def create_world():
    return World(config_name="env/locobot_base_env.yaml")


def test_vel_cmd_v(create_world):
    world = create_world
    bot = world.robots["locobot"]

    state_before = np.array(world.algorithms["odom_localizer"].get_odom_state())
    bot["base"].set_vel(fwd_speed=0.2, turn_speed=0.0, exe_time=2)
    bot["base"].stop()
    state_after = np.array(world.algorithms["odom_localizer"].get_odom_state())
    dist = np.linalg.norm(state_after[:2] - state_before[:2])
    assert 0.3 < dist < 0.5


def test_vel_cmd_w(create_world):
    world = create_world
    bot = world.robots["locobot"]

    state_before = np.array(world.algorithms["odom_localizer"].get_odom_state())
    bot["base"].set_vel(fwd_speed=0.0, turn_speed=0.5, exe_time=2)
    bot["base"].stop()
    state_after = np.array(world.algorithms["odom_localizer"].get_odom_state())
    dist = np.linalg.norm(state_after[:2] - state_before[:2])
    dt = state_after[2] - state_before[2]
    dt = np.mod(dt + np.pi, 2 * np.pi) - np.pi
    assert dist < 0.1
    assert 0.75 < dt < 1.25

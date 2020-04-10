# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import numpy as np
from pyrobot import Robot


@pytest.fixture(scope="module")
def create_robot():
    return Robot(
        "locobot", use_camera=True, use_base=False, use_arm=False, use_gripper=False
    )


@pytest.mark.parametrize(
    "target_position", [[0, 0.7], [0.4, 0.4], [0.4, -0.4], [-0.4, 0.4], [-0.4, -0.4]]
)
def test_position_control(create_robot, target_position):
    bot = create_robot
    bot.camera.reset()
    bot.camera.set_pan_tilt(target_position[0], target_position[1], wait=True)
    achieved_position = bot.camera.get_state()
    ag_error = np.fabs(np.array(achieved_position) - np.array(target_position))
    assert np.max(ag_error) < 0.10


def test_get_images(create_robot):
    bot = create_robot
    rgb_img = bot.camera.get_rgb()
    depth_img = bot.camera.get_depth()
    assert depth_img is not None
    assert rgb_img is not None
    rgb_img, depth_img = bot.camera.get_rgb_depth()
    assert rgb_img is not None
    assert depth_img is not None

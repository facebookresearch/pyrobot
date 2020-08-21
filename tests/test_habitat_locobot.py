# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest
import os
import sys
import numpy as np
from pyrobot import Robot

from pyrobot.locobot.base_control_utils import _get_absolute_pose


@pytest.fixture(scope="module")
def create_robot():

    # Please change this to match your habitat_sim repo's path
    path_to_habitat_scene = os.path.dirname(os.path.realpath(__file__))
    relative_path = "../examples/habitat/scenes/skokloster-castle.glb"

    common_config = dict(scene_path=os.path.join(path_to_habitat_scene, relative_path))
    bot = Robot("habitat", common_config=common_config)

    return bot


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


@pytest.mark.parametrize("posn", posns)
def _test_relative_position_control(
    create_robot, posn,
):
    bot = create_robot
    start_state = np.array(bot.base.get_state("odom"))
    desired_target = _get_absolute_pose(posn, start_state)
    bot.base.go_to_relative(posn)
    end_state = np.array(bot.base.get_state("odom"))

    dist = np.linalg.norm(end_state[:2] - desired_target[:2])
    angle = end_state[2] - desired_target[2]
    angle = np.abs(wrap_theta(angle))
    assert dist < trans_thresh
    assert angle * 180.0 / np.pi < angular_thresh

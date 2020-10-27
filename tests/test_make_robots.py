# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import rospy
from pyrobot import Robot


@pytest.fixture
def botname(request):
    return request.config.getoption("botname")


@pytest.mark.parametrize("base_controller", ["ilqr", "proportional", "movebase"])
@pytest.mark.parametrize("base_planner", ["movebase", "none"])
def test_make_robot_base(botname, base_controller, base_planner):
    bot = Robot(
        botname,
        base_config={"base_controller": base_controller, "base_planner": base_planner},
        use_arm=False,
        use_camera=False,
    )
    return bot

@pytest.mark.parametrize("use_moveit", ["True", "False"])
def test_make_robot_arm(botname, use_moveit):
    bot = Robot(botname, arm_config={"use_moveit": use_moveit}, use_base=False)
    return bot

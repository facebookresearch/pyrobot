# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

import rospy
from pyrobot import World

@pytest.mark.parametrize("world_config", ["env/simple_env.yaml"])
def test_make_world(world_config):
    world = World(config_name=world_config)
    return world

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot
from pyrobot.locobot.base_control_utils import (
    get_trajectory_circle,
    get_trajectory_negcircle,
)

FLAGS = flags.FLAGS
flags.DEFINE_string("type", "circle", "Trajectory tracking to run.")
flags.DEFINE_string("botname", "locobot", "Robot name, locobot, locobot_lite, ...")
flags.DEFINE_bool("close_loop", True, "")
flags.DEFINE_string("base_controller", "ilqr", "One of ilqr, proportional, movebase.")


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def get_trajectory(bot, trajectory_type):
    dt = 1.0 / bot.configs.BASE.BASE_CONTROL_RATE
    v = bot.configs.BASE.MAX_ABS_FWD_SPEED / 2.0
    w = bot.configs.BASE.MAX_ABS_TURN_SPEED / 2.0

    if trajectory_type == "circle":
        r = v / w
        start_state = np.array(bot.base.get_state("odom"))
        states, _ = get_trajectory_circle(start_state, dt, r, v, 2 * np.pi)

    elif trajectory_type == "twocircles":
        r = v / w
        start_state = np.array(bot.base.get_state("odom"))
        states1, _ = get_trajectory_circle(start_state, dt, r, v, np.pi)
        states2, _ = get_trajectory_negcircle(states1[-1, :].copy(), dt, r, v, np.pi)
        states = np.concatenate([states1, states2], 0)
    else:
        raise ValueError("Trajectory type [%s] not implemented" % trajectory_type)

    return states


def main(_):
    bot = Robot(
        FLAGS.botname,
        base_config={"base_controller": FLAGS.base_controller, "base_planner": "none"},
    )
    states = get_trajectory(bot, FLAGS.type)
    bot.base.track_trajectory(states, close_loop=FLAGS.close_loop)


if __name__ == "__main__":
    app.run(main)

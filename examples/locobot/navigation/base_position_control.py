# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

FLAGS = flags.FLAGS
flags.DEFINE_list(
    "relative_position", "5,1,0", "Relative position to go to in the demo."
)
flags.DEFINE_bool("close_loop", True, "")
flags.DEFINE_string("base_controller", "gpmp", "One of ilqr, proportional, movebase.")
flags.DEFINE_string("base_planner", "movebase", "movebase or none")
flags.DEFINE_string("botname", "locobot", "Robot name, locobot, locobot_lite, ...")
flags.DEFINE_bool("smooth", True, "")


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def main(_):
    bot = Robot(
        FLAGS.botname,
        base_config={
            "base_controller": FLAGS.base_controller,
            "base_planner": FLAGS.base_planner,
        },
    )
    posn = np.asarray(FLAGS.relative_position, dtype=np.float64, order="C")
    bot.base.go_to_relative(
        posn, use_map=False, close_loop=FLAGS.close_loop, smooth=FLAGS.smooth
    )

    bot.base.stop()


if __name__ == "__main__":
    app.run(main)

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

import numpy as np
from absl import flags, app
from pyrobot import Robot

FLAGS = flags.FLAGS
flags.DEFINE_list(
    "relative_position", "1,0.5,1.0", "Relative position to go to in the demo."
)
flags.DEFINE_bool("close_loop", True, "")
flags.DEFINE_bool("smooth", True, "")
flags.DEFINE_string("botname", "locobot", "Robot name, locobot, locobot_lite, ...")
flags.DEFINE_string(
    "base_controller", "proportional", "One of ilqr, proportional, movebase."
)
flags.DEFINE_string("base_planner", "movebase", "movebase or none")


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


# Note - need to run the python script base_navigation/nodes/mapToGrid.py before running this script


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
        posn, use_map=True, close_loop=FLAGS.close_loop, smooth=FLAGS.smooth
    )

    if hasattr(bot.base.controller, "plot_plan_execution"):
        posn = FLAGS.relative_position
        posn_str = ",".join(posn)
        file_name = os.path.join(
            "position_{:s}_close{:d}_smooth{:d}-{:s}.pdf".format(
                posn_str, int(FLAGS.close_loop), int(FLAGS.smooth), get_time_str()
            )
        )
        tmp_dir = os.path.join(os.path.dirname(__file__), "tmp")
        if not os.path.exists(tmp_dir):
            os.makedirs(tmp_dir)
        file_name = os.path.join(tmp_dir, file_name)
        print("Trajectory saved to:", file_name)
        bot.base.plot_plan_execution(file_name)
    bot.base.stop()


if __name__ == "__main__":
    app.run(main)

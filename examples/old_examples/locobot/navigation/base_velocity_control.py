# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from absl import flags, app
from pyrobot import Robot

FLAGS = flags.FLAGS
flags.DEFINE_string("base_controller", "ilqr", "One of ilqr, proportional, movebase.")
flags.DEFINE_string("base_planner", "movebase", "movebase or none")
flags.DEFINE_string("botname", "locobot", "Robot name, locobot, locobot_lite, ...")
flags.DEFINE_float("linear_speed", 0.2, "Linear speed.")
flags.DEFINE_float("angular_speed", 0.5, "Angular velocity.")
flags.DEFINE_float("duration", 1.0, "Time to run velocity commands for.")


def main(_):
    bot = Robot(
        FLAGS.botname,
        use_arm=False,
        base_config={
            "base_planner": FLAGS.base_planner,
            "base_controller": FLAGS.base_controller,
        },
    )
    bot.base.set_vel(
        fwd_speed=FLAGS.linear_speed,
        turn_speed=FLAGS.angular_speed,
        exe_time=FLAGS.duration,
    )
    bot.base.stop()


if __name__ == "__main__":
    app.run(main)

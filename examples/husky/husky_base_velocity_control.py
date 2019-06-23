# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

FLAGS = flags.FLAGS
flags.DEFINE_string('base_controller', 'proportional', 'One of ilqr, proportional, movebase.')
flags.DEFINE_string('base_planner', 'none', 'movebase or none')
flags.DEFINE_string('botname', 'husky', 'Robot name, locobot, locobot_lite, ...')
flags.DEFINE_float('linear_speed', 1.0, 'Linear speed.')
flags.DEFINE_float('angular_speed', 2.0, 'Angular velocity.')
flags.DEFINE_float('duration', 50.0, 'Time to run velocity commands for.')

def main(_):
    bot = Robot(FLAGS.botname,
    		use_base=True,
    		use_arm=False,
    		use_camera=False,
    		use_gripper=False,
            base_config={'base_controller': FLAGS.base_controller,
                         'base_planner': FLAGS.base_planner})
    bot.base.set_vel(fwd_speed=FLAGS.linear_speed,
                     turn_speed=FLAGS.angular_speed,
                     exe_time=FLAGS.duration)
    bot.base.stop()


if __name__ == '__main__':
    app.run(main)

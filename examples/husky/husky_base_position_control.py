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
flags.DEFINE_list('relative_position', '10,10,1', 'Relative position to go to in the demo.')
flags.DEFINE_bool('close_loop', True, '')
flags.DEFINE_string('base_controller', 'proportional', 'One of ilqr, proportional, movebase.')
flags.DEFINE_string('base_planner', 'none', 'movebase or none')
flags.DEFINE_string('botname', 'husky', 'Robot name, locobot, locobot_lite, husky ...')
flags.DEFINE_bool('smooth', True, '')


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def main(_):
    bot = Robot(FLAGS.botname, use_arm=False, use_camera=False,use_gripper=False,
                base_config={'base_controller': FLAGS.base_controller,
                             'base_planner': FLAGS.base_planner})
    posn = np.asarray(FLAGS.relative_position, dtype=np.float64, order='C')
    bot.base.controller.goto(posn)


    # if hasattr(bot.base.controller, 'plot_plan_execution'):
    #     posn = FLAGS.relative_position
    #     posn_str = ','.join(posn)
    #     file_name = 'position_{:s}_controller_{:s}_close{:d}_smooth{:d}-{:s}.pdf'
    #     file_name = file_name.format(posn_str, FLAGS.base_controller,
    #                                  int(FLAGS.close_loop), int(FLAGS.smooth), get_time_str())
    #     tmp_dir = os.path.join(os.path.dirname(__file__), 'tmp')
    #     if not os.path.exists(tmp_dir):
    #         os.makedirs(tmp_dir)
    #     file_name = os.path.join(tmp_dir, file_name)
    #     rospy.loginfo('Trajectory saved to %s.', file_name)
    #     bot.base.controller.plot_plan_execution(file_name)

    bot.base.stop()


if __name__ == "__main__":
    app.run(main)

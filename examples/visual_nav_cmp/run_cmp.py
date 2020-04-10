# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()

import numpy as np
import rospy
from absl import flags, app
from pyrobot import Robot

from cmp_runner import CMPRunner

"""
To run experiments, use appropriate flags with the following command:
    python run_cmp.py
See instructions in README.md
"""

FLAGS = flags.FLAGS
flags.DEFINE_float("goal_x", 0, "Desired goal x (in cm).")
flags.DEFINE_float("goal_y", 0, "Desired goal y (in cm).")
flags.DEFINE_float("goal_t", 0, "Desired goal theta (in radians).")
flags.DEFINE_string("model_path", "./model.ckpt-120003", "Path for model")
flags.DEFINE_string("botname", "locobot", "Robot name: locobot, locobot_lite.")
flags.DEFINE_string(
    "logdir", None, "Where to log results, if None results are not logged."
)
flags.DEFINE_bool("compensate", True, "Compensate controller inaccuracies")

action_strings = ["stay", "right", "left", "forward"]
action_relative_offset = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, -np.pi / 2.0],
    [0.0, 0.0, np.pi / 2.0],
    [0.4, 0.0, 0.0],
]


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def compensate_controller_error(init_state, current_state, next_state, action):
    # Imperfections in controllers can lead to each step being more inaccurate
    # than the underlying odometry sensors. We compensate for these
    # inaccuracies by calculating the location on the grid that we want the
    # robot to be at, at each time step, and adjusting the next macro-action to
    # arrive at the exact grid location as determined by the odometry sensors.

    # Compute next_state in init_state coordinate frame.

    goal_pos1 = init_state.copy()
    x, y, theta = init_state
    dx, dy, dtheta = next_state
    goal_pos1[0] = x + dx * np.cos(theta) - dy * np.sin(theta)
    goal_pos1[1] = y + dx * np.sin(theta) + dy * np.cos(theta)
    goal_pos1[2] = theta + dtheta

    # Compute next_state in current state coordinate frame.
    goal_pos2 = goal_pos1.copy()
    x, y, theta = current_state
    X, Y, Theta = goal_pos1
    goal_pos2[0] = (X - x) * np.cos(theta) + (Y - y) * np.sin(theta)
    goal_pos2[1] = -(X - x) * np.sin(theta) + (Y - y) * np.cos(theta)
    goal_pos2[2] = Theta - theta
    goal_pos2[2] = np.mod(goal_pos2[2] + np.pi, 2 * np.pi) - np.pi

    if action == 1 or action == 2 or action == 0:
        goal_pos2[:2] = 0.0

    return goal_pos2


def main(_):
    # CMP policies were trained using a coordinate frame where robot faces the
    # +Y axis. PyRobot and this example assumes that the robot is facing +X
    # axis. Thus, we appropriately transform the goal before feeding into CMP.
    # CMP also assumes a grid world, where the size of the grid is 40cm. Thus,
    # we snap the goal to the nearest grid location.
    rospy.loginfo("Loading CMP policy and model.")
    cmp_runner = CMPRunner(FLAGS.model_path)

    goal = [
        -np.round(FLAGS.goal_y * 100.0 / 40.0) * 40.0,
        np.round(FLAGS.goal_x * 100.0 / 40.0) * 40.0,
        np.round(FLAGS.goal_t / np.pi * 2.0) * np.pi / 2.0 + np.pi / 2.0,
    ]
    rospy.loginfo("Goal in CMP coordinate frame: %s", str(goal))
    cmp_runner.set_new_goal(goal)

    rospy.loginfo("Making robot")
    bot = Robot(
        FLAGS.botname, base_config={"base_controller": "ilqr", "base_planner": "none"}
    )
    logdir = None
    if FLAGS.logdir is not None:
        logdir = os.path.join(FLAGS.logdir, get_time_str())
        rospy.loginfo("Logging images and actions to %s.", logdir)
        if not os.path.exists(logdir):
            os.makedirs(logdir)

    if logdir is not None:
        # Write goal to file
        file_name = os.path.join(logdir, "goal.txt")
        np.savetxt(file_name, goal)

    init_state = np.array(bot.base.get_state("odom"))
    for i in range(100):
        rgb = bot.camera.get_rgb()
        img = cv2.resize(rgb, (0, 0), fx=0.5, fy=0.5)
        img = img[np.newaxis, np.newaxis, np.newaxis, :, :, ::-1]
        img = img.astype(np.float32)
        model_action, next_state = cmp_runner.compute_action(img)
        model_action = model_action[0]
        next_state = next_state[0]

        # Log things if necessary.
        if logdir is not None:
            fname = "{:03d}_{:s}.jpg".format(i, action_strings[model_action])
            file_name = os.path.join(logdir, fname)
            cv2.imwrite(file_name, rgb[:, :, ::-1])

        if FLAGS.compensate:
            next_state_ros = [
                next_state[1] / 100.0,
                -next_state[0] / 100.0,
                next_state[2] - np.pi / 2.0,
            ]
            next_state_ros = np.array(next_state_ros, dtype=np.float64)
            current_state = np.array(bot.base.get_state("odom"))
            posn = compensate_controller_error(
                init_state, current_state, next_state_ros, model_action
            )
        else:
            posn = action_relative_offset[model_action]

        posn_str = ["{:0.3f}".format(x) for x in posn]
        rospy.loginfo(
            "CMP action: %s, relative_posn: %s",
            action_strings[model_action],
            ", ".join(posn_str),
        )
        if model_action == 0:
            rospy.loginfo("Goal achieved")
            break
        bot.base.go_to_relative(posn, use_map=False, close_loop=True, smooth=False)


if __name__ == "__main__":
    app.run(main)

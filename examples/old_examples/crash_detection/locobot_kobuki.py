#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Locomotion without crashing example with the PyRobot API.

Associated launch command:
roslaunch locobot_control main.launch use_base:=true use_arm:=true base:=kobuki use_camera:=true

Follow the associated README for installation instructions.
"""

import sys

import crash_utils.model as model
from crash_utils.test import Tester

sys.modules["model"] = model

from pyrobot import Robot
import os
import errno
import time
import torch
import argparse

DEFAULT_PAN_TILT = [0, -0.0]
MODEL_URL = "https://www.dropbox.com/s/dr4npkwz9j6c9rk/checkpoint.pth.bst?dl=0"
SAVE_DIR = "./models"
STRAIGHT_VELOCITY = 0.2
TURNING_VELOCITY = 1.0
TIME_STEP = 0.5
STRAIGHT_THRESHOLD = 0.5


def download_if_not_present(model_path, url):
    """
    Function that downloads a file from a url to a given location.

    :param model_path: Path where the file should be downlaoded to
    :param url: URL from where the file will be downloaded from
    :type model_path: string
    :type url: string
    """
    if not os.path.isfile(model_path):
        if not os.path.exists(os.path.dirname(model_path)):
            try:
                os.makedirs(os.path.dirname(model_path))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        print("CRASH MODEL NOT FOUND! DOWNLOADING IT!")
        os.system("wget {} -O {}".format(url, model_path))


def go_straight(bot, vel=STRAIGHT_VELOCITY, t=TIME_STEP):
    """
    Make the robot go straight
    
    :param bot: A pyrobot.Robot object
    :param vel: Velocity with which to go straight
    :param t: Amount of time to go staight for
 
    :type bot: pyrobot.Robot
    :type vel: float
    :type t: float
    """
    print("Straight!!")
    bot.base.set_vel(vel, 0.0, t)


def turn_left(bot, vel=TURNING_VELOCITY, t=TIME_STEP):
    """
    Make the robot turn left
    
    :param bot: A pyrobot.Robot object
    :param vel: Velocity with which to turn left
    :param t: Amount of time to turn left for
 
    :type bot: pyrobot.Robot
    :type vel: float
    :type t: float
    """
    print("Left!!")
    bot.base.set_vel(0.0, vel, t)


def turn_right(bot, vel=TURNING_VELOCITY, t=TIME_STEP):
    """
    Make the robot turn right
    
    :param bot: A pyrobot.Robot object
    :param vel: Velocity with which to turn right
    :param t: Amount of time to turn right for
 
    :type bot: pyrobot.Robot
    :type vel: float
    :type t: float
    """
    print("Right!!")
    bot.base.set_vel(0.0, -vel, t)


def main(args):
    """
    This is the main function for running the locomotion without crashing demo.
    """

    if args.display_images == True:
        from pyrobot.utils.util import try_cv2_import

        cv2 = try_cv2_import()

    bot = Robot("locobot", base_config={"base_planner": "none"})
    bot.camera.reset()
    print("Setting pan: {}, tilt: {}".format(*DEFAULT_PAN_TILT))
    bot.camera.set_pan_tilt(*DEFAULT_PAN_TILT, wait=True)

    model_path = os.path.join(args.save_dir, "crash_model.pth")
    download_if_not_present(model_path, args.model_url)
    crash_model = torch.load(model_path)
    evaluator = Tester(crash_model)

    control_start = time.time()
    hist = "straight"
    for _ in range(args.n_loops):
        start_time = time.time()
        rgb, _ = bot.camera.get_rgb_depth()
        evals = evaluator.test(rgb)
        print(evals)
        if evals[3] > STRAIGHT_THRESHOLD:
            hist = "straight"
            go_straight(bot)
        else:
            if hist == "straight":
                if evals[1] > evals[2]:
                    hist = "left"
                    turn_left(bot)
                else:
                    hist = "right"
                    turn_right(bot)
            elif hist == "left":
                turn_left(bot)
            elif hist == "right":
                turn_right(bot)

        stop_time = time.time()
        time_elapsed = stop_time - start_time
        if args.display_images == True:
            cv2.imshow("image", evaluator.image)
            cv2.waitKey(10)
        if time.time() - control_start >= args.n_secs:
            print("Time limit exceeded")
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process args for moving without crashing"
    )
    parser.add_argument(
        "-n",
        "--n_secs",
        help="Number of seconds to run the avoiding crashing controller",
        type=int,
        default=60,
    )
    parser.add_argument(
        "-l",
        "--n_loops",
        help="Number of loops to run the avoiding crashing controller",
        type=int,
        default=1000,
    )
    parser.add_argument(
        "-u", "--model_url", help="URL to download model from", default=MODEL_URL
    )
    parser.add_argument(
        "-s", "--save_dir", help="Directory to save model", default=SAVE_DIR
    )
    parser.add_argument(
        "-d",
        "--visualize",
        help="True to visualize images at each control loop, False otherwise",
        dest="display_images",
        action="store_true",
    )
    parser.set_defaults(display_images=False)

    args = parser.parse_args()

    main(args)

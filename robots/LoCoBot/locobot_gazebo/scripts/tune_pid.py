# *******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
# *******************************************************************************
import argparse
import copy
import time

import matplotlib.pyplot as plt
import numpy as np
from pyrobot import Robot


def main(args):
    joint_name = "joint_%d" % args.joint_id
    bot = Robot("locobot")
    bot.arm.go_home()
    time.sleep(1)
    angle = 0.5
    joint_angles = np.zeros(5)
    joint_angles[args.joint_id - 1] = angle
    time_len = 2
    sampling_freq = 100.0
    sleep_time = 1 / sampling_freq
    angles = []
    bot.arm.set_joint_positions(joint_angles, plan=False, wait=False)
    for i in range(int(time_len / sleep_time)):
        actual_angle = bot.arm.get_joint_angle(joint_name)
        angles.append(copy.deepcopy(actual_angle))
        time.sleep(sleep_time)
    times = np.arange(len(angles)) * sleep_time
    print("error:", np.mean(angles[-10]) - angle)
    plt.plot(times, angles)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Argument Parser")
    parser.add_argument("--joint_id", type=int, default=1, help="joint id")
    args = parser.parse_args()
    main(args)

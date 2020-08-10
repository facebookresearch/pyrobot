# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with velocity control
"""

import math
import signal
import sys
import time

from pyrobot import Robot


def signal_handler(sig, frame):
    print("Exit")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def sin_wave(t, f, A):
    return A * math.cos(2 * math.pi * f * t)


def main():
    bot = Robot(
        "sawyer", use_arm=True, use_base=False, use_camera=False, use_gripper=True
    )
    bot.arm.move_to_neutral()

    A = 0.2
    f = 0.4
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        vels = [sin_wave(elapsed_time, f, A)] * bot.arm.arm_dof
        bot.arm.set_joint_velocities(vels)
        time.sleep(0.01)


if __name__ == "__main__":
    main()

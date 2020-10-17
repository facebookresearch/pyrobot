#!/usr/bin/env python
# coding: utf-8

import time
from pyrobot import Robot


def main():
    """
    Example controlling the base of the Pepper robot using positions and
    velocities. Please ensure that Pepper is not placed in a cluttered
    environment before running the example
    """
    pepper = Robot(
        'pepper',
        use_arm=False,
        use_base=True,
        use_camera=False,
        use_gripper=False)

    # Set the velocity of Pepper's base to 0.03 m/s along the X axis for 1
    # second
    pepper.base.set_vel(0.1, 0.0, 0.0, exe_time=1)
    print("Current base state: " + str(pepper.base.get_state()))
    time.sleep(1.0)

    # Move Pepper to the origin of the world frame
    pepper.base.go_to_absolute([-1.0, 0.0, 0.0], blocking=False)
    time.sleep(6.0)
    print("Current base state: " + str(pepper.base.get_state()))

    # Move Pepper to [0.5, 0.2, 0.2] in the robot frame
    pepper.base.go_to_relative([0.5, 0.2, 0.2], timeout=10.0)
    print("Current base state: " + str(pepper.base.get_state()))


if __name__ == "__main__":
    main()

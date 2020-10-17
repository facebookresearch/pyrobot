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
        use_arm=True,
        use_base=False,
        use_camera=False,
        use_gripper=True)

    # Simultaneously open Pepper's right and left hands at the default speed
    # (60% of the max gripper joints speed)
    pepper.gripper.open()
    pepper.gripper.open(r_hand=False)
    time.sleep(1.0)

    # Close Pepper's right hand and then left hand (at 20% of the max gripper
    # joints speed)
    pepper.gripper.close(speed=0.2)
    time.sleep(1.0)
    pepper.gripper.close(r_hand=False, speed=0.2)
    time.sleep(1.0)

    # Set the articular position (opening percentage) of the left gripper
    pepper.gripper.set_angle(0.4, r_hand=False)
    time.sleep(1.0)
    pepper.gripper.close(r_hand=False)
    time.sleep(1.0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python
# coding: utf-8

import time
from pyrobot import Robot


def main():
    """
    Example controlling the arms of Pepper
    """
    pepper = Robot(
        'pepper',
        use_arm=True,
        use_base=False,
        use_camera=False,
        use_gripper=False)

    # Putting the right and left arms of the robot in the "home" position. This
    # is an asynchronous call
    pepper.arm.go_home()
    pepper.arm.go_home(r_arm=False)
    time.sleep(1)

    # Putting the right and left arms of the robot in the "neutral" position.
    # This is an asynchronous call
    pepper.arm.move_to_neutral()
    pepper.arm.move_to_neutral(r_arm=False)
    time.sleep(1.0)

    # Putting the right and left arms of the robot in the "home" position. This
    # is an blocking call
    pepper.arm.go_home(speed=0.4, wait=True)
    pepper.arm.go_home(r_arm=False, speed=0.4, wait=True)

    # Calling set_joint_positions on the right arm, with 20% of the max joint
    # speed. This is a blocking call
    pepper.arm.set_joint_positions(
        [1.207, -0.129, 1.194, 1.581, 1.632],
        speed=0.2,
        wait=True)

    # Calling set_joint_positions on the left arm. This is a blocking call
    pepper.arm.set_joint_positions(
        [1.207, 0.129, -1.194, -1.581, -1.632],
        r_arm=False,
        speed=0.2,
        wait=True)

    # Positions of the enf effectors
    print("ee (right) pose: " + str(pepper.arm.pose_ee))
    print("extra_ee (left) pose: " + str(pepper.arm.pose_extra_ee))

    # Move to the neutral position for the right and left arms. This is a
    # blocking call
    pepper.arm.move_to_neutral(speed=0.5, wait=True)
    pepper.arm.move_to_neutral(r_arm=False, speed=0.5, wait=True)


if __name__ == "__main__":
    main()

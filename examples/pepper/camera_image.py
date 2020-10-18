#!/usr/bin/env python
# coding: utf-8

import cv2
import time
from pyrobot import Robot


def main():
    """
    Example retrieveing camera images from the Pepper robot
    """
    pepper = Robot(
        'pepper',
        use_arm=False,
        use_base=False,
        use_camera=True,
        use_gripper=False)

    rgb = pepper.camera.get_rgb_top()
    print(rgb.shape)


if __name__ == "__main__":
    main()

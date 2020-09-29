#!/home/m_dyse/pyenvs/pyro3_env/bin/python
# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time
import math
import time
import rospy
import numpy as np
from absl import app
from pyrobot import Robot
from pyrobot.utils.util import try_cv2_import
cv = try_cv2_import()


IMAGE_PATH = '/home/m_dyse/workspaces/pyro_ws/src/pyrobot/robots/LoCoBot/locobot_riss/Data/'
IMG_EXT = 'depth_img_'
IMG_TYPE = '.png'
BOT_NAME = 'locobot'
CONFIG = {'base_controller': 'ilqr', 'base_planner': 'movebase'} # not sure why but the tutorials say so
TARGETS = np.array([[1.8,1.8,3*np.pi/2],[.5,2,np.pi],[-1.5,2.3,np.pi],[-3,3,-np.pi/2],[-4,3,-np.pi/2],[-4,0,-np.pi/2]])


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def main(_):

    bot = Robot(BOT_NAME, CONFIG)
    print('Locobot Initialized')
    current_state = bot.base.get_state('odom') # can also be 'vslam' for visual slam
    print(f'Starting at {current_state}')
    img_ctr = len([f for f in os.listdir(IMAGE_PATH) if IMG_EXT in f])

    for target in TARGETS:
    #while np.linalg.norm(bot.base.get_state('odom') - target):
        print(f'Moving to target {target}')
        bot.base.go_to_absolute(target)

        image = bot.camera.get_depth().astype(np.float64) * 1000
        im_path = os.path.join(IMAGE_PATH,IMG_EXT + str(img_ctr) + IMG_TYPE)
        print(f'Image captured: Saving Image {im_path}...')
        cv.imwrite(im_path, image)
        img_ctr += 1

        bot.base.stop()

    bot.base.stop()

if __name__ == "__main__":
    try:
        app.run(main)
    except KeyboardInterrupt:
        print('USER KILL: Exiting ...')

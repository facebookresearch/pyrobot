from pyrobot import Robot

import sys
import os

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
	sys.path.remove(ros_path)
	import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# Please change this to match your habitat_sim repo's path
path_to_habitat_sim = "/home/kalyan/temp/habitat-sim"
relative_path = "examples/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"

common_config = dict(scene_path=os.path.join(path_to_habitat_sim, relative_path))
bot = Robot('habitat', common_config=common_config)

rgb_img = bot.camera.get_rgb()
cv2.imshow('Color', rgb_img[..., 0:3][..., ::-1])
base_state = bot.base.get_state()
print("Base State:")
print(base_state)
cv2.waitKey(4000)

# Execute an action on the base to move forward
bot.base.execute_action("move_forward")

rgb_img = bot.camera.get_rgb()
cv2.imshow('Color', rgb_img[..., 0:3][..., ::-1])
base_state = bot.base.get_state()
print("Base State:")
print(base_state)
cv2.waitKey(4000)
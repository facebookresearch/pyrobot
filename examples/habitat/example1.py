from pyrobot import Robot

import sys
import os

from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()


def visualize(bot):
    rgb_img = bot.camera.get_rgb()
    cv2.imshow("Color", rgb_img[..., 0:3][..., ::-1])
    base_state = bot.base.get_state()
    print("Base State:")
    print(base_state)
    cv2.waitKey(4000)


# Please change this to match your habitat_sim repo's path
path_to_habitat_scene = os.path.dirname(os.path.realpath(__file__))
relative_path = "scenes/skokloster-castle.glb"

common_config = dict(scene_path=os.path.join(path_to_habitat_scene, relative_path))
bot = Robot("habitat", common_config=common_config)
visualize(bot)

# Execute an action on the base to move forward
bot.base.execute_action("move_forward")
print("Move forward using discreate actions")
visualize(bot)

# Make the agent go to a relative pose
print("Move the robot to a relative pose")
bot.base.go_to_relative([1.0, 1.0, 0.0])
visualize(bot)

# Make the agent adjust its camera motors
print("Set pan/tilt camera motor values.")
bot.camera.set_pan(0.1)
visualize(bot)

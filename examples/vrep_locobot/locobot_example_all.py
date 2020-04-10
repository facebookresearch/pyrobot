from __future__ import print_function

from pyrobot import Robot
from os.path import dirname, join, abspath
import numpy as np


common_config = {}
common_config["scene_path"] = join(
    dirname(abspath(__file__)), "scene_locobot_stack_cube_with_camera.ttt"
)

robot = Robot("vrep_locobot", common_config=common_config)


################ start of base ###############################

pos = [1, 1, 3.14 / 2]  # beware x, y is fliped!
robot.base.go_to_absolute(pos, smooth=False, close_loop=True)
print("Robot state = ")
print(robot.base.get_state())

###############################################################

################ start of camera ###############################

robot.camera.set_tilt(0.5)
robot.camera.set_pan(0.5)
print(robot.camera.get_tilt())

import scipy.misc  # Camera cv2.imshow has known issues with vrep

rgb, depth = robot.camera.get_rgb_depth()
scipy.misc.imsave("outfile_rgb.jpg", rgb)
scipy.misc.imsave("outfile_depth.jpg", depth)

############ end of camera #####################################

################ start of arm ##################################


# Joint control
robot.arm.set_joint_positions([0.1, 0, 0, 0, 0])
print(robot.arm.get_joint_angles())


robot.arm.set_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0])
# Inverse Kinematics
ee_pos, ee_rot, ee_quat = robot.arm.pose_ee
print(robot.arm.compute_ik(ee_pos, ee_quat))

# Taskpace control
pos = np.array([0.5, 0, 0.2])
quat = np.array([0, 0, 0, 1])
robot.arm.set_ee_pose(pos, quat)

# Task displacement control
dis = np.array([+0.05, 0, -0.05])
robot.arm.move_ee_xyz(dis)


################ End of arm ##################################

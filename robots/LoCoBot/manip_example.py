from pyrobot import Robot
import time
import numpy as np
robot = Robot('locobot')

target_poses = [{'position': np.array([0.279, 0.176, 0.217]),
                 'orientation': np.array([[0.5380200, -0.6650449, 0.5179283],
                                          [0.4758410, 0.7467951, 0.4646209],
                                          [-0.6957800, -0.0035238, 0.7182463]])},
                {'position': np.array([0.339, 0.0116, 0.255]),
                 'orientation': np.array([0.245, 0.613, -0.202, 0.723])},
                ]
robot.arm.go_home()

for pose in target_poses:
    robot.arm.set_ee_pose(**pose)
    time.sleep(1)
robot.arm.go_home()



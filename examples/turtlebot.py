from pyrobot import Robot

robot = Robot('turtlebot',
		use_base=True,
		use_arm=False,
		use_camera=False,
		use_gripper=False)

while True:
	print(robot.base.get_state('odom'))


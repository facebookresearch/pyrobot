from pyrobot import Robot

robot = Robot('husk',
		use_base=True,
		use_arm=False,
		use_camera=False,
		use_gripper=False)

# linear_velocity in m/s
linear_velocity = 2.0 

# rotational_velocity in radian / s
rotational_velocity = 0.5 

# execution_time in seconds
execution_time = 2 
i = 0
while (i<100):
# Command to execute motion
	robot.base.set_vel(fwd_speed=linear_velocity, 
                   turn_speed=rotational_velocity, 
                   exe_time=execution_time)
	print(robot.base.get_state('odom'))



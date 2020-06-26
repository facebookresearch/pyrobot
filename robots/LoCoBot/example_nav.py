from pyrobot import Robot
import time

# base_config_dict is a dictionary that contains different base configuration
# parameters. 'base_controller' can be set to 'ilqr' or 'proportional' or
# 'movebase' to use the respective controllers.
base_config_dict={'base_controller': 'ilqr'} 

# crate the Robot object with the specified base_config
robot = Robot('locobot', base_config=base_config_dict)

print(robot.base)

home = [0,0,0]

# linear_velocity in m/s
linear_velocity = 5 

# rotational_velocity in radian / s
rotational_velocity = 0 

#robot.base.set_vel(fwd_speed=linear_velocity, 
#                   turn_speed=rotational_velocity,
#		   exe_time=6)


# Now command the robot to go to the target pose in the enviroment
# 'go_to_absolute' assumes that the target is in world frame.
robot.base.go_to_relative([1, 0, 0])


time.sleep(2) 
#robot.base.go_to_relative([0,0,1])
robot.base.go_to_absolute(home)


# Targets can be specified in robot's coordinate frame.
# robot.base.go_to_relative(target_position)
